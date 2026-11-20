"""Read-only access to rosbag2 sqlite3 bags (Humble, storage schema v3).

No ROS install is needed -- the robot's bags are read on development machines
that have none -- so this talks to the ``.db3`` files with the standard
library's ``sqlite3``. A rosbag2 sqlite3 file holds four tables:

- ``schema(schema_version, ros_distro)`` -- ``(3, 'humble')`` here;
- ``metadata`` -- empty in Humble;
- ``topics(id, name, type, serialization_format, offered_qos_profiles)``;
- ``messages(id, topic_id, timestamp, data)`` with an index on ``timestamp``.

``timestamp`` is when the recorder *received* the message (ns since the
epoch), not the stamp in its header; it is what a player paces itself by.
Topic ids are per file -- the same topic has different ids in the two files of
a split bag -- so every lookup goes through that file's own ``topics`` table.

Every file is opened ``mode=ro&immutable=1``: read-only, and with no locking
or journal, so nothing (no ``-journal``, ``-wal`` or ``-shm`` file) is ever
created next to the data, which may be a shared or read-only folder.
"""

from __future__ import annotations

import heapq
import os
import re
import shlex
import sqlite3
import zipfile
from dataclasses import dataclass
from operator import attrgetter
from typing import Iterable, Iterator
from urllib.request import pathname2url

from ramms_tools.rosbag.cdr import BagError, decode

__all__ = ["TopicInfo", "RawMessage", "BagReader", "open_bag"]


@dataclass(frozen=True)
class TopicInfo:
    """One topic over the whole bag. ``first_ns``/``last_ns`` are 0 when ``count`` is 0."""

    name: str
    type: str
    count: int
    first_ns: int
    last_ns: int


@dataclass(frozen=True)
class RawMessage:
    """One recorded message, undecoded. ``timestamp_ns`` is the bag (receive) time."""

    topic: str
    type: str
    timestamp_ns: int
    data: bytes

    def __repr__(self) -> str:
        # The generated repr would print the whole payload -- 768 KB for a camera frame.
        return (f"RawMessage(topic={self.topic!r}, type={self.type!r}, "
                f"timestamp_ns={self.timestamp_ns}, data=<{len(self.data)} bytes>)")


# ── opening ─────────────────────────────────────────────────────────

_SPLIT_INDEX = re.compile(r"_(\d+)\.db3$")
_COMPRESSION_MODE = re.compile(r"^\s*compression_mode:\s*(.*?)\s*$", re.MULTILINE)
_COMPRESSION_FORMAT = re.compile(r"^\s*compression_format:\s*(.*?)\s*$", re.MULTILINE)


def open_bag(path) -> BagReader:
    """Open a bag directory (all its ``*.db3`` files) or a single ``.db3`` file.

    Raises :class:`BagError` for a path that does not exist or holds no
    ``.db3`` file, for a ``.zip`` (sqlite cannot read a file inside an
    archive; the message gives the ``unzip`` command), for a bag whose
    ``metadata.yaml`` declares compression, and for a file that is not a
    rosbag2 sqlite3 database.
    """
    path = os.path.abspath(os.fspath(path))
    if path.lower().endswith(".zip"):
        if not os.path.isfile(path):
            raise BagError(f"no such file: {path}")
        raise BagError(_zip_message(path))
    if os.path.isdir(path):
        _refuse_compressed(os.path.join(path, "metadata.yaml"))
        files = _db3_files(path)
        if not files:
            others = sorted(n for n in os.listdir(path) if n.endswith((".mcap", ".zstd")))
            hint = f" (found {', '.join(others)}: only uncompressed sqlite3 bags are read)" if others else ""
            raise BagError(f"no .db3 files in {path}{hint}")
    elif os.path.isfile(path):
        if not path.endswith(".db3"):
            raise BagError(f"{path} is neither a bag directory nor a .db3 file")
        # A single file of a compressed bag is just as unreadable, so check its folder's metadata too.
        _refuse_compressed(os.path.join(os.path.dirname(path), "metadata.yaml"))
        files = [path]
    else:
        raise BagError(f"no such bag: {path}")
    return BagReader(files)


def _db3_files(directory: str) -> list[str]:
    """The directory's ``*.db3`` files in split order: by the trailing ``_N``, numerically.

    A plain name sort would put ``_10`` before ``_2``. Files without an index sort first.
    """
    def key(name: str):
        m = _SPLIT_INDEX.search(name)
        return (1, int(m.group(1)), name) if m else (0, 0, name)

    names = [n for n in os.listdir(directory) if n.endswith(".db3")]
    return [os.path.join(directory, n) for n in sorted(names, key=key)]


def _unquote(value: str) -> str:
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        return value[1:-1]
    return value


def _refuse_compressed(metadata_path: str) -> None:
    """Refuse a bag whose metadata.yaml declares a compression mode.

    A ``FILE``-compressed bag's files are whole zstd archives and a
    ``MESSAGE``-compressed one's blobs are zstd frames: neither is CDR. Read
    with a regex because a YAML parser would be a new dependency for one key.
    """
    try:
        with open(metadata_path, encoding="utf-8", errors="replace") as fh:
            text = fh.read()
    except FileNotFoundError:
        return  # metadata.yaml is optional: the db3 files are self-describing
    m = _COMPRESSION_MODE.search(text)
    mode = _unquote(m.group(1)) if m else ""
    if not mode:
        return
    f = _COMPRESSION_FORMAT.search(text)
    fmt = (_unquote(f.group(1)) if f else "") or "unknown format"
    raise BagError(
        f"{os.path.dirname(metadata_path)} is compressed ({fmt}, compression_mode {mode}); only "
        "uncompressed bags are read. Write an uncompressed copy first, e.g. with `ros2 bag convert` "
        "and an output bag whose compression_mode is empty."
    )


def _zip_message(zip_path: str) -> str:
    """Why a zip is refused, and the exact command that makes it readable.

    sqlite opens files, not archive members, and the RAMMP bags are Deflate
    zips (no way to read them in place), so they have to be extracted. The
    zip's member list (its central directory, cheap to read) tells where the
    bag lands: a zip with one top-level folder extracts next to itself.
    """
    parent = os.path.dirname(zip_path)
    stem = os.path.splitext(os.path.basename(zip_path))[0]
    try:
        with zipfile.ZipFile(zip_path) as zf:
            tops = {n.split("/", 1)[0] for n in zf.namelist() if n.strip("/")}
            nested = all("/" in n for n in zf.namelist() if n.strip("/"))
    except (zipfile.BadZipFile, OSError):
        tops, nested = set(), False
    if len(tops) == 1 and nested:
        target, bag_dir = parent, os.path.join(parent, next(iter(tops)))
    else:
        target = bag_dir = os.path.join(parent, stem)
    lines = [
        f"{zip_path} is a zip archive; sqlite cannot read a bag inside one. Extract it first:",
        f"  unzip -d {shlex.quote(target)} {shlex.quote(zip_path)}",
        f"then open {bag_dir}",
    ]
    if os.path.isdir(bag_dir) and _db3_files(bag_dir):
        lines.append(f"(an extracted copy is already there: open {bag_dir})")
    return "\n".join(lines)


def _connect(path: str) -> sqlite3.Connection:
    # pathname2url percent-escapes the path, so a '?' or '#' in a folder name
    # cannot end the file name early and open some other file.
    uri = "file:" + pathname2url(path) + "?mode=ro&immutable=1"
    # check_same_thread=False: a player opens the bag on one thread and plays
    # it on another. Safe here: the connection only ever reads.
    return sqlite3.connect(uri, uri=True, check_same_thread=False)


# ── the reader ──────────────────────────────────────────────────────

class _File:
    """One open ``.db3``: its connection and its own topic-id table."""

    __slots__ = ("path", "con", "topics")

    def __init__(self, path: str):
        self.path = path
        self.con = None
        try:
            self.con = _connect(path)
            tables = {row[0] for row in self.con.execute("SELECT name FROM sqlite_master WHERE type = 'table'")}
            if not {"topics", "messages"} <= tables:
                raise BagError(f"{path} is not a rosbag2 sqlite3 file (it has no topics/messages tables)")
            # id -> (name, type)
            self.topics = {row[0]: (row[1], row[2]) for row in self.con.execute("SELECT id, name, type FROM topics")}
        except sqlite3.Error as exc:
            self.close()
            raise BagError(f"cannot read {path}: {exc}") from exc
        except BagError:
            self.close()
            raise

    def ids_for(self, names: set[str] | None) -> list[int] | None:
        """This file's ids for *names*; None means every topic."""
        if names is None:
            return None
        return [tid for tid, (name, _) in self.topics.items() if name in names]

    def close(self) -> None:
        if self.con is not None:
            self.con.close()
            self.con = None


class BagReader:
    """A rosbag2 sqlite3 bag of one or more files, read in timestamp order.

    Usually made by :func:`open_bag`. Use it as a context manager, or call
    :meth:`close`.

    Attributes:
        paths: the ``.db3`` files, in split order.
        topics: name -> :class:`TopicInfo`, counted over all files.
        start_ns, end_ns: the first and last message timestamps (0 for an empty bag).
        duration_s: ``(end_ns - start_ns) / 1e9``.
    """

    def __init__(self, paths: Iterable[str]):
        self.paths: list[str] = [os.fspath(p) for p in paths]
        self._files: list[_File] = []
        self._closed = False
        try:
            for p in self.paths:
                self._files.append(_File(p))
            self.topics: dict[str, TopicInfo] = self._count_topics()
        except BaseException:
            self.close()
            raise
        live = [t for t in self.topics.values() if t.count]
        self.start_ns: int = min((t.first_ns for t in live), default=0)
        self.end_ns: int = max((t.last_ns for t in live), default=0)
        self.duration_s: float = (self.end_ns - self.start_ns) / 1e9

    def _count_topics(self) -> dict[str, TopicInfo]:
        """Per-topic count and first/last timestamps, summed over files with SQL aggregates.

        One GROUP BY per file reads only the small fixed columns ahead of each
        blob, so it costs tens of milliseconds even on a 4 GB file.
        """
        acc: dict[str, list] = {}  # name -> [type, count, first, last]
        for f in self._files:
            per_id = {
                tid: (n, lo, hi)
                for tid, n, lo, hi in f.con.execute(
                    "SELECT topic_id, COUNT(*), MIN(timestamp), MAX(timestamp) FROM messages GROUP BY topic_id"
                )
            }
            for tid, (name, typ) in f.topics.items():
                n, lo, hi = per_id.get(tid, (0, None, None))
                entry = acc.setdefault(name, [typ, 0, None, None])
                if entry[0] != typ:
                    raise BagError(f"topic {name} is {entry[0]} in one file and {typ} in {f.path}")
                entry[1] += n
                if n:
                    entry[2] = lo if entry[2] is None else min(entry[2], lo)
                    entry[3] = hi if entry[3] is None else max(entry[3], hi)
        return {
            name: TopicInfo(name, typ, n, lo or 0, hi or 0)
            for name, (typ, n, lo, hi) in sorted(acc.items())
        }

    # ── reading ──

    def messages(self, topics=None, start_ns=None, end_ns=None) -> Iterator[RawMessage]:
        """Yield the messages in timestamp order across all files.

        *topics* limits it to those names (one name may be given as a str);
        names the bag does not have are ignored, so one list can serve bags
        that lack some topics -- check :attr:`topics` to tell. *start_ns* is
        inclusive and *end_ns* exclusive, so ``end_ns=reader.end_ns`` leaves
        out the last message; pass None for no bound. Both go to the
        ``timestamp`` index, so starting late in a 4 GB file costs nothing.

        Each file is read through its own cursor and the cursors are merged
        (``heapq.merge``), so files whose time ranges overlap still come out in
        order. Messages with equal timestamps keep file order, then recording order.
        """
        self._check_open()
        if isinstance(topics, str):
            topics = (topics,)
        wanted = None if topics is None else set(topics)
        clauses, params = [], []
        if start_ns is not None:
            clauses.append("timestamp >= ?")
            params.append(int(start_ns))
        if end_ns is not None:
            clauses.append("timestamp < ?")
            params.append(int(end_ns))
        streams = []
        for f in self._files:
            ids = f.ids_for(wanted)
            if ids is not None and not ids:
                continue
            where = list(clauses)
            if ids is not None:
                where.append(f"topic_id IN ({','.join(map(str, ids))})")
            sql = "SELECT topic_id, timestamp, data FROM messages"
            if where:
                sql += " WHERE " + " AND ".join(where)
            # (timestamp, id) is the index's own order, so sqlite sorts nothing.
            sql += " ORDER BY timestamp, id"
            streams.append(self._rows(f, sql, params))
        if len(streams) == 1:
            return streams[0]
        return heapq.merge(*streams, key=attrgetter("timestamp_ns"))

    @staticmethod
    def _rows(f: _File, sql: str, params: list) -> Iterator[RawMessage]:
        if f.con is None:  # closed between messages() and the first next()
            raise BagError("the bag is closed")
        try:
            cur = f.con.execute(sql, params)
        except sqlite3.Error as exc:
            raise BagError(f"reading {f.path} failed: {exc}") from exc
        try:
            while True:
                # Only the fetch is guarded: an error raised into the generator
                # at its yield (GeneratorExit on close) must pass through untouched.
                try:
                    row = cur.fetchone()
                except sqlite3.Error as exc:  # e.g. the reader was closed under a running iterator
                    raise BagError(f"reading {f.path} failed: {exc}") from exc
                if row is None:
                    return
                tid, ts, data = row
                try:
                    name, typ = f.topics[tid]
                except KeyError:
                    raise BagError(f"{f.path}: a message has topic id {tid}, which its topics table lacks") from None
                yield RawMessage(name, typ, ts, data)
        finally:
            try:
                cur.close()
            except sqlite3.Error:
                pass  # its connection is already closed, which finalized it

    def decode(self, raw: RawMessage, keep_data: bool = True) -> dict:
        """Decode *raw* (see :func:`ramms_tools.rosbag.decode`)."""
        return decode(raw.type, raw.data, keep_data)

    def first(self, topic: str) -> dict | None:
        """The first message on *topic*, decoded; None if the bag has none."""
        it = self.messages(topics=(topic,))
        try:
            raw = next(it, None)
        finally:
            it.close()  # release the cursors now rather than at garbage collection
        return None if raw is None else self.decode(raw)

    # ── lifetime ──

    def _check_open(self) -> None:
        if self._closed:
            raise BagError("the bag is closed")

    def close(self) -> None:
        """Close every file. Safe to call twice."""
        self._closed = True
        for f in self._files:
            f.close()

    def __enter__(self) -> BagReader:
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __repr__(self) -> str:
        return (f"BagReader({len(self.paths)} file(s), {len(self.topics)} topics, "
                f"{self.duration_s:.3f} s)")
