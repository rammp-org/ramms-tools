"""ramms_tools.rosbag's reader, on real rosbag2 sqlite3 bags written by rosbag_fixtures.

The opt-in tests at the end read the RAMMP recordings and are skipped unless
their directories are given:

    RAMMS_BAG_DIR=.../nov_19_perception_bag          (split: _0.db3, _1.db3)
    RAMMS_BAG_VO_DIR=.../nov_19_perception_bag_vo

They decode every message of each bag, write it back with encode() and compare
the bytes, and check the per-topic counts against the bag's metadata.yaml.
"""

from __future__ import annotations

import collections
import json
import os
import re
import shlex
import shutil
import sqlite3
import stat
import threading
import zipfile

import pytest
from rosbag_fixtures import RAMMP_TOPICS, sample_messages, without_payloads, write_bag

from ramms_tools.rosbag import BagError, BagReader, RawMessage, TopicInfo, encode, open_bag

STRING = "std_msgs/msg/String"
TOPICS = {"/a": STRING, "/b": STRING, "/quiet": STRING}


def text(topic: str, ts: int) -> tuple[str, int, bytes]:
    """A message whose payload names its own topic and time, so a mixed-up id or order shows."""
    return (topic, ts, encode(STRING, {"data": f"{topic}@{ts}"}))


def seen(reader: BagReader, **kw) -> list[tuple[str, int]]:
    out = []
    for raw in reader.messages(**kw):
        assert reader.decode(raw) == {"data": f"{raw.topic}@{raw.timestamp_ns}"}
        out.append((raw.topic, raw.timestamp_ns))
    return out


# Inserted out of time order, as a recorder receiving from several publishers may.
MESSAGES = [text("/a", 30), text("/b", 10), text("/a", 50), text("/b", 20), text("/b", 60), text("/a", 40),
            text("/a", 10)]
IN_ORDER = [("/b", 10), ("/a", 10), ("/b", 20), ("/a", 30), ("/a", 40), ("/a", 50), ("/b", 60)]


@pytest.fixture
def split_bag(tmp_path):
    return write_bag(tmp_path / "bag", TOPICS, MESSAGES, split_at_ns=35)


# ── ordering ───────────────────────────────────────────────────────

def test_messages_come_in_time_order_across_split_files(split_bag):
    with open_bag(split_bag) as bag:
        assert [os.path.basename(p) for p in bag.paths] == ["bag_0.db3", "bag_1.db3"]
        # Equal timestamps keep file order, then recording order: /b@10 was recorded before /a@10.
        assert seen(bag) == IN_ORDER


def test_topic_ids_are_looked_up_per_file(split_bag):
    """write_bag numbers the topics in opposite orders in the two files, as a real split bag may."""
    ids = []
    for name in ("bag_0.db3", "bag_1.db3"):
        con = sqlite3.connect(os.path.join(split_bag, name))
        ids.append(dict(con.execute("SELECT name, id FROM topics")))
        con.close()
    assert ids[0]["/a"] != ids[1]["/a"]
    with open_bag(split_bag) as bag:
        assert seen(bag) == IN_ORDER  # each payload names its topic, so a mix-up would fail here


def _combine(tmp_path, parts: dict[str, list]) -> str:
    """One bag directory whose files come from separate write_bag calls, renamed to *parts*' keys."""
    out = tmp_path / "combined"
    out.mkdir()
    for i, (file_name, messages) in enumerate(parts.items()):
        src = write_bag(tmp_path / f"src{i}", TOPICS, messages)
        shutil.move(os.path.join(src, f"src{i}_0.db3"), out / file_name)
    return str(out)


def test_files_whose_times_overlap_are_merged(tmp_path):
    bag_dir = _combine(tmp_path, {
        "x_0.db3": [text("/a", 10), text("/a", 30), text("/a", 50)],
        "x_1.db3": [text("/b", 20), text("/b", 40), text("/b", 60)],
    })
    with open_bag(bag_dir) as bag:
        assert seen(bag) == [("/a", 10), ("/b", 20), ("/a", 30), ("/b", 40), ("/a", 50), ("/b", 60)]


def test_split_files_sort_by_their_number(tmp_path):
    bag_dir = _combine(tmp_path, {
        "x_10.db3": [text("/a", 3)], "x_2.db3": [text("/a", 2)], "x_1.db3": [text("/a", 1)],
    })
    with open_bag(bag_dir) as bag:
        assert [os.path.basename(p) for p in bag.paths] == ["x_1.db3", "x_2.db3", "x_10.db3"]


# ── topics and bounds ──────────────────────────────────────────────

def test_topics_are_summed_over_files(split_bag):
    with open_bag(split_bag) as bag:
        assert bag.topics == {
            "/a": TopicInfo("/a", STRING, 4, 10, 50),
            "/b": TopicInfo("/b", STRING, 3, 10, 60),
            "/quiet": TopicInfo("/quiet", STRING, 0, 0, 0),  # registered, never published
        }
        assert (bag.start_ns, bag.end_ns) == (10, 60)
        assert bag.duration_s == pytest.approx(50e-9)


def test_an_empty_bag(tmp_path):
    with open_bag(write_bag(tmp_path / "e", TOPICS, [])) as bag:
        assert (bag.start_ns, bag.end_ns, bag.duration_s) == (0, 0, 0.0)
        assert list(bag.messages()) == []
        assert bag.first("/a") is None


def test_start_is_inclusive_and_end_exclusive(split_bag):
    with open_bag(split_bag) as bag:
        assert seen(bag, start_ns=20, end_ns=50) == [("/b", 20), ("/a", 30), ("/a", 40)]
        assert seen(bag, start_ns=40) == [("/a", 40), ("/a", 50), ("/b", 60)]
        assert seen(bag, end_ns=bag.end_ns) == IN_ORDER[:-1]  # end_ns=None to include the last
        assert seen(bag, start_ns=50, end_ns=50) == []


def test_topic_filter(split_bag):
    with open_bag(split_bag) as bag:
        only_a = [m for m in IN_ORDER if m[0] == "/a"]
        assert seen(bag, topics=["/a"]) == only_a
        assert seen(bag, topics="/a") == only_a  # one name as a str, not its characters
        assert seen(bag, topics=["/a", "/not/in/this/bag"]) == only_a
        assert seen(bag, topics=["/not/in/this/bag"]) == []
        assert seen(bag, topics=["/b"], start_ns=15, end_ns=61) == [("/b", 20), ("/b", 60)]


def test_first_is_the_earliest_not_the_first_inserted(split_bag):
    with open_bag(split_bag) as bag:
        assert bag.first("/a") == {"data": "/a@10"}  # inserted last
        assert bag.first("/quiet") is None
        assert bag.first("/nope") is None


def test_reader_decodes_every_type(tmp_path):
    samples = sample_messages()
    topics = {f"/t{i}": t for i, t in enumerate(samples)}
    messages = [(topic, 100 + i, encode(t, samples[t])) for i, (topic, t) in enumerate(topics.items())]
    with open_bag(write_bag(tmp_path / "all", topics, messages, split_at_ns=104)) as bag:
        raws = list(bag.messages())
        assert [r.type for r in raws] == list(samples)
        for raw in raws:
            assert bag.decode(raw) == samples[raw.type]
            assert bag.decode(raw, keep_data=False) == without_payloads(raw.type, samples[raw.type])
            assert bag.first(raw.topic) == samples[raw.type]


def test_a_single_db3_file_opens_alone(split_bag):
    with open_bag(os.path.join(split_bag, "bag_1.db3")) as bag:
        assert len(bag.paths) == 1
        assert seen(bag) == [("/a", 40), ("/a", 50), ("/b", 60)]


def test_raw_message_repr_leaves_out_the_payload():
    raw = RawMessage("/scene_camera/color/image_raw", "sensor_msgs/msg/Image", 1, bytes(768_080))
    assert repr(raw).endswith("data=<768080 bytes>)")


def test_messages_can_be_read_on_another_thread(split_bag):
    """A player opens the bag on one thread and plays it on another."""
    got = []
    with open_bag(split_bag) as bag:
        t = threading.Thread(target=lambda: got.extend(seen(bag)))
        t.start()
        t.join(timeout=10)
    assert got == IN_ORDER


# ── refusals ───────────────────────────────────────────────────────

def test_a_zip_is_refused_with_the_command_to_extract_it(tmp_path):
    folder = tmp_path / "my data"  # a space, so the command must quote
    folder.mkdir()
    archive = folder / "nov_19_perception_bag.zip"
    with zipfile.ZipFile(archive, "w") as zf:
        zf.writestr("nov_19_perception_bag/", "")
        zf.writestr("nov_19_perception_bag/metadata.yaml", "")
        zf.writestr("nov_19_perception_bag/nov_19_perception_bag_0.db3", "")
    with pytest.raises(BagError) as info:
        open_bag(archive)
    message = str(info.value)
    # The zip holds one top-level folder, so it extracts next to itself.
    assert f"unzip -d {shlex.quote(str(folder))} {shlex.quote(str(archive))}" in message
    assert f"then open {folder / 'nov_19_perception_bag'}" in message
    assert "already" not in message

    write_bag(folder / "nov_19_perception_bag", TOPICS, MESSAGES)
    with pytest.raises(BagError, match="an extracted copy is already there"):
        open_bag(archive)


def test_a_flat_zip_extracts_into_a_folder_of_its_own(tmp_path):
    archive = tmp_path / "flat.zip"
    with zipfile.ZipFile(archive, "w") as zf:
        zf.writestr("metadata.yaml", "")
        zf.writestr("flat_0.db3", "")
    with pytest.raises(BagError) as info:
        open_bag(str(archive))
    assert f"unzip -d {shlex.quote(str(tmp_path / 'flat'))} {shlex.quote(str(archive))}" in str(info.value)


def test_a_missing_zip_is_just_missing(tmp_path):
    with pytest.raises(BagError, match="no such file"):
        open_bag(tmp_path / "nope.zip")


@pytest.mark.parametrize("mode", ["FILE", "MESSAGE"])
def test_a_compressed_bag_is_refused(tmp_path, mode):
    bag_dir = write_bag(tmp_path / "z", TOPICS, MESSAGES, compression_mode=mode)
    for path in (bag_dir, os.path.join(bag_dir, "z_0.db3")):
        with pytest.raises(BagError, match=f"compressed \\(zstd, compression_mode {mode}\\)"):
            open_bag(path)


def test_a_quoted_compression_mode_is_read_too(tmp_path):
    bag_dir = write_bag(tmp_path / "q", TOPICS, MESSAGES)
    meta = os.path.join(bag_dir, "metadata.yaml")
    with open(meta, encoding="utf-8") as fh:
        text_ = fh.read()
    assert 'compression_mode: ""' in text_
    with open(meta, "w", encoding="utf-8") as fh:
        fh.write(text_.replace('compression_mode: ""', "compression_mode: 'FILE'"))
    with pytest.raises(BagError, match="compression_mode FILE"):
        open_bag(bag_dir)


def test_paths_that_are_not_bags(tmp_path):
    with pytest.raises(BagError, match="no such bag"):
        open_bag(tmp_path / "missing")
    (tmp_path / "empty").mkdir()
    with pytest.raises(BagError, match="no .db3 files"):
        open_bag(tmp_path / "empty")
    (tmp_path / "mcap").mkdir()
    (tmp_path / "mcap" / "x_0.mcap").write_bytes(b"")
    with pytest.raises(BagError, match="x_0.mcap"):
        open_bag(tmp_path / "mcap")
    (tmp_path / "notes.txt").write_text("hi")
    with pytest.raises(BagError, match="neither a bag directory nor a .db3 file"):
        open_bag(tmp_path / "notes.txt")
    (tmp_path / "junk.db3").write_bytes(b"not a database at all, just some bytes" * 100)
    with pytest.raises(BagError, match="cannot read"):
        open_bag(tmp_path / "junk.db3")
    con = sqlite3.connect(tmp_path / "other.db3")
    con.execute("CREATE TABLE t (x)")
    con.close()
    with pytest.raises(BagError, match="not a rosbag2 sqlite3 file"):
        open_bag(tmp_path / "other.db3")


def test_a_closed_reader_says_so(split_bag):
    bag = open_bag(split_bag)
    started = bag.messages()
    next(started)
    bag.close()
    bag.close()  # twice is fine
    with pytest.raises(BagError, match="closed"):
        bag.messages()
    with pytest.raises(BagError):
        list(started)


# ── nothing is written next to the data ────────────────────────────

def _snapshot(directory):
    return {n: os.stat(os.path.join(directory, n)).st_mtime_ns for n in sorted(os.listdir(directory))}


def test_reading_writes_nothing_next_to_the_bag(split_bag):
    before = _snapshot(split_bag)
    with open_bag(split_bag) as bag:
        list(bag.messages())
        bag.first("/b")
    assert _snapshot(split_bag) == before  # no -journal, -wal or -shm, and no file touched


@pytest.mark.skipif(hasattr(os, "geteuid") and os.geteuid() == 0, reason="root ignores file permissions")
def test_a_read_only_bag_can_be_read(split_bag):
    paths = [os.path.join(split_bag, n) for n in os.listdir(split_bag)]
    try:
        for p in paths:
            os.chmod(p, stat.S_IRUSR)
        os.chmod(split_bag, stat.S_IRUSR | stat.S_IXUSR)
        with open_bag(split_bag) as bag:
            assert seen(bag) == IN_ORDER
    finally:
        os.chmod(split_bag, stat.S_IRWXU)
        for p in paths:
            os.chmod(p, stat.S_IRUSR | stat.S_IWUSR)


# ── opt-in: the real RAMMP bags ────────────────────────────────────

def _opt_in(var: str):
    return pytest.param(var, marks=pytest.mark.skipif(
        not os.environ.get(var), reason=f"set {var} to a RAMMP bag directory to read it"))


REAL_BAGS = [_opt_in("RAMMS_BAG_DIR"), _opt_in("RAMMS_BAG_VO_DIR")]


def _metadata(bag_dir: str) -> dict:
    """What metadata.yaml says, read with regexes (no YAML dependency)."""
    with open(os.path.join(bag_dir, "metadata.yaml"), encoding="utf-8") as fh:
        text_ = fh.read()
    topics = re.findall(
        r"^ +name: (\S+)\n +type: (\S+)\n(?: +\S+: .*\n)*? +message_count: (\d+)$", text_, re.MULTILINE)
    return {
        "types": {name: typ for name, typ, _ in topics},
        "counts": {name: int(n) for name, _, n in topics},
        "start_ns": int(re.search(r"^  starting_time:\n +nanoseconds_since_epoch: (\d+)", text_, re.M).group(1)),
        "duration_ns": int(re.search(r"^  duration:\n +nanoseconds: (\d+)", text_, re.M).group(1)),
        "message_count": int(re.search(r"^  message_count: (\d+)", text_, re.M).group(1)),
        "files": re.findall(r"^    - path: (\S+)", text_, re.M),
    }


@pytest.mark.parametrize("var", REAL_BAGS)
def test_real_bag_every_message_decodes_and_re_encodes(var):
    bag_dir = os.environ[var]
    meta = _metadata(bag_dir)
    counts = collections.Counter()
    previous = None
    with open_bag(bag_dir) as bag:
        assert [os.path.basename(p) for p in bag.paths] == meta["files"]
        assert {n: t.type for n, t in bag.topics.items()} == meta["types"]
        assert {n: t.count for n, t in bag.topics.items()} == meta["counts"]
        assert sum(meta["counts"].values()) == meta["message_count"]
        assert bag.start_ns == meta["start_ns"]
        assert bag.end_ns - bag.start_ns == meta["duration_ns"]
        for raw in bag.messages():
            assert previous is None or raw.timestamp_ns >= previous
            previous = raw.timestamp_ns
            msg = bag.decode(raw)  # raises on more than 3 bytes left over
            again = encode(raw.type, msg)
            # Byte for byte, up to the zero padding: robot-recorded messages carry
            # it (as encode() writes it), the vo bag's re-written topics do not.
            n = min(len(again), len(raw.data))
            assert again[:n] == raw.data[:n] and not any(again[n:]) and not any(raw.data[n:]), raw
            assert len(raw.data) - n <= 3
            # repr, not ==: /joint_states holds NaN (the gripper's velocity and effort), and NaN != NaN.
            assert repr(bag.decode(raw, keep_data=False)) == repr(without_payloads(raw.type, msg))
            counts[raw.topic] += 1
    assert dict(counts) == {n: c for n, c in meta["counts"].items() if c}


@pytest.mark.parametrize("var", [_opt_in("RAMMS_BAG_DIR")])
def test_real_bag_window_across_the_split(var):
    """A window over the boundary of _0.db3 and _1.db3 matches a direct count in each file."""
    bag_dir = os.environ[var]
    with open_bag(bag_dir) as bag:
        assert len(bag.paths) == 2
        con = sqlite3.connect(f"file:{bag.paths[1]}?mode=ro&immutable=1", uri=True)
        boundary = con.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
        con.close()
        start, end = boundary - 250_000_000, boundary + 250_000_000
        expected = 0
        for path in bag.paths:
            con = sqlite3.connect(f"file:{path}?mode=ro&immutable=1", uri=True)
            expected += con.execute(
                "SELECT COUNT(*) FROM messages WHERE timestamp >= ? AND timestamp < ?", (start, end)).fetchone()[0]
            con.close()
        stamps = [raw.timestamp_ns for raw in bag.messages(start_ns=start, end_ns=end)]
        assert len(stamps) == expected
        assert stamps == sorted(stamps) and start <= stamps[0] < boundary <= stamps[-1] < end

        info = bag.first("/scene_camera/color/camera_info")
        assert info["header"]["frame_id"] == "scene_camera_color_optical_frame"
        assert (info["width"], info["height"]) == (640, 400)
        assert info["k"][0] == pytest.approx(283.775, abs=1e-3)
        assert bag.first("/base/odom")["child_frame_id"] == "base_link"
        assert set(bag.topics) == set(RAMMP_TOPICS) - {"/perception/odom3d/ui_payload"}


@pytest.mark.parametrize("var", [_opt_in("RAMMS_BAG_VO_DIR")])
def test_real_vo_bag_ui_payload_is_json(var):
    with open_bag(os.environ[var]) as bag:
        payloads = [bag.decode(raw)["data"] for raw in bag.messages(topics="/perception/odom3d/ui_payload")]
    assert len(payloads) == 1548
    for p in payloads:
        assert isinstance(json.loads(p)["objects"], list)
