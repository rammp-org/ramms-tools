"""Offline tests for ramms_tools.streaming (run: python -m pytest tests)."""
import io
import json
import socket
import struct
import threading
import time

import pytest

from ramms_tools.streaming.client import StreamClient, _decompress_rgbd
from ramms_tools.streaming.protocol import (
    HEADER_SIZE, Compression, MessageType, StreamHeader, StreamMessage,
)
from ramms_tools.streaming.sender import StreamSender


def message(mtype, channel=0, payload=b"", meta=b"", seq=0) -> bytes:
    m = StreamMessage()
    m.header.message_type = mtype
    m.header.channel_id = channel
    m.header.sequence_num = seq
    m.metadata = meta
    m.payload = payload
    return m.serialize()


def receive(chunks, count, timeout=3.0):
    """Feed chunks to a client's receive loop over a socket pair; return what it delivers."""
    client = StreamClient()
    ours, theirs = socket.socketpair()
    client._sock = ours
    client._connected.set()
    t = threading.Thread(target=client._recv_loop, daemon=True)
    t.start()
    for c in chunks:
        theirs.sendall(c)
        time.sleep(0.02)
    got, end = [], time.monotonic() + timeout
    while len(got) < count and time.monotonic() < end:
        m = client.poll(timeout=0.1)
        if m is not None:
            got.append(m)
    client._stopping.set()
    t.join(timeout=2)
    theirs.close()
    ours.close()
    return got


def test_unknown_opcode_is_skipped_not_fatal():
    unknown = bytearray(message(MessageType.FRAME_RGB, payload=b"x"))
    unknown[5] = 0x7E                      # an opcode this client doesn't know
    got = receive([bytes(unknown) + message(MessageType.FRAME_RGB, channel=3, payload=b"abc")], 1)
    assert [m.header.channel_id for m in got] == [3]


def test_resyncs_after_garbage():
    garbage = b"\x00junk-not-a-header" * 4  # more than a header's worth
    got = receive([garbage + message(MessageType.FRAME_RGB, channel=1, payload=b"ok")], 1)
    assert len(got) == 1 and got[0].payload == b"ok"


def test_large_frame_in_pieces():
    payload = bytes(range(256)) * 4096     # 1 MiB
    data = message(MessageType.FRAME_RGB, channel=2, payload=payload)
    got = receive([data[i:i + 65536] for i in range(0, len(data), 65536)], 1)
    assert len(got) == 1 and got[0].payload == payload


def test_compressed_rgbd_keeps_depth():
    Image = pytest.importorskip("PIL.Image")
    pytest.importorskip("lz4")
    from ramms_tools.streaming.compression import compress_lz4

    w, h = 8, 4
    buf = io.BytesIO()
    Image.new("RGB", (w, h), (10, 20, 30)).save(buf, "JPEG")
    jpeg = buf.getvalue()
    depth_raw = struct.pack("<%df" % (w * h), *([123.5] * (w * h)))
    depth = compress_lz4(depth_raw)
    m = StreamMessage()
    m.header.message_type = MessageType.FRAME_RGBD
    m.header.set_compression(Compression.JPEG)
    m.metadata = json.dumps({"w": w, "h": h, "rgb_size": len(jpeg), "depth_size": len(depth),
                             "rgb_comp": "jpeg", "depth_comp": "lz4"}).encode()
    m.payload = jpeg + depth
    payload, meta = _decompress_rgbd(m)
    meta = json.loads(meta)
    assert meta["rgb_size"] == w * h * 4 and meta["depth_size"] == len(depth_raw)
    assert payload[meta["rgb_size"]:] == depth_raw
    assert "rgb_comp" not in meta


def test_sender_context_sends_what_it_queued():
    server = socket.create_server(("127.0.0.1", 0))
    port = server.getsockname()[1]
    received = bytearray()

    def serve():
        conn, _ = server.accept()
        with conn:
            while chunk := conn.recv(65536):
                received.extend(chunk)

    t = threading.Thread(target=serve, daemon=True)
    t.start()
    meta = {"camera": "front"}   # one dict reused for every frame
    with StreamSender("127.0.0.1", port) as s:
        for i in range(1, 5):
            s.send_image(channel=0, image_bytes=b"\x00" * (4 * i), width=i, height=1,
                         fmt="bgra8", metadata=meta)
    t.join(timeout=5)
    server.close()
    assert meta == {"camera": "front"}
    widths, off = [], 0
    while off + HEADER_SIZE <= len(received):
        h = StreamHeader.from_bytes(bytes(received[off:off + HEADER_SIZE]))
        m = json.loads(bytes(received[off + HEADER_SIZE:off + HEADER_SIZE + h.metadata_len]))
        widths.append(m["w"])
        off += h.total_message_size
    assert widths == [1, 2, 3, 4]   # all four sent, each with its own size


def test_frame_without_its_codec_is_delivered_as_sent(monkeypatch):
    from ramms_tools.streaming import compression
    monkeypatch.setattr(compression, "_has_lz4", False)
    monkeypatch.setattr(compression, "_has_pillow", False)
    sent = []
    for mtype, comp, meta in (
        (MessageType.FRAME_DEPTH, Compression.LZ4, b'{"w": 2, "h": 1}'),
        (MessageType.FRAME_RGBD, Compression.JPEG,
         b'{"w": 2, "h": 1, "rgb_size": 3, "rgb_comp": "jpeg", "depth_comp": "lz4"}'),
    ):
        m = StreamMessage()
        m.header.message_type = mtype
        m.header.set_compression(comp)
        m.metadata = meta
        m.payload = b"\x01\x02\x03compressed"
        sent.append(m)
    got = receive([m.serialize() for m in sent], 2)
    assert [(m.header.get_compression(), m.payload, m.metadata) for m in got] == \
        [(m.header.get_compression(), m.payload, m.metadata) for m in sent]


def test_send_capture_dir_raises_without_openexr(monkeypatch, tmp_path):
    import sys
    monkeypatch.setitem(sys.modules, "OpenEXR", None)   # import OpenEXR -> ImportError
    (tmp_path / "frame_0000000.json").write_text("{}")
    (tmp_path / "frame_0000000.exr").write_bytes(b"")
    with pytest.raises(ImportError, match=r"ramms-tools\[exr\]"):
        StreamSender().send_capture_dir(tmp_path)


def test_stream_test_cli_exits_without_openexr(monkeypatch, tmp_path, capsys):
    import sys
    from ramms_tools.scripts import stream_test
    monkeypatch.setitem(sys.modules, "OpenEXR", None)
    monkeypatch.setattr(sys, "argv", ["ramms-stream-test", "--mask-dir", str(tmp_path)])
    with pytest.raises(SystemExit) as e:
        stream_test.main()
    assert e.value.code == 1 and "ramms-tools[exr]" in capsys.readouterr().out
