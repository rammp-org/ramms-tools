"""The CDR decoders and encoders in ramms_tools.rosbag (no bag files, no ROS).

The layouts were checked against every message of two real RAMMP bags (the
opt-in tests in test_rosbag_reader.py). These tests pin the rules that check
rests on -- where CDR puts padding, how strings and empty sequences are
written -- with byte layouts spelled out by hand, independent of the encoder,
and they check that a message of the wrong layout is refused rather than
half-decoded.
"""

from __future__ import annotations

import struct

import pytest
from rosbag_fixtures import sample_messages, without_payloads

from ramms_tools.rosbag import SUPPORTED_TYPES, BagDecodeError, BagError, decode, encode

CDR_LE = b"\x00\x01\x00\x00"  # encapsulation: CDR little-endian, options 0

DETECTIONS = "rammp_perception_interfaces/msg/ObjectDetections"
STRING = "std_msgs/msg/String"
JOINTS = "sensor_msgs/msg/JointState"


def le(fmt: str, *values) -> bytes:
    return struct.pack("<" + fmt, *values)


def cdr_string(raw: bytes) -> bytes:
    """uint32 length counting the NUL, the bytes, the NUL. Callers add the padding that follows."""
    return le("I", len(raw) + 1) + raw + b"\0"


# ── alignment ──────────────────────────────────────────────────────

POSE = {"position": {"x": 0.25, "y": -0.5, "z": 1.5}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
SIZE = {"x": 0.5, "y": 0.25, "z": 0.125}
BOX = [187.25, 23.25, 352.0, 299.0]

# An empty label_image (what the 15 Hz tracker sends), starting 8-aligned.
EMPTY_LABEL = (
    le("iI", 5, 250) + cdr_string(b"cam")   # header
    + le("II", 0, 0)                        # height, width
    + cdr_string(b"")                       # encoding ""
    + b"\0"                                 # is_bigendian
    + b"\0\0"                               # pad: step is uint32
    + le("I", 0)                            # step
    + le("I", 0)                            # data: no bytes
)


def detections_bytes(instance_id: bytes, pad_after_id: bytes, pad_before_pose: bytes) -> bytes:
    # Payload offsets are noted for instance_id "t"; "table_2" shifts them so the Pose pad differs.
    return (
        CDR_LE
        + le("iI", 5, 250) + cdr_string(b"cam")  # 0..16  header
        + le("I", 1)                             # 16..20 one object
        + cdr_string(instance_id) + pad_after_id
        + cdr_string(b"table") + b"\0\0"         # pad: score is float32
        + le("f", 0.75)                          # score
        + b"\x01" + b"\0\0\0"                    # state, then pad: box is float32[4]
        + le("4f", *BOX)
        + le("H", 7) + b"\x01"                   # mask_id, has_pose
        + pad_before_pose                        # pad: Pose is float64, 8-aligned from the payload start
        + le("7d", 0.25, -0.5, 1.5, 0.0, 0.0, 0.0, 1.0)
        + le("3d", 0.5, 0.25, 0.125)
        + EMPTY_LABEL
    )


@pytest.mark.parametrize("instance_id, pad_after_id, pad_before_pose", [
    # "t": id ends at payload 26 -> 2 pad; has_pose ends at 67 -> 5 pad to 72.
    (b"t", b"\0\0", b"\0" * 5),
    # "table_2": id ends at 32 -> no pad; has_pose ends at 71 -> 1 pad to 72.
    (b"table_2", b"", b"\0"),
])
def test_detected_object_padding(instance_id, pad_after_id, pad_before_pose):
    data = detections_bytes(instance_id, pad_after_id, pad_before_pose)
    expected = {
        "header": {"stamp_ns": 5_000_000_250, "frame_id": "cam"},
        "objects": [{
            "instance_id": instance_id.decode(), "perception_class": "table", "score": 0.75, "state": 1,
            "box": BOX, "mask_id": 7, "has_pose": True, "pose": POSE, "size": SIZE,
        }],
        "label_image": {"header": {"stamp_ns": 5_000_000_250, "frame_id": "cam"}, "height": 0, "width": 0,
                        "encoding": "", "is_bigendian": 0, "step": 0, "data": b""},
    }
    assert decode(DETECTIONS, data) == expected
    assert encode(DETECTIONS, expected) == data


def test_alignment_counts_from_the_payload_not_the_buffer():
    """A float64 is 8-aligned from byte 4 (after the encapsulation header), not from byte 0."""
    data = (
        CDR_LE
        + le("I", 1)                            # 0..4   one transform
        + le("iI", 1, 2) + cdr_string(b"w")     # 4..18  header
        + b"\0\0"                               # pad: the next length is uint32
        + cdr_string(b"c")                      # 20..26 child_frame_id
        + b"\0" * 6                             # pad to payload 32 (buffer 36); buffer-relative would stop at 28
        + le("3d", 1.0, 2.0, 3.0) + le("4d", 0.0, 0.0, 0.0, 1.0)
    )
    msg = decode("tf2_msgs/msg/TFMessage", data)
    assert msg["transforms"][0]["transform"]["translation"] == {"x": 1.0, "y": 2.0, "z": 3.0}
    assert encode("tf2_msgs/msg/TFMessage", msg) == data


def test_an_empty_sequence_is_only_its_count():
    """Fast-CDR aligns for a sequence's elements only when there are some."""
    header = le("iI", 0, 0) + cdr_string(b"abcde") + b"\0\0"  # 0..20, then pad: the count is uint32
    empty = CDR_LE + header + le("I", 0) + le("I", 0) + le("I", 0) + le("I", 0)
    # position's count ends at payload 28: an empty float64[] there must not pad to 32.
    assert decode(JOINTS, empty) == {"header": {"stamp_ns": 0, "frame_id": "abcde"},
                                     "name": [], "position": [], "velocity": [], "effort": []}
    one = CDR_LE + header + le("I", 0) + le("I", 1) + b"\0" * 4 + le("d", 1.5) + le("I", 0) + le("I", 0)
    assert decode(JOINTS, one)["position"] == [1.5]
    for data in (empty, one):
        assert encode(JOINTS, decode(JOINTS, data)) == data


# ── round trips ────────────────────────────────────────────────────

def test_samples_cover_every_supported_type():
    assert set(sample_messages()) == set(SUPPORTED_TYPES) == {
        "sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/CameraInfo",
        "sensor_msgs/msg/JointState", "nav_msgs/msg/Odometry", "tf2_msgs/msg/TFMessage",
        "visualization_msgs/msg/MarkerArray", DETECTIONS, STRING,
    }


@pytest.mark.parametrize("type_name", SUPPORTED_TYPES)
def test_round_trip(type_name):
    msg = sample_messages()[type_name]
    data = encode(type_name, msg)
    assert data[:4] == CDR_LE
    assert len(data) % 4 == 0  # zero-padded like rmw_fastrtps
    assert decode(type_name, data) == msg
    assert encode(type_name, decode(type_name, data)) == data


@pytest.mark.parametrize("type_name", SUPPORTED_TYPES)
def test_round_trip_with_everything_empty(type_name):
    """Empty strings and sequences everywhere they can be."""
    def empty(value):
        if isinstance(value, dict):
            return {k: empty(v) for k, v in value.items()}
        if isinstance(value, str):
            return ""
        if isinstance(value, bytes):
            return b""
        if isinstance(value, list) and len(value) not in (4, 9, 12, 36):  # keep fixed-size arrays
            return []
        return value

    msg = empty(sample_messages()[type_name])
    assert decode(type_name, encode(type_name, msg)) == msg


def test_header_stamp_ns_joins_sec_and_nanosec():
    data = (CDR_LE + le("iI", 1_795_133_217, 258_787_762) + cdr_string(b"f") + b"\0\0"  # header, pad
            + cdr_string(b"") + b"\0" * 3 + le("I", 0))                                 # format "", pad, no data
    assert decode("sensor_msgs/msg/CompressedImage", data)["header"] == {
        "stamp_ns": 1_795_133_217_258_787_762, "frame_id": "f"}


def test_a_stamp_before_1970_keeps_ros_form():
    msg = {"header": {"stamp_ns": -1, "frame_id": ""}, "format": "", "data": b""}
    data = encode("sensor_msgs/msg/CompressedImage", msg)
    assert data[4:12] == le("iI", -1, 999_999_999)  # sec negative, nanosec in [0, 1e9)
    assert decode("sensor_msgs/msg/CompressedImage", data) == msg


def test_decode_takes_any_bytes_like():
    data = encode(STRING, {"data": "hi"})
    assert decode(STRING, memoryview(data)) == decode(STRING, bytearray(data)) == {"data": "hi"}


# ── strings ────────────────────────────────────────────────────────

def test_an_empty_string_is_a_lone_nul():
    data = encode(STRING, {"data": ""})
    assert data == CDR_LE + le("I", 1) + b"\0" + b"\0\0\0"
    assert decode(STRING, data) == {"data": ""}


def test_a_zero_length_string_decodes_as_empty():
    assert decode(STRING, CDR_LE + le("I", 0)) == {"data": ""}


def test_the_terminator_is_dropped_and_a_missing_one_tolerated():
    assert decode(STRING, CDR_LE + cdr_string(b"abc")) == {"data": "abc"}
    # Fast-CDR accepts a string without its NUL; so does this decoder.
    assert decode(STRING, CDR_LE + le("I", 3) + b"abc" + b"\0") == {"data": "abc"}


@pytest.mark.parametrize("text", ["a\0b", "café ☕", "\0"])
def test_strings_round_trip(text):
    data = encode(STRING, {"data": text})
    raw = text.encode()
    assert data[4:8] == le("I", len(raw) + 1)  # the length counts bytes, not characters
    assert decode(STRING, data) == {"data": text}


def test_bytes_that_are_not_utf8_survive():
    data = CDR_LE + cdr_string(b"\xff\xfe ok") + b"\0\0"  # 10 bytes of payload, padded to 12
    msg = decode(STRING, data)
    assert msg["data"].endswith(" ok")
    assert encode(STRING, msg) == data


# ── keep_data=False ────────────────────────────────────────────────

@pytest.mark.parametrize("type_name", SUPPORTED_TYPES)
def test_keep_data_false_drops_only_payload_bytes(type_name):
    msg = sample_messages()[type_name]
    assert decode(type_name, encode(type_name, msg), keep_data=False) == without_payloads(type_name, msg)


def test_keep_data_false_still_checks_the_skipped_bytes_exist():
    data = encode("sensor_msgs/msg/Image", sample_messages()["sensor_msgs/msg/Image"])
    assert decode("sensor_msgs/msg/Image", data, keep_data=False)["data"] is None
    with pytest.raises(BagDecodeError, match="truncated"):
        decode("sensor_msgs/msg/Image", data[:-6], keep_data=False)  # 18 data bytes + 2 pad; cut 4 of data


# ── refusals ───────────────────────────────────────────────────────

def test_unknown_type_is_refused():
    with pytest.raises(BagDecodeError, match="geometry_msgs/msg/Twist") as info:
        decode("geometry_msgs/msg/Twist", CDR_LE + bytes(48))
    assert isinstance(info.value, BagError)
    assert "sensor_msgs/msg/Image" in str(info.value)  # it says what it does read


@pytest.mark.parametrize("encapsulation, says", [
    (b"\x00\x00", "big-endian"),
    (b"\x00\x07", "not plain CDR"),  # XCDR2 (DELIMITED_CDR2_LE)
    (b"\x01\x00", "not plain CDR"),
])
def test_other_encapsulations_are_refused(encapsulation, says):
    data = encapsulation + encode(STRING, {"data": "x"})[2:]
    with pytest.raises(BagDecodeError, match=says):
        decode(STRING, data)


@pytest.mark.parametrize("type_name", SUPPORTED_TYPES)
def test_every_truncation_is_refused(type_name):
    data = encode(type_name, sample_messages()[type_name])
    # At most 3 trailing bytes are padding, so cutting 4 or more always cuts into a field.
    for cut in range(len(data) - 3):
        with pytest.raises(BagDecodeError):
            decode(type_name, data[:cut])


def test_up_to_three_bytes_may_be_left_over_but_not_four():
    data = encode(STRING, {"data": "abcd"})  # 4 + 9 bytes of payload, padded with 3
    assert len(data) == 16
    assert decode(STRING, data[:-3]) == decode(STRING, data) == {"data": "abcd"}  # unpadded, as other tools write
    with pytest.raises(BagDecodeError, match="4 bytes left over"):
        decode(STRING, data + b"\0")


@pytest.mark.parametrize("type_name", SUPPORTED_TYPES)
def test_trailing_bytes_beyond_padding_are_refused(type_name):
    with pytest.raises(BagDecodeError, match="left over"):
        decode(type_name, encode(type_name, sample_messages()[type_name]) + bytes(4))


def test_an_impossible_sequence_count_is_refused_at_once():
    data = CDR_LE + le("iI", 0, 0) + cdr_string(b"") + b"\0\0\0" + le("I", 0xFFFFFFFF)
    with pytest.raises(BagDecodeError, match="4294967295 elements"):
        decode(JOINTS, data)


# ── encoder errors ─────────────────────────────────────────────────

def test_encode_refuses_what_it_cannot_write():
    with pytest.raises(ValueError, match="no encoder"):
        encode("geometry_msgs/msg/Twist", {})
    image = sample_messages()["sensor_msgs/msg/Image"]
    with pytest.raises(ValueError, match="'height'"):
        encode("sensor_msgs/msg/Image", {k: v for k, v in image.items() if k != "height"})
    with pytest.raises(ValueError, match="keep_data=False"):
        encode("sensor_msgs/msg/Image", {**image, "data": None})
    with pytest.raises(ValueError):
        encode("sensor_msgs/msg/Image", {**image, "data": 18})  # a length is not a payload
    info = sample_messages()["sensor_msgs/msg/CameraInfo"]
    with pytest.raises(ValueError, match="fixed array of 9"):
        encode("sensor_msgs/msg/CameraInfo", {**info, "k": info["k"][:8]})
