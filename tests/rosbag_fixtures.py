"""Build real rosbag2 bags for tests: ``write_bag``, ``sample_messages``, ``without_payloads``.

``write_bag`` writes what rosbag2 Humble's sqlite3 storage writes (schema v3):
the same four tables, columns and ``timestamp_idx`` as the RAMMP recordings
(copied from their ``sqlite_master``), so tests exercise the reader's real SQL
and query plans, not a lookalike. ``sample_messages`` gives one message of
every type ``ramms_tools.rosbag`` decodes, shaped like the RAMMP bags' and
with float32 fields set to values float32 holds exactly, so that
``decode(t, encode(t, m)) == m`` compares equal.

Imported by other tests (``from rosbag_fixtures import write_bag``).
"""

from __future__ import annotations

import os
import sqlite3

__all__ = ["write_bag", "sample_messages", "without_payloads", "RAMMP_TOPICS"]

# Verbatim from nov_19_perception_bag_0.db3's sqlite_master.
_SCHEMA = """
CREATE TABLE schema(schema_version INTEGER PRIMARY KEY,ros_distro TEXT NOT NULL);
CREATE TABLE metadata(id INTEGER PRIMARY KEY,metadata_version INTEGER NOT NULL,metadata TEXT NOT NULL);
CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT NOT NULL,type TEXT NOT NULL,serialization_format TEXT NOT NULL,offered_qos_profiles TEXT NOT NULL);
CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER NOT NULL,timestamp INTEGER NOT NULL, data BLOB NOT NULL);
CREATE INDEX timestamp_idx ON messages (timestamp ASC);
"""

# The topics of the RAMMP perception bags and their types (main bag plus the vo bag's ui_payload).
RAMMP_TOPICS = {
    "/scene_camera/color/image_raw": "sensor_msgs/msg/Image",
    "/scene_camera/color/camera_info": "sensor_msgs/msg/CameraInfo",
    "/scene_camera/depth/image_rect_raw": "sensor_msgs/msg/Image",
    "/scene_camera/depth/camera_info": "sensor_msgs/msg/CameraInfo",
    "/wrist_camera/color/image_raw": "sensor_msgs/msg/Image",
    "/wrist_camera/color/camera_info": "sensor_msgs/msg/CameraInfo",
    "/wrist_camera/depth/image_rect_raw": "sensor_msgs/msg/Image",
    "/wrist_camera/depth/camera_info": "sensor_msgs/msg/CameraInfo",
    "/perception/object_detections": "rammp_perception_interfaces/msg/ObjectDetections",
    "/perception/object_detections_raw": "rammp_perception_interfaces/msg/ObjectDetections",
    "/perception/scene_segmenter/overlay/compressed": "sensor_msgs/msg/CompressedImage",
    "/perception/odom3d/markers": "visualization_msgs/msg/MarkerArray",
    "/perception/odom3d/ui_payload": "std_msgs/msg/String",
    "/joint_states": "sensor_msgs/msg/JointState",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/base/odom": "nav_msgs/msg/Odometry",
}


def write_bag(directory, topics: dict[str, str], messages: list[tuple[str, int, bytes]],
              split_at_ns: int | None = None, compression_mode: str = "") -> str:
    """Write a rosbag2 sqlite3 bag into *directory* and return the directory.

    *topics* maps name -> type; *messages* are ``(topic, timestamp_ns, cdr_bytes)``,
    inserted in the order given (rosbag2 inserts in arrival order, which need
    not be timestamp order). With *split_at_ns* the bag is two files, like a
    split recording: ``<name>_0.db3`` holds the messages before it and
    ``<name>_1.db3`` the rest. Both files register every topic, as rosbag2
    does, but with the ids in opposite orders -- in a real split bag the same
    topic has different ids per file, and a reader that mixed them up would
    pass a test whose files happened to agree. *compression_mode* only goes
    into metadata.yaml (the files stay plain), which is all a reader needs to
    see to refuse the bag.
    """
    directory = os.fspath(directory)
    os.makedirs(directory, exist_ok=True)
    name = os.path.basename(os.path.normpath(directory))
    for topic, _, _ in messages:
        if topic not in topics:
            raise ValueError(f"a message is on {topic!r}, which is not in topics")
    if split_at_ns is None:
        parts = [list(messages)]
    else:
        parts = [[m for m in messages if m[1] < split_at_ns], [m for m in messages if m[1] >= split_at_ns]]

    files = []
    for index, part in enumerate(parts):
        file_name = f"{name}_{index}.db3"
        path = os.path.join(directory, file_name)
        if os.path.exists(path):
            raise FileExistsError(path)
        order = list(topics) if index % 2 == 0 else list(reversed(topics))
        ids = {topic: i for i, topic in enumerate(order, start=1)}
        con = sqlite3.connect(path)
        try:
            con.executescript(_SCHEMA)
            con.execute("INSERT INTO schema (schema_version, ros_distro) VALUES (3, 'humble')")
            con.executemany(
                "INSERT INTO topics (id, name, type, serialization_format, offered_qos_profiles) "
                "VALUES (?, ?, ?, 'cdr', '')",
                [(ids[t], t, topics[t]) for t in order],
            )
            con.executemany(
                "INSERT INTO messages (topic_id, timestamp, data) VALUES (?, ?, ?)",
                [(ids[t], ts, bytes(data)) for t, ts, data in part],
            )
            con.commit()
        finally:
            con.close()
        files.append((file_name, part))

    with open(os.path.join(directory, "metadata.yaml"), "w", encoding="utf-8") as fh:
        fh.write(_metadata_yaml(topics, files, compression_mode))
    return directory


def _span(part) -> tuple[int, int]:
    stamps = [ts for _, ts, _ in part]
    return (min(stamps), max(stamps) - min(stamps)) if stamps else (0, 0)


def _metadata_yaml(topics: dict[str, str], files, compression_mode: str) -> str:
    """The parts of rosbag2's metadata.yaml (version 5) a reader or a person looks at."""
    every = [m for _, part in files for m in part]
    start, duration = _span(every)
    lines = [
        "rosbag2_bagfile_information:",
        "  version: 5",
        "  storage_identifier: sqlite3",
        "  duration:",
        f"    nanoseconds: {duration}",
        "  starting_time:",
        f"    nanoseconds_since_epoch: {start}",
        f"  message_count: {len(every)}",
        "  topics_with_message_count:",
    ]
    for topic, typ in topics.items():
        lines += [
            "    - topic_metadata:",
            f"        name: {topic}",
            f"        type: {typ}",
            "        serialization_format: cdr",
            '        offered_qos_profiles: ""',
            f"      message_count: {sum(1 for t, _, _ in every if t == topic)}",
        ]
    # rosbag2 quotes an empty value and writes a set one bare (compression_mode: FILE).
    lines += [
        f"  compression_format: {'zstd' if compression_mode else chr(34) * 2}",
        f"  compression_mode: {compression_mode or chr(34) * 2}",
        "  relative_file_paths:",
        *[f"    - {file_name}" for file_name, _ in files],
        "  files:",
    ]
    for file_name, part in files:
        file_start, file_duration = _span(part)
        lines += [
            f"    - path: {file_name}",
            "      starting_time:",
            f"        nanoseconds_since_epoch: {file_start}",
            "      duration:",
            f"        nanoseconds: {file_duration}",
            f"      message_count: {len(part)}",
        ]
    return "\n".join(lines) + "\n"


# ── sample messages ─────────────────────────────────────────────────

T0 = 1_795_133_217_258_787_762  # the main RAMMP bag's first message, ns
SCENE = "scene_camera_color_optical_frame"


def _hdr(stamp_ns: int, frame_id: str = SCENE) -> dict:
    return {"stamp_ns": stamp_ns, "frame_id": frame_id}


def _pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0) -> dict:
    return {"position": {"x": x, "y": y, "z": z}, "orientation": {"x": qx, "y": qy, "z": qz, "w": qw}}


def sample_messages(stamp_ns: int = T0) -> dict[str, dict]:
    """One message per supported type (fresh dicts every call), keyed by type name."""
    image = {
        "header": _hdr(stamp_ns), "height": 2, "width": 3, "encoding": "bgr8", "is_bigendian": 0,
        "step": 9, "data": bytes(range(18)),
    }
    marker_base = {
        "header": _hdr(stamp_ns, "odom"), "ns": "odom3d", "id": 4, "type": 1, "action": 0,
        "pose": _pose(1.25, -0.5, 0.75), "scale": {"x": 0.5, "y": 0.25, "z": 0.125},
        "color": {"r": 0.125, "g": 0.75, "b": 0.25, "a": 0.5}, "lifetime": {"sec": 0, "nanosec": 0},
        "frame_locked": False, "points": [], "colors": [], "texture_resource": "",
        "texture": {"header": _hdr(0, ""), "format": "", "data": b""}, "uv_coordinates": [],
        "text": "", "mesh_resource": "", "mesh_file": {"filename": "", "data": b""},
        "mesh_use_embedded_materials": False,
    }
    return {
        "sensor_msgs/msg/Image": image,
        "sensor_msgs/msg/CompressedImage": {
            "header": _hdr(stamp_ns), "format": "jpeg", "data": b"\xff\xd8\xff\xe0fake-jpeg\xff\xd9",
        },
        "sensor_msgs/msg/CameraInfo": {
            "header": _hdr(stamp_ns), "height": 400, "width": 640, "distortion_model": "rational_polynomial",
            "d": [0.0] * 14,
            "k": [283.77505, 0.0, 316.84305, 0.0, 283.69711, 196.45135, 0.0, 0.0, 1.0],
            "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "p": [283.77505, 0.0, 316.84305, 0.0, 0.0, 283.69711, 196.45135, 0.0, 0.0, 0.0, 1.0, 0.0],
            "binning_x": 0, "binning_y": 0,
            "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": False},
        },
        "sensor_msgs/msg/JointState": {
            "header": _hdr(stamp_ns, ""),
            "name": [f"arm_joint_{i}" for i in range(1, 8)] + ["arm_robotiq_85_left_knuckle_joint"],
            "position": [0.3735, -0.7645, -1.6092, -1.9307, -1.0259, -1.6371, -2.8920, 0.793],
            "velocity": [0.0] * 8,
            "effort": [0.5, -1.25, 0.0, 2.0, 0.0, 0.25, 0.0, 0.0],
        },
        "nav_msgs/msg/Odometry": {
            "header": _hdr(stamp_ns, "odom"), "child_frame_id": "base_link",
            "pose": {"pose": _pose(0.0819, -0.5931, 0.0, 0.0, 0.0, -0.9862, 0.1657),
                     "covariance": [0.001 if i in (0, 7) else 0.0 for i in range(36)]},
            "twist": {"twist": {"linear": {"x": 0.25, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": -0.1}},
                      "covariance": [0.0] * 36},
        },
        "tf2_msgs/msg/TFMessage": {"transforms": [
            {"header": _hdr(stamp_ns, "base_link"), "child_frame_id": "arm_mount",
             "transform": {"translation": {"x": 0.6017, "y": 0.2817, "z": 0.502},
                           "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}},
            {"header": _hdr(stamp_ns, "arm_mount"), "child_frame_id": "arm_base_link",
             "transform": {"translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                           "rotation": {"x": 0.0, "y": 0.2588, "z": 0.0, "w": 0.9659}}},
        ]},
        "visualization_msgs/msg/MarkerArray": {"markers": [
            {**marker_base, "header": _hdr(0, ""), "ns": "", "id": 0, "type": 0, "action": 3},  # DELETEALL
            marker_base,
            {**marker_base, "id": 5, "type": 9, "text": "food container_14494",  # TEXT_VIEW_FACING
             "points": [{"x": 1.0, "y": 2.0, "z": 3.0}], "colors": [{"r": 1.0, "g": 1.0, "b": 1.0, "a": 1.0}],
             "uv_coordinates": [{"u": 0.25, "v": 0.75}], "lifetime": {"sec": 1, "nanosec": 500_000_000},
             "frame_locked": True, "mesh_file": {"filename": "mesh.stl", "data": b"solid x"},
             "mesh_use_embedded_materials": True},
        ]},
        "rammp_perception_interfaces/msg/ObjectDetections": {
            "header": _hdr(stamp_ns),
            "objects": [
                {"instance_id": "table_2", "perception_class": "table", "score": 0.90625, "state": 0,
                 "box": [56.25, 182.75, 639.0, 328.0], "mask_id": 0, "has_pose": True,
                 "pose": _pose(0.302, 0.059, 0.559, 0.291, 0.655, -0.647, 0.261),
                 "size": {"x": 0.197, "y": 0.191, "z": 0.133}},
                {"instance_id": "fridge_5", "perception_class": "fridge", "score": 0.5, "state": 1,
                 "box": [0.0, 0.0, 0.0, 0.0], "mask_id": 0, "has_pose": True,
                 "pose": _pose(-0.25, 0.125, 2.5), "size": {"x": 0.75, "y": 1.75, "z": 0.75}},
            ],
            "label_image": {
                "header": _hdr(stamp_ns), "height": 2, "width": 2, "encoding": "mono16", "is_bigendian": 0,
                "step": 4, "data": b"\x01\x00\x00\x00\x02\x00\x01\x00",
            },
        },
        "std_msgs/msg/String": {"data": '{"stream_id":"camera/scene/color","objects":[]}'},
    }


def without_payloads(type_name: str, msg: dict) -> dict:
    """*msg* as ``decode(..., keep_data=False)`` returns it: payload bytes replaced by None."""
    def image(m):
        return {**m, "data": None}

    if type_name in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"):
        return image(msg)
    if type_name == "rammp_perception_interfaces/msg/ObjectDetections":
        return {**msg, "label_image": image(msg["label_image"])}
    if type_name == "visualization_msgs/msg/MarkerArray":
        return {"markers": [
            {**m, "texture": image(m["texture"]), "mesh_file": {**m["mesh_file"], "data": None}}
            for m in msg["markers"]
        ]}
    return msg
