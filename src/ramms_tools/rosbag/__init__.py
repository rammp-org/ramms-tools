"""Read recorded RAMMP ROS 2 bags without a ROS install.

``open_bag`` reads a rosbag2 sqlite3 bag (Humble, storage schema v3; one
``.db3`` or a split directory of them) in timestamp order, and ``decode``
turns each message's CDR bytes into a plain dict for the nine message types
the RAMMP perception bags record. ``encode`` is its inverse, used to build
test bags. Standard library only.

    from ramms_tools.rosbag import open_bag

    with open_bag("nov_19_perception_bag") as bag:
        print(bag.topics["/perception/object_detections"].count, bag.duration_s)
        for raw in bag.messages(topics=["/joint_states"], start_ns=bag.start_ns + 5_000_000_000):
            js = bag.decode(raw)
            ...
"""

from ramms_tools.rosbag.cdr import SUPPORTED_TYPES, BagDecodeError, BagError, decode, encode
from ramms_tools.rosbag.reader import BagReader, RawMessage, TopicInfo, open_bag

__all__ = [
    "BagError",
    "BagDecodeError",
    "TopicInfo",
    "RawMessage",
    "BagReader",
    "open_bag",
    "decode",
    "encode",
    "SUPPORTED_TYPES",
]
