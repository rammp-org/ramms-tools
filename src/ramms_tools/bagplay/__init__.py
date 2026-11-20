"""Replay a recorded RAMMP ROS 2 bag into RammpUI through the interfaces the robot uses.

RMSS (camera colour, depth and segmentation, with intrinsics and the camera's pose from the
bag's TF) and Remote Control (detected objects, 2D and 3D; the robot display's joints). See
`ramms-bag --help` and the README's "Replaying a bag" section.
"""

from ramms_tools.bagplay.player import BagSession, PlayOptions, Player, PlayStats  # noqa: F401
from ramms_tools.bagplay.sinks import LiveSinks, RecordingSinks  # noqa: F401
