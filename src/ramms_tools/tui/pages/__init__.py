"""TUI pages for RAMMS robotics control."""

from .dashboard import DashboardPage
from .arm_page import ArmPage
from .mebot_page import MebotPage
from .imu_page import IMUPage

__all__ = ["DashboardPage", "ArmPage", "MebotPage", "IMUPage"]
