"""Data-source abstraction for joint torques.

This module deliberately has no ROS dependency (no rclpy import). A torque
source connects to some upstream, and on demand returns the current joint
torques in N.m. Swapping the data source (different robot interface, a replay
file, a simulator, ...) only means implementing this interface; the ROS node
shell stays untouched.
"""

from abc import ABC, abstractmethod
from typing import List, Optional, Tuple

# UR7e standard joint order. Torques are always reported in this order so the
# downstream contract (/joint_torques) is stable regardless of the source.
UR7E_JOINT_ORDER: List[str] = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class TorqueSource(ABC):
    """Abstract joint-torque data source."""

    @abstractmethod
    def connect(self) -> bool:
        """Try to establish the upstream connection.

        Returns True on success, False otherwise. Must not raise on a simple
        connection failure (return False instead) so callers can retry.
        """

    @abstractmethod
    def disconnect(self) -> None:
        """Release the upstream connection. Safe to call when not connected."""

    @abstractmethod
    def is_connected(self) -> bool:
        """Whether the source currently holds a live connection."""

    @abstractmethod
    def get_torques(self) -> Tuple[List[str], Optional[List[float]]]:
        """Return (joint_names, torques_nm).

        joint_names is always the UR7e standard order. torques_nm is a list of
        six floats in N.m, or None when no fresh sample is available (e.g. not
        connected). Callers should skip publishing when torques is None.
        """
