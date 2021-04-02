"""Unitree A1 robot."""
from quadsim.robots.robot import Motor
from quadsim.robots.robot import Robot
from quadsim.robots.robot import Sensor
from typing import Any
from typing import List
from typing import Tuple


class A1(Robot):
    """A1 Robot."""

    def __init__(
        self,
        pybullet_client: Any,  # TODO: is this necessary as robot attribute?
        motors: List[Motor] = None,
        sensors: List[Sensor] = None,
        urdf_path: str = "a1/a1.urdf",
        base_link_names: Tuple[str, ...] = ("trunk",),
        motor_joint_names: Tuple[str, ...] = (
            "FR_hip_joint",
            "FR_upper_joint",
            "FR_lower_joint",
            "FL_hip_joint",
            "FL_upper_joint",
            "FL_lower_joint",
            "RR_hip_joint",
            "RR_upper_joint",
            "RR_lower_joint",
            "RL_hip_joint",
            "RL_upper_joint",
            "RL_lower_joint",
        ),
        init_motor_angles: Tuple[float] = (0.0, 0.9, -1.8) * 4,
        foot_link_names: Tuple[str] = ("FR_toe", "FL_toe", "RR_toe", "RL_toe"),
        on_rack: bool = False,  # TODO: should this move to simulator only?
    ) -> None:
        """Constructs an A1 robot and resets it to the initial states.

        Args:
        """
        super().__init__()

    def apply_action(self, action, motor_control_mode=None):
        pass
