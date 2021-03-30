"""Base class for all robots."""
from typing import Any
from typing import List

import numpy as np


class Sensor:
    pass


class Motor:
    pass


class Robot:
    """Robot Base."""

    def __init__(
        self,
        pybullet_client: Any,  # TODO: is this necessary as robot attribute?
        motors: List[Motor],
        sensors: List[Sensor],
        urdf_path: str,
        base_link_names: List[str],
        motor_joint_names: List[str],
        init_motor_angles: List[float],
        foot_link_names: List[str],
        on_rack: bool,  # TODO: should this move to simulator only?
    ) -> None:
        """Constructs a base robot and resets it to the initial states.

        Args:
          pybullet_client: The instance of BulletClient to manage different
            simulations.
          motors: A list of motor models.
          sensors: A list of sensor models.
          on_rack: Whether the robot is hanging in mid-air or not. Used for debugging.
          TODO: populate the rest of the class attribute descriptions
        """
        self._pybullet_client = pybullet_client
        self._motors = motors
        self._sensors = sensors
        self._urdf_path = urdf_path
        self._base_link_names = base_link_names
        self._motor_joint_name = motor_joint_names
        self._init_motor_angles = np.array(init_motor_angles)
        self._foot_link_names = foot_link_names
        self._on_rack = on_rack

    def apply_action(self, action, motor_control_mode=None):
        pass
