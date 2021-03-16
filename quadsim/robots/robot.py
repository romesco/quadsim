from typing import Any
from typing import List

# TODO: move Sensor and Motor to appropriate files


class Sensor:
    pass


class Motor:
    pass


class Robot:
    """Robot Base."""

    def __init__(
        self,
        pybullet_client: Any,
        motors: List[Motor],
        sensors: List[Sensor],
        on_rack: bool = False,
    ) -> None:
        """Constructs a base robot and resets it to the initial states.

        Args:

          pybullet_client: The instance of BulletClient to manage different
            simulations.
          motors: A list of motor models.
          sensors: A list of sensor models.
          on_rack: Whether the robot is hanging in mid-air or not. Used for debugging.
        """

        self.motors = motors
        self.num_motors = len(self.motors)
        self.sensors = sensors
        self._step_counter = 0

    def step(self, action, motor_control_mode=None):
        """Steps simulation."""
        # TODO: desired step functionality
        self._step_counter += 1
