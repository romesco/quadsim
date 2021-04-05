"""Implements a simple DC motor model."""
import enum
import numpy as np
from typing import Sequence, Union, Optional

POSITION_INDEX = 0
KP_INDEX = 1
VELOCITY_INDEX = 2
KD_INDEX = 3
TORQUE_INDEX = 4
COMMAND_DIMENSION = 5


class MotorControlMode(enum.Enum):
    """Different motor control modes.
    See the documentation of MotorModel class for details."""

    POSITION = 0
    TORQUE = 1
    HYBRID = 2


class MotorModel:
    """Implements a simple DC motor model for simulation.

    To accurately model the motor behaviors, this class converts all motor
    commands into torques, which could be send directly to the simulator.
    Right now, 3 motor control modes are supported:
    - POSITION: performs joint-level PD control
    - TORQUE: directly takes in motor torque command
    - HYBRID: takes in a 5-d tuple (pos, kp, vel, kd, torque), and output
      torque is a sum of PD torque and additional torque.
    """

    def __init__(
        self,
        motor_control_mode: MotorControlMode,
        min_position: float,
        max_position: float,
        min_velocity: float,
        max_velocity: float,
        min_torque: float,
        max_torque: float,
        kp: float,
        kd: float,
    ) -> None:
        """Initializes the class."""
        self._motor_control_mode = motor_control_mode
        self._kp = kp
        self._kd = kd
        self._strength_ratio = 1
        # Thresholds
        self._max_torque = max_torque
        self._min_torque = min_torque
        self._max_velocity = max_velocity
        self._min_velocity = min_velocity
        self._max_position = max_position
        self._min_position = min_position

    def set_motor_gain(self, kp: float, kd: float) -> None:
        self._kp = kp
        self._kd = kd

    def set_strength_ratio(self, strength_ratio: float) -> None:
        self._strength_ratio = strength_ratio

    def _clip_torque(self, desired_torque: float, current_motor_velocity: float):
        """Clips the applied torque based on motor constraints.

        See documentation in convert_to_torque function for more details.
        """
        del current_motor_velocity  # unused
        return np.clip(desired_torque, self._min_torque, self._max_torque)

    def convert_to_torque(
        self,
        motor_commands: Union[float, Sequence[float]],
        current_angle: float,
        current_velocity: float,
        motor_control_mode: Optional[MotorControlMode],
    ):
        """Converts the given motor command into motor torques.

        This function converts the user-supplied motor command into torque
        command, which is sent to either the simulator or the real robot.
        First, the function computes the desired torque based on the command
        and control mode. For a more realistic representation, the desired
        torque is then clipped based on motor characteristics.
        """
        motor_control_mode = motor_control_mode or self._motor_control_mode
        motor_commands = np.array(motor_commands)
        if motor_control_mode == MotorControlMode.POSITION:
            desired_angle = motor_commands
            desired_velocity = 0
            desired_torque = self._kp * (desired_angle - current_angle) + self._kd * (
                desired_velocity - current_velocity
            )
        elif motor_control_mode == MotorControlMode.TORQUE:
            desired_torque = motor_commands
        else:
            desired_angle = motor_commands[POSITION_INDEX]
            kp = motor_commands[KP_INDEX]
            desired_velocity = motor_commands[VELOCITY_INDEX]
            kd = motor_commands[KD_INDEX]
            torque = motor_commands[TORQUE_INDEX]
            desired_torque = (
                kp * (desired_angle - current_angle)
                + kd * (desired_velocity - current_velocity)
                + torque
            )

        applied_torque = self._clip_torque(desired_torque, current_velocity)
        applied_torque *= self._strength_ratio
        return applied_torque, desired_torque

    @property
    def num_motors(self):
        return self._num_motors
