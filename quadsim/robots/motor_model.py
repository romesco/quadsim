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

    def __init__(self, config) -> None:
        """Initializes the class."""
        self.config = config
        self._motor_control_mode = self.config.motor_control_mode
        self._num_motors = self.config.num_motors
        self._kp = self.config.kps
        self._kd = self.config.kds
        self._strength_ratios = np.ones(self._num_motors)
        # Thresholds
        self._max_torque = self.config.max_torque
        self._min_torque = self.config.min_torque
        self._max_velocity = self.config.max_velocity
        self._min_velocity = self.config.min_velocity
        self._max_position = self.config.max_position
        self._min_position = self.config.min_position

    def set_motor_gains(
        self, kp: Union[float, Sequence[float]], kd: Union[float, Sequence[float]]
    ) -> None:
        self._kp = np.ones(self._num_motors) * kp
        self._kd = np.ones(self._num_motors) * kd

    def set_strength_ratios(
        self, strength_ratios: Union[float, Sequence[float]]
    ) -> None:
        self._strength_ratios = np.ones(self._num_motors) * strength_ratios

    def _clip_torques(
        self, desired_torque: Sequence[float], current_motor_velocity: Sequence[float]
    ):
        del current_motor_velocity  # unused
        return np.clip(desired_torque, self._min_torque, self._max_torque)

    def convert_to_torque(
        self,
        motor_commands: Sequence[float],
        current_angle: Sequence[float],
        current_velocity: Sequence[float],
        motor_control_mode: Optional[MotorControlMode],
    ):
        """Converts the given motor command into motor torques."""
        motor_control_mode = motor_control_mode or self._motor_control_mode
        motor_commands = np.array(motor_commands)
        if motor_control_mode == MotorControlMode.POSITION:
            desired_angle = motor_commands.copy()
            desired_velocity = np.zeros(self._num_motors)
            desired_torque = self._kp * (desired_angle - current_angle) + self._kd * (
                desired_velocity - current_velocity
            )
        elif motor_control_mode == MotorControlMode.TORQUE:
            desired_torque = motor_commands.copy()
        else:
            desired_angle = motor_commands[POSITION_INDEX::COMMAND_DIMENSION]
            kp = motor_commands[KP_INDEX::COMMAND_DIMENSION]
            desired_velocity = motor_commands[VELOCITY_INDEX::COMMAND_DIMENSION]
            kd = motor_commands[KD_INDEX::COMMAND_DIMENSION]
            torque = motor_commands[TORQUE_INDEX::COMMAND_DIMENSION]
            desired_torque = (
                kp * (desired_angle - current_angle)
                + kd * (desired_velocity - current_velocity)
                + torque
            )

        applied_torque = self._clip_torques(desired_torque, current_velocity)
        applied_torque *= self._strength_ratios
        return applied_torque, desired_torque

    @property
    def num_motors(self):
        return self._num_motors
