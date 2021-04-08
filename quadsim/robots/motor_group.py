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


class MotorGroup:
    """Implements a simple DC motor model for simulation.

    To accurately model the motor behaviors, this class converts all motor
    commands into torques, which could be send directly to the simulator.
    Right now, 3 motor control modes are supported:
    - POSITION: performs joint-level PD control
    - TORQUE: directly takes in motor torque command
    - HYBRID: takes in a 5-d tuple (pos, kp, vel, kd, torque), and output
      torque is a sum of PD torque and additional torque.
    """

    def __init__(self, motor_configs) -> None:
        """Initializes the class."""
        self._num_motors = len(motor_configs)
        # Check motor control mode
        motor_control_mode = motor_configs[0].motor_control_mode
        for motor_config in motor_configs:
            if motor_control_mode != motor_config.motor_control_mode:
                raise ValueError(
                    "Using different control mode for different motors is "
                    "not currently supported."
                )
        self._motor_control_mode = motor_control_mode

        # Vectorize motors into to improve performance
        self._motor_joint_names = [
            motor_config.joint_name for motor_config in motor_configs
        ]
        self._kp = np.array([motor_config.kp for motor_config in motor_configs])
        self._kd = np.array([motor_config.kd for motor_config in motor_configs])
        self._strength_ratios = np.ones(self._num_motors)
        self._max_torque = np.array(
            [motor_config.max_torque for motor_config in motor_configs]
        )
        self._min_torque = np.array(
            [motor_config.min_torque for motor_config in motor_configs]
        )
        self._max_velocity = np.array(
            [motor_config.max_velocity for motor_config in motor_configs]
        )
        self._min_velocity = np.array(
            [motor_config.min_velocity for motor_config in motor_configs]
        )
        self._max_position = np.array(
            [motor_config.max_position for motor_config in motor_configs]
        )
        self._min_position = np.array(
            [motor_config.min_position for motor_config in motor_configs]
        )

    @property
    def kp(self):
        return self._kp

    @kp.setter
    def kp(self, value: Union[float, Sequence[float]]):
        self._kp = np.ones(self._num_motors) * value

    @property
    def kd(self):
        return self._kd

    @kd.setter
    def kd(self, value: Union[float, Sequence[float]]):
        self._kd = np.ones(self._num_motors) * value

    @property
    def strength_ratios(self):
        return self._strength_ratios

    @strength_ratios.setter
    def strength_ratios(self, value: Union[float, Sequence[float]]):
        self._strength_ratios = np.ones(self._num_motors) * value

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

    @property
    def motor_joint_names(self):
        return self._motor_joint_names
