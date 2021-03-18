"""Implements a simple DC motor model."""
import enum
import numpy as np
from typing import List, Union, Optional

POSITION_INDEX = 0
KP_INDEX = 1
VELOCITY_INDEX = 2
KD_INDEX = 3
TORQUE_INDEX = 4
COMMAND_DIMENSION = 5


class MotorControlMode(enum.Enum):
    POSITION = 0
    TORQUE = 1
    HYBRID = 2


class MotorModel:
    def __init__(self, config):
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
        self, kp: Union[float, List[float]], kd: Union[float, List[float]]
    ):
        self._kp = np.ones(self._num_motors) * kp
        self._kd = np.ones(self._num_motors) * kd

    def set_strength_ratios(self, strength_ratios: Union[float, List[float]]):
        self._strength_ratios = np.ones(self._num_motors) * strength_ratios

    def _clip_torques(
        self, desired_torque: List[float], current_motor_velocity: List[float]
    ):
        del current_motor_velocity  # unused
        return np.clip(desired_torque, self._min_torque, self._max_torque)

    def convert_to_torque(
        self,
        motor_commands: List[float],
        current_angle: List[float],
        current_velocity: List[float],
        motor_control_mode: Optional[MotorControlMode],
    ):
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
            # Action is in the form of
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
