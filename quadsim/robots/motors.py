""" Implements models for single DC-DC motors and compositional motor groups.
"""
import enum
from typing import Optional
from typing import Sequence
from typing import Tuple
from typing import Union

import numpy as np

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


# NOTE: Until functionality is added to MotorModel,
# it is effectively equivalent to a `dataclass`.
class MotorModel:
    """Implements a simple DC motor model for simulation.

    `MotorModel` describes a particular individual motor and can be useful
    for specifying physical characteristics that a singular motor might have.

    To accurately model the motor behaviors, the `MotorGroup` class converts all motor
    commands into torques, which can be sent directly to the simulator.
    Right now, 3 motor control modes are supported:
    - POSITION: performs joint-level PD control  (rad)
    - TORQUE: directly takes in motor torque command
    - HYBRID: takes in a 5-d tuple (pos, kp, vel, kd, torque), and output
      torque is a sum of PD torque and additional torque.
    """

    # TODO(yxyang): Complete documentation of motors with description of units (e.g. rads/s etc.)

    def __init__(
        self,
        name: str = None,
        motor_control_mode: MotorControlMode = MotorControlMode.POSITION,
        init_position: float = 0.0,
        min_position: float = 0.0,
        max_position: float = 0.0,
        min_velocity: float = 0.0,
        max_velocity: float = 0.0,
        min_torque: float = 0.0,
        max_torque: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0,
    ) -> None:

        self._name = name
        self._motor_control_mode = motor_control_mode
        self._init_position = init_position
        self._min_position = min_position
        self._max_position = max_position
        self._min_velocity = min_velocity
        self._max_velocity = max_velocity
        self._min_torque = min_torque
        self._max_torque = max_torque
        self._kp = kp
        self._kd = kd

    def inject_noise(self) -> None:
        """Stub to show potential functionality at `MotorModel` abstraction.
        This method might specify a noise model and then use it when initializing the motor.
        """
        raise NotImplementedError()


class MotorGroup:
    """A group of motors.
    This abstraction level allows vectorized motor control
    which is a computationally advantageous. At least 4x faster.

    For the time being, all functionality is provided in `MotorGroup`. Please
    note that `MotorGroup` does not change the state of attributes in each
    `MotorModel` it is initialized with.
    """

    # TODO: support instantiation by lists of values:
    # e.g. instead of creating motors and adding them to group to instantiate,
    # directly instantiate MotorGroup with 'motor_joint_names=...', ..., '_kds=...'
    def __init__(self, motors: Tuple[MotorModel, ...] = ()) -> None:

        self._motors = motors
        self._num_motors = len(motors)

        # Check motor control mode
        motor0_control_mode = motors[0]._motor_control_mode
        for motor in motors:
            if motor0_control_mode != motor._motor_control_mode:
                raise ValueError(
                    "Using different control mode for different motors is "
                    "not currently supported."
                )
        self._motor_control_mode = motor0_control_mode

        # TODO(romesco): Reevaluate whether protected vars in motors make sense.
        # assignees: romesco
        self._motor_joint_names = [motor._name for motor in motors]
        self._kps = np.array([motor._kp for motor in motors])
        self._kds = np.array([motor._kd for motor in motors])
        self._strength_ratios = np.ones(self._num_motors)
        self._init_positions = np.array([motor._init_position for motor in motors])
        self._min_positions = np.array([motor._min_position for motor in motors])
        self._max_positions = np.array([motor._max_position for motor in motors])
        self._min_velocities = np.array([motor._min_velocity for motor in motors])
        self._max_velocities = np.array([motor._max_velocity for motor in motors])
        self._min_torques = np.array([motor._min_torque for motor in motors])
        self._max_torques = np.array([motor._max_torque for motor in motors])

    @property
    def kps(self):
        return self._kps

    @kps.setter
    def kps(self, value: Union[float, Sequence[float]]):
        self._kps = np.ones(self._num_motors) * value

    @property
    def kds(self):
        return self._kds

    @kds.setter
    def kds(self, value: Union[float, Sequence[float]]):
        self._kds = np.ones(self._num_motors) * value

    @property
    def strength_ratios(self):
        return self._strength_ratios

    @strength_ratios.setter
    def strength_ratios(self, value: Union[float, Sequence[float]]):
        self._strength_ratios = np.ones(self._num_motors) * value

    @property
    def init_positions(self):
        return self._init_positions

    @init_positions.setter
    def init_positions(self, value: Union[float, Sequence[float]]):
        self._init_positions = value

    def _clip_torques(
        self, desired_torque: Sequence[float], current_motor_velocity: Sequence[float]
    ):
        del current_motor_velocity  # unused
        return np.clip(desired_torque, self._min_torques, self._max_torques)

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
            desired_torque = self._kps * (desired_angle - current_angle) + self._kds * (
                desired_velocity - current_velocity
            )
        elif motor_control_mode == MotorControlMode.TORQUE:
            desired_torque = motor_commands.copy()
        else:  # HYBRID case
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
