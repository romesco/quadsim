# Autogenerated by configen, do not edit.
# If encountering an error, please file an issue @
# https://github.com/romesco/quadsim
# fmt: off
# isort: skip_file
# flake8: noqa
# Hydra + Quadsim 

from dataclasses import dataclass, field
from quadsim.robots.motor_model import MotorControlMode
from typing import List


@dataclass
class MotorModelConf:
    _target_: str = "quadsim.robots.motor_model.MotorModel"
    motor_control_mode: MotorControlMode = MotorControlMode.POSITION
    num_motors: int = 0
    kps: List[float] = field(default_factory=lambda: [0.0])
    kds: List[float] = field(default_factory=lambda: [0.0])
    max_torque: float = 0.0
    min_torque: float = 0.0
    max_velocity: float = 0.0
    min_velocity: float = 0.0
    max_position: float = 0.0
    min_position: float = 0.0