# Autogenerated by configen, do not edit.
# If encountering an error, please file an issue @
# https://github.com/romesco/quadsim
# fmt: off
# isort: skip_file
# flake8: noqa
# Hydra + Quadsim 

from dataclasses import dataclass, field
from omegaconf import MISSING
from quadsim.simulator import SimulatorConf
from typing import Any
from typing import Tuple


@dataclass
class RobotConf:
    _target_: str = "quadsim.robots.robot.Robot"
    pybullet_client: Any = None
    sim_conf: SimulatorConf = None
    urdf_path: str = "None"
    motors: Any = MISSING  # Tuple[MotorGroup, ...]
    base_joint_names: Tuple[str, ...] = ()
    foot_joint_names: Tuple[str, ...] = ()