from dataclasses import dataclass
from typing import Any
from omegaconf import MISSING

import pybullet
import pybullet_data
from pybullet_utils import bullet_client

import hydra
from hydra.core.config_store import ConfigStore
from hydra_configs.quadsim.robots.motors import MotorModelConf, MotorGroupConf
from hydra_configs.quadsim.robots.robot import RobotConf
from quadsim.simulator import SimulatorConf
from quadsim.robots.motors import MotorControlMode

sim_conf = SimulatorConf(
    show_gui=False,
    on_rack=True,
)
sim_conf.init_position = list(sim_conf.init_position)

# Set up pybullet client
if sim_conf.show_gui:
    p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
else:
    p = bullet_client.BulletClient(connection_mode=pybullet.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


@dataclass
class TopLvlConf:
    robot: Any = RobotConf(
        pybullet_client=MISSING,
        sim_conf=sim_conf,
        motors=MotorGroupConf(
            motors=[
                MotorModelConf(
                    name="FR_hip_joint",
                    motor_control_mode=MotorControlMode.POSITION,
                ),
                MotorModelConf(
                    name="FR_upper_joint",
                    motor_control_mode=MotorControlMode.POSITION,
                )
            ]
        )
    )


cs = ConfigStore.instance()
cs.store(name="toplvlconf", node=TopLvlConf)


@hydra.main(config_name="toplvlconf")
def main(cfg):
    print(cfg.pretty())


if __name__ == "__main__":
    main()
