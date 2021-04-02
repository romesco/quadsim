from dataclasses import dataclass
from hydra_configs.quadsim.robots.a1 import A1Conf
from hydra_configs.quadsim.robots.motor_model import MotorModelConf
from quadsim.robots.motor_model import MotorControlMode
from quadsim.simulator import SimulatorConf
from typing import Any

import hydra
from hydra.core.config_store import ConfigStore


@dataclass
class TopLvlConf:
    simulator: SimulatorConf = SimulatorConf(
        on_rack=True,
        action_repeat=2,
        # override any other attribute beyond the defaults specified in the Conf
    )
    robot: Any = A1Conf(
        motors=MotorModelConf(
            motor_control_mode=MotorControlMode.POSITION,
            num_motors=12,
            max_torque=[33.5] * 12,
            min_torque=[-33.5] * 12,
            max_velocity=[16] * 12,
            min_velocity=[-16] * 12,
            max_position=[0.802851455917, 4.18879020479, -0.916297857297] * 4,
            min_position=[-0.802851455917, -1.0471975512, -2.69653369433] * 4,
            kps=[100] * 12,
            kds=[1, 2, 2] * 4,
        )
    )


cs = ConfigStore.instance()
cs.store(name="toplvlconf", node=TopLvlConf)


@hydra.main(config_name="toplvlconf")
def main(cfg):
    print(cfg.pretty())


if __name__ == "__main__":
    main()
