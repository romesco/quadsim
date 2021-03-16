from quadsim.robots.robot import Motor
from quadsim.robots.robot import Robot
from quadsim.robots.robot import Sensor
from typing import Any

import pybullet
import pytest
from pybullet_utils import bullet_client

p = bullet_client.BulletClient(connection_mode=pybullet.DIRECT)


@pytest.mark.parametrize(
    "robot",
    [
        pytest.param(
            Robot(p, [Motor()], [Sensor()]),
            id="BaseRobot",
        ),
    ],
)
def test_step(
    robot: Any,
) -> None:
    robot.step("dummy_val")
    assert robot._step_counter == 1
