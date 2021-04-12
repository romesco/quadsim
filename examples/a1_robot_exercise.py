"""Example of running A1 robot with position control.

To run the example with gui:
python a1_robot_exercise.py gui=true
"""

import numpy as np
# import time

from quadsim.simulator import Simulator
# from quadsim.robots.robot import Robot


def get_action(t):
    mid_action = np.array([0.0, 0.9, -1.8] * 4)
    amplitude = np.array([0.0, 0.2, -0.4] * 4)
    freq = 1.0
    return mid_action + amplitude * np.sin(2 * np.pi * freq * t)


def main():
    sim = Simulator(
        show_gui=False,
        on_rack=True,
    )
    print(sim)

    # robot = Robot(p, FLAGS.robot_config)
    # for _ in range(10000):
    #     action = get_action(robot.time_since_reset)
    #     robot.step(action)
    #     time.sleep(0.002)
    #     print(robot.base_orientation_rpy)


if __name__ == "__main__":
    main()
