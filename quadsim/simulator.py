""" A convenience class for configuring and instantiating the simulator (pybullet).
The idea is to encapsulate most simulator (but robot agnostic) calls.
We use this to simplify what must be done in the top level code (and the Robot class).
"""
from typing import Tuple

import pybullet
import pybullet_data
from pybullet_utils import bullet_client


class Simulator:
    """Simulator configuration and utility class.
    """

    def __init__(
        self,
        # pybullet_client: Any = None,
        # TODO: if we find it necessary, we can pass an existing one
        show_gui: bool = False,
        timestep: float = 0.002,
        action_repeat: int = 1,
        reset_time: float = 3,
        num_solver_iterations: int = 30,
        init_position: Tuple[float, float, float] = (0.0, 0.0, 0.32),
        init_rack_position: Tuple[float, float, float] = (0.0, 0.0, 1),
        on_rack: bool = False,
    ) -> None:

        if show_gui:
            p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
        else:
            p = bullet_client.BulletClient(connection_mode=pybullet.DIRECT)

        # Set Simulator Parameters
        p.setPhysicsEngineParameter(
            numSolverIterations=num_solver_iterations
        )
        p.setTimeStep(timestep)

        # Initialize world (once)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        p.setGravity(0.0, 0.0, -9.8)

        self.p = p
