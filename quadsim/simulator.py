""" A convenience class for configuring and instantiating the simulator (pybullet).
"""
from dataclasses import dataclass
from typing import Any
from typing import Tuple

import pybullet
import pybullet_data
from pybullet_utils import bullet_client


@dataclass
class SimulatorConf:
    """Simulator configuration dataclass.
    connection_mode:
        `None` connects to an existing simulation or, if fails, creates a
          new headless simulation,
        `pybullet.GUI` creates a new simulation with a GUI,
        `pybullet.DIRECT` creates a headless simulation,
        `pybullet.SHARED_MEMORY` connects to an existing simulation.
    """

    connection_mode: Any = pybullet.GUI
    timestep: float = 0.002
    action_repeat: int = 1
    reset_time: float = 3
    num_solver_iterations: int = 30
    # TODO: discuss why init_position is in the simulator
    # TODO: add to docs what this is, e.g. vector in R^3
    init_position: Tuple[float, float, float] = (0.0, 0.0, 0.32)
    init_rack_position: Tuple[float, float, float] = (0.0, 0.0, 1)
    on_rack: bool = False


class Simulator:
    """THIS CLASS IS CURRENTLY UNUSED
    A wrapper class for simulators which serves to:
    1. Provide configuration.
    2. Run boilerplate simulator instantiation code.
    """

    def __init__(
        self,
        show_gui: bool = False,
        timestep: float = 0.002,
        action_repeat: int = 1,
        reset_time: float = 3,
        num_solver_iterations: int = 30,
        init_position: Tuple[float, float, float] = (0.0, 0.0, 0.32),
        init_rack_position: Tuple[float, float, float] = (0.0, 0.0, 1),
        on_rack: bool = False,
    ) -> None:

        # TODO: should these be immutable since after init they cannot change?
        self.init_position = init_position
        self.init_rack_position = init_rack_position
        self.on_rack = on_rack

        if show_gui:
            p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
        else:
            p = bullet_client.BulletClient(connection_mode=pybullet.DIRECT)

        # Set Simulator Parameters
        p.setPhysicsEngineParameter(numSolverIterations=num_solver_iterations)
        p.setTimeStep(timestep)

        # Initialize world (once)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        p.setGravity(0.0, 0.0, -9.8)

        # The pybullet client reference:
        # e.g. in your external code, access via 'simulator.pybullet_client'
        self.pybullet_client = p
