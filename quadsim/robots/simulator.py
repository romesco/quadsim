from dataclasses import dataclass
from typing import List


@dataclass
class SimulatorConf:
    timestep: float = 0.002
    action_repeat: int = 1
    hard_reset: bool = True
    on_rack: bool = False
    init_position: List[int]
    init_rack_position: List[int]
    reset_time: int = 3
