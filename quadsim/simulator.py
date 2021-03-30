from dataclasses import dataclass
from dataclasses import field
from typing import List


@dataclass
class SimulatorConf:
    timestep: float = 0.002
    action_repeat: int = 1
    hard_reset: bool = True
    on_rack: bool = False
    init_position: List[int] = field(default_factory=lambda: [None])
    init_rack_position: List[int] = field(default_factory=lambda: [None])
    reset_time: int = 3
