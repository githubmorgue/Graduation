from dataclasses import dataclass, field
from typing import Dict


@dataclass
class DualVars:
    u: Dict[int, Dict[int, int]] = field(default_factory=dict)
    z: Dict[int, Dict[int, int]] = field(default_factory=dict)
    g: Dict[int, Dict[int, int]] = field(default_factory=dict)
    h: Dict[int, Dict[int, int]] = field(default_factory=dict)
    v: Dict[int, Dict[int, Dict[int, int]]] = field(default_factory=dict)
    w: Dict[int, Dict[int, Dict[int, int]]] = field(default_factory=dict)
