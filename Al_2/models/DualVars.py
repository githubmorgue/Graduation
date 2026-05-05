from dataclasses import dataclass, field
from typing import Dict, List


@dataclass
class DualVars:
    # 修改为：dual_vars.u[r] 是一个列表，包含第r轮的多对对偶变量
    # 每个元素是一个 Dict[int, float]，表示一对对偶变量
    u: Dict[int, List[Dict[int, float]]] = field(default_factory=dict)
    v: Dict[int, List[Dict[int, Dict[int, float]]]] = field(default_factory=dict)
    w: Dict[int, List[Dict[int, Dict[int, float]]]] = field(default_factory=dict)
    g: Dict[int, List[Dict[int, float]]] = field(default_factory=dict)
    h: Dict[int, List[Dict[int, float]]] = field(default_factory=dict)
    z: Dict[int, List[Dict[int, float]]] = field(default_factory=dict)
