from dataclasses import dataclass, field
from typing import Dict, List, Tuple


@dataclass
class Params:
    # 基础维度参数
    T: int = 0
    L: int = 0
    B: int = 0

    # 约束参数 (Section 1)
    alpha_param: float = 0.0
    z_l: Dict[int, int] = field(default_factory=dict)  # 二元变量，z_l ∈ {0, 1}
    p_t: Dict[int, float] = field(default_factory=dict)  # 承运商绩效因子
    q_t: Dict[int, int] = field(default_factory=dict)  # 中标下限
    Q_t: Dict[int, int] = field(default_factory=dict)  # 中标上限
    N_min: int = 0
    N_max: int = 0

    # 函数参数 (Section 2)
    d_bar_l: Dict[int, float] = field(default_factory=dict)  # 名义需求 d_bar_l
    d_hat_l: Dict[int, float] = field(default_factory=dict)  # 最大偏差 d_hat_l = alpha_param * d_bar_l
    d_l: Dict[int, float] = field(default_factory=dict)  # 实际需求 d_l = d_bar_l + z_l * d_hat_l
    LB_tb: Dict[int, Dict[int, int]] = field(default_factory=dict)  # 投标下限
    UB_tb: Dict[int, Dict[int, int]] = field(default_factory=dict)  # 投标上限
    c_tb: Dict[int, Dict[int, float]] = field(default_factory=dict)  # 投标价格
    ce_l: Dict[int, int] = field(default_factory=dict)  # 即期市场成本
    M: Dict[int, int] = field(default_factory=dict)  # 辅助参数，等于ce_l
    a_tb_l: Dict[int, Dict[int, Dict[int, int]]] = field(default_factory=dict)  # 覆盖矩阵 a_{tb}^l

    # 决策相关变量
    A_r: float = 0.0
    LB: float = float('-inf')
    UB: float = float('inf')
    x_tb_r: Dict[int, Dict[int, Dict[int, int]]] = field(default_factory=dict)  # 每轮迭代的主问题解 x_tb_r[r][t][b]，值为0或1
