from dataclasses import dataclass, field
from typing import Dict, List, Tuple

@dataclass
class SolutionInfo:
    """存储解的信息：包含目标值和决策变量"""
    obj_value: float
    x_vars: List[Tuple[int, int]]

@dataclass
class Params:
    # 基础维度参数
    T: int = 0
    L: int = 0
    B: int = 0

    # 约束参数 (Section 1)
    alpha_param: float = 0.0  # 重命名为alpha_param避免与频率向量alpha混淆
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
    LB: float = float('-inf')  # 当前下界 LB^r
    UB: float = float('inf')   # 当前上界 UB^r
    LB_history: Dict[int, float] = field(default_factory=dict)  # LB的历史值，LB^{r-1}用于公式(20)
    UB_history: Dict[int, float] = field(default_factory=dict)  # UB的历史值，UB^{r-1}用于公式(20)
    x_tb_r: Dict[int, Dict[int, Dict[int, int]]] = field(default_factory=dict)  # 每轮迭代的主问题解 x_tb_r[r][t][b]，值为0或1
    D_r: Dict[int, List[SolutionInfo]] = field(default_factory=dict)  # 每轮迭代的解集合，按目标值降序排列，最后一个是最优解x^r
    K_r: Dict[int, int] = field(default_factory=dict)  # 每轮迭代的对偶变量对数量 K_r[r] = k，k为正整数
    NB_mem: int = 0  # 记忆池大小，人工设定的参数
    D_r_mem: Dict[int, List[List[Tuple[int, int]]]] = field(default_factory=dict)  # 按轮次索引的记忆池，D_r_mem[r]存储第r轮的记忆池（大小为NB_mem）
    alpha_freq: Dict[int, Dict[int, Dict[int, int]]] = field(default_factory=dict)  # 频率向量 alpha_freq[t][b][r] = 投标(t,b)在第r轮时于D_r_mem中被选中的次数
    phi_r: Dict[int, int] = field(default_factory=dict)  # 局部搜索参数，每轮迭代的值，phi_r[r] = |D_r[r]| / 2
    B_tilde_r: Dict[int, List[List[Tuple[int, int]]]] = field(default_factory=dict)  # 每轮迭代的LS随机子集集合，B_tilde_r[r] = 包含NB_LS个随机子集的列表，每个子集大小为phi_r[r]
    NB_LS: int = 2  # LS解的数量，人工设定的参数，默认为2
    X_r_LS: Dict[int, List[List[Tuple[int, int]]]] = field(default_factory=dict)  # 每轮迭代的LS解集合 X_r_LS[r] = 包含NB_LS个限制主问题最优解的列表
    N_x_r: Dict[int, List[Dict[int, Dict[int, int]]]] = field(default_factory=dict)  # 每轮迭代的邻域集合 N_x_r[r] = 包含NB_LS个邻域约束的列表，每个邻域N(x^r)用\bar{x}_{tb}^r表示
