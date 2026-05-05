"""
初始化模块 (Initialization Module)

该模块负责算法初始阶段的数据生成与参数配置。
主要功能包括：
1. 初始化基础约束参数（如承运商数量、时间周期、预算等）。
2. 生成函数相关参数（如名义需求、投标报价、即期市场成本等）。
3. 构建覆盖矩阵，定义承运商标书对线路的覆盖关系。
4. 初始化对偶变量空间，用于后续的对偶信息更新。
5. 初始化决策变量及算法边界（LB/UB），为迭代过程做准备。
"""

import random


from Al_2.models.Params import Params
from Al_2.models.DualVars import DualVars


def initialize_constraints_params(L: int, T: int, B: int) -> Params:
    """
    初始化约束相关的核心参数。

    该函数负责创建 Params 对象并设置算法的基础规模参数，同时生成
    随机的绩效因子、需求量范围以及中标承运商数量的限制。

    参数:
        L (int): 线路（Lane）的数量。
        T (int): 承运商（Truck/Carrier）的数量。
        B (int): 每个承运商的标书（Bid）数量。

    返回:
        Params: 填充了基础约束参数的 Params 对象。
    """
    params = Params()

    # 1.1 确定参数数量
    params.L = L
    params.T = T
    params.B = B

    # 1.2 生成参数
    # alpha_param: 需求波动系数，范围 [0.10, 0.50]
    params.alpha_param = round(random.uniform(0.10, 0.50), 2)

    for l in range(1, L + 1):
        # z_l: 需求波动二元变量，初始化为 0
        params.z_l[l] = 0

    for t in range(1, T + 1):
        # p_t: 承运商绩效因子，范围 [-0.05, 0.05]，保留4位小数
        params.p_t[t] = round(random.uniform(-0.05, 0.05), 4)
        # q_t: 最小中标量，范围 [10, 15]
        params.q_t[t] = random.randint(10, 15)
        # Q_t: 最大中标量，范围 [60, 75]
        params.Q_t[t] = random.randint(60, 75)

    # N_min: 最小中标承运商数量，固定为 2
    # N_max: 最大中标承运商数量，约为承运商总数的 20%
    params.N_min = 2
    params.N_max = max(2, int(T * 0.2))

    return params


def initialize_function_params(params: Params) -> Params:
    """
    初始化目标函数及约束所需的详细参数。

    在基础参数的基础上，进一步生成每条线路的名义需求、实际需求量，
    以及每个承运商对每条线路的投标价格区间和即期市场成本。

    参数:
        params (Params): 已包含基础规模参数（L, T, B）的 Params 对象。

    返回:
        Params: 填充了完整函数参数的 Params 对象。
    """
    # 2.1 确定参数数量
    T = params.T
    L = params.L
    B = params.B

    # 2.2 运输量相关参数
    for l in range(1, L + 1):
        # d_bar_l: 名义需求量，范围 [10, 50]
        params.d_bar_l[l] = random.randint(10, 50)
        # d_hat_l: 最大需求偏差，等于 alpha_param * d_bar_l
        params.d_hat_l[l] = params.alpha_param * params.d_bar_l[l]
        # d_l: 实际需求，根据波动变量 z_l 计算
        z_l_val = params.z_l.get(l, 0)
        params.d_l[l] = params.d_bar_l[l] + z_l_val * params.d_hat_l[l]

    # 2.3 投标相关参数 (针对每个承运商 t 和每条线路 l 的标书 b)
    for t in range(1, T + 1):
        params.LB_tb[t] = {}
        params.UB_tb[t] = {}
        params.c_tb[t] = {}
        for b in range(1, B + 1):
            # LB_tb: 投标最低运量，范围 [10, 20]
            params.LB_tb[t][b] = random.randint(10, 20)
            # UB_tb: 投标最高运量，范围 [40, 75]
            params.UB_tb[t][b] = random.randint(40, 75)
            # c_tb: 投标单价，范围 [10, 80]，保留2位小数
            params.c_tb[t][b] = round(random.uniform(10, 80), 2)

    # ce_l: 即期市场（Spot Market）单位运输成本，范围 [50, 100]
    for l in range(1, L + 1):
        params.ce_l[l] = random.randint(50, 100)
        # M[l]: 惩罚系数，此处设为与即期市场成本相等
        params.M[l] = params.ce_l[l]

    return params


def initialize_coverage_matrix(params: Params, density: float = 0.235) -> Params:
    """
    初始化覆盖矩阵 a_{tb}^l。

    该矩阵定义了承运商 t 的标书 b 是否能够覆盖线路 l。
    采用随机生成的方式，通过密度参数控制矩阵中 1 的比例。

    参数:
        params (Params): 包含 L, T, B 信息的 Params 对象。
        density (float): 覆盖矩阵的稀疏度（默认 0.235，即约 23.5% 的元素为 1）。

    返回:
        Params: 更新了三维字典 a_tb_l 的 Params 对象。
    """
    T = params.T
    L = params.L
    B = params.B

    # 初始化三维字典结构
    for t in range(1, T + 1):
        params.a_tb_l[t] = {}
        for b in range(1, B + 1):
            params.a_tb_l[t][b] = {}
            for l in range(1, L + 1):
                # 根据密度概率随机赋值 0 或 1
                params.a_tb_l[t][b][l] = 1 if random.random() < density else 0

    return params


def initialize_dual_variables(params: Params) -> DualVars:
    """
    初始化对偶变量空间。

    为算法的第 0 轮迭代准备对偶变量容器。对偶变量以列表形式存储，
    用于在后续迭代中记录切割平面（Benders Cuts）或拉格朗日乘子。

    参数:
        params (Params): 当前算法的参数对象（用于获取维度信息，可选）。

    返回:
        DualVars: 初始化后的对偶变量对象，第 0 轮索引已创建但内容为空。
    """
    # 创建对偶变量对象
    dual_vars = DualVars()

    # 初始化第 0 轮的对偶变量存储空间（初始为空列表）
    # 后续在 Step_3 中会根据求解结果向这些列表中添加具体的对偶变量值
    dual_vars.u[0] = []
    dual_vars.v[0] = []
    dual_vars.w[0] = []
    dual_vars.g[0] = []
    dual_vars.h[0] = []
    dual_vars.z[0] = []

    print(f"初始化完成 - 第0轮对偶变量空间已创建（初始为空列表）")

    return dual_vars


def initialize_decision_variables(params: Params) -> Params:
    """
    初始化决策变量集合及算法历史边界。

    设置初始轮次（r=0）的解集为空，并根据算法要求初始化
    下界（LB）和上界（UB）的历史记录，包括虚拟的 r=-1 时刻。

    参数:
        params (Params): 待初始化的 Params 对象。

    返回:
        Params: 包含了初始化解集 D_r, X_r_LS 以及边界历史 LB_history, UB_history 的对象。
    """    
    # 初始化 D_r[0] 为空列表，用于存储主问题的解
    params.D_r[0] = []
    
    # 初始化 X_r_LS[0] 为空列表，用于存储局部搜索的解
    params.X_r_LS[0] = []
    
    # 根据图片算法2 Step 0: 初始化 LB^{-1} ← -∞, UB^{-1} ← +∞, LB^0 ← -∞, UB^0 ← +∞
    # LB^{r-1} 和 UB^{r-1} 是目标函数 A 的历史边界值
    if not hasattr(params, 'LB_history'):
        params.LB_history = {}
    if not hasattr(params, 'UB_history'):
        params.UB_history = {}
    
    # 初始化 LB^{-1} = -∞, UB^{-1} = +∞
    # 使用 -1 作为索引表示 r=-1（即 r=0 时的 r-1）
    params.LB_history[-1] = float('-inf')
    params.UB_history[-1] = float('inf')
    
    # 初始化 LB^0 = -∞, UB^0 = +∞（算法2 Step 0）
    params.LB_history[0] = float('-inf')
    params.UB_history[0] = float('inf')
    
    print(f"初始化完成 - r=0, D_r[0]={params.D_r[0]}, X_r_LS[0]={params.X_r_LS[0]}")
    print(f"初始化完成 - LB^{{-1}}={params.LB_history[-1]}, UB^{{-1}}={params.UB_history[-1]}")
    print(f"初始化完成 - LB^{{0}}={params.LB_history[0]}, UB^{{0}}={params.UB_history[0]}")
    
    return params
