import random

from Al_1.models.Params import Params
from Al_1.models.DualVars import DualVars


def initialize_constraints_params(L: int, T: int, B : int) -> Params:
    """
    初始化 Benders 分解算法的约束参数。

    该函数负责设置问题的基础规模（线路、承运商、投标数量），并随机生成
    承运商绩效因子、中标数量上下限以及不确定性预算比例。

    参数:
        L (int): 线路 (Lane) 的数量。
        T (int): 承运商 (Truck/Carrier) 的数量。
        B (int): 每个承运商提供的投标 (Bid) 数量。

    返回:
        Params: 填充了基础约束参数的 Params 对象。
    
    生成规则:
        - alpha_param: 不确定性预算比例，范围 [0.10, 0.50]。
        - p_t: 承运商绩效因子，范围 [-1, 1]。
        - q_t: 单个承运商中标下限，范围 [10, 15]。
        - Q_t: 单个承运商中标上限，范围 [60, 75]。
        - N_min: 全局中标承运商数量下限，固定为 2。
        - N_max: 全局中标承运商数量上限，约为承运商总数的 20%。
    """
    params = Params()

    # 1.1 确定参数数量
    params.L = L
    params.T = T
    params.B = B

    # 1.2 生成参数
    # alpha_param: [0.10, 0.50]
    params.alpha_param = round(random.uniform(0.10, 0.50), 2)

    # z_l: 针对每条车道的二元波动变量 {0, 1}
    for l in range(1, L + 1):
        params.z_l[l] = random.choice([0, 1])

    # p_t: 承运商绩效因子 [-1, 1] (针对每个承运商)
    # q_t: 中标下限 [10, 15], Q_t: 中标上限 [60, 75]
    for t in range(1, T + 1):
        params.p_t[t] = round(random.uniform(-1, 1), 4)
        params.q_t[t] = random.randint(10, 15)
        params.Q_t[t] = random.randint(60, 75)

    # N_min: 固定为2
    # N_max: 承运商数量的 20%
    params.N_min = 2
    params.N_max = max(2, int(T * 0.2))

    return params


def initialize_function_params(params: Params) -> Params:
    """
    初始化鲁棒优化模型所需的函数参数。

    基于已有的基础参数，生成名义需求、最大需求偏差、投标上下限、
    投标价格以及即期市场成本。

    参数:
        params (Params): 已包含 L, T, B, alpha_param 等基础参数的对象。

    返回:
        Params: 补充了函数参数的 Params 对象。
        
    生成规则:
        - d_bar_l: 名义需求量，范围 [10, 50]。
        - d_hat_l: 最大需求偏差，等于 alpha_param * d_bar_l。
        - d_l: 实际需求，基于 z_l 的取值计算。
        - LB_tb: 投标运输量下限，范围 [10, 20]。
        - UB_tb: 投标运输量上限，范围 [40, 75]。
        - c_tb: 投标报价，范围 [10, 80]。
        - ce_l: 即期市场补货成本，范围 [50, 100]。
    """
    # 如果传入了之前的参数，则在此基础上更新
    if not params:
        params = Params()

    # 2.1 确定参数数量
    T = params.T
    L = params.L
    B = params.B

    # 2.2 运输量与需求参数
    for l in range(1, L + 1):
        # d_bar_l: 名义需求量，范围 [10, 50]
        params.d_bar_l[l] = random.randint(10, 50)
        # d_hat_l: 最大需求偏差，等于 alpha_param * d_bar_l
        params.d_hat_l[l] = params.alpha_param * params.d_bar_l[l]
        # d_l: 实际需求，根据波动变量 z_l 计算 (d_l = d_bar_l + z_l * d_hat_l)
        z_l_val = params.z_l.get(l, 0)
        params.d_l[l] = params.d_bar_l[l] + z_l_val * params.d_hat_l[l]

    # 2.3 投标相关参数 (针对每个承运商 t 和每个投标 b)
    for t in range(1, T + 1):
        params.LB_tb[t] = {}
        params.UB_tb[t] = {}
        params.c_tb[t] = {}
        for b in range(1, B + 1):
            # LB: 投标下限 [10, 20], UB: 投标上限 [40, 75]
            params.LB_tb[t][b] = random.randint(10, 20)
            params.UB_tb[t][b] = random.randint(40, 75)
            # c_tb: 投标报价 [10, 80]
            params.c_tb[t][b] = round(random.uniform(10, 80), 2)

    # ce_l: 即期市场成本 [50, 100]，M 作为辅助参数通常等于 ce_l
    for l in range(1, L + 1):
        params.ce_l[l] = random.randint(50, 100)
        params.M[l] = params.ce_l[l]

    return params


def initialize_coverage_matrix(params: Params, density: float = 0.235) -> Params:
    """
    初始化覆盖矩阵 a_{tb}^l。

    该矩阵表示承运商 t 的投标 b 是否覆盖线路 l。矩阵采用稀疏生成方式，
    通过设定的密度参数控制 1 的出现概率。

    参数:
        params (Params): 包含维度参数 (T, L, B) 的对象。
        density (float): 覆盖矩阵的密度（默认 23.5%，即 1 出现的概率）。

    返回:
        Params: 更新了三维字典 a_tb_l 的对象。
    """
    T = params.T
    L = params.L
    B = params.B

    # 初始化三维字典 a_tb_l[t][b][l]
    for t in range(1, T + 1):
        params.a_tb_l[t] = {}
        for b in range(1, B + 1):
            params.a_tb_l[t][b] = {}
            for l in range(1, L + 1):
                # 根据密度随机生成 0 或 1
                params.a_tb_l[t][b][l] = 1 if random.random() < density else 0

    return params


def initialize_dual_variables(params: Params) -> DualVars:
    """
    初始化对偶变量空间。

    为第 0 轮迭代 (r=0) 分配所有对偶变量 (u, v, w, g, h, z) 的存储空间，
    并将其初始值设为 0。这些变量将用于构建主问题中的 Benders 割平面。

    参数:
        params (Params): 用于获取维度参数 (T, B, L) 的对象。

    返回:
        DualVars: 初始化完成的对偶变量对象。
    """
    T = params.T
    B = params.B
    L = params.L

    # 3.2 生成参数: 所有变量在第1轮初始化为 0
    dual_vars = DualVars()

    # 初始化对偶变量
    dual_vars.u[0] = {}
    dual_vars.v[0] = {}
    dual_vars.w[0] = {}
    dual_vars.g[0] = {}
    dual_vars.h[0] = {}
    dual_vars.z[0] = {}

    # 初始化车道维度的变量 u_l, z_l
    for l in range(1, L + 1):
        dual_vars.u[0][l] = 0
        dual_vars.z[0][l] = 0

    # 初始化承运商维度的变量 g_t, h_t
    for t in range(1, T + 1):
        dual_vars.g[0][t] = 0
        dual_vars.h[0][t] = 0

    # 初始化承运商-投标维度的变量 v_tb, w_tb
    for t in range(1, T + 1):
        dual_vars.w[0][t] = {}
        dual_vars.v[0][t] = {}
        for b in range(1, B + 1):
            dual_vars.w[0][t][b] = 0
            dual_vars.v[0][t][b] = 0

    return dual_vars


def initialize_decision_variables(params: Params) -> Params:
    """
    初始化决策变量 x_tb_r 及相关迭代容器。

    设置第 1 轮 (r=1) 的主问题决策变量初始值为 0，为算法迭代做准备。
    （注：Al_1 版本通常从 r=1 开始迭代，Al_2 版本从 r=0 开始）。

    参数:
        params (Params): 包含维度参数 (T, B) 的对象。

    返回:
        Params: 更新了 x_tb_r 容器的 Params 对象。
    """
    T = params.T
    B = params.B
    
    # 初始化第 1 轮的 x_tb_r[1][t][b]，所有元素设为 0
    params.x_tb_r[1] = {}
    for t in range(1, T + 1):
        params.x_tb_r[1][t] = {}
        for b in range(1, B + 1):
            params.x_tb_r[1][t][b] = 0
    
    return params

