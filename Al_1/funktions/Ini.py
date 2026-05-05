import random


from Al_1.models.Params import Params
from Al_1.models.DualVars import DualVars


def initialize_constraints_params(L: int, T: int, B : int) -> Params:
    """
    初始化约束参数
    """
    params = Params()

    # 1.1 确定参数数量
    params.L = L
    params.T = T
    params.B = B

    # 1.2 生成参数
    # alpha_param: [0.10, 0.50]
    params.alpha_param = round(random.uniform(0.10, 0.50), 2)

    # z_l: {0, 1} 变量 (针对每条车道)
    for l in range(1, L + 1):
        params.z_l[l] = random.choice([0, 1])

    # p_t: 绩效因子 [-1, 1] (针对每个承运商)
    for t in range(1, T + 1):
        params.p_t[t] = round(random.uniform(-1, 1), 4)
        # q_t: [10, 15], Q_t: [60, 75]
        params.q_t[t] = random.randint(10, 15)
        params.Q_t[t] = random.randint(60, 75)

    # N_min: 固定为2
    # N_max: 承运商数量的 20%
    params.N_min = 2
    params.N_max = max(2, int(T * 0.2))

    return params


def initialize_function_params(params: Params) -> Params:
    """
    初始化函数参数
    """
    # 如果传入了之前的参数，则在此基础上更新
    if not params:
        params = Params()

    # 2.1 确定参数数量
    T = params.T
    L = params.L
    B = params.B

    # 2.2 运输量
    for l in range(1, L + 1):
        # d_bar_l: 名义需求量，范围 [10, 50]
        params.d_bar_l[l] = random.randint(10, 50)
        # d_hat_l: 最大需求偏差，等于 alpha_param * d_bar_l
        params.d_hat_l[l] = params.alpha_param * params.d_bar_l[l]
        # d_l: 实际需求，根据波动变量 z_l 计算
        z_l_val = params.z_l.get(l, 0)
        params.d_l[l] = params.d_bar_l[l] + z_l_val * params.d_hat_l[l]

    # 2.3 投标相关 (针对每个承运商t和每条线路l的投标b)
    for t in range(1, T + 1):
        params.LB_tb[t] = {}
        params.UB_tb[t] = {}
        params.c_tb[t] = {}
        for b in range(1, B + 1):
            # LB: [10, 20], UB: [40, 75]
            params.LB_tb[t][b] = random.randint(10, 20)
            params.UB_tb[t][b] = random.randint(40, 75)
            # c_tb: 报价 [10, 80]
            params.c_tb[t][b] = round(random.uniform(10, 80), 2)

    # ce_l: 即期市场成本 [50, 100]
    for l in range(1, L + 1):
        params.ce_l[l] = random.randint(50, 100)
        params.M[l] = params.ce_l[l]

    return params


def initialize_coverage_matrix(params: Params, density: float = 0.235) -> Params:
    """
    初始化覆盖矩阵 a_{tb}^l

    参数:
        params: Params 对象
        density: 覆盖矩阵的密度（默认 23.5%，范围 22%-25%）

    返回:
        更新了 a_tb_l 的 Params 对象
    """
    T = params.T
    L = params.L
    B = params.B

    # 初始化三维字典
    for t in range(1, T + 1):
        params.a_tb_l[t] = {}
        for b in range(1, B + 1):
            params.a_tb_l[t][b] = {}
            for l in range(1, L + 1):
                # 根据密度随机生成 0 或 1
                params.a_tb_l[t][b][l] = 1 if random.random() < density else 0

    return params


def initialize_dual_variables(params: Params, r: int = 1) -> DualVars:
    """
    初始化对偶变量
    """
    # 确保轮数为1
    if r != 1:
        print("警告: 文档要求初始化轮数 r 必须为 1。强制设置为 1。")
        r = 1

    T = params.T
    B = params.B
    L = params.L

    # 3.2 生成参数: 所有变量在第1轮初始化为 0
    dual_vars = DualVars()

    # 初始化第 r 轮的变量
    dual_vars.u[r-1] = {}
    dual_vars.v[r-1] = {}
    dual_vars.w[r-1] = {}
    dual_vars.g[r-1] = {}
    dual_vars.h[r-1] = {}
    dual_vars.z[r-1] = {}

    for l in range(1, L + 1):
        # 使用字典存储每个车道l的变量值
        dual_vars.u[r-1][l] = 0
        dual_vars.z[r-1][l] = 0

    for t in range(1, T + 1):
        dual_vars.g[r-1][t] = 0
        dual_vars.h[r-1][t] = 0

    for t in range(1, T + 1):
        dual_vars.w[r-1][t] = {}
        dual_vars.v[r-1][t] = {}
        for b in range(1, B + 1):
            dual_vars.w[r-1][t][b] = 0
            dual_vars.v[r-1][t][b] = 0

    return dual_vars


def initialize_decision_variables(params: Params) -> Params:
    """
    初始化决策变量 x_tb_r（第1轮设为0）和 D_r 集合
    
    参数:
        params: Params 对象
    
    返回:
        更新了 x_tb_r 和 D_r 的 Params 对象
    """
    T = params.T
    B = params.B
    
    # 初始化第1轮的 x_tb_r[1][t][b]，所有元素设为 0
    params.x_tb_r[1] = {}
    for t in range(1, T + 1):
        params.x_tb_r[1][t] = {}
        for b in range(1, B + 1):
            params.x_tb_r[1][t][b] = 0
    
    # 初始化第1轮的 D_r[1] 为空列表
    params.D_r[1] = []
    
    return params

