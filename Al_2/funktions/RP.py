from gurobipy import Model, GRB, quicksum


def solve_rp_model(gamma, xtb, params):
    """
    构建并求解鲁棒优化子问题 (Robust Problem)。

    该函数用于 Benders 分解算法的 Step 3 和 Step 5，对给定的候选解求解
    鲁棒优化子问题，提取对偶变量用于生成 Benders 割平面。

    数学模型（对偶形式）：
    max  sum_{l∈L} d_bar_l*u_l + sum_{l∈L} d_hat_l*s_l
         + sum_{t∈T} sum_{b∈B_t} LB_tb*x_tb*v_tb - sum_{t∈T} sum_{b∈B_t} UB_tb*x_tb*w_tb
         + sum_{t∈T} sum_{b∈B_t} x_tb*q_t*g_t - sum_{t∈T} sum_{b∈B_t} x_tb*Q_t*h_t

    s.t.
    (1) sum_{l∈L} a_tbl*u_l + v_tb - w_tb + g_t - h_t <= (1+p_t)*c_tb,  ∀t∈T, b∈B_t
    (2) u_l <= ce_l,  ∀l∈L
    (3) sum_{l∈L} z_l <= Gamma
    (4) s_l <= M*z_l,  ∀l∈L
    (5) s_l <= u_l,  ∀l∈L
    (6) u_l >= 0, s_l >= 0, v_tb >= 0, w_tb >= 0, g_t >= 0, h_t >= 0, z_l ∈ {0,1}

    参数:
        gamma (int): 不确定性预算参数 Γ，控制最坏情况下允许偏离名义需求的车道数量
        xtb (dict): 二元变量 x_{tb} 的值（来自主问题的解），格式为 xtb[t][b]，值为 0 或 1
        params (Params): 包含模型参数的 Params 对象，包括 d_bar_l, d_hat_l, LB_tb, UB_tb 等

    返回:
        model: Gurobi 模型对象。如果求解成功，可通过 model.getVarByName() 获取变量值
    """

    # 创建模型
    model = Model("RP_Model")

    # ========== 创建决策变量 ==========

    # u_l: 对应名义需求 d_bar_l 的对偶变量（连续变量，非负）- 索引从1到L
    u = {}
    for l in range(1, params.L + 1):
        u[l] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"u_{l}")

    # s_l: 对应需求偏差 d_hat_l 的松弛变量（连续变量，非负）- 索引从1到L
    s = {}
    for l in range(1, params.L + 1):
        s[l] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"s_{l}")

    # z_l: 二元变量，标识车道 l 是否处于最坏情况（z_l=1 表示需求为 d_bar_l + d_hat_l）
    z = {}
    for l in range(1, params.L + 1):
        z[l] = model.addVar(vtype=GRB.BINARY, name=f"z_{l}")

    # v_{tb}, w_{tb}: 对应运输量上下限约束的对偶变量（连续变量，非负）
    # g_t, h_t: 对应承运商运输量约束的对偶变量（连续变量，非负）
    v = {}
    w = {}
    
    # g_t 和 h_t - 索引从1到T
    g = {}
    for t in range(1, params.T + 1):
        g[t] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"g_{t}")
    
    h = {}
    for t in range(1, params.T + 1):
        h[t] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"h_{t}")

    for t in range(1, params.T + 1):
        for b in range(1, params.B + 1):
            v[t, b] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"v_{t}_{b}")
            w[t, b] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"w_{t}_{b}")

    # ========== 设置目标函数 ==========
    # max sum(d_bar_l * u_l) + sum(d_hat_l * s_l) 
    #     + sum(LB_tb * x_tb * v_tb) - sum(UB_tb * x_tb * w_tb) 
    #     + sum(x_tb * q_t * g_t) - sum(x_tb * Q_t * h_t)
    obj_expr = (
            quicksum(params.d_bar_l[l] * u[l] for l in range(1, params.L + 1)) +
            quicksum(params.d_hat_l[l] * s[l] for l in range(1, params.L + 1)) +
            quicksum(params.LB_tb[t][b] * xtb[t][b] * v[t, b] for t in range(1, params.T + 1) for b in range(1, params.B + 1)) -
            quicksum(params.UB_tb[t][b] * xtb[t][b] * w[t, b] for t in range(1, params.T + 1) for b in range(1, params.B + 1)) +
            quicksum(xtb[t][b] * params.q_t[t] * g[t] for t in range(1, params.T + 1) for b in range(1, params.B + 1)) -
            quicksum(xtb[t][b] * params.Q_t[t] * h[t] for t in range(1, params.T + 1) for b in range(1, params.B + 1))
    )

    model.setObjective(obj_expr, GRB.MAXIMIZE)

    # ========== 添加约束 ==========

    # 约束类型 1: 投标价格约束
    # sum_l(a_tbl * u_l) + v_tb - w_tb + g_t - h_t <= (1 + p_t) * c_tb, ∀t∈T, b∈B_t
    for t in range(1, params.T + 1):
        for b in range(1, params.B + 1):
            expr = quicksum(params.a_tb_l[t][b][l] * u[l] for l in range(1, params.L + 1)) + v[t, b] - w[t, b] + g[t] - h[t]
            model.addConstr(expr <= (1 + params.p_t[t]) * params.c_tb[t][b], name=f"C1_{t}_{b}")

    # 约束类型 2: 即期市场成本上界约束
    # u_l <= ce_l, ∀l∈L
    for l in range(1, params.L + 1):
        model.addConstr(u[l] <= params.ce_l[l], name=f"C2_{l}")

    # 约束类型 3: 不确定性预算约束
    # sum_l(z_l) <= Gamma，限制最多有 Gamma 个车道处于最坏情况
    model.addConstr(quicksum(z[l] for l in range(1, params.L + 1)) <= gamma, name="C3_Budget")

    # 约束类型 4: 松弛变量与二元变量的关联约束
    # s_l <= M * z_l, 其中 M[l] = ce_l[l]
    # 当 z_l=0 时，s_l 必须为 0；当 z_l=1 时，s_l 可以取正值
    for l in range(1, params.L + 1):
        model.addConstr(s[l] <= params.ce_l[l] * z[l], name=f"C4_slack_{l}")

    # 约束类型 5: 松弛变量上界约束
    # s_l <= u_l，确保松弛变量不超过对应的对偶变量
    for l in range(1, params.L + 1):
        model.addConstr(s[l] <= u[l], name=f"C5_link_{l}")

    # ========== 求解模型 ==========
    model.optimize()

    # 返回模型对象（包含求解结果）
    return model



def solve_deterministic_rp_model(xtb, params, z_l):
    """
    构建并求解确定性鲁棒优化子问题（z_l 已固定）。

    该函数用于 Step_1 中求解补救问题，其中 z_l 已由 generate_demand_scenario 
    确定为固定值（按最坏需求降序排列，前 Gamma 个车道 z_l=1，其余 z_l=0）。

    与 solve_rp_model 的区别：
    - solve_rp_model: z_l 是二元决策变量，需要在优化中确定
    - solve_deterministic_rp_model: z_l 是已知的固定值，作为参数传入并被固定

    数学模型（对偶形式，z_l 固定）：
    max  sum_{l∈L} d_bar_l*u_l + sum_{l∈L} d_hat_l*s_l
         + sum_{t∈T} sum_{b∈B_t} LB_tb*x_tb*v_tb - sum_{t∈T} sum_{b∈B_t} UB_tb*x_tb*w_tb
         + sum_{t∈T} sum_{b∈B_t} x_tb*q_t*g_t - sum_{t∈T} sum_{b∈B_t} x_tb*Q_t*h_t

    s.t.
    (1) sum_{l∈L} a_tbl*u_l + v_tb - w_tb + g_t - h_t <= (1+p_t)*c_tb,  ∀t∈T, b∈B_t
    (2) u_l <= ce_l,  ∀l∈L
    (3) s_l <= M*z_l,  ∀l∈L（z_l 为已知固定值）
    (4) s_l <= u_l,  ∀l∈L
    (5) u_l >= 0, s_l >= 0, v_tb >= 0, w_tb >= 0, g_t >= 0, h_t >= 0

    参数:
        xtb (dict): 二元变量 x_{tb} 的值（来自主问题的解），格式为 xtb[t][b]，值为 0 或 1
        params (Params): 包含模型参数的 Params 对象
        z_l (dict): 已确定的 z_l 值，格式为 z_l[l]，l=1,...,L，值为 0 或 1

    返回:
        model: Gurobi 模型对象。如果求解成功，可通过 model.getVarByName() 获取变量值
    """

    # 创建模型
    model = Model("RP_Model_Fixed_Z")

    # ========== 创建决策变量 ==========

    # u_l: 对应名义需求 d_bar_l 的对偶变量（连续变量，非负）- 索引从1到L
    u = {}
    for l in range(1, params.L + 1):
        u[l] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"u_{l}")

    # s_l: 对应需求偏差 d_hat_l 的松弛变量（连续变量，非负）- 索引从1到L
    s = {}
    for l in range(1, params.L + 1):
        s[l] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"s_{l}")

    # z_l: 二元变量，虽然在此函数中被固定为已知值，但仍需创建为模型变量以便提取
    z = {}
    for l in range(1, params.L + 1):
        z[l] = model.addVar(vtype=GRB.BINARY, name=f"z_{l}")

    # 固定 z_l 为传入的值（通过等式约束强制 z_l 等于给定值）
    for l in range(1, params.L + 1):
        model.addConstr(z[l] == z_l[l], name=f"Fix_z_{l}")

    # v_{tb}, w_{tb}: 对应运输量上下限约束的对偶变量（连续变量，非负）
    # g_t, h_t: 对应承运商运输量约束的对偶变量（连续变量，非负）
    v = {}
    w = {}

    # g_t 和 h_t - 索引从1到T
    g = {}
    for t in range(1, params.T + 1):
        g[t] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"g_{t}")

    h = {}
    for t in range(1, params.T + 1):
        h[t] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"h_{t}")

    for t in range(1, params.T + 1):
        for b in range(1, params.B + 1):
            v[t, b] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"v_{t}_{b}")
            w[t, b] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"w_{t}_{b}")

    # ========== 设置目标函数 ==========
    # max sum(d_bar_l * u_l) + sum(d_hat_l * s_l) 
    #     + sum(LB_tb * x_tb * v_tb) - sum(UB_tb * x_tb * w_tb) 
    #     + sum(x_tb * q_t * g_t) - sum(x_tb * Q_t * h_t)
    obj_expr = (
            quicksum(params.d_bar_l[l] * u[l] for l in range(1, params.L + 1)) +
            quicksum(params.d_hat_l[l] * s[l] for l in range(1, params.L + 1)) +
            quicksum(params.LB_tb[t][b] * xtb[t][b] * v[t, b] for t in range(1, params.T + 1) for b in
                     range(1, params.B + 1)) -
            quicksum(params.UB_tb[t][b] * xtb[t][b] * w[t, b] for t in range(1, params.T + 1) for b in
                     range(1, params.B + 1)) +
            quicksum(xtb[t][b] * params.q_t[t] * g[t] for t in range(1, params.T + 1) for b in range(1, params.B + 1)) -
            quicksum(xtb[t][b] * params.Q_t[t] * h[t] for t in range(1, params.T + 1) for b in range(1, params.B + 1))
    )

    model.setObjective(obj_expr, GRB.MAXIMIZE)

    # ========== 添加约束 ==========

    # 约束类型 1: 投标价格约束
    # sum_l(a_tbl * u_l) + v_tb - w_tb + g_t - h_t <= (1 + p_t) * c_tb, ∀t∈T, b∈B_t
    for t in range(1, params.T + 1):
        for b in range(1, params.B + 1):
            expr = quicksum(params.a_tb_l[t][b][l] * u[l] for l in range(1, params.L + 1)) + v[t, b] - w[t, b] + g[t] - \
                   h[t]
            model.addConstr(expr <= (1 + params.p_t[t]) * params.c_tb[t][b], name=f"C1_{t}_{b}")

    # 约束类型 2: 即期市场成本上界约束
    # u_l <= ce_l, ∀l∈L
    for l in range(1, params.L + 1):
        model.addConstr(u[l] <= params.ce_l[l], name=f"C2_{l}")

    # 约束类型 4: 松弛变量与固定二元变量的关联约束
    # s_l <= M * z_l, 其中 M[l] = ce_l[l]，z_l 是已知的固定值
    # 由于 z_l 已被固定，此约束直接限制 s_l 的上界
    for l in range(1, params.L + 1):
        model.addConstr(s[l] <= params.ce_l[l] * z_l[l], name=f"C4_slack_{l}")

    # 约束类型 5: 松弛变量上界约束
    # s_l <= u_l，确保松弛变量不超过对应的对偶变量
    for l in range(1, params.L + 1):
        model.addConstr(s[l] <= u[l], name=f"C5_link_{l}")

    # ========== 求解模型 ==========
    model.optimize()

    # 返回模型对象（包含求解结果）
    return model

