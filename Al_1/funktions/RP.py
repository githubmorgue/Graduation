from gurobipy import Model, GRB, quicksum


def solve_rp_model(r, gamma, xtb, params):
    """
    该函数用于 Benders 分解算法的 Step_3，构建并求解给定主问题解 xtb 的鲁棒对偶子问题。
    子问题在不确定性集合内寻找最坏情况，生成对应的对偶变量信息用于构建 Benders 割平面。

    数学模型：
    max  sum_{l∈L} d_bar_l*u_l + sum_{l∈L} d_hat_l*s_l 
         + sum_{t∈T} sum_{b∈B_t} LB_tb*x_tb*v_tb 
         - sum_{t∈T} sum_{b∈B_t} UB_tb*x_tb*w_tb 
         + sum_{t∈T} sum_{b∈B_t} x_tb*q_t*g_t 
         - sum_{t∈T} sum_{b∈B_t} x_tb*Q_t*h_t

    s.t.
    (4.1) sum_{l∈L} a_{tb}^l * u_l + v_tb - w_tb + g_t - h_t ≤ (1 + p_t) * c_tb,  ∀t∈T, b∈B_t

    (4.2) u_l ≤ ce_l,  ∀l∈L

    (4.3) sum_{l∈L} z_l ≤ Γ

    (4.4) s_l ≤ ce_l * z_l,  ∀l∈L

    (4.5) s_l ≤ u_l,  ∀l∈L

    u_l ≥ 0, s_l ≥ 0, v_tb ≥ 0, w_tb ≥ 0, g_t ≥ 0, h_t ≥ 0, z_l ∈ {0,1}

    参数:
        r (int): 当前迭代轮次，表示第 r 轮 Benders 分解
        gamma (float): 不确定性预算参数 Γ，控制鲁棒性程度
        xtb (dict): 主问题给定的二元决策变量值，xtb[t][b] 表示承运商 t 的投标 b 是否被选中
        params (Params): 包含模型参数的 Params 对象，包括：
            - T: 承运商数量
            - L: 车道数量
            - B: 投标数量
            - alpha_param: 需求偏差系数 α
            - z_l: 二进制变量（预留）
            - p_t: 承运商价格调整因子
            - q_t, Q_t: 承运商运输量下限和上限
            - N_min, N_max: 中标数量下限和上限
            - d_bar_l: 名义需求
            - d_hat_l: 最大需求偏差
            - d_l: 实际需求
            - LB_tb, UB_tb: 投标上下限
            - c_tb: 投标价格
            - ce_l: 即期市场成本
            - M: 辅助参数（大M值）
            - a_tb_l: 覆盖矩阵

    返回:
        model: Gurobi 模型对象，包含求解结果和对偶变量值

    异常:
        GurobiError: 当模型构建或求解失败时抛出

    示例:
        >>> model = solve_rp_model(r=0, gamma=5.0, xtb=xtb_solution, params=params)
        >>> if model.status == GRB.OPTIMAL:
        ...     print(f"最坏情况目标值: {model.objVal}")
    """

    # 创建模型
    model = Model("RP_Model")

    # 创建变量 (Create Variables)

    # u_l (连续变量) - 索引从1到L
    u = {}
    for l in range(1, params.L + 1):
        u[l] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"u_{l}")

    # s_l (连续变量) - 索引从1到L
    s = {}
    for l in range(1, params.L + 1):
        s[l] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"s_{l}")

    # z_l (二元变量) - 索引从1到L
    z = {}
    for l in range(1, params.L + 1):
        z[l] = model.addVar(vtype=GRB.BINARY, name=f"z_{l}")

    # v_{tb}, w_{tb}, g_t, h_t (连续变量)
    # 使用嵌套字典存储
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

    # 设置目标函数 (Set Objective)
    # max sum(d_bar_l * u_l) + sum(d_hat_l * s_l) + sum(LB_tb * x_tb * v_tb) 
    #     - sum(UB_tb * x_tb * w_tb) + sum(x_tb * q_t * g_t) - sum(x_tb * Q_t * h_t)
    obj_expr = (
            quicksum(params.d_bar_l[l] * u[l] for l in range(1, params.L + 1)) +
            quicksum(params.d_hat_l[l] * s[l] for l in range(1, params.L + 1)) +
            quicksum(params.LB_tb[t][b] * xtb[t][b] * v[t, b] for t in range(1, params.T + 1) for b in range(1, params.B + 1)) -
            quicksum(params.UB_tb[t][b] * xtb[t][b] * w[t, b] for t in range(1, params.T + 1) for b in range(1, params.B + 1)) +
            quicksum(xtb[t][b] * params.q_t[t] * g[t] for t in range(1, params.T + 1) for b in range(1, params.B + 1)) -
            quicksum(xtb[t][b] * params.Q_t[t] * h[t] for t in range(1, params.T + 1) for b in range(1, params.B + 1))
    )

    model.setObjective(obj_expr, GRB.MAXIMIZE)

    # 添加约束 (Add Constraints)

    # 约束类型 1 (公式 4.1)
    # sum_l(a_tbl * u_l) + v_tb - w_tb + g_t - h_t <= (1 + p_t) * c_tb, ∀t∈T, b∈B_t
    for t in range(1, params.T + 1):
        for b in range(1, params.B + 1):
            expr = quicksum(params.a_tb_l[t][b][l] * u[l] for l in range(1, params.L + 1)) + v[t, b] - w[t, b] + g[t] - h[t]
            model.addConstr(expr <= (1 + params.p_t[t]) * params.c_tb[t][b], name=f"C1_{t}_{b}")

    # 约束类型 2 (公式 4.2)
    # u_l <= ce_l, ∀l∈L
    for l in range(1, params.L + 1):
        model.addConstr(u[l] <= params.ce_l[l], name=f"C2_{l}")

    # 约束类型 3 (公式 4.3)
    # sum_l(z_l) <= Gamma
    model.addConstr(quicksum(z[l] for l in range(1, params.L + 1)) <= gamma, name="C3_Budget")

    # 约束类型 4 (公式 4.4)
    # s_l <= ce_l * z_l, ∀l∈L
    for l in range(1, params.L + 1):
        model.addConstr(s[l] <= params.ce_l[l] * z[l], name=f"C4_slack_{l}")

    # 约束类型 5 (公式 4.5)
    # s_l <= u_l, ∀l∈L
    for l in range(1, params.L + 1):
        model.addConstr(s[l] <= u[l], name=f"C5_link_{l}")

    # 优化模型
    model.optimize()

    # 返回模型 (包含解)
    return model

