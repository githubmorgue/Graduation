from gurobipy import Model, GRB, quicksum
import math


def solve_mp_model(r, params, dual_vars):
    """
    求解主问题 W^r(Γ)（Master Problem）。

    该函数用于 Benders 分解算法的 Step_2，构建并求解当前迭代轮次 r 的主问题。
    主问题基于历史对偶变量信息生成 Benders 割平面约束，逐步逼近最优解。

    数学模型：
    min  A

    s.t.
    (3.1) A ≥ sum_{l∈L} d_l*u_l^i + sum_{l∈L} d_hat_l*u_l^i*z_l^i
              + sum_{t∈T} sum_{b∈B_t} x_tb*q_t*g_t^i
              - sum_{t∈T} sum_{b∈B_t} x_tb*Q_t*h_t^i
              + sum_{t∈T} sum_{b∈B_t} LB_tb*x_tb*v_tb^i
              - sum_{t∈T} sum_{b∈B_t} UB_tb*x_tb*w_tb^i,  ∀i=0,...,r-1

    (3.2) sum_{b∈B_t} x_tb ≤ 1,  ∀t∈T

    (3.3) sum_{t∈T} sum_{b∈B_t} a_{tb}^l * x_tb ≤ 1,  ∀l∈L

    (3.4) N_min ≤ sum_{t∈T} sum_{b∈B_t} x_tb ≤ N_max

    x_tb ∈ {0,1}, A ≥ 0

    参数:
        r (int): 当前循环数。
        params (Params): 包含模型参数的对象。
        dual_vars (DualVars): 包含对偶变量的对象。

    返回:
        model: Gurobi 模型对象，包含求解结果

    异常:
        GurobiError: 当模型构建或求解失败时抛出

    示例:
        >>> model = solve_mp_model(r=0, params=params, dual_vars=dual_vars)
        >>> if model.status == GRB.OPTIMAL:
        ...     print(f"最优目标值: {model.objVal}")
    """

    # 创建模型
    model = Model("MP_Model")

    # 创建变量 (Create Variables)

    # A >= 0
    A = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name="A")

    # x_tb 是二元变量，使用元组 (t, b) 作为键
    x = {}
    for t in range(1, params.T + 1):
        for b in range(1, params.B + 1):
            x[t, b] = model.addVar(vtype=GRB.BINARY, name=f"x_{t}_{b}")

    # 设置目标函数 (Set Objective)
    # 目标函数 min A
    model.setObjective(A, GRB.MINIMIZE)

    # 添加约束 (Add Constraints)

    # 约束类型 1 (公式 3.1)
    # 对于每一轮 i = 0 ... r-1，使用该轮的对偶变量构建 Benders 割平面
    for i in range(r):
        # 获取当前场景的对偶变量系数
        u = dual_vars.u[i]
        z = dual_vars.z[i]
        g = dual_vars.g[i]
        h = dual_vars.h[i]
        v = dual_vars.v[i]
        w = dual_vars.w[i]

        # 构建右侧表达式 (RHS)
        expr = 0

        # 求和部分 1: sum_{l∈L} d_l * u_l^i
        for l in range(1, params.L + 1):
            expr += params.d_l[l] * u[l]

        # 求和部分 2: sum_{l∈L} d_hat_l * u_l^i * z_l^i
        for l in range(1, params.L + 1):
            expr += params.d_hat_l[l] * u[l] * z[l]

        # 求和部分 3: sum_{t∈T} sum_{b∈B_t} x_tb * q_t * g_t^i
        for t in range(1, params.T + 1):
            for b in range(1, params.B + 1):
                expr += x[t, b] * params.q_t[t] * g[t]

        # 求和部分 4: - sum_{t∈T} sum_{b∈B_t} x_tb * Q_t * h_t^i
        for t in range(1, params.T + 1):
            for b in range(1, params.B + 1):
                expr -= x[t, b] * params.Q_t[t] * h[t]

        # 求和部分 5: + sum_{t∈T} sum_{b∈B_t} LB_tb * x_tb * v_tb^i
        for t in range(1, params.T + 1):
            for b in range(1, params.B + 1):
                expr += params.LB_tb[t][b] * x[t, b] * v[t][b]

        # 求和部分 6: - sum_{t∈T} sum_{b∈B_t} UB_tb * x_tb * w_tb^i
        for t in range(1, params.T + 1):
            for b in range(1, params.B + 1):
                expr -= params.UB_tb[t][b] * x[t, b] * w[t][b]

        # 添加约束: A >= expr (公式 3.1)
        model.addConstr(A >= expr, name=f"Type1_Scenario_{i}")

    # 约束类型 2 (公式 3.2)
    # sum_{b∈B_t} x_tb ≤ 1, for t ∈ T
    for t in range(1, params.T + 1):
        model.addConstr(quicksum(x[t, b] for b in range(1, params.B + 1)) <= 1, name=f"Type2_Time_{t}")

    # 约束类型 3 (公式 3.3)
    # sum_{t∈T} sum_{b∈B_t} a_{tb}^l * x_tb ≤ 1, for l ∈ L
    for l in range(1, params.L + 1):
        line_expr = 0
        for t in range(1, params.T + 1):
            for b in range(1, params.B + 1):
                if params.a_tb_l[t][b][l] == 1:
                    line_expr += x[t, b]
        model.addConstr(line_expr <= 1, name=f"Type3_Line_{l}")

    # 约束类型 4 (公式 3.4)
    # N_min ≤ sum_{t∈T} sum_{b∈B_t} x_tb ≤ N_max
    total_x = quicksum(x[t, b] for t in range(1, params.T + 1) for b in range(1, params.B + 1))
    model.addConstr(total_x >= params.N_min, "MinCount")
    model.addConstr(total_x <= params.N_max, "MaxCount")

    # 优化模型
    model.optimize()

    return model
