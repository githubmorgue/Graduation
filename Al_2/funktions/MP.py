from gurobipy import Model, GRB, quicksum
import math
from ..models.Params import SolutionInfo


def solve_mp_model(r, params, dual_vars):
    """
    求解主问题 W^r(Γ)（Master Problem）。

    该函数用于 Benders 分解算法的 Step_2，构建并求解当前迭代轮次 r 的主问题。
    主问题基于历史对偶变量信息生成 Benders 割平面约束，逐步逼近最优解。

    数学模型：
    min  A

    s.t.
    (19) A ≥ sum_{l∈L} d_bar_l*u_l^{ik} + sum_{l∈L} d_hat_l*u_l^{ik}*z_l^{ik}
              + sum_{t∈T} sum_{b∈B_t} x_tb*q_t*g_t^{ik}
              - sum_{t∈T} sum_{b∈B_t} x_tb*Q_t*h_t^{ik}
              + sum_{t∈T} sum_{b∈B_t} LB_tb*x_tb*v_tb^{ik}
              - sum_{t∈T} sum_{b∈B_t} UB_tb*x_tb*w_tb^{ik},  ∀i=0,...,r, k=0,...,K^i

    (2)  sum_{b∈B_t} x_tb ≤ 1,  ∀t∈T

    (3)  sum_{t∈T} sum_{b∈B_t} a_{tb}^l * x_tb ≤ 1,  ∀l∈L

    (4)  N_min ≤ sum_{t∈T} sum_{b∈B_t} x_tb ≤ N_max

    (20) LB^{r-1} ≤ A ≤ UB^{r-1}

    x_tb ∈ {0,1}, A ≥ 0

    其中 K^r 的计算公式为：
    - 当 r = 0 时：K^0 = 0
    - 当 r ≠ 0 时：K^r = |D^{r-1}| + |X^{r-1,LS}| - 1

    参数:
        r (int): 当前迭代轮次，表示第 r 轮 Benders 分解
        params (Params): 包含模型参数的 Params 对象，包括：
            - T: 承运商数量
            - L: 车道数量
            - B: 投标数量
            - d_bar_l: 名义需求
            - d_hat_l: 最大需求偏差
            - q_t, Q_t: 承运商运输量下限和上限
            - LB_tb, UB_tb: 投标上下限
            - a_tb_l: 覆盖矩阵
            - N_min, N_max: 中标数量下限和上限
            - D_r: 每轮迭代的解集合
            - X_r_LS: 每轮迭代的局部搜索解集合
            - K_r: 每轮迭代的对偶变量对数量
            - LB_history, UB_history: 历史边界值
        dual_vars (DualVars): 包含对偶变量的 DualVars 对象，结构为：
            - u[i][k], v[i][k], w[i][k], g[i][k], h[i][k], z[i][k]: 
              第 i 轮第 k 对对偶变量，用于构建 Benders 割平面

    返回:
        list: 包含所有可行解的列表，每个元素为SolutionInfo对象，按目标值降序排列
              （最后一个元素是最优解x^r，符合算法要求）

    异常:
        GurobiError: 当模型构建或求解失败时抛出

    示例:
        >>> solutions = solve_mp_model(r=0, params=params, dual_vars=dual_vars)
        >>> if solutions:
        ...     print(f"找到{len(solutions)}个可行解")
        ...     print(f"最优目标值: {solutions[-1].obj_value}")
    """

    # 计算当前轮次的对偶变量对数量 K^r
    # 根据图片公式：K^r = |D^{r-1}| + |X^{r-1,LS}| - 1 (当 r≠0)，K^0 = 0
    if r == 0:
        params.K_r[r] = 0
    else:
        D_r_minus_1_size = len(params.D_r.get(r - 1, []))
        X_r_minus_1_LS_size = len(params.X_r_LS.get(r - 1, []))
        params.K_r[r] = D_r_minus_1_size + X_r_minus_1_LS_size - 1

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

    # 约束类型 1 (公式19)
    # 对于每一轮 i = 0 ... r，遍历该轮的所有对偶变量 k = 0 ... K^i
    constraint_idx = 0  # 约束计数器
    
    for i in range(0, r+1):
        for k in range(0, params.K_r[i] + 1):
            u = dual_vars.u[i][k]
            v = dual_vars.v[i][k]
            g = dual_vars.g[i][k]
            h = dual_vars.h[i][k]
            z = dual_vars.z[i][k]
            w = dual_vars.w[i][k]

            # 构建右侧表达式 (RHS)
            expr = 0

            # 求和部分 1: sum_{l∈L} d_bar_l * u_l^{ik}
            for l in range(1, params.L + 1):
                expr += params.d_bar_l[l] * u[l]

            # 求和部分 2: sum_{l∈L} d_hat_l * u_l^{ik} * z_l^{ik}
            for l in range(1, params.L + 1):
                expr += params.d_hat_l[l] * u[l] * z[l]

            # 求和部分 3: sum_{t∈T} sum_{b∈B_t} x_tb * q_t * g_t^{ik}
            for t in range(1, params.T + 1):
                for b in range(1, params.B + 1):
                    expr += x[t, b] * params.q_t[t] * g[t]

            # 求和部分 4: - sum_{t∈T} sum_{b∈B_t} x_tb * Q_t * h_t^{ik}
            for t in range(1, params.T + 1):
                for b in range(1, params.B + 1):
                    expr -= x[t, b] * params.Q_t[t] * h[t]

            # 求和部分 5: + sum_{t∈T} sum_{b∈B_t} LB_tb * x_tb * v_tb^{ik}
            for t in range(1, params.T + 1):
                for b in range(1, params.B + 1):
                    expr += params.LB_tb[t][b] * x[t, b] * v[t][b]

            # 求和部分 6: - sum_{t∈T} sum_{b∈B_t} UB_tb * x_tb * w_tb^{ik}
            for t in range(1, params.T + 1):
                for b in range(1, params.B + 1):
                    expr -= params.UB_tb[t][b] * x[t, b] * w[t][b]

            # 添加约束: A >= expr (公式19)
            model.addConstr(A >= expr, name=f"Type1_i{i}_k{k}")
            constraint_idx += 1

    # 约束类型 2 (公式第二行)
    # sum_{b∈B_t} x_tb ≤ 1, for t ∈ T
    for t in range(1, params.T + 1):
        model.addConstr(quicksum(x[t, b] for b in range(1, params.B + 1)) <= 1, name=f"Type2_Time_{t}")

    # 约束类型 3 (公式第三行)
    # sum_{t∈T} sum_{b∈B_t} a_{tb}^l * x_tb ≤ 1, for l ∈ L
    for l in range(1, params.L + 1):
        line_expr = 0
        for t in range(1, params.T + 1):
            for b in range(1, params.B + 1):
                if params.a_tb_l[t][b][l] == 1:
                    line_expr += x[t, b]
        model.addConstr(line_expr <= 1, name=f"Type3_Line_{l}")

    # 约束类型 4 (公式第四行)
    # N_min ≤ sum_{t∈T} sum_{b∈B_t} x_tb ≤ N_max
    total_x = quicksum(x[t, b] for t in range(1, params.T + 1) for b in range(1, params.B + 1))
    model.addConstr(total_x >= params.N_min, "MinCount")
    model.addConstr(total_x <= params.N_max, "MaxCount")

    # 边界约束 (公式20)
    # LB^{r-1} ≤ A ≤ UB^{r-1}
    # 根据图片算法2 Step 0: LB^{-1} ← -∞, UB^{-1} ← +∞
    # LB^{r-1} 和 UB^{r-1} 是目标函数A的历史边界值（不是LB_tb和UB_tb）
    
    # 获取 LB^{r-1} 和 UB^{r-1}
    LB_prev = params.LB_history.get(r - 1, float('-inf'))
    UB_prev = params.UB_history.get(r - 1, float('inf'))
    
    # 添加边界约束（仅当边界值有效时）
    if LB_prev > float('-inf'):
        model.addConstr(A >= LB_prev, f"LowerBound_r{r}")
    if UB_prev < float('inf'):
        model.addConstr(A <= UB_prev, f"UpperBound_r{r}")

    # 设置求解参数以收集多个可行解
    # model.setParam('PoolSearchMode', 2)  # 寻找多个最优解模式
    # model.setParam('PoolSolutions', 100)  # 最多保存100个解
    # model.setParam('MIPGap', 0)  # 要求找到最优解
    
    # 求解模型
    model.optimize()

    # 检查模型是否成功求解
    if model.status != GRB.OPTIMAL:
        print(f"主问题求解失败,状态码: {model.status}")
        return []

    # 从解池中获取所有可行解
    all_solutions = []
    num_solutions = model.SolCount
    
    print(f"\n--- 分支定界求解完成,共找到 {num_solutions} 个可行解 ---")
    
    for sol_idx in range(num_solutions):
        # 设置获取第 sol_idx 个解
        model.setParam(GRB.Param.SolutionNumber, sol_idx)
        
        # 获取目标值(使用 PoolObjVal 获取解池中的目标值)
        obj_value = model.PoolObjVal
        
        # 提取当前解的x_tb变量
        selected_vars = []
        x_values = {}
        for t in range(1, params.T + 1):
            for b in range(1, params.B + 1):
                x_var = model.getVarByName(f"x_{t}_{b}")
                if x_var and x_var.Xn > 0.5:  # Xn 表示当前解号下的变量值
                    selected_vars.append((t, b))
                    x_values[(t, b)] = 1.0
                else:
                    x_values[(t, b)] = 0.0
        
        # 检查是否与之前的解重复
        is_duplicate = False
        for existing_sol in all_solutions:
            if len(existing_sol.x_vars) == len(selected_vars):
                if set(existing_sol.x_vars) == set(selected_vars):
                    is_duplicate = True
                    break
        
        if is_duplicate or not selected_vars:
            continue
        
        # 验证该解是否满足所有历史约束(公式19的所有Benders割平面)
        satisfies_all_constraints = True
        for i in range(0, r+1):
            for k in range(0, params.K_r[i] + 1):
                u = dual_vars.u[i][k]
                v = dual_vars.v[i][k]
                g = dual_vars.g[i][k]
                h = dual_vars.h[i][k]
                z = dual_vars.z[i][k]
                w = dual_vars.w[i][k]

                # 计算约束右侧表达式
                rhs = 0

                # 求和部分 1: sum_{l∈L} d_bar_l * u_l^{ik}
                for l in range(1, params.L + 1):
                    rhs += params.d_bar_l[l] * u[l]

                # 求和部分 2: sum_{l∈L} d_hat_l * u_l^{ik} * z_l^{ik}
                for l in range(1, params.L + 1):
                    rhs += params.d_hat_l[l] * u[l] * z[l]

                # 求和部分 3-6: 涉及 x_tb 的项
                for t in range(1, params.T + 1):
                    for b in range(1, params.B + 1):
                        rhs += x_values[(t, b)] * params.q_t[t] * g[t]
                        rhs -= x_values[(t, b)] * params.Q_t[t] * h[t]
                        rhs += params.LB_tb[t][b] * x_values[(t, b)] * v[t][b]
                        rhs -= params.UB_tb[t][b] * x_values[(t, b)] * w[t][b]

                # 检查 A >= rhs 是否成立
                if obj_value < rhs - 1e-6:  # 允许小的数值误差
                    satisfies_all_constraints = False
                    print(f"  ✗ 解#{len(all_solutions)+1}: 不满足约束 i={i}, k={k} (obj={obj_value:.4f} < rhs={rhs:.4f})")
                    break
            
            if not satisfies_all_constraints:
                break
        
        if satisfies_all_constraints:
            all_solutions.append(SolutionInfo(obj_value=obj_value, x_vars=selected_vars))
            print(f"  ✓ 解#{len(all_solutions)}: obj={obj_value:.4f}, vars={len(selected_vars)}个, 满足所有约束")

    print(f"主问题共收集到 {len(all_solutions)} 个满足最终约束的可行解")
    
    return all_solutions
