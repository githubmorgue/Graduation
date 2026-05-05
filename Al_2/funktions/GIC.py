from .RP import solve_deterministic_rp_model
from ..models.Params import Params
from ..models.DualVars import DualVars
from gurobipy import Model, GRB, quicksum


def generate_demand_scenario(params: Params, gamma: int) -> Params:
    """
    生成需求场景：按最坏需求降序排列车道，对前Γ个车道设置z_l=1，并更新实际需求d_l
    
    算法逻辑：
    1. 计算每条车道的最坏需求：d_bar_l + d_hat_l
    2. 按最坏需求降序排列车道集合L
    3. 对前Gamma个车道：z_l = 1
    4. 对其余车道：z_l = 0
    5. 更新实际需求：d_l = d_bar_l + z_l * d_hat_l
    
    参数:
        params: Params 对象，包含 d_bar_l (名义需求) 和 d_hat_l (最大偏差)
        gamma: 不确定性预算参数 Γ，控制需要设置z_l=1的车道数量
    
    返回:
        更新了 z_l 和 d_l 的 Params 对象
    """
    L = params.L
    
    # 1. 计算每条车道的最坏需求并存储为 (lane_id, worst_demand) 的列表
    lane_worst_demands = []
    for l in range(1, L + 1):
        worst_demand = params.d_bar_l[l] + params.d_hat_l[l]
        lane_worst_demands.append((l, worst_demand))
    
    # 2. 按最坏需求降序排列
    lane_worst_demands.sort(key=lambda x: x[1], reverse=True)
    
    # 3. 提取有序的车道ID列表
    sorted_lanes = [lane_id for lane_id, _ in lane_worst_demands]
    
    # 4. 确定前Gamma个车道
    gamma_effective = gamma
    top_gamma_lanes = set(sorted_lanes[:gamma_effective])
    
    # 5. 设置z_l的值并更新实际需求 d_l
    for l in range(1, L + 1):
        if l in top_gamma_lanes:
            # 前Gamma个车道：z_l = 1
            params.z_l[l] = 1
        else:
            # 其余车道：z_l = 0
            params.z_l[l] = 0
        
        # 更新实际需求：d_l = d_bar_l + z_l * d_hat_l
        params.d_l[l] = params.d_bar_l[l] + params.z_l[l] * params.d_hat_l[l]
    
    print(f"GFC: 已设置z_l - 前{gamma_effective}个车道z_l=1，其余z_l=0")
    print(f"GFC: 最坏需求最高的{gamma_effective}个车道: {sorted_lanes[:gamma_effective]}")
    print(f"GFC: 已更新实际需求 d_l = d_bar_l + z_l * d_hat_l")
    
    return params

def solve_deterministic_wdp(params: Params):
    """
    求解确定性 Winner Determination Problem (WDP)
    
    数学模型：
    min  sum_{t∈T} sum_{b∈B_t} (1+p_t)*c_tb*y_tb + sum_{l∈L} ce_l*e_l
    
    s.t.
    (1) sum_{t∈T} sum_{b∈B_t} a_tbl*y_tb + e_l >= d_l,  ∀l∈L
    (2) LB_tb*x_tb <= y_tb <= UB_tb*x_tb,  ∀t∈T, b∈B_t
    (3) sum_{b∈B_t} x_tb <= 1,  ∀t∈T
    (4) sum_{t∈T} sum_{b∈B_t} a_tbl*x_tb <= 1,  ∀l∈L
    (5) N_min <= sum_{t∈T} sum_{b∈B_t} x_tb <= N_max
    (6) q_t*sum_{b∈B_t} x_tb <= sum_{b∈B_t} y_tb <= Q_t*sum_{b∈B_t} x_tb,  ∀t∈T
    (7) x_tb ∈ {0,1}, y_tb >= 0, e_l >= 0, t∈T, b∈B_t, l∈L
    
    参数:
        params: Params 对象，包含已设置的 z_l 和计算好的 d_l
        r: 当前迭代轮次
        dual_vars: 对偶变量对象（暂未使用，预留接口）
    
    返回:
        tuple: (A_value, selected_vars, y_values)
            - A_value: 目标函数值
            - selected_vars: 选中的(t,b)对列表 [(t1,b1), (t2,b2), ...]
            - y_values: y_tb 的值，Dict[int, Dict[int, float]]
    """
    try:
        print(f"D-WDP: 开始求解确定性WDP")
        
        # 2. 创建模型
        model = Model("Deterministic_WDP_Model")
        model.Params.OutputFlag = 0  # 关闭Gurobi输出
        
        # 3. 创建变量
        x = {}  # 二元变量：是否选择投标(t,b)
        y = {}  # 连续变量：投标(t,b)的运输量
        e = {}  # 连续变量：车道l的外包量
        
        for t in range(1, params.T + 1):
            for b in range(1, params.B + 1):
                x[t, b] = model.addVar(vtype=GRB.BINARY, name=f"x_{t}_{b}")
                y[t, b] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"y_{t}_{b}")
        
        for l in range(1, params.L + 1):
            e[l] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"e_{l}")
        
        # 4. 设置目标函数
        # min sum_{t,b} (1+p_t)*c_tb*y_tb + sum_l ce_l*e_l
        obj_expr = (
            quicksum((1 + params.p_t[t]) * params.c_tb[t][b] * y[t, b] for t in range(1, params.T + 1) for b in range(1, params.B + 1)) +
            quicksum(params.ce_l[l] * e[l] for l in range(1, params.L + 1))
        )
        model.setObjective(obj_expr, GRB.MINIMIZE)
        
        # 5. 添加约束
        
        # 约束(1): 需求满足约束
        # sum_{t,b} a_tbl*y_tb + e_l >= d_l, ∀l∈L
        for l in range(1, params.L + 1):
            demand_expr = (
                quicksum(params.a_tb_l[t][b][l] * y[t, b] for t in range(1, params.T + 1) for b in range(1, params.B + 1)) +
                e[l]
            )
            model.addConstr(demand_expr >= params.d_l[l], name=f"Demand_Lane_{l}")
        
        # 约束(2): 运输量上下限约束
        # LB_tb*x_tb <= y_tb <= UB_tb*x_tb, ∀t∈T, b∈B_t
        for t in range(1, params.T + 1):
            for b in range(1, params.B + 1):
                model.addConstr(y[t, b] >= params.LB_tb[t][b] * x[t, b], name=f"LB_y_{t}_{b}")
                model.addConstr(y[t, b] <= params.UB_tb[t][b] * x[t, b], name=f"UB_y_{t}_{b}")
        
        # 约束(3): 每个承运商最多选择一个投标
        # sum_{b∈B_t} x_tb <= 1, ∀t∈T
        for t in range(1, params.T + 1):
            model.addConstr(quicksum(x[t, b] for b in range(1, params.B + 1)) <= 1, name=f"OneBid_Carrier_{t}")
        
        # 约束(4): 每条车道的容量约束
        # sum_{t,b} a_tbl*x_tb <= 1, ∀l∈L
        for l in range(1, params.L + 1):
            capacity_expr = quicksum(params.a_tb_l[t][b][l] * x[t, b] for t in range(1, params.T + 1) for b in range(1, params.B + 1))
            model.addConstr(capacity_expr <= 1, name=f"Capacity_Lane_{l}")
        
        # 约束(5): 中标数量约束
        # N_min <= sum_{t,b} x_tb <= N_max
        total_x = quicksum(x[t, b] for t in range(1, params.T + 1) for b in range(1, params.B + 1))
        model.addConstr(total_x >= params.N_min, "MinCount")
        model.addConstr(total_x <= params.N_max, "MaxCount")
        
        # 约束(6): 承运商运输量约束
        # q_t*sum_b x_tb <= sum_b y_tb <= Q_t*sum_b x_tb, ∀t∈T
        for t in range(1, params.T + 1):
            total_x_t = quicksum(x[t, b] for b in range(1, params.B + 1))
            total_y_t = quicksum(y[t, b] for b in range(1, params.B + 1))
            model.addConstr(total_y_t >= params.q_t[t] * total_x_t, name=f"MinVolume_Carrier_{t}")
            model.addConstr(total_y_t <= params.Q_t[t] * total_x_t, name=f"MaxVolume_Carrier_{t}")
        
        # 6. 优化模型
        model.optimize()
        
        if model.status != GRB.OPTIMAL:
            print(f"D-WDP: 警告 - 未找到最优解，状态码: {model.status}")
            return None, [], {}
        
        # 7. 提取解
        selected_vars = []

        for t in range(1, params.T + 1):
            for b in range(1, params.B + 1):
                x_val = x[t, b].X

                if x_val == 1:
                    selected_vars.append((t, b))

        print(f"D-WDP: 找到最优解, 选中{len(selected_vars)}个投标")

        return selected_vars
        
    except Exception as e:
        print(f"D-WDP: 求解失败 - {str(e)}")
        import traceback
        traceback.print_exc()
        return None, [], {}


def solve_recourse_problem(params, dual_vars):
    """
    求解补救问题（Recourse Problem），并提取对偶变量用于生成割平面。

    该函数基于给定的x_t_b求解鲁棒优化子问题，提取对偶变量并存储到
    dual_vars[next_r][pair_idx]中，用于在主问题中生成Benders割平面。

    数学模型（对偶形式）：
    max  sum_{l∈L} d_l*u_l + sum_{l∈L} d_hat_l*s_l
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
        params (Params): 模型参数对象
        dual_vars (DualVars): 对偶变量对象
        gamma (int): 不确定性预算参数 Γ，由外部传入

    返回:
        dual_vars: 对偶变量对象
               如果求解失败，返回 None
    """
    try:
        from .RP import solve_rp_model
        
        xtb = params.x_tb_r[0]
        z_l = params.z_l

        # 调用RP模型求解
        rp_model = solve_deterministic_rp_model(xtb, params, z_l)

        # ========== 提取对偶变量并存储到 dual_vars[0][0] ==========

        # 初始化数据结构 - 检查键是否存在且列表是否为空

        dual_vars.u[0] = [{}]  # 第0轮，第0对变量
        dual_vars.z[0] = [{}]
        dual_vars.g[0] = [{}]
        dual_vars.h[0] = [{}]
        dual_vars.v[0] = [{}]
        dual_vars.w[0] = [{}]

        # 提取 u_l
        for l in range(1, params.L + 1):
            u_var = rp_model.getVarByName(f"u_{l}")
            dual_vars.u[0][0][l] = u_var.X if u_var else 0.0

        # 提取 z_l
        for l in range(1, params.L + 1):
            z_var = rp_model.getVarByName(f"z_{l}")
            dual_vars.z[0][0][l] = z_var.X if z_var else 0.0

        # 提取 g_t 和 h_t
        for t in range(1, params.T + 1):
            g_var = rp_model.getVarByName(f"g_{t}")
            h_var = rp_model.getVarByName(f"h_{t}")
            dual_vars.g[0][0][t] = g_var.X if g_var else 0.0
            dual_vars.h[0][0][t] = h_var.X if h_var else 0.0

        # 提取 v_{t,b} 和 w_{t,b}
        for t in range(1, params.T + 1):
            for b in range(1, params.B + 1):
                # 初始化嵌套字典
                if t not in dual_vars.v[0][0]:
                    dual_vars.v[0][0][t] = {}
                if t not in dual_vars.w[0][0]:
                    dual_vars.w[0][0][t] = {}
                    
                v_var = rp_model.getVarByName(f"v_{t}_{b}")
                w_var = rp_model.getVarByName(f"w_{t}_{b}")
                dual_vars.v[0][0][t][b] = v_var.X if v_var else 0.0
                dual_vars.w[0][0][t][b] = w_var.X if w_var else 0.0

        print(f"RP: 已提取对偶变量到 dual_vars[0][0]")
        print(f"{'-' * 60}\n")

        return dual_vars

    except Exception as e:
        print(f"RP: 求解补救问题失败 - {str(e)}")
        import traceback
        traceback.print_exc()
        return None
