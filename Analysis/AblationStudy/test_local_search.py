"""
消融实验：验证"基于局部搜索生成捆绑割（Local Search Cuts）"对算法性能的影响

实验设计：
- Baseline: Al_1 (基础约束生成，无局部搜索)
- Variant: Algorithm_Ablation_LocalSearch (在Al_1基础上添加局部搜索割)

核心差异：
Variant在每次迭代中：
1. 求解主问题时收集所有中间可行解（通过Gurobi解池）
2. 基于这些解执行局部搜索生成LS解
3. 对所有中间解和LS解求解子问题，批量生成割平面

注意：此变体不包含初次割和边界约束，仅验证局部搜索割的效果。

对比指标：
1. 迭代次数
2. 运行时间
3. 收敛质量(LB, UB, Gap)
4. 收敛率
5. 生成的割平面总数
"""

import sys
import os
import time
import csv
import random
import numpy as np
from datetime import datetime

# 将项目根目录加入系统路径
project_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# 导入算法1的相关模块（Baseline：基础约束生成）
from Al_1.funktions.SD import Step_0 as Step_0_Al1
from Al_1.funktions.SD import Step_1 as Step_1_Al1
from Al_1.funktions.SD import Step_2 as Step_2_Al1

# 导入Gurobi相关模块
from gurobipy import GRB


def solve_restricted_mp_local(r, params, dual_vars, neighborhood_constraints):
    """
    求解限制主问题 (兼容 Al_1/Al_2 混合数据结构)
    
    在标准主问题基础上，添加邻域约束 x_{tb} = 1
    
    Args:
        r: 当前迭代轮次
        params: 模型参数
        dual_vars: 对偶变量
        neighborhood_constraints: 邻域约束列表 [(t,b), ...]
    
    Returns:
        ls_solution: LS解列表 [(t,b), ...]
    """
    from gurobipy import Model, GRB, quicksum
    
    model = Model("Restricted_MP_Local")
    model.setParam('OutputFlag', 0)
    
    # 创建变量
    A = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name="A")
    x = {}
    for t in range(1, params.T + 1):
        for b in range(1, params.B + 1):
            x[t, b] = model.addVar(vtype=GRB.BINARY, name=f"x_{t}_{b}")
    
    model.setObjective(A, GRB.MINIMIZE)
    
    # 安全提取字典的辅助函数
    def safe_extract(data):
        if isinstance(data, dict):
            return data
        elif isinstance(data, list) and len(data) > 0:
            return data[0] if isinstance(data[0], dict) else {}
        return {}
    
    # 添加 Benders 割 (遍历 0 到 r-1)
    for i in range(0, r):
        u_data = dual_vars.u.get(i, {})
        v_data = dual_vars.v.get(i, {})
        g_data = dual_vars.g.get(i, {})
        h_data = dual_vars.h.get(i, {})
        z_data = dual_vars.z.get(i, {})
        w_data = dual_vars.w.get(i, {})
        
        u_dict = safe_extract(u_data)
        v_dict = safe_extract(v_data)
        g_dict = safe_extract(g_data)
        h_dict = safe_extract(h_data)
        z_dict = safe_extract(z_data)
        w_dict = safe_extract(w_data)
        
        if not u_dict:
            continue
            
        expr = 0
        for l in range(1, params.L + 1):
            u_val = u_dict.get(l, 0.0)
            z_val = z_dict.get(l, 0.0)
            expr += params.d_bar_l[l] * u_val
            expr += params.d_hat_l[l] * u_val * z_val
        
        for t in range(1, params.T + 1):
            g_val = g_dict.get(t, 0.0)
            h_val = h_dict.get(t, 0.0)
            
            v_t = safe_extract(v_dict.get(t, {}))
            w_t = safe_extract(w_dict.get(t, {}))
            
            for b in range(1, params.B + 1):
                v_val = v_t.get(b, 0.0)
                w_val = w_t.get(b, 0.0)
                
                expr += x[t, b] * params.q_t[t] * g_val
                expr -= x[t, b] * params.Q_t[t] * h_val
                expr += params.LB_tb[t][b] * x[t, b] * v_val
                expr -= params.UB_tb[t][b] * x[t, b] * w_val
        
        model.addConstr(A >= expr, name=f"BendersCut_{i}")
    
    # 基本约束
    for t in range(1, params.T + 1):
        model.addConstr(quicksum(x[t, b] for b in range(1, params.B + 1)) <= 1, name=f"OneBid_{t}")
    
    for l in range(1, params.L + 1):
        model.addConstr(quicksum(x[t, b] for t in range(1, params.T + 1) for b in range(1, params.B + 1) 
                                 if params.a_tb_l[t][b][l] == 1) <= 1, name=f"Capacity_{l}")
    
    total_x = quicksum(x[t, b] for t in range(1, params.T + 1) for b in range(1, params.B + 1))
    model.addConstr(total_x >= params.N_min, "MinCount")
    model.addConstr(total_x <= params.N_max, "MaxCount")
    
    # 邻域约束
    for t, b in neighborhood_constraints:
        model.addConstr(x[t, b] == 1, name=f"Neighborhood_{t}_{b}")
        
    model.optimize()
    
    ls_solution = []
    if model.status == GRB.OPTIMAL:
        ls_solution = [(t, b) for t in range(1, params.T + 1) for b in range(1, params.B + 1) if x[t, b].X > 0.5]
    
    model.dispose()
    return ls_solution


def perform_local_search(params, dual_vars, r, gamma_value):
    """
    执行局部搜索生成LS解
    
    算法逻辑（基于论文Algorithm 4和5.2.4章节）：
    1. 基于当前解x^r，随机选择一部分变量保持不变（邻域约束）
    2. 在受限邻域内求解主问题，得到LS解
    3. 返回生成的LS解列表
    
    Args:
        params: 模型参数对象
        dual_vars: 对偶变量对象
        r: 当前迭代轮次
        gamma_value: gamma值
    
    Returns:
        ls_solutions: LS解列表，每个元素为[(t,b), ...]格式
    """
    print(f"\n--- 局部搜索（Local Search）开始 ---")
    
    NB_LS = params.NB_LS if hasattr(params, 'NB_LS') else 2
    current_solution = params.x_tb_r[r]
    
    # 计算当前解中选中变量的总数
    num_selected = sum(1 for t in range(1, params.T + 1) 
                      for b in range(1, params.B + 1) 
                      if current_solution.get(t, {}).get(b, 0) == 1)
    
    # phi^r = num_selected / 2
    phi_r = max(1, int(num_selected / 2))
    
    ls_solutions = []
    
    for i in range(NB_LS):
        print(f"  LS迭代 {i+1}/{NB_LS}")
        
        # 找出所有被选中的(t,b)对
        active_pairs = [(t, b) for t in range(1, params.T + 1) 
                        for b in range(1, params.B + 1) 
                        if current_solution.get(t, {}).get(b, 0) == 1]
        
        # 随机选择phi_r个(t,b)对
        if len(active_pairs) > 0 and phi_r > 0:
            num_to_select = min(phi_r, len(active_pairs))
            B_tilde_subset = random.sample(active_pairs, num_to_select)
        else:
            B_tilde_subset = []
        
        # 构造邻域约束：随机修改前后值保持不变的变量
        neighborhood_constraints = []
        for (t, b) in B_tilde_subset:
            # 随机决定是否保持该变量为1
            if random.random() < 0.5:
                neighborhood_constraints.append((t, b))
        
        print(f"    邻域约束数: {len(neighborhood_constraints)}")
        
        # 在受限邻域上求解主问题
        if neighborhood_constraints:
            # 修改点：使用本地兼容的求解函数
            ls_solution = solve_restricted_mp_local(r, params, dual_vars, neighborhood_constraints)
            if ls_solution:
                ls_solutions.append(ls_solution)
                print(f"    生成LS解: {len(ls_solution)}个变量")
    
    print(f"局部搜索完成，共生成了 {len(ls_solutions)} 个LS解")
    
    return ls_solutions


def run_baseline_al1(L, T, B, gamma_value, instance_id, max_iterations=1000):
    """
    运行算法1（Baseline：基础约束生成，无局部搜索）

    算法流程：
    Step 0: 初始化参数（无初次割）
    Step 1: 求解主问题 -> 更新LB
    Step 2: 求解子问题 -> 更新UB，提取对偶变量
    Step 3: 检查收敛条件

    Args:
        L: 车道数量
        T: 承运商数量
        B: 投标数量
        gamma_value: gamma值
        instance_id: 实例ID
        max_iterations: 最大迭代次数

    Returns:
        tuple: (iterations, elapsed_time, final_LB, final_UB, converged, total_cuts)
    """
    print(f"\n{'='*70}")
    print(f"[Baseline Al_1] 实例{instance_id} - [L={L}, T={T}, B={B}] - Gamma={gamma_value:.1f}")
    print(f"{'='*70}")

    start_time = time.time()

    # Step 0: 初始化参数（无初次割）
    params, dual_vars = Step_0_Al1(L, T, B, 1)
    r = 1
    converged = False
    total_cuts = 0

    while r <= max_iterations:
        # 检查是否超时（1200秒）
        current_time = time.time()
        elapsed = current_time - start_time
        if elapsed >= 1200:
            print(f"达到时间限制(1200秒)，终止迭代")
            break

        # Step 1: 求解主问题 - 更新下界
        A_value, selected_vars = Step_1_Al1(r, params, dual_vars)

        if not selected_vars:
            print("主问题未找到可行解，终止迭代")
            break

        # Step 2: 求解子问题 - 更新上界，提取对偶变量
        model = Step_2_Al1(r, gamma_value, params, dual_vars)

        # 统计割平面数量（每轮生成1个割）
        total_cuts += 1

        # Step 3: 检查收敛条件
        gap = params.UB - params.LB
        print(f"第{r}轮 - Gap: {gap:.6f} (UB={params.UB:.4f}, LB={params.LB:.4f}), 耗时: {elapsed:.2f}秒, 割数: {total_cuts}")

        if gap < 1e-6:
            print(f"算法在第{r}轮收敛")
            converged = True
            break

        r += 1

    end_time = time.time()
    elapsed_time = end_time - start_time

    final_LB = params.LB
    final_UB = params.UB
    final_gap = final_UB - final_LB

    print(f"[Baseline Al_1] 实例{instance_id}完成 - 迭代: {r}, 耗时: {elapsed_time:.2f}秒, "
          f"LB: {final_LB:.4f}, UB: {final_UB:.4f}, Gap: {final_gap:.6f}, "
          f"割数: {total_cuts}, 收敛: {converged}")

    return r, elapsed_time, final_LB, final_UB, converged, total_cuts

def solve_mp_with_pool(r, params, dual_vars):
    """
    求解主问题并收集解池中的所有可行解 (兼容 Al_1 数据结构)

    Args:
        r: 当前迭代轮次
        params: 模型参数对象
        dual_vars: 对偶变量对象 (Al_1 结构: {iteration: {lane: value}})

    Returns:
        tuple: (A_value, selected_vars, all_solutions)
    """
    from gurobipy import Model, GRB, quicksum

    # 创建模型
    model = Model("MP_Model_Pool")
    model.setParam('OutputFlag', 0)

    # 创建变量
    A = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name="A")
    x = {}
    for t in range(1, params.T + 1):
        for b in range(1, params.B + 1):
            x[t, b] = model.addVar(vtype=GRB.BINARY, name=f"x_{t}_{b}")

    # 设置目标函数
    model.setObjective(A, GRB.MINIMIZE)

    # 安全提取字典的辅助函数（兼容列表包裹或纯字典结构）
    def safe_extract(data):
        if isinstance(data, dict):
            return data
        elif isinstance(data, list) and len(data) > 0:
            return data[0] if isinstance(data[0], dict) else {}
        return {}

    for i in range(0, r):
        u_data = dual_vars.u.get(i, {})
        v_data = dual_vars.v.get(i, {})
        g_data = dual_vars.g.get(i, {})
        h_data = dual_vars.h.get(i, {})
        z_data = dual_vars.z.get(i, {})
        w_data = dual_vars.w.get(i, {})
        
        u_dict = safe_extract(u_data)
        v_dict = safe_extract(v_data)
        g_dict = safe_extract(g_data)
        h_dict = safe_extract(h_data)
        z_dict = safe_extract(z_data)
        w_dict = safe_extract(w_data)
        
        if not u_dict: 
            continue
            
        expr = 0
        for l in range(1, params.L + 1):
            u_val = u_dict.get(l, 0.0)
            z_val = z_dict.get(l, 0.0)
            expr += params.d_bar_l[l] * u_val
            expr += params.d_hat_l[l] * u_val * z_val

        for t in range(1, params.T + 1):
            g_val = g_dict.get(t, 0.0)
            h_val = h_dict.get(t, 0.0)
            
            # 处理 v 和 w 的二级结构
            v_t = safe_extract(v_dict.get(t, {}))
            w_t = safe_extract(w_dict.get(t, {}))
            
            for b in range(1, params.B + 1):
                v_val = v_t.get(b, 0.0)
                w_val = w_t.get(b, 0.0)
                
                expr += x[t, b] * params.q_t[t] * g_val
                expr -= x[t, b] * params.Q_t[t] * h_val
                expr += params.LB_tb[t][b] * x[t, b] * v_val
                expr -= params.UB_tb[t][b] * x[t, b] * w_val

        model.addConstr(A >= expr, name=f"Type1_Scenario_{i}")

    # 基本约束
    for t in range(1, params.T + 1):
        model.addConstr(quicksum(x[t, b] for b in range(1, params.B + 1)) <= 1, name=f"Type2_Time_{t}")

    for l in range(1, params.L + 1):
        line_expr = quicksum(x[t, b] for t in range(1, params.T + 1) for b in range(1, params.B + 1)
                            if params.a_tb_l[t][b][l] == 1)
        model.addConstr(line_expr <= 1, name=f"Type3_Line_{l}")

    total_x = quicksum(x[t, b] for t in range(1, params.T + 1) for b in range(1, params.B + 1))
    model.addConstr(total_x >= params.N_min, "MinCount")
    model.addConstr(total_x <= params.N_max, "MaxCount")

    # 解池配置
    model.setParam('PoolSearchMode', 2)
    model.setParam('PoolSolutions', 100)
    model.optimize()

    if model.status != GRB.OPTIMAL:
        print(f"主问题求解失败,状态码: {model.status}")
        return None, [], []

    A_value = model.ObjVal
    selected_vars = [(t, b) for t in range(1, params.T + 1) for b in range(1, params.B + 1)
                     if x[t, b].X > 0.5]

    all_solutions = []
    num_solutions = model.SolCount
    for sol_idx in range(min(num_solutions, 50)):
        model.setParam(GRB.Param.SolutionNumber, sol_idx)
        sol_vars = [(t, b) for t in range(1, params.T + 1) for b in range(1, params.B + 1)
                    if x[t, b].Xn > 0.5]
        if sol_vars and not any(set(s) == set(sol_vars) for s in all_solutions):
            all_solutions.append(sol_vars)

    model.dispose()
    return A_value, selected_vars, all_solutions


def run_variant_with_local_search(L, T, B, gamma_value, instance_id, max_iterations=1000):
    """
    运行增强算法（添加了局部搜索割的Al_1）

    算法流程（Algorithm_Ablation_LocalSearch）：
    Step 0: 初始化参数（无初次割）
    Step 1: 求解主问题，收集所有中间可行解
    Step 2: 执行局部搜索生成LS解
    Step 3: 对所有中间解和LS解求解子问题，批量生成割平面
    Step 4: 检查收敛条件

    注意：此变体不包含初次割和边界约束

    Args:
        L: 车道数量
        T: 承运商数量
        B: 投标数量
        gamma_value: gamma值
        instance_id: 实例ID
        max_iterations: 最大迭代次数

    Returns:
        tuple: (iterations, elapsed_time, final_LB, final_UB, converged, total_cuts)
    """
    print(f"\n{'='*70}")
    print(f"[Variant LS] 实例{instance_id} - [L={L}, T={T}, B={B}] - Gamma={gamma_value:.1f}")
    print(f"{'='*70}")

    start_time = time.time()

    # Step 0: 初始化参数（无初次割）
    from Al_1.funktions.Ini import (
        initialize_constraints_params,
        initialize_function_params,
        initialize_coverage_matrix,
        initialize_dual_variables
    )

    params = initialize_constraints_params(L, T, B)
    params = initialize_function_params(params)
    density = np.random.uniform(0.22, 0.25)
    params = initialize_coverage_matrix(params, density=round(density, 4))
    dual_vars = initialize_dual_variables(params)

    # 初始化决策变量和相关数据结构
    params.x_tb_r = {}
    params.K_r = {0: 0}  # 初始化K_r[0] = 0
    params.NB_LS = 2  # 设置LS解数量
    
    r = 1
    converged = False
    total_cuts = 0

    # ========== 主循环 ==========
    while r <= max_iterations:
        # 检查是否超时（1200秒）
        current_time = time.time()
        elapsed = current_time - start_time
        if elapsed >= 1200:
            print(f"达到时间限制(1200秒)，终止迭代")
            break

        print(f"\n{'#'*60}")
        print(f"# 第{r}轮迭代开始")
        print(f"{'#'*60}")

        # Step 1: 求解主问题，收集所有中间可行解
        print(f"\n--- Step 1: 求解主问题并收集中间可行解 ---")
        A_value, selected_vars, all_solutions = solve_mp_with_pool(r, params, dual_vars)

        if not selected_vars:
            print("主问题未找到可行解，终止迭代")
            break

        # 存储当前最优解
        params.x_tb_r[r] = {}
        for t in range(1, params.T + 1):
            params.x_tb_r[r][t] = {}
            for b in range(1, params.B + 1):
                params.x_tb_r[r][t][b] = 1 if (t, b) in selected_vars else 0

        # 更新LB
        params.LB = A_value
        params.A_r = A_value

        print(f"主问题最优解: {len(selected_vars)}个变量, A={A_value:.4f}")
        print(f"收集到 {len(all_solutions)} 个中间可行解")

        # Step 2: 执行局部搜索生成LS解
        print(f"\n--- Step 2: 执行局部搜索 ---")
        ls_solutions = perform_local_search(params, dual_vars, r, gamma_value)
        print(f"生成 {len(ls_solutions)} 个LS解")

        # Step 3: 对所有中间解和LS解求解子问题，批量生成割平面
        print(f"\n--- Step 3: 批量求解子问题生成割平面 ---")

        # 合并所有需要求解的候选解
        candidate_solutions = all_solutions + ls_solutions

        cuts_generated = 0
        for idx, candidate_solution in enumerate(candidate_solutions):
            print(f"\n  处理候选解 {idx+1}/{len(candidate_solutions)}")

            # 将candidate_solution转换为dict格式
            xtb_candidate = {}
            for t in range(1, params.T + 1):
                xtb_candidate[t] = {}
                for b in range(1, params.B + 1):
                    xtb_candidate[t][b] = 1 if (t, b) in candidate_solution else 0

            # 求解子问题
            from Al_2.funktions.RP import solve_rp_model
            rp_model = solve_rp_model(gamma_value, xtb_candidate, params)

            if rp_model.status == GRB.OPTIMAL:
                # 提取对偶变量
                iteration_idx = r
                k_idx = cuts_generated

                # 初始化对偶变量结构
                if iteration_idx not in dual_vars.u:
                    dual_vars.u[iteration_idx] = []
                    dual_vars.v[iteration_idx] = []
                    dual_vars.w[iteration_idx] = []
                    dual_vars.g[iteration_idx] = []
                    dual_vars.h[iteration_idx] = []
                    dual_vars.z[iteration_idx] = []

                while len(dual_vars.u[iteration_idx]) <= k_idx:
                    dual_vars.u[iteration_idx].append({})
                    dual_vars.v[iteration_idx].append({})
                    dual_vars.w[iteration_idx].append({})
                    dual_vars.g[iteration_idx].append({})
                    dual_vars.h[iteration_idx].append({})
                    dual_vars.z[iteration_idx].append({})

                # 提取并存储对偶变量
                for l in range(1, params.L + 1):
                    u_var = rp_model.getVarByName(f"u_{l}")
                    dual_vars.u[iteration_idx][k_idx][l] = u_var.X if u_var else 0.0

                    z_var = rp_model.getVarByName(f"z_{l}")
                    dual_vars.z[iteration_idx][k_idx][l] = z_var.X if z_var else 0.0

                for t in range(1, params.T + 1):
                    g_var = rp_model.getVarByName(f"g_{t}")
                    h_var = rp_model.getVarByName(f"h_{t}")
                    dual_vars.g[iteration_idx][k_idx][t] = g_var.X if g_var else 0.0
                    dual_vars.h[iteration_idx][k_idx][t] = h_var.X if h_var else 0.0

                    dual_vars.v[iteration_idx][k_idx][t] = {}
                    dual_vars.w[iteration_idx][k_idx][t] = {}
                    for b in range(1, params.B + 1):
                        v_var = rp_model.getVarByName(f"v_{t}_{b}")
                        w_var = rp_model.getVarByName(f"w_{t}_{b}")
                        dual_vars.v[iteration_idx][k_idx][t][b] = v_var.X if v_var else 0.0
                        dual_vars.w[iteration_idx][k_idx][t][b] = w_var.X if w_var else 0.0

                # 更新UB
                rp_obj_val = rp_model.ObjVal
                params.UB = min(params.UB, rp_obj_val)

                cuts_generated += 1
                print(f"    生成割平面 {cuts_generated}, RP目标值={rp_obj_val:.4f}, 当前UB={params.UB:.4f}")

        total_cuts += cuts_generated

        # Step 4: 检查收敛条件
        print(f"\n--- Step 4: 收敛性检查 ---")
        gap = params.UB - params.LB
        print(f"第{r}轮 - Gap: {gap:.6f} (UB={params.UB:.4f}, LB={params.LB:.4f}), "
              f"耗时: {elapsed:.2f}秒, 总割数: {total_cuts}")

        if gap < 1e-6:
            print(f"算法在第{r}轮收敛")
            converged = True
            break

        # 更新K_r为下一轮使用
        params.K_r[r] = cuts_generated - 1 if cuts_generated > 0 else 0

        r += 1

    end_time = time.time()
    elapsed_time = end_time - start_time

    final_LB = params.LB
    final_UB = params.UB
    final_gap = final_UB - final_LB

    print(f"\n[Variant LS] 实例{instance_id}完成 - 迭代: {r}, 耗时: {elapsed_time:.2f}秒, "
          f"LB: {final_LB:.4f}, UB: {final_UB:.4f}, Gap: {final_gap:.6f}, "
          f"总割数: {total_cuts}, 收敛: {converged}")

    return r, elapsed_time, final_LB, final_UB, converged, total_cuts


def run_ablation_experiment(instance_sets, gamma_ratios, num_instances=5):
    """
    运行消融实验，对比Baseline和Variant

    Args:
        instance_sets: 实例集列表 [(L, T, B), ...]
        gamma_ratios: Gamma比例列表
        num_instances: 每个配置运行的实例数

    Returns:
        results: 实验结果字典
    """
    results = {}

    output_file = os.path.join(os.path.dirname(__file__),
                               'results',
                               'ablation_study_local_search.csv')
    os.makedirs(os.path.dirname(output_file), exist_ok=True)

    print("="*100)
    print("开始消融实验：验证'基于局部搜索生成捆绑割（Local Search Cuts）'的效果")
    print("="*100)
    print("\n实验设计:")
    print("  Baseline: Al_1 (基础约束生成，无局部搜索)")
    print("  Variant:  Algorithm_Ablation_LocalSearch (添加局部搜索割)")
    print("  控制变量: 不包含初次割和边界约束")
    print("="*100)

    for L, T, B in instance_sets:
        print(f"\n\n{'#'*100}")
        print(f"# 处理实例集: L={L}, T={T}, B={B}")
        print(f"{'#'*100}")

        if (L, T, B) not in results:
            results[(L, T, B)] = {}

        for gamma_ratio in gamma_ratios:
            gamma_value = int(gamma_ratio * L)

            print(f"\n{'-'*100}")
            print(f"--- 测试 Gamma比例={gamma_ratio*100:.0f}% (Gamma={gamma_value}) ---")
            print(f"{'-'*100}")

            # 初始化结果存储
            baseline_results = {'iterations': [], 'time': [], 'LB': [], 'UB': [], 'gap': [],
                               'converged': [], 'cuts': []}
            variant_results = {'iterations': [], 'time': [], 'LB': [], 'UB': [], 'gap': [],
                              'converged': [], 'cuts': []}

            # 运行多个实例取平均
            for instance_id in range(1, num_instances + 1):
                print(f"\n>>> 实例 {instance_id}/{num_instances}")

                try:
                    # 运行Baseline (Al_1)
                    iter_al1, time_al1, lb_al1, ub_al1, conv_al1, cuts_al1 = run_baseline_al1(
                        L, T, B, gamma_value, instance_id
                    )
                    baseline_results['iterations'].append(iter_al1)
                    baseline_results['time'].append(time_al1)
                    baseline_results['LB'].append(lb_al1)
                    baseline_results['UB'].append(ub_al1)
                    baseline_results['gap'].append(ub_al1 - lb_al1)
                    baseline_results['converged'].append(conv_al1)
                    baseline_results['cuts'].append(cuts_al1)

                    # 运行Variant (with Local Search Cuts)
                    iter_ls, time_ls, lb_ls, ub_ls, conv_ls, cuts_ls = run_variant_with_local_search(
                        L, T, B, gamma_value, instance_id
                    )
                    variant_results['iterations'].append(iter_ls)
                    variant_results['time'].append(time_ls)
                    variant_results['LB'].append(lb_ls)
                    variant_results['UB'].append(ub_ls)
                    variant_results['gap'].append(ub_ls - lb_ls)
                    variant_results['converged'].append(conv_ls)
                    variant_results['cuts'].append(cuts_ls)

                except Exception as e:
                    print(f"实例{instance_id}运行出错: {e}")
                    import traceback
                    traceback.print_exc()
                    continue

            # 计算平均值
            if baseline_results['iterations'] and variant_results['iterations']:
                avg_baseline = {
                    'avg_iterations': np.mean(baseline_results['iterations']),
                    'avg_time': np.mean(baseline_results['time']),
                    'avg_LB': np.mean(baseline_results['LB']),
                    'avg_UB': np.mean(baseline_results['UB']),
                    'avg_gap': np.mean(baseline_results['gap']),
                    'convergence_rate': sum(baseline_results['converged']) / len(baseline_results['converged']),
                    'avg_cuts': np.mean(baseline_results['cuts'])
                }

                avg_variant = {
                    'avg_iterations': np.mean(variant_results['iterations']),
                    'avg_time': np.mean(variant_results['time']),
                    'avg_LB': np.mean(variant_results['LB']),
                    'avg_UB': np.mean(variant_results['UB']),
                    'avg_gap': np.mean(variant_results['gap']),
                    'convergence_rate': sum(variant_results['converged']) / len(variant_results['converged']),
                    'avg_cuts': np.mean(variant_results['cuts'])
                }

                # 计算比值
                iter_ratio = avg_baseline['avg_iterations'] / avg_variant['avg_iterations'] if avg_variant['avg_iterations'] > 0 else 0
                time_ratio = avg_baseline['avg_time'] / avg_variant['avg_time'] if avg_variant['avg_time'] > 0 else 0

                results[(L, T, B)][gamma_ratio] = {
                    'baseline': avg_baseline,
                    'variant': avg_variant,
                    'iter_ratio': iter_ratio,
                    'time_ratio': time_ratio
                }

                # 实时写入CSV
                write_results_to_csv(output_file, results, instance_sets, gamma_ratios)
            else:
                print(f"警告: Gamma={gamma_ratio*100:.0f}% 没有成功的数据点")

    # 打印最终汇总表格
    print_final_summary_table(results, instance_sets, gamma_ratios)

    print(f"\n\n{'='*100}")
    print(f"实验结果已保存到: {output_file}")
    print(f"{'='*100}")
    return results


def print_final_summary_table(results, instance_sets, gamma_ratios):
    """
    打印最终汇总表格（学术论文格式）
    """
    print("\n\n" + "="*120)
    print("消融实验结果汇总")
    print("="*120)

    # 打印表头
    print(f"{'|L|-|T|-|B|':<12} {'Γ(%)':<6} {'Baseline (Al_1)':<25} {'Variant (Local Search)':<25} {'Ratio (Baseline/Variant)':<30}")
    print(f"{'':<12} {'':<6} {'# iter.':<12} {'Time (s)':<12} {'# iter.':<12} {'Time (s)':<12} {'# iter.':<12} {'Time (s)':<12}")
    print("-"*120)

    # 打印数据
    for L, T, B in instance_sets:
        instance_label = f"{L}-{T}-{B}"

        for idx, ratio in enumerate(gamma_ratios):
            if ratio not in results.get((L, T, B), {}):
                continue

            gamma_percent = int(ratio * 100)
            baseline = results[(L, T, B)][ratio]['baseline']
            variant = results[(L, T, B)][ratio]['variant']
            iter_ratio = results[(L, T, B)][ratio]['iter_ratio']
            time_ratio = results[(L, T, B)][ratio]['time_ratio']

            # 第一个gamma比例显示实例集标签
            if idx == 0:
                print(f"{instance_label:<12} {gamma_percent:<6} "
                      f"{baseline['avg_iterations']:<12.1f} {baseline['avg_time']:<12.0f} "
                      f"{variant['avg_iterations']:<12.1f} {variant['avg_time']:<12.0f} "
                      f"{iter_ratio:<12.2f} {time_ratio:<12.2f}")
            else:
                print(f"{'':<12} {gamma_percent:<6} "
                      f"{baseline['avg_iterations']:<12.1f} {baseline['avg_time']:<12.0f} "
                      f"{variant['avg_iterations']:<12.1f} {variant['avg_time']:<12.0f} "
                      f"{iter_ratio:<12.2f} {time_ratio:<12.2f}")

        # 实例集之间添加空行
        print()


def write_results_to_csv(output_file, results, instance_sets, gamma_ratios):
    """
    将消融实验结果写入CSV文件（学术论文格式）
    """
    with open(output_file, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)

        # 写入表头
        writer.writerow([
            '|L|-|T|-|B|',
            'Gamma (%)',
            'Baseline - # iter.',
            'Baseline - Time (s)',
            'Variant - # iter.',
            'Variant - Time (s)',
            'Ratio - # iter.',
            'Ratio - Time (s)'
        ])

        # 写入数据
        for L, T, B in instance_sets:
            instance_label = f"{L}-{T}-{B}"

            for idx, ratio in enumerate(gamma_ratios):
                if ratio not in results.get((L, T, B), {}):
                    continue

                gamma_percent = int(ratio * 100)
                baseline = results[(L, T, B)][ratio]['baseline']
                variant = results[(L, T, B)][ratio]['variant']
                iter_ratio = results[(L, T, B)][ratio]['iter_ratio']
                time_ratio = results[(L, T, B)][ratio]['time_ratio']

                writer.writerow([
                    instance_label if idx == 0 else '',
                    gamma_percent,
                    f"{baseline['avg_iterations']:.1f}",
                    f"{baseline['avg_time']:.0f}",
                    f"{variant['avg_iterations']:.1f}",
                    f"{variant['avg_time']:.0f}",
                    f"{iter_ratio:.2f}",
                    f"{time_ratio:.2f}"
                ])

            # 实例集之间添加空行
            writer.writerow([])


def main():
    """
    主函数：执行消融实验
    """
    # 定义测试实例集（从小到大，逐步增加规模）
    instance_sets = [
        [10, 4, 10],
        # [20, 8, 10],
        # [40, 16, 10],
        # [80, 32, 10],
    ]

    # Gamma比例（覆盖低、中、高三种情况）
    gamma_ratios = [0.1, 0.3, 0.5, 0.7, 0.9]

    # 每个配置运行5个实例取平均
    num_instances = 5

    # 运行消融实验
    results = run_ablation_experiment(instance_sets, gamma_ratios, num_instances)

    print("\n\n" + "="*100)
    print("消融实验完成！")
    print("="*100)
    print("\n实验总结:")
    print("  本实验验证了'基于局部搜索生成捆绑割（Local Search Cuts）'对算法性能的单独影响")
    print("  通过对比Baseline和Variant，可以量化局部搜索割带来的加速效果")
    print("="*100)


if __name__ == "__main__":
    main()
