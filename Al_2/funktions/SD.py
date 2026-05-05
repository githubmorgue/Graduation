import random
from gurobipy import *

from .MP import solve_mp_model
from .Ini import (
    initialize_constraints_params,
    initialize_function_params,
    initialize_coverage_matrix,
    initialize_dual_variables,
    initialize_decision_variables
)
from .RP import solve_rp_model
from .GIC import generate_demand_scenario, solve_deterministic_wdp, solve_recourse_problem
from ..models.DualVars import DualVars


def Step_0(L, T, B):
    """
    步骤0：初始化 Benders 分解算法所需的问题参数和对偶变量。

    Args:
        L (int): 车道数量
        T (int): 承运商数量
        B (int): 投标数量
        r (int): 当前迭代次数
        gamma (int): 不确定性预算参数 Γ
        NB_mem (int): 记忆池大小，默认为10

    Returns:
        tuple: 包含初始化后的参数字典 (params) 和对偶变量字典 (dual_vars)
    """
    # 1. 初始化约束参数
    params = initialize_constraints_params(L, T, B)
    # 2. 初始化函数参数
    params = initialize_function_params(params)
    # 2.5. 初始化覆盖矩阵
    density = random.uniform(0.22, 0.25)  # 密度在 22% 到 25% 之间
    params = initialize_coverage_matrix(params, density=round(density, 4))
    # 3. 初始化对偶变量
    dual_vars = initialize_dual_variables(params)
    # 4. 初始化决策变量 D_r[0] = X_r_LS[0] = 空集
    params = initialize_decision_variables(params)

    return params, dual_vars


def Step_1(params, dual_vars, gamma):
    """
    步骤1：生成需求场景，求解确定性WDP，获取x_t_b用于后续补救问题求解。
    
    Args:
        r (int): 当前迭代次数
        params (Params): 模型参数
        dual_vars (DualVars): 对偶变量
        gamma (int): 不确定性预算参数 Γ
    
    Returns:
        tuple: (A_value, selected_vars, y_values) - 目标函数值、选中的变量列表、运输量
    """
    # 生成需求场景：按最坏需求降序排列车道，对前Γ个车道设置z_l=1
    params = generate_demand_scenario(params, gamma)
    
    # 求解确定性WDP，获取x_t_b和y_tb
    selected_vars = solve_deterministic_wdp(params)

    # 存储x_t_b到params.x_tb_r[0]
    params.x_tb_r[0] = {}
    for t in range(1, params.T + 1):
        params.x_tb_r[0][t] = {}
        for b in range(1, params.B + 1):
            params.x_tb_r[0][t][b] = 1 if (t, b) in selected_vars else 0

    # 求解补救问题，并提取对偶变量并存储到 dual_vars[0][0]，返回 dual_vars
    dual_vars = solve_recourse_problem(params, dual_vars)

    return params, dual_vars


def Step_2(r, params, dual_vars, gamma):
    """
    步骤2：求解主问题 W^r(Γ)，收集所有可行解到D^r（按目标值降序），更新LB。
    
    算法逻辑（Algorithm 2 Step 2 - 根据图片公式21-22）：
    1. 求解主问题 W^r(Γ)，通过分支定界算法收集所有中间可行解
    2. 将所有可行解按目标值降序存储在 D^r 中
    3. D^r的最后一个元素对应当前最优解x^r（最小化问题，数值最小排最后）
    4. 更新 LB^r = A^r
    5. 转到 Step 3
    
    Args:
        r (int): 当前迭代次数
        params (Params): 模型参数
        dual_vars (DualVars): 对偶变量（包含之前轮次的对偶信息）
        gamma (int): 不确定性预算参数 Γ
    
    Returns:
        tuple: (A_value, selected_vars) - 目标函数值、当前最优解的变量列表
    """
    try:
        print(f"\n{'='*60}")
        print(f"Step_2: 开始执行Algorithm 2 Step 2 - 求解主问题 W^{{{r}}}(Γ)")
        print(f"{'='*60}")
        
        # ========== 步骤1: 求解主问题并收集所有中间可行解 ==========
        print(f"\n--- Step 2.1: 求解主问题 W^{{{r}}}(Γ) 并收集可行解 ---")
        
        # 初始化 D^r = ∅
        params.D_r[r] = []
        
        # K^r 的计算在 solve_mp_model 函数中完成
        # 直接使用 dual_vars（包含之前轮次的对偶变量信息）
        all_solutions = solve_mp_model(r, params, dual_vars)
        
        if not all_solutions:
            print(f"Step_2: 主问题未找到可行解")
            print(f"Step_2: D^r 保持为空")
            return None, []

        print(f"\n--- Step 2.2: 按目标值降序排列并构建 D^r ---")
        
        # 按目标值降序排列（从大到小）
        # 根据图片描述："根据相应的目标值，解按降序排列"
        all_solutions.sort(key=lambda sol: sol.obj_value, reverse=True)
        
        # 存储到 D^r
        params.D_r[r] = all_solutions
        
        print(f"Step_2: D^r 包含 {len(params.D_r[r])} 个解（按目标值降序排列）")
        for idx, sol in enumerate(params.D_r[r]):
            marker = " <-- 最优解x^r" if idx == len(params.D_r[r]) - 1 else ""
            print(f"Step_2:   D^r[{idx}]: obj={sol.obj_value:.4f}, vars={len(sol.x_vars)}个{marker}")
        
        print(f"Step_2: 最优解x^r是D^r的最后一个元素（索引{len(params.D_r[r])-1}），obj={params.D_r[r][-1].obj_value:.4f}")
        
        # ========== 步骤3: 获取最优解x^r并更新相关变量 ==========
        print(f"\n--- Step 2.3: 提取最优解x^r ---")
        
        # D^r的最后一个元素是最优解x^r（因为按降序排列，最小值在最后）
        optimal_solution = params.D_r[r][-1]
        A_value = optimal_solution.obj_value
        selected_vars = optimal_solution.x_vars
        
        # 更新 x_tb_r[r] 为最优解
        params.x_tb_r[r] = {}
        for t in range(1, params.T + 1):
            params.x_tb_r[r][t] = {}
            for b in range(1, params.B + 1):
                params.x_tb_r[r][t][b] = 1 if (t, b) in selected_vars else 0
        
        print(f"Step_2: 最优解x^r包含 {len(selected_vars)} 个选中的(t,b)对")
        
        # ========== 步骤4: 更新下界 LB^r = A^r ==========
        print(f"\n--- Step 2.4: 更新下界 ---")
        
        # 保存当前轮次的LB和UB到历史（用于下一轮的边界约束 LB^{r-1} 和 UB^{r-1}）
        # 根据图片算法2 Step 2: Update LB^r ← A^r
        # 公式(20)中的 LB^{r-1} 和 UB^{r-1} 就是这里保存的历史值
        params.LB_history[r] = A_value  # LB^r = A^r
        params.UB_history[r] = params.UB  # 保留当前的UB
        
        params.LB = A_value
        params.A_r = A_value
        
        print(f"Step_2: LB^r = A^r = {A_value:.4f}")
        print(f"Step_2: 当前 LB = {params.LB:.4f}, UB = {params.UB:.4f}")
        
        print(f"\n{'='*60}")
        print(f"Step_2: 完成！转到 Step 3")
        print(f"{'='*60}\n")
        
        return A_value, selected_vars
    
    except Exception as e:
        print(f"Step_2 第{r}轮发生异常: {str(e)}")
        import traceback
        traceback.print_exc()
        raise


def Step_3(r, gamma, params, dual_vars):
    """
    步骤3：对主问题的最优解x^r求解补救问题，提取对偶变量并更新UB
    
    算法逻辑（Algorithm 2 Step 3）：
    1. 对固定的x^r求解补救问题Q(x^r, Γ)
    2. 提取最优解和对偶变量(u^{(r+1)0}, v^{(r+1)0}, w^{(r+1)0}, g^{(r+1)0}, h^{(r+1)0}, z^{(r+1)0}, ζ^{(r+1)0})
    3. 更新UB^r ← min{UB^{r-1}, Θ^r}，其中Θ^r是补救问题的最优目标值
    4. 如果UB^r == LB^r，则返回(x^r, A^r)作为最优解并终止
    5. 否则，转到Step 4
    
    Returns:
        tuple: (model, is_optimal) - Gurobi模型对象和是否找到最优解的标志
    """
    try:
        print(f"\n{'='*60}")
        print(f"Step_3: 开始执行Algorithm 2 Step 3 - 求解x^r的补救问题")
        print(f"{'='*60}")

        # ========== 步骤1: 获取主问题的最优解x^r ==========
        print(f"\n--- Step 3.1: 获取主问题最优解x^r ---")

        x_current = params.x_tb_r[r]
        
        # 将x_current转换为List[Tuple]格式用于求解RP
        x_current_list = [(t, b) for t in range(1, params.T + 1) 
                          for b in range(1, params.B + 1) 
                          if x_current.get(t, {}).get(b, 0) == 1]
        
        print(f"Step_3: x^r包含 {len(x_current_list)} 个选中的(t,b)对")

        # ========== 步骤2: 求解补救问题Q(x^r, Γ) ==========
        print(f"\n--- Step 3.2: 求解补救问题 Q(x^r, Γ) ---")
        
        # 求解RP
        rp_model = solve_rp_model(gamma, x_current, params)
        
        if rp_model.status != GRB.OPTIMAL:
            print(f"Step_3: 警告 - 补救问题未找到最优解，状态码: {rp_model.status}")
            # 即使失败也要初始化对偶变量空间
            next_r = r + 1
            dual_vars.u[next_r] = [{}]
            dual_vars.v[next_r] = [{}]
            dual_vars.w[next_r] = [{}]
            dual_vars.g[next_r] = [{}]
            dual_vars.h[next_r] = [{}]
            dual_vars.z[next_r] = [{}]
            return rp_model, False
        
        # 获取补救问题的最优目标函数值Θ^r
        theta_r = rp_model.ObjVal
        
        print(f"Step_3: 补救问题Q(x^r, Γ)找到最优解")
        print(f"Step_3: Θ^r = {theta_r:.4f}")
        
        # ========== 步骤3: 初始化 dual_vars[r+1] 并存储第0对对偶变量 ==========
        print(f"\n--- Step 3.3: 提取对偶变量并存储到 dual_vars[r+1][0] ---")
        
        next_r = r + 1
        
        # 创建 dual_vars[next_r]，初始大小为1（只存储k=0）
        dual_vars.u[next_r] = [{}]
        dual_vars.v[next_r] = [{}]
        dual_vars.w[next_r] = [{}]
        dual_vars.g[next_r] = [{}]
        dual_vars.h[next_r] = [{}]
        dual_vars.z[next_r] = [{}]
        
        # 提取对偶变量并存储到 dual_vars[next_r][0]
        # 提取 u_l^{(r+1)0}
        for l in range(1, params.L + 1):
            u_var = rp_model.getVarByName(f"u_{l}")
            if u_var:
                dual_vars.u[next_r][0][l] = u_var.X
            else:
                dual_vars.u[next_r][0][l] = 0.0
        
        # 提取 z_l^{(r+1)0}
        for l in range(1, params.L + 1):
            z_var = rp_model.getVarByName(f"z_{l}")
            if z_var:
                dual_vars.z[next_r][0][l] = z_var.X
            else:
                dual_vars.z[next_r][0][l] = 0.0
        
        # 提取 g_t^{(r+1)0} 和 h_t^{(r+1)0}
        for t in range(1, params.T + 1):
            g_var = rp_model.getVarByName(f"g_{t}")
            h_var = rp_model.getVarByName(f"h_{t}")
            dual_vars.g[next_r][0][t] = g_var.X if g_var else 0.0
            dual_vars.h[next_r][0][t] = h_var.X if h_var else 0.0
        
        # 提取 v_{t,b}^{(r+1)0} 和 w_{t,b}^{(r+1)0}
        for t in range(1, params.T + 1):
            dual_vars.v[next_r][0][t] = {}
            dual_vars.w[next_r][0][t] = {}
            for b in range(1, params.B + 1):
                v_var = rp_model.getVarByName(f"v_{t}_{b}")
                w_var = rp_model.getVarByName(f"w_{t}_{b}")
                dual_vars.v[next_r][0][t][b] = v_var.X if v_var else 0.0
                dual_vars.w[next_r][0][t][b] = w_var.X if w_var else 0.0
        
        print(f"Step_3: 已提取第0对对偶变量到 dual_vars[{next_r}][0]")

        # ========== 步骤4: 更新UB^r ← min{UB^{r-1}, Θ^r} ==========
        print(f"\n--- Step 3.4: 更新上界UB ---")
        
        params.UB = min(params.UB, theta_r)
        
        print(f"Step_3: 当前 LB = {params.LB:.4f}, UB = {params.UB:.4f}")

        # ========== 步骤5: 检查终止条件 ==========
        print(f"\n--- Step 3.5: 检查终止条件 ---")
        
        # 检查是否 UB^r == LB^r
        if abs(params.UB - params.LB) < 1e-6:
            print(f"\n{'='*60}")
            print(f"Step_3: 满足终止条件！UB^r == LB^r = {params.LB:.4f}")
            print(f"Step_3: 找到最优解 (x^r, A^r)")
            print(f"Step_3: 算法终止")
            print(f"{'='*60}\n")
            
            # 返回最优解
            return rp_model, True
        else:
            print(f"Step_3: UB^r ≠ LB^r ({params.UB:.4f} ≠ {params.LB:.4f})")
            print(f"Step_3: 继续执行Step 4")
            print(f"\n{'='*60}")
            print(f"Step_3: 完成！转到 Step 4")
            print(f"{'='*60}\n")
            
            return rp_model, False
    
    except Exception as e:
        print(f"Step_3 第{r}轮发生异常: {str(e)}")
        import traceback
        traceback.print_exc()
        raise


def Step_4(r, gamma, params, dual_vars):
    """
    步骤4：执行Algorithm 4 - 在第r轮生成LS解的完整流程
    
    算法逻辑（Algorithm 4）：
    Step 0: 初始化 - D^{r,mem} = D^{r-1,mem}（r>0时）
    Step 1: Update D^{r,mem} - 基于FIFO原则更新记忆池
    Step 2: Local search - 基于记忆池生成LS解
    Step 3: Update D^{r,mem} - 基于LS解集合再次更新记忆池
    
    Args:
        r: 当前迭代轮次
        gamma: 不确定性预算参数Γ
        params: 模型参数对象
        dual_vars: 对偶变量对象
    
    Returns:
        None (直接修改params对象)
    """
    try:
        print(f"\n{'='*70}")
        print(f"Step_4: 开始执行Algorithm 4 - 第{r}轮LS解生成流程")
        print(f"{'='*70}")
        
        # ========== Step 0: Initialization ==========
        # 如果r > 0，初始化记忆池 D^{r,mem} = D^{r-1,mem}

        from .LS import initialize
        initialize(params, r)
        print(f"Step_4: 已初始化 D_r_mem[{r}] = D_r_mem[{r-1}]")
        
        # ========== Step 1: Update D^{r,mem} ==========
        # 基于D^r更新记忆池（FIFO原则）
        print(f"\n--- Algorithm 4 Step 1: 更新记忆池D^{{r,mem}}（基于D^r）---")
        from .LS import update_pool_1
        update_pool_1(params, r)
        
        # ========== Step 2: Local search ==========
        # 基于记忆池构造频率向量alpha^r，生成LS邻域解
        print(f"\n--- Algorithm 4 Step 2: 局部搜索生成LS解 ---")
        from .LS import local_search
        local_search(params, r, dual_vars)
        
        # ========== Step 3: Update D^{r,mem} ==========
        # 基于X^{LS}再次更新记忆池
        print(f"\n--- Algorithm 4 Step 3: 更新记忆池D^{{r,mem}}（基于X^{{LS}}）---")
        from .LS import update_pool_2
        update_pool_2(params, r)
        
        print(f"\n{'='*70}")
        print(f"Step_4: Algorithm 4执行完成！")
        print(f"Step_4: 第{r}轮生成了 {len(params.X_r_LS.get(r, []))} 个LS解")
        print(f"Step_4: 记忆池D_r_mem当前大小: {len(params.D_r_mem)}")
        print(f"{'='*70}\n")
        
        return
    
    except Exception as e:
        print(f"Step_4 第{r}轮发生异常: {str(e)}")
        import traceback
        traceback.print_exc()
        raise


def Step_5(r, gamma, params, dual_vars):
    """
    步骤5：对所有候选解求解补救问题，提取对偶变量并更新UB
    
    算法逻辑（Algorithm 2 Step 5）：
    1. 构建候选解集合：{x̄_k} ∈ D^r \ {x^r} ∪ X^{r,LS}，其中 k = 1, ..., |D^r| - 1 + |X^{r,LS}|
    2. 对每个候选解 x̄_k 求解补救问题 Q(x̄_k, Γ)
    3. 提取最优解和对偶变量 (u^{(r+1)k}, v^{(r+1)k}, w^{(r+1)k}, g^{(r+1)k}, h^{(r+1)k}, z^{(r+1)k}, ζ^{(r+1)k})
    4. 扩展 dual_vars[r+1] 并赋值 dual_vars[r+1][1..k]（k=0已在Step_3赋值）
    
    Args:
        r: 当前迭代轮次
        gamma: 不确定性预算参数Γ
        params: 模型参数对象
        dual_vars: 对偶变量对象
    
    Returns:
        None (直接修改params和dual_vars对象)
    """
    try:
        print(f"\n{'='*70}")
        print(f"Step_5: 开始执行Algorithm 2 Step 5 - 求解所有候选解的RP")
        print(f"{'='*70}")
        
        # ========== 步骤1: 构建候选解集合 {x̄_k} ==========
        print(f"\n--- Step 5.1: 构建候选解集合 ---")
        
        # 获取当前轮的解 x^r（D^r的最后一个元素，因为按降序排列，最小值在最后）
        if r not in params.D_r or len(params.D_r[r]) == 0:
            raise KeyError(f"第{r}轮的 D_r 为空")
        
        x_r_solution = params.D_r[r][-1]  # 最后一个元素是最优解x^r
        x_current_list = x_r_solution.x_vars
        
        print(f"Step_5: x^r（最优解）包含 {len(x_current_list)} 个(t,b)对，目标值={x_r_solution.obj_value:.4f}")
        
        # 获取 D^r \ {x^r}（排除最后一个元素）
        D_r_without_current = params.D_r[r][:-1]  # 所有除了最后一个的元素
        
        print(f"Step_5: D^r \\ {{x^r}} 包含 {len(D_r_without_current)} 个解")
        for idx, sol in enumerate(D_r_without_current):
            print(f"Step_5:   D^r[{idx}]: obj={sol.obj_value:.4f}, vars={len(sol.x_vars)}个")
        
        # 获取 X^{r,LS}
        X_r_LS = params.X_r_LS.get(r, [])
        print(f"Step_5: X^{{r,LS}} 包含 {len(X_r_LS)} 个解")
        
        # 合并得到候选解集合 {x̄_k}
        # D^r \ {x^r} 中的解已经是 SolutionInfo 格式
        # X^{r,LS} 中的解是 List[Tuple] 格式，需要转换
        
        candidate_solutions_info = []
        
        # 添加 D^r \ {x^r} 中的解
        for sol_info in D_r_without_current:
            candidate_solutions_info.append(sol_info)
        
        # 添加 X^{r,LS} 中的解（需要重新计算目标值或设为None）
        # 注意：X^{r,LS}中的解需要通过求解限制主问题得到，这里先存储变量列表
        # 如果需要目标值，需要重新求解或存储
        for ls_solution_vars in X_r_LS:
            # 对于LS解，我们暂时无法直接获得目标值，设为None或使用一个默认值
            # 在实际算法中，LS解的目标值应该在local_search时就已经计算过了
            # 这里我们需要从params中获取或重新计算
            from Al_2.models.Params import SolutionInfo
            candidate_solutions_info.append(SolutionInfo(obj_value=float('inf'), x_vars=ls_solution_vars))
        
        k_total = len(candidate_solutions_info)
        
        print(f"Step_5: 候选解集合大小 k_total = {k_total}")
        print(f"Step_5: k = 1, ..., |D^r| - 1 + |X^{{r,LS}}| = {len(params.D_r.get(r, []))} - 1 + {len(X_r_LS)} = {k_total}")
        
        if k_total == 0:
            print(f"Step_5: 警告 - 候选解集合为空，跳过Step 5")
            return
        
        # ========== 步骤2: 扩展第r+1轮的对偶变量空间 ==========
        print(f"\n--- Step 5.2: 扩展第{r+1}轮对偶变量空间 ---")
        
        next_r = r + 1
        
        # 检查dual_vars[next_r]是否已在Step_3中创建
        if not (hasattr(dual_vars, 'u') and next_r in dual_vars.u and dual_vars.u[next_r]):
            print(f"Step_5: 警告 - dual_vars[{next_r}]不存在，需要先执行Step_3")
            raise KeyError(f"dual_vars[{next_r}]未初始化，请先执行Step_3")
        
        current_size = len(dual_vars.u[next_r])
        print(f"Step_5: dual_vars[{next_r}]已存在，当前大小={current_size}（索引0..{current_size-1}）")
        print(f"Step_5: 需要扩展至大小={k_total + 1}（新增索引{current_size}..{k_total}）")
        
        # 扩展每个对偶变量列表到大小 k_total + 1
        target_size = k_total + 1
        while len(dual_vars.u[next_r]) < target_size:
            dual_vars.u[next_r].append({})
            dual_vars.v[next_r].append({})
            dual_vars.w[next_r].append({})
            dual_vars.g[next_r].append({})
            dual_vars.h[next_r].append({})
            dual_vars.z[next_r].append({})
        
        # 初始化新增的对偶变量结构（k从current_size开始到k_total）
        for k in range(current_size, target_size):
            for l in range(1, params.L + 1):
                dual_vars.u[next_r][k][l] = 0.0
                dual_vars.z[next_r][k][l] = 0.0
            
            for t in range(1, params.T + 1):
                dual_vars.g[next_r][k][t] = 0.0
                dual_vars.h[next_r][k][t] = 0.0
                dual_vars.v[next_r][k][t] = {}
                dual_vars.w[next_r][k][t] = {}
                for b in range(1, params.B + 1):
                    dual_vars.v[next_r][k][t][b] = 0.0
                    dual_vars.w[next_r][k][t][b] = 0.0
        
        print(f"Step_5: 已为第{next_r}轮扩展对偶变量空间至大小={target_size}")
        print(f"Step_5: 本次将赋值k={current_size}..{k_total}")
        
        # ========== 步骤3: 对每个候选解求解RP并提取对偶变量 ==========
        print(f"\n--- Step 5.3: 求解所有候选解的补救问题 ---")
        
        from .RP import solve_rp_model
        
        for idx, candidate_solution_info in enumerate(candidate_solutions_info):
            k = idx + 1  # k从1开始
            candidate_solution = candidate_solution_info.x_vars
            
            print(f"\n{'-'*60}")
            print(f"Step_5: 处理第 k={k}/{k_total} 个候选解...")
            print(f"Step_5: 候选解包含 {len(candidate_solution)} 个(t,b)对")
            print(f"{'-'*60}")
            
            # 将candidate_solution（List[Tuple]）转换为dict格式
            xtb_candidate = {}
            for t in range(1, params.T + 1):
                xtb_candidate[t] = {}
                for b in range(1, params.B + 1):
                    xtb_candidate[t][b] = 1 if (t, b) in candidate_solution else 0
            
            # 求解RP: Q(x̄_k, Γ)
            rp_model = solve_rp_model(gamma, xtb_candidate, params)
            
            if rp_model.status != GRB.OPTIMAL:
                print(f"Step_5: 警告 - 第k={k}个候选解的RP未找到最优解，状态码: {rp_model.status}")
                continue
            
            # 获取RP目标函数值
            rp_obj_val = rp_model.ObjVal
            
            # 更新UB: UB^r ← min{UB^{r-1}, Θ^r}
            params.UB = min(params.UB, rp_obj_val)
            
            print(f"Step_5: 候选解k={k} - RP目标值: {rp_obj_val:.4f}, 当前UB: {params.UB:.4f}")
            
            # 提取对偶变量并存储到 dual_vars[next_r][k]（k从1开始）
            # 提取 u_l^{(r+1)k}
            for l in range(1, params.L + 1):
                u_var = rp_model.getVarByName(f"u_{l}")
                if u_var:
                    dual_vars.u[next_r][k][l] = u_var.X
                else:
                    dual_vars.u[next_r][k][l] = 0.0
            
            # 提取 z_l^{(r+1)k}
            for l in range(1, params.L + 1):
                z_var = rp_model.getVarByName(f"z_{l}")
                if z_var:
                    dual_vars.z[next_r][k][l] = z_var.X
                else:
                    dual_vars.z[next_r][k][l] = 0.0
            
            # 提取 g_t^{(r+1)k} 和 h_t^{(r+1)k}
            for t in range(1, params.T + 1):
                g_var = rp_model.getVarByName(f"g_{t}")
                h_var = rp_model.getVarByName(f"h_{t}")
                dual_vars.g[next_r][k][t] = g_var.X if g_var else 0.0
                dual_vars.h[next_r][k][t] = h_var.X if h_var else 0.0
            
            # 提取 v_{tb}^{(r+1)k} 和 w_{tb}^{(r+1)k}
            for t in range(1, params.T + 1):
                for b in range(1, params.B + 1):
                    v_var = rp_model.getVarByName(f"v_{t}_{b}")
                    w_var = rp_model.getVarByName(f"w_{t}_{b}")
                    dual_vars.v[next_r][k][t][b] = v_var.X if v_var else 0.0
                    dual_vars.w[next_r][k][t][b] = w_var.X if w_var else 0.0
            
            print(f"Step_5: 已提取第k={k}对对偶变量到 dual_vars[{next_r}][{k}]")
        
        print(f"\n{'='*70}")
        print(f"Step_5: 完成！共处理{k_total}个候选解")
        print(f"Step_5: dual_vars[{next_r}][0]在Step_3已赋值")
        print(f"Step_5: dual_vars[{next_r}][1..{k_total}]在Step_5已赋值")
        print(f"Step_5: 当前 UB = {params.UB:.4f}")
        print(f"{'='*70}\n")
        
        return
    
    except Exception as e:
        print(f"Step_5 第{r}轮发生异常: {str(e)}")
        import traceback
        traceback.print_exc()
        raise
