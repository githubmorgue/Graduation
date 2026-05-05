import random
import numpy as np
from typing import Dict, List, Tuple
from gurobipy import Model, GRB, quicksum

from Al_2.models import Params, DualVars

def initialize(params: Params, r: int):
    """
    初始化记忆池 D_r_mem
    
    算法逻辑：
    - 当 r = 0 时：初始化 D_r_mem[0] 为空列表（或预设大小的空池）
    - 当 r > 0 时：将上一轮的记忆池状态复制到当前轮
      D_r_mem[r] = D_r_mem[r-1]
    
    Args:
        params: 模型参数对象，包含 D_r_mem 记忆池
        r: 当前迭代轮次（r >= 0）
    
    Returns:
        None (直接修改 params 对象)
    
    Note:
        此函数应在每次迭代开始时调用，确保 D_r_mem[r] 已存在
    """
    if r == 0:
        # 第一轮迭代：初始化空记忆池
        params.D_r_mem[r] = []
        print(f"LS: 初始化D_r_mem[{r}]为空列表")
    else:
        # 后续迭代：将上一轮的记忆池状态复制到当前轮（深拷贝）
        if r - 1 in params.D_r_mem:
            params.D_r_mem[r] = [sol.copy() for sol in params.D_r_mem[r - 1]]
            print(f"LS: 从D_r_mem[{r-1}]复制到D_r_mem[{r}]，大小={len(params.D_r_mem[r])}")
        else:
            # 如果上一轮不存在，初始化为空列表
            params.D_r_mem[r] = []
            print(f"LS: D_r_mem[{r-1}]不存在，初始化D_r_mem[{r}]为空列表")

    return

def update_pool_1(params: Params, r: int):
    """
    更新记忆池 D_r_mem
    
    算法逻辑（基于FIFO原则）：
    - 如果 |D^r| < NB_mem：用最新的 D^r 解替代 D^{r,mem} 中最早的 |D^r| 个解（最差的）
    - 如果 |D^r| >= NB_mem：用 D^r 中最新的 NB_mem 个解完全取代 D^{r,mem}
    
    注意：这里将 D^r 视为包含多个候选解的集合，|D^r| 表示解的数量
    
    Args:
        params: 模型参数对象
        r: 当前迭代轮次
    """
    # 获取当前轮的解集合 D^r（现在是SolutionInfo列表）
    current_solutions_info = params.D_r[r]
    num_current_solutions = len(current_solutions_info)
    
    print(f"LS: 开始更新D_r_mem[{r}]，当前D_r[{r}]包含{num_current_solutions}个解, NB_mem={params.NB_mem}")
    
    # 提取变量列表（从SolutionInfo中提取x_vars）
    current_solutions = [sol.x_vars for sol in current_solutions_info]
    
    if num_current_solutions < params.NB_mem:
        # 情况1: |D^r| < NB_mem
        # 用最新的 |D^r| 个解替代 D^{r,mem} 中最早的 |D^r| 个解（队首，即最旧的/最差的）
        print(f"LS: |D^r|({num_current_solutions}) < NB_mem({params.NB_mem})")
        print(f"LS: 移除D_r_mem[{r}]中最早的{num_current_solutions}个旧解（最差的）")
        
        # 移除最早的 |D^r| 个解（队首，即最旧的/最差的）
        for i in range(num_current_solutions):
            if params.D_r_mem[r]:
                removed = params.D_r_mem[r].pop(0)
                print(f"LS: 移除第{i+1}个旧解（长度={len(removed)}）")
        
        # 将新的 D^r 中的所有解添加到队尾（按顺序，最新在后）
        for solution in current_solutions:
            params.D_r_mem[r].append(solution.copy())
            print(f"LS: 添加新解到记忆池（长度={len(solution)}）")
        
    else:
        # 情况2: |D^r| >= NB_mem
        print(f"LS: |D^r|({num_current_solutions}) >= NB_mem({params.NB_mem})")
        print(f"LS: 从D_r[{r}]中提取最新的{params.NB_mem}个解")
        
        # 提取最新的 NB_mem 个解（列表末尾的NB_mem个元素，即最新的）
        latest_solutions = current_solutions[-params.NB_mem:]
        
        # 清空并替换整个记忆池
        params.D_r_mem[r].clear()
        params.D_r_mem[r].extend([sol.copy() for sol in latest_solutions])
        
        print(f"LS: 记忆池已完全替换，新大小={len(params.D_r_mem[r])}")
    
    print(f"LS: D_r_mem[{r}]更新完成，当前记忆池大小={len(params.D_r_mem[r])}")


def local_search(params: Params, r: int, dual_vars):
    """
    基于记忆池 D_r_mem 构造频率向量 alpha_param^r
    
    算法逻辑：
    - 对于所有 t ∈ T, b ∈ B_t，计算 alpha_b^r = sum_{x ∈ D^{r,mem}} x_tb
    - alpha_param^r 表示在迭代 r 之前，遇到的最佳 NB_mem 个第一阶段解中，投标 (t,b) 中标的次数
    - 如果 alpha_param[t][b][r] 很大，说明这个投标组合是个"好主意"
    
    Args:
        params: 模型参数对象
        r: 当前迭代轮次
        dual_vars: 对偶变量对象
    
    Returns:
        None (直接修改 params.alpha_freq)
    """
    # 从params中提取所需参数
    T = params.T
    B = params.B
    D_r_mem = params.D_r_mem.get(r, [])  # 获取当前轮的记忆池
    
    # ========== 步骤1: 基于D_r_mem构造频率向量alpha^r ==========
    # alpha_freq[t][b][r] = sum_{x in D_r_mem} x_tb
    # 表示在D_r_mem中，投标(t,b)被选中的总次数
    alpha = {}
    for t in range(1, T + 1):
        alpha[t] = {}
        for b in range(1, B + 1):
            # 统计D_r_mem中所有解里，(t,b)出现的次数
            count = 0
            for solution in D_r_mem:
                # solution是一个List[Tuple[int, int]]，例如 [(1,2), (3,4), ...]
                if (t, b) in solution:
                    count += 1
            alpha[t][b] = count
    
    # 保存alpha到params的第r轮
    if r not in params.alpha_freq:
        params.alpha_freq[r] = {}
    for t in range(1, T + 1):
        if t not in params.alpha_freq[r]:
            params.alpha_freq[r][t] = {}
        for b in range(1, B + 1):
            params.alpha_freq[r][t][b] = alpha.get(t, {}).get(b, 0)
    
    print(f"LS: 已构造alpha向量（基于{len(D_r_mem)}个历史解）")
    if T > 0 and B > 0:
        print(f"LS: alpha示例 - alpha_param[1][1][{r}]={alpha.get(1, {}).get(1, 0)}, alpha_param[1][2][{r}]={alpha.get(1, {}).get(2, 0)}")

    # ========== 步骤2: 计算并存储phi^r[r] ==========
    # phi^r[r] = |D_r[r]| / 2，其中|D_r[r]|表示第r轮解集合的大小
    # 需要先获取第r轮的解集合D_r[r]
    if r in params.D_r:
        D_r_current_info = params.D_r[r]  # D_r[r]现在是SolutionInfo列表
        phi_value = len(D_r_current_info) // 2  # 使用整数除法
        params.phi_r[r] = phi_value
        print(f"LS: 已计算phi^{r} = {phi_value} (基于|D_r[{r}]| = {len(D_r_current_info)})")
    else:
        print(f"LS: 警告 - D_r[{r}]不存在，无法计算phi^{r}")
        params.phi_r[r] = 0

    i = 0

    # 初始化B_tilde_r[r]为空列表
    if r not in params.B_tilde_r:
        params.B_tilde_r[r] = []
    
    # 获取第r轮的解，找到所有x_tb^r=1的(t,b)对
    if r in params.x_tb_r:
        x_tb_current = params.x_tb_r[r]  # x_tb_r[r][t][b]
        
        # 收集所有x_tb^r=1的(t,b)对
        selected_pairs = []
        for t in range(1, params.T + 1):
            for b in range(1, params.B + 1):
                if x_tb_current.get(t, {}).get(b, 0) == 1:
                    selected_pairs.append((t, b))
        
        print(f"LS: 找到{len(selected_pairs)}个x_tb^r=1的候选对")
        print(f"LS: phi^{r}={params.phi_r[r]}, 需要从中随机选择{params.phi_r[r]}个")
        
        # 检查是否有足够的候选对
        if len(selected_pairs) < params.phi_r[r]:
            print(f"LS: 警告 - 候选对数量({len(selected_pairs)}) < phi^{r}({params.phi_r[r]})，使用全部候选对")
            phi_actual = len(selected_pairs)
        else:
            phi_actual = params.phi_r[r]
        
        # 在while循环中生成NB_LS个随机子集和对应的LS解
        while i < params.NB_LS:
            # 从selected_pairs中随机选择phi_r个(t,b)对
            B_tilde_subset = random.sample(selected_pairs, phi_actual)
            
            # 添加到B_tilde_r[r]列表中
            params.B_tilde_r[r].append(B_tilde_subset)
            
            print(f"LS: 第{i+1}个LS子集已生成，包含{len(B_tilde_subset)}个随机选择的(t,b)对")
            
            # ========== 根据B_tilde_subset生成邻域约束x̄^r ==========
            # 对于每个(t,b) ∈ B̃^r，生成随机数ξ决定x̄_{tb}^r的值
            x_bar = {}  # 存储当前邻域约束x̄^r
            for t in range(1, params.T + 1):
                x_bar[t] = {}
                for b in range(1, params.B + 1):
                    x_bar[t][b] = 0  # 默认设为0
            
            # 处理B_tilde_subset中的每个(t,b)对
            for t, b in B_tilde_subset:
                # 生成[0,1]区间的随机数
                xi = random.random()
                
                # 计算阈值 \alpha_{tb}^r / NB^{mem}
                alpha_tb_r = params.alpha_freq.get(r, {}).get(t, {}).get(b, 0)
                threshold = alpha_tb_r / params.NB_mem if params.NB_mem > 0 else 0
                
                # 根据算法伪代码逻辑赋值
                if xi < threshold:
                    x_bar[t][b] = 1
                    print(f"LS: (t={t},b={b}) -> ξ={xi:.3f} < {threshold:.3f}, 设置x̄_{{{t},{b}}}^r=1")
                else:
                    x_bar[t][b] = 0
                    print(f"LS: (t={t},b={b}) -> ξ={xi:.3f} >= {threshold:.3f}, 设置x̄_{{{t},{b}}}^r=0")
            
            # ========== 求解限制主问题在邻域N(x^r)中的最优解 ==========
            # N(x^r) = {x : ∀(t,b) ∈ B̃^r, x_{tb} = x̄_{tb}^r}
            # 这里N(x^r)是邻域定义，不需要单独存储，直接用于求解限制主问题
            print(f"LS: 正在求解第{i+1}个限制主问题（在邻域N(x^r)中，固定{len(B_tilde_subset)}个变量）...")
            
            # 调用限制主问题求解函数，传入邻域约束x̄^r、B̃^r和对偶变量
            ls_solution = solve_restricted_mp(r, params, dual_vars, x_bar, B_tilde_subset)
            
            # 初始化X_r_LS[r]如果不存在
            if r not in params.X_r_LS:
                params.X_r_LS[r] = []
            
            # 添加LS最优解x̄^{r*}到X_r_LS[r]
            params.X_r_LS[r].append(ls_solution)
            
            print(f"LS: 第{i+1}个LS最优解x̄^{{r*}}已生成，包含{len(ls_solution)}个(t,b)对")
            print(f"LS: 最优解x̄^{{r*}} = {ls_solution}")
            
            i += 1
    else:
        print(f"LS: 警告 - x_tb_r[{r}]不存在，无法生成B_tilde_r")
        # 初始化X_r_LS[r]如果不存在
        if r not in params.X_r_LS:
            params.X_r_LS[r] = []
        # 添加空解以保证LS解数量
        while i < params.NB_LS:
            params.X_r_LS[r].append([])
            i += 1

    return

def update_pool_2(params: Params, r: int):
    """
    更新记忆池 D_r_mem（基于LS解集合）
    
    算法逻辑（Step 3 - 基于FIFO原则）：
    - 如果 |X^{LS}| < NB^{mem}：用所有的LS解替代 D^{r,mem} 中最早的 |X^{LS}| 个解（最差的）
    - 如果 |X^{LS}| >= NB^{mem}：从 X^{LS} 中随机选择 NB^{mem} 个解完全取代 D^{r,mem}
    
    Args:
        params: 模型参数对象
        r: 当前迭代轮次
    """
    # 获取当前轮的LS解集合 X^{LS}
    if r not in params.X_r_LS:
        print(f"LS: 警告 - X_r_LS[{r}]不存在，无法更新记忆池")
        return
    
    current_ls_solutions = params.X_r_LS[r]
    num_ls_solutions = len(current_ls_solutions)
    
    print(f"LS: 开始Step 3更新D_r_mem[{r}]，当前X^{{LS}}包含{num_ls_solutions}个LS解, NB_mem={params.NB_mem}")
    
    if num_ls_solutions < params.NB_mem:
        # 情况1: |X^{LS}| < NB^{mem}
        # 用所有的 |X^{LS}| 个LS解替代 D^{r,mem} 中最早的 |X^{LS}| 个解（最旧的/最差的）
        print(f"LS: |X^{{LS}}|({num_ls_solutions}) < NB_mem({params.NB_mem})")
        print(f"LS: 移除D_r_mem[{r}]中最早的{num_ls_solutions}个旧解（最差的）")
        
        # 移除最早的 |X^{LS}| 个解（队首，即最旧的/最差的）
        for i in range(num_ls_solutions):
            if params.D_r_mem[r]:
                removed = params.D_r_mem[r].pop(0)
                print(f"LS: 移除第{i+1}个旧解（长度={len(removed)}）")
        
        # 将所有LS解添加到队尾
        for solution in current_ls_solutions:
            params.D_r_mem[r].append(solution.copy())
            print(f"LS: 添加LS解到记忆池（长度={len(solution)}）")
        
    else:
        # 情况2: |X^{LS}| >= NB^{mem}
        # 从 X^{LS} 中随机选择 NB^{mem} 个解完全取代 D^{r,mem}
        print(f"LS: |X^{{LS}}|({num_ls_solutions}) >= NB_mem({params.NB_mem})")
        print(f"LS: 从X^{{LS}}中随机选择{params.NB_mem}个解")
        
        # 随机选择 NB^{mem} 个LS解
        import random
        selected_ls_solutions = random.sample(current_ls_solutions, params.NB_mem)
        
        # 清空并替换整个记忆池
        params.D_r_mem[r].clear()
        params.D_r_mem[r].extend([sol.copy() for sol in selected_ls_solutions])
        
        print(f"LS: 记忆池已完全替换，新大小={len(params.D_r_mem[r])}")
    
    print(f"LS: Step 3完成，D_r_mem[{r}]更新后大小={len(params.D_r_mem[r])}")

    return


def solve_restricted_mp(r: int, params: Params, dual_vars, x_bar: Dict[int, Dict[int, int]],
                        B_tilde_subset: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """
    求解限制主问题 W^r(Γ) 在邻域 N(x^r) 中的最优解 x̄^{r*}

    算法逻辑：
    - 邻域定义：N(x^r) = {x : ∀(t,b) ∈ B̃^r, x_{tb} = x̄_{tb}^r}
    - 对于 (t,b) ∈ B^r，固定 x_{tb} = x_{tb}^r（这是邻域约束）
    - 其他变量自由优化
    - 求解得到限制主问题的最优解 x̄^{r*}

    Args:
        r: 当前迭代轮次
        params: 模型参数对象
        dual_vars: 对偶变量对象
        x_bar: 邻域约束 x̄^r，x_bar[t][b] = 0 或 1（仅对B̃^r中的(t,b)有效）
        B_tilde_subset: 需要固定的(t,b)对列表 B^r

    Returns:
        ls_solution: 限制主问题的最优解 x̄^{r*}，格式为 [(t1,b1), (t2,b2), ...]
    """
    # 创建模型
    model = Model("Restricted_MP_Model")
    model.setParam('OutputFlag', 0)  # 关闭Gurobi输出

    # 创建变量
    # A >= 0
    A = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name="A")

    # x_tb 是二元变量
    x = {}
    for t in range(1, params.T + 1):
        for b in range(1, params.B + 1):
            x[t, b] = model.addVar(vtype=GRB.BINARY, name=f"x_{t}_{b}")

    # 设置目标函数
    model.setObjective(A, GRB.MINIMIZE)

    # 添加约束

    # 约束类型 1 - Benders割平面约束
    # 对于每一轮 i = 0 ... r，遍历该轮的所有对偶变量 k = 0 ... K^i
    constraint_idx = 0
    
    for i in range(0, r+1):
        # 确保 K_r[i] 已计算
        if i not in params.K_r:
            if i == 0:
                params.K_r[i] = 0
            else:
                D_size = len(params.D_r.get(i - 1, []))
                X_LS_size = len(params.X_r_LS.get(i - 1, []))
                params.K_r[i] = D_size + X_LS_size - 1
        
        for k in range(0, params.K_r[i] + 1):
            u = dual_vars.u[i][k]
            v = dual_vars.v[i][k]
            g = dual_vars.g[i][k]
            h = dual_vars.h[i][k]
            zl = dual_vars.z[i][k]
            w = dual_vars.w[i][k]

            # 构建右侧表达式 (RHS)
            expr = 0

            # 求和部分 1: sum_{l∈L} d_bar_l * u_l^{ik}
            for l in range(1, params.L + 1):
                expr += params.d_bar_l[l] * u[l]

            # 求和部分 2: sum_{l∈L} d_hat_l * u_l^{ik} * z_l^{ik}
            for l in range(1, params.L + 1):
                expr += params.d_hat_l[l] * u[l] * zl[l]

            # 求和部分 3: sum_{t∈T} sum_{b∈B_t} x_tb * q_t * g_t^{ik}
            for t in range(1, params.T + 1):
                for b in range(1, params.B + 1):
                    expr += x[t, b] * params.q_t[t] * g[t]

            # 求和部分 4: - sum_{t∈T} sum_{b∈B_t} x_tb * Q_t * h_t^{ik}
            for t in range(1, params.T + 1):
                for b in range(1, params.B + 1):
                    expr -= x[t, b] * params.Q_t[t] * h[t]

            # 求和部分 5: + sum_{t∈T} sum_{b∈B_t} LV_tb * x_tb * v_tb^{ik}
            for t in range(1, params.T + 1):
                for b in range(1, params.B + 1):
                    expr += params.LB_tb[t][b] * x[t, b] * v[t][b]

            # 求和部分 6: - sum_{t∈T} sum_{b∈B_t} UV_tb * x_tb * w_tb^{ik}
            for t in range(1, params.T + 1):
                for b in range(1, params.B + 1):
                    expr -= params.UB_tb[t][b] * x[t, b] * w[t][b]

            # 添加约束: A >= expr
            model.addConstr(A >= expr, name=f"Type1_i{i}_k{k}")
            constraint_idx += 1

    # 约束类型 2: sum_{b in B_t} x_tb <= 1, for t in T
    for t in range(1, params.T + 1):
        model.addConstr(quicksum(x[t, b] for b in range(1, params.B + 1)) <= 1, name=f"Type2_Time_{t}")

    # 约束类型 3: sum_{t in T} sum_{b in B_t} a_{tb}^l x_tb <= 1, for l in L
    for l in range(1, params.L + 1):
        line_expr = 0
        for t in range(1, params.T + 1):
            for b in range(1, params.B + 1):
                if params.a_tb_l.get(t, {}).get(b, {}).get(l, 0) == 1:
                    line_expr += x[t, b]
        model.addConstr(line_expr <= 1, name=f"Type3_Line_{l}")

    # 约束类型 4: N_min <= sum_{t in T} sum_{b in B_t} x_tb <= N_max
    total_x = quicksum(x[t, b] for t in range(1, params.T + 1) for b in range(1, params.B + 1))
    model.addConstr(total_x >= params.N_min, "MinCount")
    model.addConstr(total_x <= params.N_max, "MaxCount")

    # 边界约束
    model.addConstr(A >= params.LB, "LowerBound")
    model.addConstr(A <= params.UB, "UpperBound")

    # 邻域约束：N(x^r) = {x : ∀(t,b) ∈ B^r, x_{tb} = x̄_{tb}^r}
    # 对于B̃^r中的每个(t,b)对，添加等式约束 x_{tb} = x̄_{tb}^r
    # 这限制了求解空间，使得只有满足邻域定义的解才被考虑
    for t, b in B_tilde_subset:
        fixed_value = x_bar.get(t, {}).get(b, 0)
        model.addConstr(x[t, b] == fixed_value, name=f"Neighborhood_x_{t}_{b}")
        print(f"LS: 邻域约束 - 固定 x_{t}_{b} = {fixed_value}")

    # 优化模型
    model.optimize()

    # 提取最优解
    ls_solution = []
    if model.status == GRB.OPTIMAL:
        for t in range(1, params.T + 1):
            for b in range(1, params.B + 1):
                if x[t, b].X > 0.5:  # 如果变量值为1
                    ls_solution.append((t, b))

        obj_value = model.objVal
        print(f"LS: 限制主问题求解成功，目标值={obj_value:.4f}，解包含{len(ls_solution)}个(t,b)对")
    else:
        print(f"LS: 警告 - 限制主问题未找到最优解，状态码={model.status}")

    model.dispose()
    return ls_solution
