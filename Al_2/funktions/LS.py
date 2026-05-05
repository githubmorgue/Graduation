import random
import numpy as np
from typing import Dict, List, Tuple
from gurobipy import Model, GRB, quicksum

from Al_2.funktions.MP import solve_restricted_mp
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
    基于记忆池 D_r_mem 构造频率向量 alpha^r，并执行本地搜索生成LS解
    
    算法逻辑（Algorithm 4 Step 2）：
    - 对于所有 t ∈ T, b ∈ B_t，计算 alpha^r_b = sum_{x ∈ D^{r,mem}} x_{tb}
    - 设置 phi^r = (sum_{t∈T} sum_{b∈B_t} x_{tb}^r) / 2
    - 执行 NB_LS 次迭代，每次：
      * 从当前解 x^r 中随机选择 phi^r 个 (t,b) 对（其中 x_{tb}^r=1）
      * 根据阈值 alpha^r_{tb} / NB_mem 决定 x̄^r_{tb} 的值
      * 构造邻域 N(x^r) 并求解限制主问题得到 x̄^{r*}
      * 将 x̄^{r*} 添加到 X^{LS}
    
    Args:
        params: 模型参数对象
        r: 当前迭代轮次
        dual_vars: 对偶变量对象
    
    Returns:
        None (直接修改 params.X_r_LS, params.alpha_freq, params.phi_r, params.B_tilde_r, params.N_x_r)
    """
    # 从params中提取所需参数
    T = params.T
    B = params.B
    NB_mem = params.NB_mem
    NB_LS = params.NB_LS
    D_r_mem = params.D_r_mem.get(r, [])
    
    # ========== 步骤1: 基于D_r_mem构造频率向量alpha^r ==========
    # alpha_freq[t][b][r] = sum_{x in D_r_mem} x_{tb}
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

    # ========== 步骤2: 计算并存储phi^r ==========
    # 根据算法4: phi^r = (sum_{t∈T} sum_{b∈B_t} x_{tb}^r) / 2
    x_tb_current = params.x_tb_r[r]

    # 计算 sum_{t∈T} sum_{b∈B_t} x_{tb}^r
    sum_xtb = 0
    for t in range(1, params.T + 1):
        for b in range(1, params.B + 1):
            if x_tb_current.get(t, {}).get(b, 0) == 1:
                sum_xtb += 1

    # phi^r = sum_xtb / 2
    phi_value = sum_xtb / 2
    phi_r = int(phi_value)
    params.phi_r[r] = phi_r

    print(f"LS: 已计算phi^{r} = {phi_r} (基于sum x_{{tb}}^{r} = {sum_xtb})")

    # ========== 步骤3: 本地搜索（Local Search）==========
    # 初始化 X^{r,LS} = ∅
    params.X_r_LS[r] = []
    
    # 初始化 i = 0
    i = 0
    
    # while i < NB_LS do
    while i < NB_LS:
        print(f"\n--- LS迭代 {i+1}/{NB_LS} ---")
        
        # N(x^r) = ∅ （邻域约束集合）
        x_bar = {}

        # x̄^r ← alpha^r (复制alpha向量作为初始值)
        for t in range(1, T + 1):
            x_bar[t] = {}
            for b in range(1, B + 1):
                x_bar[t][b] = alpha.get(t, {}).get(b, 0)
        
        # B̃^r = 随机选择 phi^r 个 (t,b) 子集，满足 x_{tb}^r = 1
        # 首先找出所有满足 x_{tb}^r = 1 的 (t,b) 对
        active_pairs = []
        for t in range(1, T + 1):
            for b in range(1, B + 1):
                if x_tb_current.get(t, {}).get(b, 0) == 1:
                    active_pairs.append((t, b))
        
        # 随机选择 phi^r 个 (t,b) 对
        if len(active_pairs) > 0 and phi_r > 0:
            num_to_select = min(phi_r, len(active_pairs))
            B_tilde_subset = random.sample(active_pairs, num_to_select)
        else:
            B_tilde_subset = []
        
        # 保存 B^r 到 params.B_tilde_r
        if r not in params.B_tilde_r:
            params.B_tilde_r[r] = []
        params.B_tilde_r[r].append(B_tilde_subset)
        
        print(f"LS: 随机选择了 {len(B_tilde_subset)} 个 (t,b) 对到 B^r（phi^r={phi_r}）")
        
        # for (t,b) ∈ B̃^r do
        for (t, b) in B_tilde_subset:
            # ξ ← random[0,1]
            xi = random.uniform(0, 1)
            
            # if ξ < alpha^r_{tb} / NB_mem then x̄^r_{tb} = 1
            # else x̄^r_{tb} = 0
            alpha_tb = alpha.get(t, {}).get(b, 0)
            threshold = alpha_tb / NB_mem if NB_mem > 0 else 0
            
            if xi < threshold:
                x_bar[t][b] = 1
            else:
                x_bar[t][b] = 0
        
        # N(x^r) = {x : ∀(t,b) ∈ B̃^r, x_{tb} = x̄^r_{tb}}
        # 收集 B̃^r 中随机修改前后值保持不变的 (t,b) 对
        neighborhood_constraints = []
        for (t, b) in B_tilde_subset:
            x_original = x_tb_current.get(t, {}).get(b, 0)
            x_modified = x_bar.get(t, {}).get(b, 0)
            if x_original == x_modified:
                neighborhood_constraints.append((t, b))
        
        # 保存邻域约束 N(x^r) 到 params.N_x_r
        if r not in params.N_x_r:
            params.N_x_r[r] = []
        params.N_x_r[r].append(neighborhood_constraints)
        
        print(f"LS: 邻域约束N(x^r)包含 {len(neighborhood_constraints)} 个(t,b)对（修改前后值不变）")
        
        # 在受限集 N(x^r) 上求解 W^r_{rob}(Γ')，得到最优解 x̄^{r*}
        # solve_restricted_mp 返回的是最优解的 (t,b) 列表
        ls_optimal_solution = solve_restricted_mp(r, params, dual_vars, neighborhood_constraints)
        
        # X^{LS} = X^{LS} ∪ {x̄^{r*}}
        params.X_r_LS[r].append(ls_optimal_solution)
        
        print(f"LS: 第{i+1}次迭代得到LS解，包含 {len(ls_optimal_solution)} 个(t,b)对")
        print(f"LS: X^{{LS}} 当前包含 {len(params.X_r_LS[r])} 个解")
        
        # i ← i + 1
        i += 1
    
    print(f"LS: 本地搜索完成，共生成了 {len(params.X_r_LS[r])} 个LS解")
    print(f"LS: B_tilde_r[{r}] 包含 {len(params.B_tilde_r.get(r, []))} 个子集")
    print(f"LS: N_x_r[{r}] 包含 {len(params.N_x_r.get(r, []))} 个邻域约束")
    
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
        selected_ls_solutions = random.sample(current_ls_solutions, params.NB_mem)
        
        # 清空并替换整个记忆池
        params.D_r_mem[r].clear()
        params.D_r_mem[r].extend([sol.copy() for sol in selected_ls_solutions])
        
        print(f"LS: 记忆池已完全替换，新大小={len(params.D_r_mem[r])}")
    
    print(f"LS: Step 3完成，D_r_mem[{r}]更新后大小={len(params.D_r_mem[r])}")

    return
