## 项目目录结构

~~~
~\Graduation\
│
├── Al_1/                           # 主算法模块（基础版本）
│   ├── funktions/                  # 算法各阶段函数
│   │   ├── Ini.py                  # 初始化函数（约束参数、函数参数、覆盖矩阵、对偶变量、决策变量）
│   │   ├── MP.py                   # 主问题求解 (Master Problem)
│   │   ├── RP.py                   # 鲁棒子问题求解 (Robust Problem)
│   │   └── SD.py                   # 算法流程总接口（Step_0, Step_1, Step_2）
│   │
│   ├── models/                     # 数据模型定义
│   │   ├── DualVars.py             # 对偶变量数据结构
│   │   ├── Params.py               # 模型参数数据结构
│   │   └── __init__.py
│   │
│   ├── computational_results.csv   # 计算结果CSV文件
│   ├── main.py                     # 程序启动入口
│   ├── readme.md                   # 项目说明文档
│   └── __init__.py
│
├── Al_2/                           # 增强算法模块（带局部搜索和记忆池）
│   ├── funktions/                  # 算法各阶段函数
│   │   ├── Ini.py                  # 初始化函数（约束参数、函数参数、覆盖矩阵、对偶变量、决策变量）
│   │   ├── MP.py                   # 主问题求解 (Master Problem)，含分支定界解收集与约束验证
│   │   ├── RP.py                   # 鲁棒子问题求解 (Robust Problem)，含确定性和随机性两种模式
│   │   ├── GIC.py                  # 需求场景生成与确定性WDP求解 (Generate Initial Scenario)
│   │   ├── LS.py                   # 局部搜索 (Local Search)，含记忆池管理、频率向量构造、邻域约束生成
│   │   └── SD.py                   # 算法流程总接口（Step_0 ~ Step_5）
│   │
│   ├── models/                     # 数据模型定义
│   │   ├── Params.py               # 模型参数数据结构（含SolutionInfo、Params类）
│   │   ├── DualVars.py             # 对偶变量数据结构（u, v, w, g, h, z）
│   │   └── __init__.py
│   │
│   ├── computational_results_2.csv # 计算结果CSV文件
│   ├── main.py                     # 程序启动入口（批量实验控制）
│   ├── readme.md                   # 项目说明文档
│   └── __init__.py
│
├── compare_algorithms.py           # 算法对比脚本（Al_1 vs Al_2）
├── comparison_results.csv          # 对比结果CSV文件
├── test.py                         # 测试文件
└── test1.py                        # 测试文件
~~~

## 代码运行流程

~~~
   开始 → 清空CSV文件（写表头）
         ↓
   实例集1 [40-16-3]
     ├─ Gamma 10% → 运行5个实例 → 计算平均值
     ├─ Gamma 20% → 运行5个实例 → 计算平均值
     ├─ ...
     └─ Gamma 90% → 运行5个实例 → 计算平均值
         ↓
     ✅ 立即写入实例集1的所有结果到CSV
         ↓
   实例集2 [40-16-4]
     ├─ ...（同样流程）
         ↓
     ✅ 立即写入实例集2的所有结果到CSV
         ↓
   结束 → 控制台输出完整汇总表
~~~

## 代码运行流程





## 算法流程说明

### Al_1 核心模块详解

#### 启动类
- **main.py**: 程序入口，控制整体迭代流程和批量实验
  - `run_single_instance()`: 运行单个实例的Benders分解算法
  - `write_results_to_csv()`: 将结果实时写入CSV文件
  - `main()`: 批量实验控制，遍历不同实例集和Gamma比例

#### SD.py - 算法各阶段总接口（3个步骤）
1. **Step_0** - 初始化阶段
   - 调用 `Ini.py` 中的初始化函数
   - 初始化约束参数、函数参数、覆盖矩阵
   - 初始化对偶变量和决策变量
   - 返回: params, dual_vars

2. **Step_1** - 求解主问题（Master Problem）
   - 调用 `MP.py` 中的 `solve_mp_model`
   - 更新下界 LB^r = A^r
   - 提取并存储 x_tb 的解到 selected_vars
   - 返回: A_value, selected_vars

3. **Step_2** - 求解鲁棒子问题（Robust Problem）
   - 调用 `RP.py` 中的 `solve_rp_model`
   - 提取对偶变量到 dual_vars[r]
   - 更新上界 UB^r ← min{UB^{r-1}, Θ^r}
   - 检查终止条件：若 UB^r == LB^r 则算法收敛
   - 返回: model

#### MP.py - 主问题求解（Master Problem）
- **核心功能**: 
  - 构建并求解主问题 W^r(Γ)
  - 添加 Benders 割平面约束
  - 包含承运商选择约束、车道容量约束、中标数量约束
  - 输出：最优目标值 A 和决策变量 x_tb

#### RP.py - 鲁棒子问题求解（Robust Problem）
- **核心函数**:
  - `solve_rp_model()`: z_l是二元决策变量
- **数学模型**: 对偶形式的鲁棒优化问题
  - 目标函数：最大化最坏情况下的成本（对偶形式）
  - 约束类型：对偶可行性、不确定性预算约束 (sum z_l ≤ Γ)、Slack 变量约束
- **输出**: Gurobi模型对象，可提取对偶变量 u, v, w, g, h, z

#### Ini.py - 初始化模块
- **5个初始化函数**:
  - `initialize_constraints_params()`: 初始化 L, T, B, alpha_param, p_t, q_t, Q_t, N_min, N_max
  - `initialize_function_params()`: 初始化 d_bar_l, d_hat_l, LB_tb, UB_tb, c_tb, ce_l, M
  - `initialize_coverage_matrix()`: 随机生成覆盖矩阵 a_tb_l（密度 22%~25%）
  - `initialize_dual_variables()`: 初始化对偶变量空间 dual_vars[0]
  - `initialize_decision_variables()`: 初始化决策变量存储结构

---

### Al_1 算法运行流程（3步迭代）

~~~
┌─────────────────────────────────────────────────────────────────────┐
│ 1. 程序启动                                                          │
│    main.py (main函数)                                                │
│    ├─ 定义实例集: [L, T, B] 的8种组合                                │
│    ├─ 定义Gamma比例: 10% ~ 90% (步长10%)                             │
│    └─ 遍历每个实例集和Gamma比例，对每个组合运行5个随机实例            │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 2. 单个实例执行 (run_single_instance)                                │
│    main.py → run_single_instance(L, T, B, gamma_value, instance_id) │
│    ├─ 调用 Step_0 初始化参数                                         │
│    └─ 进入 while 循环 (r = 1, 2, ..., max_iterations)               │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 3. Step_0 - 初始化阶段                                               │
│    SD.py → Step_0(L, T, B, r)                                       │
│    ├─ 调用 Ini.initialize_constraints_params()                      │
│    │   └─ 生成 L, T, B, alpha_param, p_t, q_t, Q_t, N_min, N_max   │
│    ├─ 调用 Ini.initialize_function_params()                         │
│    │   └─ 生成 d_bar_l, d_hat_l, LB_tb, UB_tb, c_tb, ce_l, M       │
│    ├─ 调用 Ini.initialize_coverage_matrix()                         │
│    │   └─ 生成稀疏覆盖矩阵 a_tb_l (密度22%~25%)                     │
│    ├─ 调用 Ini.initialize_dual_variables()                          │
│    │   └─ 初始化 dual_vars[0] 的对偶变量空间                        │
│    └─ 返回: params, dual_vars                                       │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 4. Step_1 - 求解主问题 (Master Problem)                              │
│    SD.py → Step_1(r, params, dual_vars)                             │
│    ├─ 调用 MP.solve_mp_model(r, params, dual_vars)                  │
│    │   ├─ 构建 Gurobi 主问题模型                                    │
│    │   ├─ 添加 Benders 割平面约束 (基于 dual_vars[0..r-1])          │
│    │   ├─ 添加承运商选择、车道容量、中标数量约束                      │
│    │   ├─ 求解 min A                                                │
│    │   └─ 返回最优目标值 A^r 和决策变量 x_tb                        │
│    ├─ 提取 A_value = A^r                                            │
│    ├─ 存储 x_tb_r[r][t][b] ← x_tb 的解                              │
│    ├─ 收集 selected_vars = [(t,b) | x_tb = 1]                      │
│    ├─ 更新下界: params.LB ← A^r                                     │
│    └─ 返回: A_value, selected_vars                                  │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 5. Step_2 - 求解鲁棒子问题 (Robust Problem)                          │
│    SD.py → Step_2(r, gamma, params, dual_vars)                      │
│    ├─ 从 params.x_tb_r[r] 获取当前解 xtb                            │
│    ├─ 调用 RP.solve_rp_model(r, gamma, xtb, params)                 │
│    │   ├─ 构建 Gurobi 鲁棒优化模型 (对偶形式)                       │
│    │   ├─ 目标: max Σd̄_l u_l + Σd̂_l u_l z_l + ...                │
│    │   ├─ 约束: 对偶可行性、不确定性预算 Σz_l ≤ Γ                   │
│    │   ├─ 求解得到最坏情况目标值 Θ^r                                │
│    │   └─ 返回模型对象 model                                        │
│    ├─ 提取对偶变量: u, v, w, g, h, z                                │
│    ├─ 计算 rp_obj_val = Θ^r (根据对偶公式)                          │
│    ├─ 更新上界: params.UB ← min(params.UB, Θ^r)                    │
│    ├─ 存储对偶变量到 dual_vars[r][l], dual_vars[r][t][b]            │
│    └─ 返回: model                                                   │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 6. 收敛性检查                                                        │
│    main.py → 检查 gap = params.UB - params.LB                       │
│    ├─ 如果 gap < 1e-6: 算法收敛，跳出循环                           │
│    └─ 否则: r ← r + 1，返回步骤4继续迭代                            │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 7. 结果统计与输出                                                    │
│    main.py → 计算5个实例的平均值                                     │
│    ├─ avg_iterations = 总迭代次数 / 5                               │
│    ├─ avg_time = 总耗时 / 5                                         │
│    ├─ avg_LB = 总LB / 5                                             │
│    └─ avg_UB = 总UB / 5                                             │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 8. 写入CSV文件                                                       │
│    main.py → write_results_to_csv(...)                              │
│    └─ computational_results_1.csv                                   │
│        ├─ 表头: |L|-|T|-|B|, Γ(%), #Iter., Time(s), Avg LB, Avg UB │
│        └─ 数据行: 每个实例集的9个Gamma比例结果                       │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 9. 控制台输出汇总表                                                  │
│    main.py → 打印4个表格                                             │
│    ├─ 【平均迭代次数】                                                │
│    ├─ 【平均时间消耗(秒)】                                            │
│    ├─ 【平均下界 LB】                                                 │
│    └─ 【平均上界 UB】                                                 │
└─────────────────────────────────────────────────────────────────────┘
~~~

---

### Al_2 核心模块详解

#### 启动类
- **main.py**: 程序入口，控制整体迭代流程和批量实验
  - `run_single_instance()`: 运行单个实例的Benders分解算法
  - `main()`: 批量实验控制，遍历不同实例集和Gamma比例

#### SD.py - 算法各阶段总接口（6个步骤）
1. **Step_0** - 初始化阶段
   - 调用 `Ini.py` 中的4个初始化函数
   - 初始化约束参数、函数参数、覆盖矩阵
   - 初始化对偶变量和决策变量
   - 返回: params, dual_vars

2. **Step_1** - 生成初始解（仅r=0时执行一次）
   - 调用 `GIC.py` 生成需求场景（按最坏需求降序排列车道）
   - 求解确定性WDP获取初始x_tb
   - 求解补救问题提取对偶变量到dual_vars[0][0]
   - 返回: params, dual_vars

3. **Step_2** - 求解主问题（Master Problem）
   - 调用 `MP.py` 中的 `solve_mp_model`
   - **关键特性**: 开启 Gurobi 解池模式，收集分支定界过程中的所有中间可行解
   - 按目标值降序排列 D^r，最后一个元素为最优解 x^r
   - 更新下界 LB^r = A^r
   - 返回: A_value, selected_vars

4. **Step_3** - 求解最优解 x^r 的补救问题
   - 调用 `RP.py` 中的 `solve_rp_model`
   - 提取对偶变量到dual_vars[r+1][0]
   - 更新上界 UB^r ← min{UB^{r-1}, Θ^r}
   - 检查终止条件：若UB^r == LB^r则算法收敛
   - 返回: model, is_optimal

5. **Step_4** - 局部搜索生成LS解（Algorithm 4）
   - 调用 `LS.py` 中的4个函数：
     - `initialize()`: 初始化记忆池 D_r_mem[r]
     - `update_pool_1()`: 基于D^r更新记忆池（FIFO原则）
     - `local_search()`: 构造频率向量alpha^r，生成NB_LS个LS解
     - `update_pool_2()`: 基于X^{r,LS}再次更新记忆池
   - 返回: None（直接修改params对象）

6. **Step_5** - 求解所有候选解的补救问题
   - 构建候选解集合：{x̄_k} ∈ D^r \ {x^r} ∪ X^{r,LS}
   - 对每个候选解求解RP并提取对偶变量
   - 扩展dual_vars[r+1][1..k_total]
   - 更新上界 UB^r
   - 返回: None（直接修改params和dual_vars对象）

#### MP.py - 主问题求解（Master Problem）
- **核心功能**: 
  - 构建并求解主问题 W^r(Γ)
  - 动态计算 K^r = |D^{r-1}| + |X^{r-1,LS}| - 1
  - 添加 Benders 割平面约束（公式 19）
  - **关键特性**: 开启 Gurobi 的 `PoolSearchMode=2` 收集分支定界过程中的所有中间可行解
  - 利用 `model.PoolObjVal` 和 `x_var.Xn` 提取解池中的所有解
  - 包含 `solve_restricted_mp` 函数：求解带邻域约束（固定变量值）的限制主问题

#### RP.py - 鲁棒子问题求解（Robust Problem）
- **两个求解函数**:
  - `solve_rp_model()`: z_l是二元决策变量（用于Step_3和Step_5）
  - `solve_deterministic_rp_model()`: z_l已固定（用于Step_1）
- **数学模型**: 对偶形式的鲁棒优化问题
- **输出**: Gurobi模型对象，可提取对偶变量u, v, w, g, h, z

#### GIC.py - 需求场景生成与确定性WDP
- **核心函数**:
  - `generate_demand_scenario()`: 按最坏需求降序排列车道，对前Γ个设置z_l=1
  - `solve_deterministic_wdp()`: 求解确定性Winner Determination Problem
  - `solve_recourse_problem()`: 求解补救问题并提取对偶变量

#### LS.py - 局部搜索（Local Search）
- **核心函数**:
  - `initialize()`: 初始化记忆池 D_r_mem[r]（r=0时为空，r>0时复制上一轮）
  - `update_pool_1()`: 基于D^r更新记忆池（FIFO原则）
  - `local_search()`: 
    - 构造频率向量 alpha_freq[t][b][r] = sum_{x∈D_r_mem} x_tb
    - 计算 phi^r = (sum_{t∈T} sum_{b∈B_t} x_{tb}^r) / 2
    - 随机选择 B̃^r 并根据阈值 α/NB_mem 修改变量得到 x̄^r
    - 收集修改前后值不变的 (t,b) 对作为邻域约束 N(x^r)
    - 调用 `MP.solve_restricted_mp` 在受限集上求解得到 LS 最优解
  - `update_pool_2()`: 基于 X^{r,LS} 再次更新记忆池

#### Ini.py - 初始化模块
- **5个初始化函数**:
  - `initialize_constraints_params()`: 初始化 L, T, B, alpha_param, p_t, q_t, Q_t, N_min, N_max
  - `initialize_function_params()`: 初始化 d_bar_l, d_hat_l, LB_tb, UB_tb, c_tb, ce_l, M
  - `initialize_coverage_matrix()`: 随机生成覆盖矩阵 a_tb_l（密度 22%~25%）
  - `initialize_dual_variables()`: 初始化对偶变量空间 dual_vars[0]
  - `initialize_decision_variables()`: 初始化 D_r[0], X_r_LS[0], LB_history, UB_history

---

### Al_2 算法运行流程（6步迭代 + 局部搜索）

~~~
┌─────────────────────────────────────────────────────────────────────┐
│ 1. 程序启动                                                          │
│    main.py (main函数)                                                │
│    ├─ 定义实例集: [L, T, B] 的8种组合                                │
│    ├─ 定义Gamma比例: 10% ~ 90% (步长10%)                             │
│    ├─ 设置参数: NB_mem=10 (记忆池大小), NB_LS=2 (LS解数量)          │
│    └─ 遍历每个实例集和Gamma比例，对每个组合运行5个随机实例            │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 2. 单个实例执行 (run_single_instance)                                │
│    main.py → run_single_instance(gamma_value, params, dual_vars)    │
│    ├─ 调用 Step_0 初始化参数                                         │
│    ├─ 调用 Step_1 生成初始解 (仅r=0时执行一次)                       │
│    └─ 进入 while 循环 (r = 0, 1, 2, ..., max_iterations)            │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 3. Step_0 - 初始化阶段                                               │
│    SD.py → Step_0(L, T, B)                                          │
│    ├─ 调用 Ini.initialize_constraints_params()                      │
│    │   └─ 生成 L, T, B, alpha_param, p_t, q_t, Q_t, N_min, N_max   │
│    ├─ 调用 Ini.initialize_function_params()                         │
│    │   └─ 生成 d_bar_l, d_hat_l, LB_tb, UB_tb, c_tb, ce_l, M       │
│    ├─ 调用 Ini.initialize_coverage_matrix()                         │
│    │   └─ 生成稀疏覆盖矩阵 a_tb_l (密度22%~25%)                     │
│    ├─ 调用 Ini.initialize_dual_variables()                          │
│    │   └─ 初始化 dual_vars[0] 的对偶变量空间                        │
│    ├─ 调用 Ini.initialize_decision_variables()                      │
│    │   └─ 初始化 D_r[0]=∅, X_r_LS[0]=∅, LB_history, UB_history     │
│    └─ 返回: params, dual_vars                                       │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 4. Step_1 - 生成初始解 (仅r=0时执行)                                 │
│    SD.py → Step_1(params, dual_vars, gamma)                         │
│    ├─ 调用 GIC.generate_demand_scenario(params, gamma)              │
│    │   ├─ 按最坏需求 d̄_l + d̂_l 降序排列车道                        │
│    │   ├─ 对前Γ个车道设置 z_l = 1                                   │
│    │   └─ 其余车道设置 z_l = 0                                      │
│    ├─ 调用 GIC.solve_deterministic_wdp(params)                      │
│    │   ├─ 求解确定性 Winner Determination Problem                   │
│    │   ├─ 目标: min Σc_tb x_tb                                      │
│    │   ├─ 约束: 覆盖约束、承运商约束、投标约束                        │
│    │   └─ 返回 selected_vars = [(t,b) | x_tb = 1]                  │
│    ├─ 存储 x_tb_r[0][t][b] ← selected_vars                          │
│    ├─ 调用 GIC.solve_recourse_problem(params, dual_vars)            │
│    │   ├─ 求解补救问题 Q(x^0, Γ)                                    │
│    │   ├─ 提取对偶变量 u, v, w, g, h, z                             │
│    │   └─ 存储到 dual_vars[0][0]                                    │
│    └─ 返回: params, dual_vars                                       │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 5. Step_2 - 求解主问题 (Master Problem)                              │
│    SD.py → Step_2(r, params, dual_vars, gamma)                      │
│    ├─ 调用 MP.solve_mp_model(r, params, dual_vars)                  │
│    │   ├─ 动态计算 K^r = |D^{r-1}| + |X^{r-1,LS}| - 1              │
│    │   ├─ 开启 Gurobi PoolSearchMode=2 (收集所有可行解)             │
│    │   ├─ 添加 Benders 割平面约束 (公式19)                          │
│    │   ├─ 添加边界约束: LB^{r-1} ≤ A ≤ UB^{r-1}                     │
│    │   ├─ 求解并收集分支定界过程中的所有中间解                        │
│    │   └─ 返回 all_solutions (List[SolutionInfo])                   │
│    ├─ 按目标值降序排列: all_solutions.sort(reverse=True)            │
│    ├─ 存储到 D_r[r] ← all_solutions                                 │
│    ├─ 提取最优解 x^r = D_r[r][-1] (最后一个元素，目标值最小)        │
│    ├─ 存储 x_tb_r[r][t][b] ← x^r                                    │
│    ├─ 更新下界: params.LB ← A^r                                     │
│    ├─ 保存历史: LB_history[r] = A^r, UB_history[r] = params.UB     │
│    └─ 返回: A_value, selected_vars                                  │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 6. Step_3 - 求解最优解 x^r 的补救问题                                │
│    SD.py → Step_3(r, gamma, params, dual_vars)                      │
│    ├─ 从 params.x_tb_r[r] 获取当前最优解 x_current = x^r            │
│    ├─ 调用 RP.solve_rp_model(gamma, x_current, params)              │
│    │   ├─ 求解补救问题 Q(x^r, Γ)                                    │
│    │   ├─ 目标: max Σd̄_l u_l + Σd̂_l u_l z_l + ...                │
│    │   └─ 返回最优目标值 Θ^r                                        │
│    ├─ 提取对偶变量: u, v, w, g, h, z                                │
│    ├─ 存储到 dual_vars[r+1][0] (第0对对偶变量)                      │
│    ├─ 更新上界: params.UB ← min(params.UB, Θ^r)                    │
│    ├─ 检查终止条件: 如果 |UB - LB| < 1e-6                           │
│    │   ├─ 是: 返回 is_optimal=True，算法终止                        │
│    │   └─ 否: 返回 is_optimal=False，继续Step_4                     │
│    └─ 返回: model, is_optimal                                       │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 7. Step_4 - 局部搜索生成LS解 (Algorithm 4)                           │
│    SD.py → Step_4(r, gamma, params, dual_vars)                      │
│    │                                                                 │
│    ├─ Algorithm 4 Step 0: 初始化记忆池                               │
│    │   └─ 调用 LS.initialize(params, r)                             │
│    │       └─ 如果 r > 0: D_r_mem[r] ← D_r_mem[r-1].copy()         │
│    │                                                                 │
│    ├─ Algorithm 4 Step 1: 基于D^r更新记忆池                          │
│    │   └─ 调用 LS.update_pool_1(params, r)                          │
│    │       ├─ 获取 D^r 中的所有解                                    │
│    │       ├─ 如果 |D_r_mem| < NB_mem: 直接添加新解                  │
│    │       └─ 否则: FIFO原则，移除最早加入的解，添加新解             │
│    │                                                                 │
│    ├─ Algorithm 4 Step 2: 局部搜索                                   │
│    │   └─ 调用 LS.local_search(params, r, dual_vars)                │
│    │       ├─ 构造频率向量: alpha_freq[t][b] = Σ_{x∈D_r_mem} x_tb  │
│    │       ├─ 计算阈值: phi^r = (Σ x_tb^r) / 2                      │
│    │       ├─ 重复 NB_LS 次:                                        │
│    │       │   ├─ 随机选择投标集合 B̃^r                              │
│    │       │   ├─ 根据 alpha_freq/NB_mem 修改变量得到 x̄^r           │
│    │       │   ├─ 收集不变变量作为邻域约束 N(x^r)                    │
│    │       │   └─ 调用 MP.solve_restricted_mp(N(x^r))               │
│    │       │       └─ 求解限制主问题得到LS最优解                     │
│    │       └─ 存储到 X_r_LS[r] ← [LS解1, LS解2, ...]               │
│    │                                                                 │
│    ├─ Algorithm 4 Step 3: 基于X^{LS}更新记忆池                       │
│    │   └─ 调用 LS.update_pool_2(params, r)                          │
│    │       ├─ 获取 X_r_LS[r] 中的所有LS解                            │
│    │       └─ 按FIFO原则添加到 D_r_mem[r]                            │
│    │                                                                 │
│    └─ 返回: None (直接修改params对象)                                │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 8. Step_5 - 求解所有候选解的补救问题                                 │
│    SD.py → Step_5(r, gamma, params, dual_vars)                      │
│    ├─ 构建候选解集合: {x̄_k} ∈ D^r \ {x^r} ∪ X^{r,LS}              │
│    │   ├─ D^r \ {x^r}: 排除最优解的其他主问题解                      │
│    │   └─ X^{r,LS}: 局部搜索生成的LS解                               │
│    │                                                                 │
│    ├─ 扩展对偶变量空间: dual_vars[r+1]                               │
│    │   └─ 大小扩展至 k_total + 1 (k=0已在Step_3赋值)                │
│    │                                                                 │
│    ├─ 对每个候选解 x̄_k (k=1, ..., k_total):                         │
│    │   ├─ 调用 RP.solve_rp_model(gamma, x̄_k, params)               │
│    │   ├─ 提取对偶变量: u, v, w, g, h, z                            │
│    │   ├─ 存储到 dual_vars[r+1][k]                                  │
│    │   └─ 更新上界: params.UB ← min(params.UB, Θ^k)                │
│    │                                                                 │
│    └─ 返回: None (直接修改params和dual_vars对象)                     │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 9. 迭代控制                                                          │
│    SD.py → r ← r + 1                                                │
│    └─ 返回步骤5 (Step_2) 继续下一轮迭代                             │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 10. 结果统计与输出                                                   │
│    main.py → 计算5个实例的平均值                                     │
│    ├─ avg_iterations = 总迭代次数 / 5                               │
│    ├─ avg_time = 总耗时 / 5                                         │
│    ├─ avg_LB = 总LB / 5                                             │
│    └─ avg_UB = 总UB / 5                                             │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 11. 写入CSV文件                                                      │
│    main.py → write_results_to_csv(...)                              │
│    └─ computational_results_2.csv                                   │
│        ├─ 表头: |L|-|T|-|B|, Γ(%), #Iter., Time(s), Avg LB, Avg UB │
│        └─ 数据行: 每个实例集的9个Gamma比例结果                       │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ 12. 控制台输出汇总表                                                 │
│    main.py → 打印4个表格                                             │
│    ├─ 【平均迭代次数】                                                │
│    ├─ 【平均时间消耗(秒)】                                            │
│    ├─ 【平均下界 LB】                                                 │
│    └─ 【平均上界 UB】                                                 │
└─────────────────────────────────────────────────────────────────────┘
~~~

---


### 数据模型

#### Params.py
- **SolutionInfo 类**: 存储解的信息
  - obj_value: 目标函数值
  - x_vars: 决策变量列表 [(t1,b1), (t2,b2), ...]
  
- **Params 类**: 包含所有模型参数
  - 基础维度: T, L, B
  - 约束参数: alpha_param, z_l, p_t, q_t, Q_t, N_min, N_max
  - 函数参数: d_bar_l, d_hat_l, d_l, LB_tb, UB_tb, c_tb, ce_l, M, a_tb_l
  - 决策变量: A_r, LB, UB, LB_history, UB_history, x_tb_r
  - 解集合: D_r, K_r, NB_mem, D_r_mem, alpha_freq, phi_r, B_tilde_r, NB_LS, X_r_LS, N_x_r

#### DualVars.py
- **DualVars 类**: 存储对偶变量
  - u[r][k][l]: 对应名义需求的对偶变量
  - v[r][k][t][b], w[r][k][t][b]: 对应运输量上下限的对偶变量
  - g[r][k][t], h[r][k][t]: 对应承运商运输量约束的对偶变量
  - z[r][k][l]: 对应不确定性预算的二元变量
