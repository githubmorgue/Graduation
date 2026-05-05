## 项目目录结构

~~~
~\Graduation\
├── .idea/                          # PyCharm IDE 配置文件
│   ├── inspectionProfiles/
│   ├── .gitignore
│   ├── Graduation.iml
│   ├── material_theme_project_new.xml
│   ├── misc.xml
│   ├── modules.xml
│   └── workspace.xml
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

## 算法流程说明

### 启动类
- **main.py**: 程序入口，控制整体迭代流程和批量实验
  - `run_single_instance()`: 运行单个实例的Benders分解算法
  - `main()`: 批量实验控制，遍历不同实例集和Gamma比例

### Al_2 核心模块详解

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
   - 通过分支定界收集所有可行解到D^r
   - **关键功能**: 验证每个解是否满足所有历史Benders割平面约束
   - 按目标值降序排列D^r，最后一个元素为最优解x^r
   - 更新下界 LB^r = A^r
   - 返回: A_value, selected_vars

4. **Step_3** - 求解最优解x^r的补救问题
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
  - 动态计算K^r = |D^{r-1}| + |X^{r-1,LS}| - 1
  - 添加Benders割平面约束（公式19）
  - **关键特性**: 收集分支定界过程中的所有可行解，并验证其满足最终约束
  - 按目标值降序返回所有可行解（最后一个是最优解x^r）

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
    - 计算phi^r = |D_r[r]| / 2
    - 生成NB_LS个随机子集B_tilde_r[r]
    - 根据阈值α/NB_mem设置邻域约束x̄^r
    - 求解限制主问题得到LS最优解
  - `update_pool_2()`: 基于X^{r,LS}再次更新记忆池
  - `solve_restricted_mp()`: 求解限制主问题（在邻域N(x^r)中）

#### Ini.py - 初始化模块
- **5个初始化函数**:
  - `initialize_constraints_params()`: 初始化L, T, B, alpha_param, p_t, q_t, Q_t, N_min, N_max
  - `initialize_function_params()`: 初始化d_bar_l, d_hat_l, LB_tb, UB_tb, c_tb, ce_l, M
  - `initialize_coverage_matrix()`: 随机生成覆盖矩阵a_tb_l（密度22%~25%）
  - `initialize_dual_variables()`: 初始化对偶变量空间dual_vars[0]
  - `initialize_decision_variables()`: 初始化D_r[0], X_r_LS[0], LB_history, UB_history

### 数据模型

#### Params.py
- **SolutionInfo类**: 存储解的信息
  - obj_value: 目标函数值
  - x_vars: 决策变量列表 [(t1,b1), (t2,b2), ...]
  
- **Params类**: 包含所有模型参数
  - 基础维度: T, L, B
  - 约束参数: alpha_param, z_l, p_t, q_t, Q_t, N_min, N_max
  - 函数参数: d_bar_l, d_hat_l, d_l, LB_tb, UB_tb, c_tb, ce_l, M, a_tb_l
  - 决策变量: A_r, LB, UB, LB_history, UB_history, x_tb_r
  - 解集合: D_r, K_r, NB_mem, D_r_mem, alpha_freq, phi_r, B_tilde_r, NB_LS, X_r_LS, N_x_r

#### DualVars.py
- **DualVars类**: 存储对偶变量
  - u[r][k][l]: 对应名义需求的对偶变量
  - v[r][k][t][b], w[r][k][t][b]: 对应运输量上下限的对偶变量
  - g[r][k][t], h[r][k][t]: 对应承运商运输量约束的对偶变量
  - z[r][k][l]: 对应不确定性预算的二元变量

## 算法特点
- 使用 **Benders分解算法** 求解鲁棒优化问题
- 支持 **多轮迭代**，逐步收敛（UB和LB逼近）
- **动态维护** 上界（UB）和下界（LB）
- **自动提取和传递** 对偶变量信息
- **局部搜索机制**: 基于记忆池和频率向量生成高质量邻域解
- **约束验证**: 确保收集的可行解满足所有历史Benders割平面

工作流程：

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

## 算法伪代码（Algorithm 2）

