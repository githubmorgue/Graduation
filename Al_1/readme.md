## 项目目录结构

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
├── Al_1/                           # 主算法模块
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
│   ├── main.py                     # 程序启动入口
│   ├── MP.lp                       # Gurobi 导出的主问题模型文件
│   ├── readme.md                   # 项目说明文档
│   └── __init__.py
│
├── test.py                         # 测试文件
└── test1.py                        # 测试文件


## 算法流程说明

### 启动类
- **main.py**: 程序入口，控制整体迭代流程

### SD.py - 算法各阶段总接口
1. **Step_0** - 初始化阶段
   - 调用 `Ini.py` 中的初始化函数
   - 初始化约束参数、函数参数、覆盖矩阵
   - 初始化对偶变量和决策变量

2. **Step_1** - 求解主问题
   - 调用 `MP.py` 中的 `solve_mp_model`
   - 更新下界 LB
   - 提取并存储 x_tb 的解

3. **Step_2** - 求解鲁棒子问题
   - 调用 `RP.py` 中的 `solve_rp_model`
   - 更新上界 UB
   - 提取对偶变量供下一轮迭代使用

4. **Step_3** - 生成约束（待实现）
   - 调用 `GC.py` 生成 Benders 割

### 核心模块
- **MP.py**: Master Problem - 主问题模型
- **RP.py**: Robust Problem - 鲁棒优化子问题
- **Ini.py**: Initialization - 各类参数和变量的初始化
- **Params.py**: 模型参数的数据类定义
- **DualVars.py**: 对偶变量的数据类定义

## 算法特点
- 使用 Benders 分解算法求解鲁棒优化问题
- 支持多轮迭代，逐步收敛
- 动态维护上界（UB）和下界（LB）
- 自动提取和传递对偶变量信息

