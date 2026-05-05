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
from ..models.DualVars import DualVars


def Step_0(L, T, B, r):
    """
    步骤0：初始化 Benders 分解算法所需的问题参数和对偶变量。

    Args:
        L (int): 车道数量
        T (int): 承运商数量
        B (int): 投标数量
        r (int): 当前迭代次数

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
    dual_vars = initialize_dual_variables(params, r)
    # 4. 初始化决策变量 x_tb_r（第1轮设为0）
    # params = initialize_decision_variables(params)

    return params, dual_vars

def Step_1(r, params, dual_vars):
    """
    步骤1：求解主问题。
    Args:
        r (int): 当前迭代次数
        params (Params): 模型参数
        dual_vars (DualVars): 对偶变量
    Returns:
        tuple: (A_value, selected_vars) - 目标函数值、选中的变量列表
    """
    model = solve_mp_model(r, params, dual_vars)
    
    # 存储选中的变量 (t, b) 对
    selected_vars = []
    # 对应的目标函数值
    A_value = None

    if model.status == GRB.OPTIMAL:
        A_var = model.getVarByName("A")
        A_value = A_var.X
        print(f"找到最优解！目标函数 A = {A_value}")
        
        # 初始化第 r 轮的 x_tb_r 结构
        params.x_tb_r[r] = {}
        
        # 提取 x_tb 的解并存储到 params.x_tb_r[r][t][b]
        for t in range(1, params.T + 1):
            params.x_tb_r[r][t] = {}
            for b in range(1, params.B + 1):
                x_var = model.getVarByName(f"x_{t}_{b}")
                if x_var:
                    x_value = int(round(x_var.X))
                    params.x_tb_r[r][t][b] = x_value
                    if x_value == 1:
                        print(f"选择变量 x[{t}][{b}] = 1")
                        selected_vars.append((t, b))
                else:
                    params.x_tb_r[r][t][b] = 0
        
        # 将当前轮次的selected_vars存入集合D_r
        params.D_r[r] = selected_vars.copy()
        print(f"第{r}轮迭代 - D_r已更新，包含 {len(selected_vars)} 个选中的变量")
        
        # 更新下界
        params.LB = A_value
        # params.LB = max(params.LB, A_value)
        params.A_r = A_value
        
        print(f"第{r}轮迭代 - A值: {A_value:.4f}, 当前LB: {params.LB:.4f}")
        
        return A_value, selected_vars
    else:
        print(f"未找到可行解，模型状态码: {model.status}")
        return None, []


def Step_2(r, gamma, params, dual_vars):
    """
    步骤2：求解补救问题，并提取对偶变量。
    Returns:
        model: Gurobi 模型对象
    """
    try:
        # 从 params 中获取第 r 轮的 x_tb 解
        print(f"Step_2: 准备获取 x_tb_r[{r}]")
        if r not in params.x_tb_r:
            raise KeyError(f"第{r}轮的 x_tb_r 不存在，可用的轮次: {list(params.x_tb_r.keys())}")
        
        xtb = params.x_tb_r[r]
        print(f"Step_2: xtb = {xtb}")
        
        model = solve_rp_model(r, gamma, xtb, params)
        
        if model.status != GRB.OPTIMAL:
            raise Exception(f"RP模型在第{r}轮未找到最优解，状态码: {model.status}")

        # 计算 UB = min{UB, RP目标函数值}
        rp_obj_val = model.ObjVal
        
        # 更新上界：取当前UB和RP目标函数值的较小值
        params.UB = min(params.UB, rp_obj_val)
        
        print(f"第{r}轮迭代 - RP目标函数值: {rp_obj_val:.4f}, 当前UB: {params.UB:.4f}")
        
        # 提取对偶变量并存储到 dual_vars[r]
        # 注意：这里存储的是第 r 轮的对偶变量，供下一轮 MP 使用
        iteration_idx = r  # 使用 r 作为索引，因为 MP 中用 range(r) 会访问 0, 1, ..., r-1
        
        # 初始化第 r 轮的对偶变量字典
        dual_vars.u[iteration_idx] = {}
        dual_vars.v[iteration_idx] = {}
        dual_vars.w[iteration_idx] = {}
        dual_vars.g[iteration_idx] = {}
        dual_vars.h[iteration_idx] = {}
        dual_vars.z[iteration_idx] = {}
        
        # 提取 u_l 的值（u_l 是变量，直接获取变量值）
        for l in range(1, params.L + 1):
            u_var = model.getVarByName(f"u_{l}")
            if u_var:
                dual_vars.u[iteration_idx][l] = u_var.X
            else:
                dual_vars.u[iteration_idx][l] = 0.0
        
        # 提取 z_l 的值（二元变量的解）
        for l in range(1, params.L + 1):
            z_var = model.getVarByName(f"z_{l}")
            if z_var:
                dual_vars.z[iteration_idx][l] = z_var.X
            else:
                dual_vars.z[iteration_idx][l] = 0.0
        
        # 提取 g_t 和 h_t 的值
        for t in range(1, params.T + 1):
            g_var = model.getVarByName(f"g_{t}")
            h_var = model.getVarByName(f"h_{t}")
            dual_vars.g[iteration_idx][t] = g_var.X if g_var else 0.0
            dual_vars.h[iteration_idx][t] = h_var.X if h_var else 0.0
        
        # 提取 v_{t,b} 和 w_{t,b} 的值
        for t in range(1, params.T + 1):
            dual_vars.v[iteration_idx][t] = {}
            dual_vars.w[iteration_idx][t] = {}
            for b in range(1, params.B + 1):
                v_var = model.getVarByName(f"v_{t}_{b}")
                w_var = model.getVarByName(f"w_{t}_{b}")
                dual_vars.v[iteration_idx][t][b] = v_var.X if v_var else 0.0
                dual_vars.w[iteration_idx][t][b] = w_var.X if w_var else 0.0
        
        print(f"第{r}轮对偶变量已提取并存储")

        return model
    except Exception as e:
        print(f"Step_2 第{r}轮发生异常: {str(e)}")
        import traceback
        traceback.print_exc()
        raise
