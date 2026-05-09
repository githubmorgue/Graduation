"""
消融实验：验证"生成初次割（Initial Cuts）"对算法性能的影响

实验设计：
- Baseline: Al_1 (基础约束生成，无初次割)
- Variant: Algorithm_Ablation_InitialCuts (在Al_1基础上添加初次割)

核心差异：
Variant在Step 0初始化阶段执行Algorithm 3：
1. 场景生成：按最坏需求降序排列车道，对前Γ个车道设置z_l=1
2. 求解确定性WDP：获取初始可行解x^0
3. 求解补救问题：提取对偶变量，生成初次割平面
4. 将初次割添加到主问题中

注意：此变体不包含局部搜索和边界约束，仅验证初次割的效果。

对比指标：
1. 迭代次数
2. 运行时间
3. 收敛质量(LB, UB, Gap)
4. 收敛率
"""

import sys
import os
import time
import csv
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

# 导入算法2中用于初次割生成的模块
from Al_2.funktions.GIC import generate_demand_scenario, solve_deterministic_wdp
from Al_2.funktions.RP import solve_rp_model
from gurobipy import GRB


def run_baseline_al1(L, T, B, gamma_value, instance_id, max_iterations=1000):
    """
    运行算法1（Baseline：基础约束生成，无初次割）
    
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
        tuple: (iterations, elapsed_time, final_LB, final_UB, converged)
    """
    print(f"\n{'='*70}")
    print(f"[Baseline Al_1] 实例{instance_id} - [L={L}, T={T}, B={B}] - Gamma={gamma_value:.1f}")
    print(f"{'='*70}")
    
    start_time = time.time()
    
    # Step 0: 初始化参数（无初次割）
    params, dual_vars = Step_0_Al1(L, T, B, 1)
    r = 1
    converged = False
    
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
        
        # Step 3: 检查收敛条件
        gap = params.UB - params.LB
        print(f"第{r}轮 - Gap: {gap:.6f} (UB={params.UB:.4f}, LB={params.LB:.4f}), 耗时: {elapsed:.2f}秒")
        
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
          f"LB: {final_LB:.4f}, UB: {final_UB:.4f}, Gap: {final_gap:.6f}, 收敛: {converged}")
    
    return r, elapsed_time, final_LB, final_UB, converged


def generate_initial_cuts(params, dual_vars, gamma_value):
    """
    Algorithm 3: 生成初始割平面（Initial Cuts Generation）
    
    算法逻辑（基于论文Algorithm 3）：
    Step 1: 场景生成
        - 计算每条车道的最坏需求：d_bar_l + d_hat_l
        - 按最坏需求降序排列车道
        - 对前Gamma个车道设置z_l=1，其余z_l=0
        - 更新实际需求：d_l = d_bar_l + z_l * d_hat_l
    
    Step 2: 求解确定性WDP
        - 在生成的场景下，求解确定性Winner Determination Problem
        - 得到初始可行解 x^0
    
    Step 3: 求解补救问题并生成割平面
        - 固定x^0，求解鲁棒补救问题（Recourse Problem）
        - 提取对偶变量 (u, v, w, g, h, z)
        - 生成Benders割平面约束（公式17）
        - 将割平面存储到dual_vars[0]，供主问题使用
    
    Args:
        params: 模型参数对象
        dual_vars: 对偶变量对象
        gamma_value: 不确定性预算参数Γ
    
    Returns:
        tuple: (params, dual_vars, num_cuts_generated)
    """
    print(f"\n{'='*60}")
    print(f"Algorithm 3: 开始生成初始割平面（Initial Cuts）")
    print(f"{'='*60}")
    
    # ========== Step 1: 场景生成 ==========
    print(f"\n--- Step 1: 生成需求场景（Gamma={gamma_value}）---")
    params = generate_demand_scenario(params, gamma_value)
    
    print(f"Scenario Generation: 已设置z_l - 前{gamma_value}个车道z_l=1")
    print(f"Scenario Generation: 已更新实际需求 d_l = d_bar_l + z_l * d_hat_l")
    
    # ========== Step 2: 求解确定性WDP ==========
    print(f"\n--- Step 2: 求解确定性WDP，获取初始解x^0 ---")
    selected_vars = solve_deterministic_wdp(params)
    
    if not selected_vars:
        print("警告: 确定性WDP未找到可行解，跳过初次割生成")
        return params, dual_vars, 0
    
    print(f"D-WDP: 找到初始可行解，选中{len(selected_vars)}个投标")
    
    # 存储x^0到params.x_tb_r[0]
    params.x_tb_r[0] = {}
    for t in range(1, params.T + 1):
        params.x_tb_r[0][t] = {}
        for b in range(1, params.B + 1):
            params.x_tb_r[0][t][b] = 1 if (t, b) in selected_vars else 0
    
    # ========== Step 3: 求解补救问题并生成割平面 ==========
    print(f"\n--- Step 3: 求解补救问题，提取对偶变量生成割平面 ---")
    
    xtb = params.x_tb_r[0]
    
    # 求解补救问题（Recourse Problem）
    rp_model = solve_rp_model(gamma_value, xtb, params)
    
    if rp_model.status != GRB.OPTIMAL:
        print(f"警告: 补救问题未找到最优解，状态码: {rp_model.status}")
        return params, dual_vars, 0

    # 获取补救问题的目标函数值
    theta_0 = rp_model.ObjVal
    print(f"RP: 补救问题最优目标值 Θ^0 = {theta_0:.4f}")
    
    # 提取对偶变量并存储到 dual_vars[0]
    # 初始化dual_vars[0]的结构
    dual_vars.u[0] = {}
    dual_vars.v[0] = {}
    dual_vars.w[0] = {}
    dual_vars.g[0] = {}
    dual_vars.h[0] = {}
    dual_vars.z[0] = {}
    
    # 提取 u_l^{0}
    for l in range(1, params.L + 1):
        u_var = rp_model.getVarByName(f"u_{l}")
        dual_vars.u[0][l] = u_var.X if u_var else 0.0
    
    # 提取 z_l^{0}
    for l in range(1, params.L + 1):
        z_var = rp_model.getVarByName(f"z_{l}")
        dual_vars.z[0][l] = z_var.X if z_var else 0.0
    
    # 提取 g_t^{0} 和 h_t^{0}
    for t in range(1, params.T + 1):
        g_var = rp_model.getVarByName(f"g_{t}")
        h_var = rp_model.getVarByName(f"h_{t}")
        dual_vars.g[0][t] = g_var.X if g_var else 0.0
        dual_vars.h[0][t] = h_var.X if h_var else 0.0
    
    # 提取 v_{t,b}^{0} 和 w_{t,b}^{0}
    for t in range(1, params.T + 1):
        dual_vars.v[0][t] = {}
        dual_vars.w[0][t] = {}
        for b in range(1, params.B + 1):
            v_var = rp_model.getVarByName(f"v_{t}_{b}")
            w_var = rp_model.getVarByName(f"w_{t}_{b}")
            dual_vars.v[0][t][b] = v_var.X if v_var else 0.0
            dual_vars.w[0][t][b] = w_var.X if w_var else 0.0
    
    print(f"Initial Cuts: 已提取对偶变量到 dual_vars[0]")
    print(f"Initial Cuts: 生成了 1 个初始割平面约束（公式17）")
    print(f"{'='*60}\n")
    
    return params, dual_vars, 1


def run_variant_with_initial_cuts(L, T, B, gamma_value, instance_id, max_iterations=1000):
    """
    运行增强算法（添加了初次割的Al_1）
    
    算法流程（Algorithm_Ablation_InitialCuts）：
    Step 0: 初始化（增强版）
        - 执行Algorithm 3生成初始割平面
        - 将初次割添加到主问题中
    
    Step 1: 求解主问题（包含初次割约束）-> 更新LB
    Step 2: 求解子问题 -> 更新UB，提取对偶变量
    Step 3: 检查收敛条件
    
    注意：此变体不包含局部搜索和边界约束
    
    Args:
        L: 车道数量
        T: 承运商数量
        B: 投标数量
        gamma_value: gamma值
        instance_id: 实例ID
        max_iterations: 最大迭代次数
    
    Returns:
        tuple: (iterations, elapsed_time, final_LB, final_UB, converged)
    """
    print(f"\n{'='*70}")
    print(f"[Variant IC] 实例{instance_id} - [L={L}, T={T}, B={B}] - Gamma={gamma_value:.1f}")
    print(f"{'='*70}")
    
    start_time = time.time()
    
    # ========== Step 0: 初始化（增强版 - 包含初次割生成）==========
    print(f"\n{'#'*60}")
    print(f"# Step 0: 初始化参数并生成初始割平面（Algorithm 3）")
    print(f"{'#'*60}")
    
    # 基础初始化
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
    
    # 初始化决策变量结构
    params.x_tb_r = {}
    
    # 执行Algorithm 3: 生成初始割平面
    params, dual_vars, num_cuts = generate_initial_cuts(params, dual_vars, gamma_value)
    
    if num_cuts == 0:
        print("警告: 初次割生成失败，退化为标准Al_1")
    
    # 设置迭代计数器从1开始（因为初次割对应r=0）
    r = 1
    converged = False
    
    # ========== 主循环 ==========
    while r <= max_iterations:
        # 检查是否超时（1200秒）
        current_time = time.time()
        elapsed = current_time - start_time
        if elapsed >= 1200:
            print(f"达到时间限制(1200秒)，终止迭代")
            break
        
        # Step 1: 求解主问题（包含初次割约束）
        print(f"\n--- Step 1: 求解主问题（第{r}轮）---")
        A_value, selected_vars = Step_1_Al1(r, params, dual_vars)
        
        if not selected_vars:
            print("主问题未找到可行解，终止迭代")
            break
        
        # Step 2: 求解子问题
        print(f"\n--- Step 2: 求解子问题（第{r}轮）---")
        model = Step_2_Al1(r, gamma_value, params, dual_vars)
        
        # Step 3: 检查收敛条件
        gap = params.UB - params.LB
        print(f"\n--- Step 3: 收敛性检查 ---")
        print(f"第{r}轮 - Gap: {gap:.6f} (UB={params.UB:.4f}, LB={params.LB:.4f}), 耗时: {elapsed:.2f}秒")
        
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
    
    print(f"\n[Variant IC] 实例{instance_id}完成 - 迭代: {r}, 耗时: {elapsed_time:.2f}秒, "
          f"LB: {final_LB:.4f}, UB: {final_UB:.4f}, Gap: {final_gap:.6f}, 收敛: {converged}")
    
    return r, elapsed_time, final_LB, final_UB, converged


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
                               'ablation_study_initial_cut.csv')
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    
    print("="*100)
    print("开始消融实验：验证'生成初次割（Initial Cuts）'的效果")
    print("="*100)
    print("\n实验设计:")
    print("  Baseline: Al_1 (基础约束生成，无初次割)")
    print("  Variant:  Algorithm_Ablation_InitialCuts (添加初次割)")
    print("  控制变量: 不包含局部搜索和边界约束")
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
            baseline_results = {'iterations': [], 'time': [], 'LB': [], 'UB': [], 'gap': [], 'converged': []}
            variant_results = {'iterations': [], 'time': [], 'LB': [], 'UB': [], 'gap': [], 'converged': []}
            
            # 运行多个实例取平均
            for instance_id in range(1, num_instances + 1):
                print(f"\n>>> 实例 {instance_id}/{num_instances}")
                
                try:
                    # 运行Baseline (Al_1)
                    iter_al1, time_al1, lb_al1, ub_al1, conv_al1 = run_baseline_al1(
                        L, T, B, gamma_value, instance_id
                    )
                    baseline_results['iterations'].append(iter_al1)
                    baseline_results['time'].append(time_al1)
                    baseline_results['LB'].append(lb_al1)
                    baseline_results['UB'].append(ub_al1)
                    baseline_results['gap'].append(ub_al1 - lb_al1)
                    baseline_results['converged'].append(conv_al1)
                    
                    # 运行Variant (with Initial Cuts)
                    iter_ic, time_ic, lb_ic, ub_ic, conv_ic = run_variant_with_initial_cuts(
                        L, T, B, gamma_value, instance_id
                    )
                    variant_results['iterations'].append(iter_ic)
                    variant_results['time'].append(time_ic)
                    variant_results['LB'].append(lb_ic)
                    variant_results['UB'].append(ub_ic)
                    variant_results['gap'].append(ub_ic - lb_ic)
                    variant_results['converged'].append(conv_ic)
                    
                except Exception as e:
                    print(f"实例{instance_id}运行出错: {e}")
                    import traceback
                    traceback.print_exc()
                    continue
            
            # 计算平均值（只计算成功运行的实例）
            if baseline_results['iterations'] and variant_results['iterations']:
                avg_baseline = {
                    'avg_iterations': np.mean(baseline_results['iterations']),
                    'avg_time': np.mean(baseline_results['time']),
                    'avg_LB': np.mean(baseline_results['LB']),
                    'avg_UB': np.mean(baseline_results['UB']),
                    'avg_gap': np.mean(baseline_results['gap']),
                    'convergence_rate': sum(baseline_results['converged']) / len(baseline_results['converged'])
                }
                
                avg_variant = {
                    'avg_iterations': np.mean(variant_results['iterations']),
                    'avg_time': np.mean(variant_results['time']),
                    'avg_LB': np.mean(variant_results['LB']),
                    'avg_UB': np.mean(variant_results['UB']),
                    'avg_gap': np.mean(variant_results['gap']),
                    'convergence_rate': sum(variant_results['converged']) / len(variant_results['converged'])
                }
                
                # 计算比值（Baseline / Variant）
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
    print(f"{'|L|-|T|-|B|':<12} {'Γ(%)':<6} {'Baseline (Al_1)':<25} {'Variant (Initial Cuts)':<25} {'Ratio (Baseline/Variant)':<30}")
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
        # [10, 4, 10],
        [20, 8, 10],
        # [40, 16, 10],
        # [80, 32, 10],
    ]
    
    # Gamma比例（覆盖低、中、高三种情况）
    gamma_ratios = [0.1, 0.3, 0.5, 0.7, 0.9]
    # gamma_ratios = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
    
    # 每个配置运行5个实例取平均
    num_instances = 5
    
    # 运行消融实验
    results = run_ablation_experiment(instance_sets, gamma_ratios, num_instances)
    
    print("\n\n" + "="*80)
    print("消融实验完成！")
    print("="*80)
    print("\n实验总结:")
    print("  本实验验证了'生成初次割（Initial Cuts）'对算法性能的单独影响")
    print("  通过对比Baseline和Variant，可以量化初次割带来的加速效果")
    print("="*80)


if __name__ == "__main__":
    main()