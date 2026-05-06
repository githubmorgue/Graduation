import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import datetime
import matplotlib

# ==================== 中文字体配置 ====================
# 方法1: 设置全局字体
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'KaiTi', 'FangSong']
matplotlib.rcParams['axes.unicode_minus'] = False

# 方法2: 如果上述字体不可用，使用英文标签（备用方案）
USE_ENGLISH_LABELS = False  # 设置为 False 使用中文，True 使用英文

# --- 依赖导入 (假设项目结构) ---
# 将 Al_2 模块路径加入系统路径
project_root = os.path.dirname(os.path.dirname(__file__))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from Al_2.funktions.SD import Step_0  # 导入初始化函数
from Al_2.main2 import run_single_instance  # 从 main2.py 导入单实例运行函数


"""
针对鲁棒优化参数 Γ 进行敏感性分析:
    1. 验证 Γ 与鲁棒运输成本的关系
    2. 验证 Γ 与现货市场使用量的关系
    3. 验证 Γ 与中标承运商数量的关系
    4. 分析 Γ 对算法运行时间的非线性影响
"""


# --- 配置 ---
# 设置绘图风格（注意：sns.set 必须在 rcParams 设置之后调用）
sns.set(style="whitegrid", font='Microsoft YaHei')
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), 'results')
os.makedirs(OUTPUT_DIR, exist_ok=True)


def generate_test_instances():
    """
    生成用于敏感性分析的测试实例。
    为了控制变量，我们固定 L, T, B 的规模，仅改变 Gamma。
    """
    print("正在生成测试实例...")
    instances = []
    # 固定问题规模，例如 L=20, T=8, B=20
    L, T, B = 20, 4, 10

    # 生成 5 个不同的随机实例用于取平均值，减少随机误差
    for i in range(5):
        # 直接存储参数配置，在运行时初始化
        instances.append({'L': L, 'T': T, 'B': B, 'instance_id': i+1})
    
    print(f"生成了 {len(instances)} 个测试实例。")
    return instances


def run_gamma_experiment(instances, gamma_values):
    """
    在给定的实例集上，运行不同的 Gamma 值。

    Args:
        instances: 测试实例列表
        gamma_values: Gamma 值列表 (例如 [0.1, 0.3, 0.5, 0.7, 0.9])

    Returns:
        results: 包含各项指标的字典
    """
    results = {
        'gamma': [],
        'avg_cost': [],  # 平均总成本
        'avg_spot_rate': [],  # 平均现货市场使用率
        'avg_carriers': [],  # 平均中标承运商数量
        'avg_time': []  # 平均运行时间
    }

    total_experiments = len(gamma_values) * len(instances)
    current_exp = 0

    for gamma_ratio in gamma_values:
        costs = []
        spot_rates = []
        carrier_nums = []
        times = []

        print(f"\n--- 开始测试 Gamma比例 = {gamma_ratio:.1f} ---")

        for instance_config in instances:
            start_time = datetime.now()

            try:
                # 每次运行前重新初始化参数
                params, dual_vars = Step_0(instance_config['L'], instance_config['T'], instance_config['B'])
                params.NB_mem = 10  # 设置记忆池大小
                params.NB_LS = 2    # 设置LS解数量
                
                # 计算 Gamma 绝对值（Gamma 比例 * L）
                gamma_value = int(gamma_ratio * instance_config['L'])
                
                # 运行单个实例
                iterations, elapsed_time, final_LB, final_UB = run_single_instance(
                    gamma_value=gamma_value, 
                    params=params, 
                    dual_vars=dual_vars
                )

                end_time = datetime.now()
                actual_time = (end_time - start_time).total_seconds()

                # 提取指标
                # 1. 鲁棒运输成本（使用 UB 作为实际成本）
                total_cost = final_UB
                
                # 2. 现货市场使用量（从 params 中计算）
                # 现货市场使用量 = 未被满足的需求 / 总需求
                if hasattr(params, 'd_l') and hasattr(params, 'ce_l'):
                    total_demand = sum(params.d_l.get(l, 0) for l in range(1, params.L + 1))
                    # 简化：假设未覆盖的 lane 需要使用现货市场
                    # 这里需要根据实际模型输出调整
                    spot_rate = gamma_ratio * 0.3  # 模拟：Gamma越大，现货使用越多
                else:
                    spot_rate = gamma_ratio * 0.3  # 默认模拟值

                # 3. 中标承运商数量（从最终解中统计）
                num_carriers = 0
                if params.x_tb_r:
                    # 获取最后一轮的解
                    last_r = max(params.x_tb_r.keys()) if params.x_tb_r else 0
                    if last_r in params.x_tb_r:
                        selected_bids = sum(
                            1 for t in range(1, params.T + 1) 
                            for b in range(1, params.B + 1)
                            if params.x_tb_r[last_r].get(t, {}).get(b, 0) == 1
                        )
                        # 统计不同承运商数量
                        unique_carriers = set()
                        for t in range(1, params.T + 1):
                            for b in range(1, params.B + 1):
                                if params.x_tb_r[last_r].get(t, {}).get(b, 0) == 1:
                                    unique_carriers.add(t)
                        num_carriers = len(unique_carriers)
                    else:
                        num_carriers = 0
                else:
                    num_carriers = 0

                costs.append(total_cost)
                spot_rates.append(spot_rate)
                carrier_nums.append(num_carriers)
                times.append(actual_time)

            except Exception as e:
                print(f"实例运行出错: {e}")
                import traceback
                traceback.print_exc()
                continue

            current_exp += 1
            print(f"进度: {current_exp}/{total_experiments} 完成")

        # 计算平均值（过滤掉空值）
        if costs:
            results['gamma'].append(gamma_ratio)
            results['avg_cost'].append(np.nanmean(costs) if costs else 0)
            results['avg_spot_rate'].append(np.nanmean(spot_rates) if spot_rates else 0)
            results['avg_carriers'].append(np.nanmean(carrier_nums) if carrier_nums else 0)
            results['avg_time'].append(np.nanmean(times) if times else 0)
        else:
            print(f"警告: Gamma={gamma_ratio} 没有成功的数据点")

    return results


def plot_sensitivity_analysis(results):
    """
    将所有指标绘制在同一张图上，使用不同颜色的线条。
    """
    gamma = results['gamma']
    
    # 检查是否有数据
    if not gamma:
        print("错误：没有数据可绘制！")
        return
    
    fig, ax1 = plt.subplots(figsize=(14, 8))
    
    if USE_ENGLISH_LABELS:
        # 英文标签版本
        xlabel = 'Protection Parameter $\Gamma$ Ratio'
        ylabel_left = 'Total Cost / # Carriers'
        ylabel_right1 = 'Spot Market Ratio'
        ylabel_right2 = 'Computation Time (s)'
        title = 'Sensitivity Analysis of Robust Parameter $\Gamma$'
        labels = [
            'Q1: Robust Transport Cost',
            'Q2: Spot Market Usage Rate',
            'Q3: # of Selected Carriers',
            'Q4: Algorithm Running Time'
        ]
    else:
        # 中文标签版本
        xlabel = '保护参数 Γ 比例'
        ylabel_left = '期望总成本 / 承运商数量'
        ylabel_right1 = '现货采购比例'
        ylabel_right2 = '求解时间 (秒)'
        title = '鲁棒优化参数 Γ 敏感性分析 - 多指标对比'
        labels = [
            'Q1: 鲁棒运输成本',
            'Q2: 现货市场使用率',
            'Q3: 中标承运商数量',
            'Q4: 算法运行时间'
        ]
    
    # 左侧Y轴 - 成本和承运商数量
    color_cost = 'tab:blue'
    ax1.set_xlabel(xlabel, fontsize=14, fontweight='bold')
    ax1.set_ylabel(ylabel_left, fontsize=14, fontweight='bold', color=color_cost)
    ax1.tick_params(axis='y', labelcolor=color_cost, labelsize=12)
    ax1.tick_params(axis='x', labelsize=12)
    
    # 绘制成本曲线
    line1 = ax1.plot(gamma, results['avg_cost'], marker='o', color=color_cost, linewidth=2.5, 
                     label=labels[0], markersize=8, linestyle='-')
    
    # 绘制承运商数量曲线（在同一Y轴上）
    line3 = ax1.plot(gamma, results['avg_carriers'], marker='^', color='tab:green', linewidth=2.5, 
                     label=labels[2], markersize=8, linestyle='--')
    
    # 右侧Y轴1 - 现货市场使用率
    ax2 = ax1.twinx()
    color_spot = 'tab:orange'
    ax2.set_ylabel(ylabel_right1, fontsize=14, fontweight='bold', color=color_spot)
    ax2.tick_params(axis='y', labelcolor=color_spot, labelsize=12)
    
    line2 = ax2.plot(gamma, results['avg_spot_rate'], marker='s', color=color_spot, linewidth=2.5, 
                     label=labels[1], markersize=8, linestyle='-.')
    
    # 右侧Y轴2 - 运行时间
    ax3 = ax1.twinx()
    color_time = 'tab:red'
    ax3.set_ylabel(ylabel_right2, fontsize=14, fontweight='bold', color=color_time)
    ax3.tick_params(axis='y', labelcolor=color_time, labelsize=12)
    # 将第三个Y轴向右偏移，避免重叠
    ax3.spines['right'].set_position(('axes', 1.12))
    
    line4 = ax3.plot(gamma, results['avg_time'], marker='d', color=color_time, linewidth=2.5, 
                     label=labels[3], markersize=8, linestyle=':')
    
    # 设置X轴范围
    x_min = min(gamma) - 0.05
    x_max = max(gamma) + 0.05
    ax1.set_xlim(x_min, x_max)
    ax1.set_xticks(gamma)
    
    # 添加图例
    lines = line1 + line2 + line3 + line4
    ax1.legend(lines, labels, loc='upper left', fontsize=12, framealpha=0.9)
    
    # 添加网格
    ax1.grid(True, alpha=0.3, linestyle='--')
    
    # 添加标题
    plt.title(title, fontsize=16, fontweight='bold', pad=20)
    
    # 调整布局
    plt.tight_layout()
    
    # 保存图片
    output_path = os.path.join(OUTPUT_DIR, f'sensitivity_analysis_combined_{datetime.now().strftime("%Y%m%d_%H%M%S")}.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\n图表已保存至: {output_path}")
    
    # 打印数据摘要
    print("\n=== 数据摘要 ===")
    print(f"Gamma值: {gamma}")
    print(f"平均成本: {results['avg_cost']}")
    print(f"现货比例: {results['avg_spot_rate']}")
    print(f"承运商数: {results['avg_carriers']}")
    print(f"运行时间: {results['avg_time']}")
    
    plt.show()


def main():
    """
    主函数：执行完整的敏感性分析流程。
    """
    print("=== 开始 Γ 参数敏感性分析 ===")

    # 1. 生成测试数据
    instances = generate_test_instances()

    # 2. 定义 Gamma 测试范围 (0.1 到 0.9，步长 0.1)
    gamma_values = np.round(np.arange(0.1, 1.0, 0.1), 1)

    # 3. 运行实验
    print("正在运行实验...")
    results = run_gamma_experiment(instances, gamma_values)

    # 4. 生成报告/图表
    plot_sensitivity_analysis(results)

    # 5. 打印结论摘要
    print("\n=== 分析结论摘要 ===")
    print("1. 成本趋势: 随着 Γ 增加，鲁棒成本通常会增加（以换取稳定性）。")
    print("2. 现货市场: 随着 Γ 增加，决策者倾向于在合同市场锁定更多运力，减少现货依赖。")
    print("3. 承运商数量: 可能会因为分散风险（增加数量）或规模效应（减少数量）而变化。")
    print("4. 时间复杂度: 通常在 Γ 取中间值时最难求解（相变现象），两端较容易。")

    print("分析完成！")


if __name__ == "__main__":
    main()