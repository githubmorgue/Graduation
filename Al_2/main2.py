import time
import csv
from Al_2.funktions.SD import Step_0, Step_1, Step_2, Step_3, Step_4, Step_5

def run_single_instance(gamma_value, params, dual_vars):
    """
    运行单个实例的Benders分解算法
    
    Args:
        gamma_value: gamma值
        params: 模型参数对象
        dual_vars: 对偶变量对象
    
    Returns:
        tuple: (迭代次数, 运行时间, 最终LB, 最终UB)
    """
    
    start_time = time.time()

    # Step_1: 生成初始需求场景并求解（只在r=0时执行一次）
    print(f"\n{'='*60}")
    print(f"开始执行Step_1 - 生成初始解")
    print(f"{'='*60}")
    params, dual_vars = Step_1(params, dual_vars, gamma_value)

    r = 0
    max_iterations = 1000

    while r <= max_iterations:

        # Step_2: 求解主问题，更新LB
        A_value, selected_vars = Step_2(r, params, dual_vars, gamma_value)
        
        if not selected_vars:
            print("Step_2: 主问题未找到可行解，终止迭代")
            break

        # Step_3: 处理第一个候选解，初始化dual_vars[r+1][0]，检查终止条件
        model, is_optimal = Step_3(r, gamma_value, params, dual_vars)
        
        # 检查Step_3是否找到最优解（UB^r == LB^r）
        if is_optimal:
            print(f"\n算法在第{r}轮收敛（Step_3检测到UB=LB）")
            break
        
        # Step_4: 局部搜索生成LS解
        Step_4(r, gamma_value, params, dual_vars)
        
        # Step_5: 处理所有其他候选解，扩展dual_vars[r+1][1..k]
        Step_5(r, gamma_value, params, dual_vars)
        
        # r正常递增1
        r += 1
    
    end_time = time.time()
    elapsed_time = end_time - start_time
    
    final_LB = params.LB
    final_UB = params.UB
    
    print(f"实例完成 - 迭代次数: {r}, 耗时: {elapsed_time:.2f}秒, LB: {final_LB:.4f}, UB: {final_UB:.4f}")

    return r, elapsed_time, final_LB, final_UB


def write_results_to_csv(output_file, instance_sets, results, gamma_ratios):
    """
    将结果实时写入CSV文件
    
    Args:
        output_file: CSV文件路径
        instance_sets: 实例集列表
        results: 结果字典
        gamma_ratios: Gamma比例列表
    """
    with open(output_file, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)

        # 写入表头
        writer.writerow(['|L| - |T| - |B|', 'Γ (%)', '# Iter.', 'Time (s)', 'Avg LB', 'Avg UB'])

        # 对每个实例集，写入9个gamma比例的结果
        for L, T, B in instance_sets:
            instance_label = f"{L}-{T}-{B}"

            for idx, ratio in enumerate(gamma_ratios):
                # 检查该gamma比例是否已有结果
                if ratio not in results.get((L, T, B), {}):
                    continue
                    
                gamma_percent = int(ratio * 100)
                avg_iter = results[(L, T, B)][ratio]['avg_iterations']
                avg_time = results[(L, T, B)][ratio]['avg_time']
                avg_LB = results[(L, T, B)][ratio]['avg_LB']
                avg_UB = results[(L, T, B)][ratio]['avg_UB']

                # 第一个gamma比例显示实例集标签，其余为空
                if idx == 0:
                    writer.writerow([instance_label, gamma_percent, f"{avg_iter:.2f}", f"{avg_time:.0f}", f"{avg_LB:.4f}", f"{avg_UB:.4f}"])
                else:
                    writer.writerow(['', gamma_percent, f"{avg_iter:.2f}", f"{avg_time:.0f}", f"{avg_LB:.4f}", f"{avg_UB:.4f}"])

            # 实例集之间添加空行
            writer.writerow([])


def main():
    # 定义8个实例集 [L, T, B]
    instance_sets = [
        [10, 4, 10],
        [10, 4, 20],
        [20, 8, 10],
        [20, 8, 20],
        # [30, 12, 10],
        # [30, 12, 20],
        [40, 16, 10],
        [40, 16, 20],
        # [50, 20, 10],
        # [50, 20, 20],
        # [60, 24, 10],
        # [60, 24, 20],
        # [70, 28, 10],
        # [70, 28, 20],
        [80, 32, 10],
        [80, 32, 20],
        # [90, 36, 10],
        # [90, 36, 20],
        [100, 40, 10],
        [100, 40, 20],
        [200, 80, 10],
        [200, 80, 20],
    ]

    # 每个实例集生成5个随机实例
    num_instances = 5

    # gamma比例：10% ~ 90%，步长10%
    gamma_ratios = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]

    # 人工设定NB_mem参数（记忆池大小）
    NB_mem = 10

    # 人工设定NB_LS参数（LS解的数量），默认为2
    NB_LS = 2

    # 存储结果：{(L, T, B): {gamma_ratio: {'avg_iterations': x, 'avg_time': y, 'avg_LB': z, 'avg_UB': w}}}
    results = {}

    # CSV输出文件
    output_file = "computational_results_2.csv"
    
    # 初始化CSV文件（只写表头）
    with open(output_file, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)
        writer.writerow(['|L| - |T| - |B|', 'Γ (%)', '# Iter.', 'Time (s)', 'Avg LB', 'Avg UB'])
    
    print("="*80)
    print("开始批量实验")
    print("="*80)

    for L, T, B in instance_sets:
        print(f"\n\n{'#'*80}")
        print(f"# 处理实例集: L={L}, T={T}, B={B}")
        print(f"{'#'*80}")

        if (L, T, B) not in results:
            results[(L, T, B)] = {}

        # 对每个gamma比例进行实验
        for gamma_ratio in gamma_ratios:
            gamma_value = int(gamma_ratio * L)

            print(f"\n{'-'*80}")
            print(f"--- 测试 Gamma比例={gamma_ratio*100:.0f}% (Gamma={gamma_value:.1f}) ---")
            print(f"{'-'*80}")

            total_iterations = 0
            total_time = 0.0
            total_LB = 0.0
            total_UB = 0.0

            # 生成并运行5个实例
            for instance_id in range(1, num_instances + 1):
                print(f"\n\n{'*'*80}")
                print(f"* 实例 {instance_id}/{num_instances}: L={L}, T={T}, B={B}, Gamma={gamma_value:.1f}")
                print(f"{'*'*80}")
                
                # 【重要】每个实例都需要重新初始化参数和对偶变量
                params, dual_vars = Step_0(L, T, B)
                params.NB_mem = NB_mem  # 设置NB_mem参数
                params.NB_LS = NB_LS  # 设置NB_LS参数
                
                iterations, elapsed_time, final_LB, final_UB = run_single_instance(gamma_value, params, dual_vars)
                total_iterations += iterations
                total_time += elapsed_time
                total_LB += final_LB
                total_UB += final_UB

            # 计算平均值
            avg_iterations = total_iterations / num_instances
            avg_time = total_time / num_instances
            avg_LB = total_LB / num_instances
            avg_UB = total_UB / num_instances

            results[(L, T, B)][gamma_ratio] = {
                'avg_iterations': avg_iterations,
                'avg_time': avg_time,
                'avg_LB': avg_LB,
                'avg_UB': avg_UB
            }

            print(f"\n>>> Gamma比例={gamma_ratio*100:.0f}% 结果汇总:")
            print(f"    平均迭代次数: {avg_iterations:.2f}")
            print(f"    平均耗时: {avg_time:.2f}秒")
            print(f"    平均LB: {avg_LB:.4f}")
            print(f"    平均UB: {avg_UB:.4f}")
        
        # 每个实例集完成后立即写入CSV
        print(f"\n>>> 正在将实例集 [{L}-{T}-{B}] 的结果写入CSV文件...")
        write_results_to_csv(output_file, instance_sets, results, gamma_ratios)
        print(f">>> 已保存到: {output_file}")

    # 输出最终对比表格到控制台
    print("\n\n")
    print("="*100)
    print("实验结果汇总")
    print("="*100)

    # 表头
    header = f"{'实例集 [L,T,B]':<25}"
    for ratio in gamma_ratios:
        header += f"{'Gamma='+str(int(ratio*100))+'%':<20}"
    print(header)
    print("-"*100)

    # 数据行 - 迭代次数
    print("\n【平均迭代次数】")
    for L, T, B in instance_sets:
        row = f"[{L:3d}, {T:2d}, {B:1d}]".ljust(25)
        for ratio in gamma_ratios:
            avg_iter = results[(L, T, B)][ratio]['avg_iterations']
            row += f"{avg_iter:<20.2f}"
        print(row)

    print("\n" + "-"*100)

    # 数据行 - 时间消耗
    print("\n【平均时间消耗(秒)】")
    for L, T, B in instance_sets:
        row = f"[{L:3d}, {T:2d}, {B:1d}]".ljust(25)
        for ratio in gamma_ratios:
            avg_time = results[(L, T, B)][ratio]['avg_time']
            row += f"{avg_time:<20.2f}"
        print(row)

    print("\n" + "-"*100)

    # 数据行 - 平均LB
    print("\n【平均下界 LB】")
    for L, T, B in instance_sets:
        row = f"[{L:3d}, {T:2d}, {B:1d}]".ljust(25)
        for ratio in gamma_ratios:
            avg_LB = results[(L, T, B)][ratio]['avg_LB']
            row += f"{avg_LB:<20.4f}"
        print(row)

    print("\n" + "-"*100)

    # 数据行 - 平均UB
    print("\n【平均上界 UB】")
    for L, T, B in instance_sets:
        row = f"[{L:3d}, {T:2d}, {B:1d}]".ljust(25)
        for ratio in gamma_ratios:
            avg_UB = results[(L, T, B)][ratio]['avg_UB']
            row += f"{avg_UB:<20.4f}"
        print(row)

    print("\n" + "="*100)
    print("实验全部完成！")
    print("="*100)

    print(f"\n最终结果已保存到文件: {output_file}")


if __name__ == "__main__":
    main()