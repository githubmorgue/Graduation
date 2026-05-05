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
        tuple: (迭代次数, 运行时间)
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

    return r, elapsed_time


def main():
    # 定义8个实例集 [L, T, B]
    # instance_sets = [
    #     # [10, 4, 2],
    #     # [10, 4, 4],
    #     # [20, 8, 2],
    #     # [20, 8, 4],
    #     # [40, 16, 2],
    #     # [40, 16, 4],
    #     # [100, 20, 2],
    #     # [120, 24, 2]
    # ]
    instance_sets = [
        [10, 4, 2],
        # [40, 16, 3],
        # [40, 16, 4],
        # [60, 24, 3],
        # [60, 24, 4],
        # [80, 32, 3],
        # [80, 32, 4],
    ]

    # 每个实例集生成5个随机实例
    num_instances = 5

    # gamma比例：10% ~ 90%，步长10%
    gamma_ratios = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]

    # 人工设定NB_mem参数（记忆池大小）
    NB_mem = 10

    # 人工设定NB_LS参数（LS解的数量），默认为2
    NB_LS = 2

    # 存储结果：{(L, T, B): {gamma_ratio: {'avg_iterations': x, 'avg_time': y}}}
    results = {}

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

            total_iterations = 0
            total_time = 0.0

            # 生成并运行5个实例
            for instance_id in range(1, num_instances + 1):
                print(f"\n\n{'*'*80}")
                print(f"* 实例 {instance_id}/{num_instances}: L={L}, T={T}, B={B}, Gamma={gamma_value:.1f}")
                print(f"{'*'*80}")
                
                # 【重要】每个实例都需要重新初始化参数和对偶变量
                params, dual_vars = Step_0(L, T, B)
                params.NB_mem = NB_mem  # 设置NB_mem参数
                params.NB_LS = NB_LS  # 设置NB_LS参数
                
                iterations, elapsed_time = run_single_instance(gamma_value, params, dual_vars)
                total_iterations += iterations
                total_time += elapsed_time

            # 计算平均值
            avg_iterations = total_iterations / num_instances
            avg_time = total_time / num_instances

            results[(L, T, B)][gamma_ratio] = {
                'avg_iterations': avg_iterations,
                'avg_time': avg_time
            }

            print(f"\n>>> Gamma比例={gamma_ratio*100:.0f}% 结果汇总:")
            print(f"    平均迭代次数: {avg_iterations:.2f}")
            print(f"    平均耗时: {avg_time:.2f}秒")

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

    print("\n" + "="*100)
    print("实验全部完成！")
    print("="*100)

    # 将结果输出到CSV文件（转置格式）
    output_file = "computational_results_2.csv"
    with open(output_file, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)

        # 写入表头
        writer.writerow(['|L| - |T| - |B|', 'Γ (%)', '# Iter.', 'Time (s)'])

        # 对每个实例集，写入9个gamma比例的结果
        for L, T, B in instance_sets:
            instance_label = f"{L}-{T}-{B}"

            for idx, ratio in enumerate(gamma_ratios):
                gamma_percent = int(ratio * 100)
                avg_iter = results[(L, T, B)][ratio]['avg_iterations']
                avg_time = results[(L, T, B)][ratio]['avg_time']

                # 第一个gamma比例显示实例集标签，其余为空
                if idx == 0:
                    writer.writerow([instance_label, gamma_percent, f"{avg_iter:.2f}", f"{avg_time:.0f}"])
                else:
                    writer.writerow(['', gamma_percent, f"{avg_iter:.2f}", f"{avg_time:.0f}"])

            # 实例集之间添加空行
            writer.writerow([])

    print(f"\n结果已保存到文件: {output_file}")


if __name__ == "__main__":
    main()