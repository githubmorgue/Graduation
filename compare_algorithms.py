import time
import csv
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Al_1.funktions.SD import Step_0 as Step_0_Al1, Step_1 as Step_1_Al1, Step_2 as Step_2_Al1
from Al_2.funktions.SD import Step_0 as Step_0_Al2, Step_1 as Step_1_Al2, Step_2 as Step_2_Al2, Step_3 as Step_3_Al2, Step_4 as Step_4_Al2, Step_5 as Step_5_Al2


def run_algorithm_1(L, T, B, gamma_value, instance_id, use_seed=None):
    """
    运行 Algorithm 1 (Al_1)
    """
    import random
    if use_seed is not None:
        random.seed(use_seed)

    print(f"  [Al_1] 实例{instance_id} - Gamma={gamma_value:.1f}")

    start_time = time.time()

    params, dual_vars = Step_0_Al1(L, T, B, 1)
    max_iterations = 1000
    r = 1

    while r <= max_iterations:
        A_value, selected_vars = Step_1_Al1(r, params, dual_vars)

        if not selected_vars:
            break

        model = Step_2_Al1(r, gamma_value, params, dual_vars)

        gap = params.UB - params.LB

        if gap < 1e-6:
            break

        r += 1

    end_time = time.time()
    elapsed_time = end_time - start_time

    return r, elapsed_time


def run_algorithm_2(L, T, B, gamma_value, instance_id, use_seed=None):
    """
    运行 Algorithm 2 (Al_2)
    """
    import random
    if use_seed is not None:
        random.seed(use_seed)

    print(f"  [Al_2] 实例{instance_id} - Gamma={gamma_value:.1f}")

    start_time = time.time()

    # 初始化参数和对偶变量
    params, dual_vars = Step_0_Al2(L, T, B)
    
    # 设置人工参数
    params.NB_mem = 10  # 记忆池大小
    params.NB_LS = 2    # LS解数量
    
    max_iterations = 1000
    r = 0

    # Step_1: 生成初始需求场景并求解（只在r=0时执行一次）
    params, dual_vars = Step_1_Al2(params, dual_vars, gamma_value)

    while r <= max_iterations:
        # Step_2: 求解主问题，更新LB
        A_value, selected_vars = Step_2_Al2(r, params, dual_vars, gamma_value)
        
        if not selected_vars:
            print("Step_2: 主问题未找到可行解，终止迭代")
            break

        # Step_3: 处理第一个候选解，初始化dual_vars[r+1][0]
        model = Step_3_Al2(r, gamma_value, params, dual_vars)
        
        # Step_4: 局部搜索生成LS解
        Step_4_Al2(r, gamma_value, params, dual_vars)
        
        # Step_5: 处理所有其他候选解，扩展dual_vars[r+1][1..k]
        Step_5_Al2(r, gamma_value, params, dual_vars)
        
        # 检查收敛条件
        gap = params.UB - params.LB
        
        if gap < 1e-6:
            print(f"算法在第{r}轮收敛")
            break
        
        # r正常递增1
        r += 1

    end_time = time.time()
    elapsed_time = end_time - start_time

    return r, elapsed_time


def write_results_to_csv(output_file, results, instance_sets, gamma_ratios, is_append=False):
    """
    将结果写入CSV文件
    
    Args:
        output_file: 输出文件路径
        results: 结果字典
        instance_sets: 实例集列表
        gamma_ratios: gamma比例列表
        is_append: 是否追加模式（True=追加，False=覆盖）
    """
    mode = 'a' if is_append else 'w'
    with open(output_file, mode, newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)

        # 如果是覆盖模式，写入表头
        if not is_append:
            writer.writerow(['|L| - |T| - B', 'Γ^d(%)', 'Algorithm 1 - # iter.', 'Algorithm 1 - Time (s)',
                            'Algorithm 2 - # iter.', 'Algorithm 2 - Time (s)',
                            'Ratio - # iter.', 'Ratio - Time (s)'])

        for L, T, B in instance_sets:
            # 只写入已经完成的实例集
            if (L, T, B) not in results:
                continue
                
            instance_label = f"{L}-{T}-{B}"

            for idx, ratio in enumerate(gamma_ratios):
                # 只写入已有数据的gamma比例
                if ratio not in results[(L, T, B)]:
                    continue
                    
                gamma_percent = int(ratio * 100)

                al1_data = results[(L, T, B)][ratio]['al1']
                al2_data = results[(L, T, B)][ratio]['al2']
                ratio_iter = results[(L, T, B)][ratio]['ratio_iter']
                ratio_time = results[(L, T, B)][ratio]['ratio_time']

                # 第一个gamma比例显示实例集标签，其余为空
                label = instance_label if idx == 0 else ""

                writer.writerow([
                    label,
                    gamma_percent,
                    f"{al1_data['avg_iterations']:.1f}",
                    f"{al1_data['avg_time']:.0f}",
                    f"{al2_data['avg_iterations']:.1f}",
                    f"{al2_data['avg_time']:.0f}",
                    f"{ratio_iter:.2f}",
                    f"{ratio_time:.2f}"
                ])

            # 实例集之间添加空行
            writer.writerow([])


def main():
    # 定义实例集 [L, T, B]
    instance_sets = [
        [10, 4, 10],
        [10, 4, 20],
        [20, 8, 10],
        [20, 8, 20],
        # [30, 12, 10],
        # [30, 12, 20],
        # [40, 16, 10],
        # [40, 16, 20],
        # [50, 20, 10],
        # [50, 20, 20],
        # [60, 24, 10],
        # [60, 24, 20],
        # [70, 28, 10],
        # [70, 28, 20],
        # [80, 32, 10],
        # [80, 32, 20],
        # [90, 36, 10],
        # [90, 36, 20],
        # [100, 40, 10],
        # [100, 40, 20],
        # [200, 80, 10],
        # [200, 80, 20],
    ]
    # instance_sets = [
    #     [10, 4, 2],
    #     [40, 16, 3],
    #     # [40, 16, 4],
    #     [60, 24, 3],
    #     # [60, 24, 4],
    #     [80, 32, 3],
    #     # [80, 32, 4],
    # ]

    # 每个实例集生成5个随机实例
    num_instances = 5

    # gamma比例：10% ~ 90%，步长10%
    gamma_ratios = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]

    # 存储结果
    # results[instance_set][gamma_ratio] = {
    #     'al1': {'avg_iterations': x, 'avg_time': y},
    #     'al2': {'avg_iterations': x, 'avg_time': y}
    # }
    results = {}

    # CSV输出文件
    output_file = "comparison_results.csv"
    
    # 清空文件并写入表头
    write_results_to_csv(output_file, {}, [], [], is_append=False)

    print("="*120)
    print("开始对比实验：Algorithm 1 vs Algorithm 2")
    print("="*120)

    for L, T, B in instance_sets:
        print(f"\n\n{'#'*120}")
        print(f"# 处理实例集: L={L}, T={T}, B={B}")
        print(f"{'#'*120}")

        if (L, T, B) not in results:
            results[(L, T, B)] = {}

        for gamma_ratio in gamma_ratios:
            gamma_value = int(gamma_ratio * L)

            print(f"\n{'='*120}")
            print(f"Gamma比例={gamma_ratio*100:.0f}% (Gamma={gamma_value:.1f})")
            print(f"{'='*120}")

            # 存储当前gamma下所有实例的结果
            al1_iterations_list = []
            al1_time_list = []
            al2_iterations_list = []
            al2_time_list = []

            for instance_id in range(1, num_instances + 1):
                # 使用相同的seed确保两个算法使用相同的随机实例
                seed = instance_id * 1000 + int(gamma_ratio * 100)

                print(f"\n--- 实例 {instance_id}/{num_instances} ---")

                # 运行 Algorithm 1
                al1_iter, al1_time = run_algorithm_1(L, T, B, gamma_value, instance_id, use_seed=seed)
                al1_iterations_list.append(al1_iter)
                al1_time_list.append(al1_time)
                print(f"  Al_1: 迭代={al1_iter}, 时间={al1_time:.2f}s")

                # 运行 Algorithm 2
                al2_iter, al2_time = run_algorithm_2(L, T, B, gamma_value, instance_id, use_seed=seed)
                al2_iterations_list.append(al2_iter)
                al2_time_list.append(al2_time)
                print(f"  Al_2: 迭代={al2_iter}, 时间={al2_time:.2f}s")

            # 计算平均值
            avg_al1_iter = sum(al1_iterations_list) / num_instances
            avg_al1_time = sum(al1_time_list) / num_instances
            avg_al2_iter = sum(al2_iterations_list) / num_instances
            avg_al2_time = sum(al2_time_list) / num_instances

            # 计算比率 Al_1 / Al_2
            ratio_iter = avg_al1_iter / avg_al2_iter if avg_al2_iter > 0 else 0
            ratio_time = avg_al1_time / avg_al2_time if avg_al2_time > 0 else 0

            results[(L, T, B)][gamma_ratio] = {
                'al1': {'avg_iterations': avg_al1_iter, 'avg_time': avg_al1_time},
                'al2': {'avg_iterations': avg_al2_iter, 'avg_time': avg_al2_time},
                'ratio_iter': ratio_iter,
                'ratio_time': ratio_time
            }

            print(f"\n>>> 汇总:")
            print(f"    Al_1 - 平均迭代: {avg_al1_iter:.1f}, 平均时间: {avg_al1_time:.0f}s")
            print(f"    Al_2 - 平均迭代: {avg_al2_iter:.1f}, 平均时间: {avg_al2_time:.0f}s")
            print(f"    比率(Al_1/Al_2) - 迭代: {ratio_iter:.2f}, 时间: {ratio_time:.2f}")

        # ========== 完成一个实例集后，立即写入文件 ==========
        print(f"\n{'='*120}")
        print(f"实例集 [{L}-{T}-{B}] 完成，结果已写入 {output_file}")
        print(f"{'='*120}")
        
        # 追加写入当前实例集的所有结果
        write_results_to_csv(output_file, results, [(L, T, B)], gamma_ratios, is_append=True)

    # 输出对比表格到控制台
    print("\n\n")
    print("="*150)
    print("对比实验结果汇总")
    print("="*150)

    # 打印表头
    header = f"{'|L|-|T|-B':<12} {'Γ^d(%)':<10}"
    header += f"{'Algorithm 1':^25} {'Algorithm 2':^25} {'Ratio: Al_1/Al_2':^30}"
    header += f"{'':<10}"
    print(header)

    subheader = f"{'':<12} {'':<10}"
    subheader += f"{'# iter.':<12} {'Time (s)':<13}"
    subheader += f"{'# iter.':<12} {'Time (s)':<13}"
    subheader += f"{'# iter.':<12} {'Time (s)':<18}"
    print(subheader)
    print("-"*150)

    for L, T, B in instance_sets:
        instance_label = f"{L}-{T}-{B}"

        for idx, ratio in enumerate(gamma_ratios):
            gamma_percent = int(ratio * 100)

            al1_data = results[(L, T, B)][ratio]['al1']
            al2_data = results[(L, T, B)][ratio]['al2']
            ratio_iter = results[(L, T, B)][ratio]['ratio_iter']
            ratio_time = results[(L, T, B)][ratio]['ratio_time']

            # 第一个gamma比例显示实例集标签，其余为空
            label = instance_label if idx == 0 else ""

            row = f"{label:<12} {gamma_percent:<10}"
            row += f"{al1_data['avg_iterations']:<12.1f} {al1_data['avg_time']:<13.0f}"
            row += f"{al2_data['avg_iterations']:<12.1f} {al2_data['avg_time']:<13.0f}"
            row += f"{ratio_iter:<12.2f} {ratio_time:<18.2f}"

            print(row)

        print()  # 实例集之间空行

    print("="*150)

    print(f"\n最终结果已保存到文件: {output_file}")
    print("="*150)
    print("对比实验全部完成！")
    print("="*150)


if __name__ == "__main__":
    main()