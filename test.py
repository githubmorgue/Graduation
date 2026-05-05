"""
测试 SD.py 的 Step_0 和 Step_1 函数
"""
import sys
from Al_2.funktions.SD import Step_0, Step_1


def test_step0_step1():
    """
    独立测试 Step_0 和 Step_1 的执行流程
    """
    print("="*80)
    print("开始测试 Step_0 和 Step_1")
    print("="*80)

    # 设置测试参数
    L = 10  # 车道数量
    T = 4   # 承运商数量
    B = 2   # 投标数量
    gamma = 5  # 不确定性预算

    print(f"\n测试参数: L={L}, T={T}, B={B}, gamma={gamma}\n")

    try:
        # ========== 执行 Step_0 ==========
        print("\n" + "="*80)
        print("【Step_0】初始化参数和对偶变量")
        print("="*80)

        params, dual_vars = Step_0(L, T, B)

        print(f"\nStep_0 完成！")
        print(f"  - params.T = {params.T}")
        print(f"  - params.L = {params.L}")
        print(f"  - params.B = {params.B}")
        print(f"  - d_bar_l (前3个): {dict(list(params.d_bar_l.items())[:3])}")
        print(f"  - d_hat_l (前3个): {dict(list(params.d_hat_l.items())[:3])}")
        print(f"  - d_l (前3个): {dict(list(params.d_l.items())[:3])}")
        print(f"  - z_l (前3个): {dict(list(params.z_l.items())[:3])}")

        # ========== 执行 Step_1 ==========
        print("\n" + "="*80)
        print("【Step_1】生成需求场景并求解WDP")
        print("="*80)

        # 记录 Step_1 执行前的状态
        print(f"\nStep_1 执行前:")
        print(f"  - d_l (前3个): {dict(list(params.d_l.items())[:3])}")
        print(f"  - z_l (前3个): {dict(list(params.z_l.items())[:3])}")

        params, dual_vars = Step_1(params, dual_vars, gamma)

        # 记录 Step_1 执行后的状态
        print(f"\nStep_1 执行后:")
        print(f"  - d_l (前3个): {dict(list(params.d_l.items())[:3])}")
        print(f"  - z_l (前3个): {dict(list(params.z_l.items())[:3])}")
        print(f"  - x_tb_r[0] 结构: {type(params.x_tb_r.get(0))}")

        if 0 in params.x_tb_r:
            selected_count = sum(
                1 for t in params.x_tb_r[0]
                for b in params.x_tb_r[0][t]
                if params.x_tb_r[0][t][b] == 1
            )
            print(f"  - x_tb_r[0] 中选中的变量数: {selected_count}")

        # 检查 dual_vars
        print(f"\n对偶变量检查:")
        print(f"  - dual_vars.u[0] 是否存在: {0 in dual_vars.u}")
        if 0 in dual_vars.u:
            print(f"  - dual_vars.u[0] 类型: {type(dual_vars.u[0])}")
            if isinstance(dual_vars.u[0], dict) and len(dual_vars.u[0]) > 0:
                print(f"  - dual_vars.u[0][0] (第一个元素): {dual_vars.u[0][0]}")

        print("\n" + "="*80)
        print("✓ 测试成功！Step_0 和 Step_1 正常运行")
        print("="*80)

        return True

    except Exception as e:
        print("\n" + "="*80)
        print("✗ 测试失败！")
        print("="*80)
        print(f"错误类型: {type(e).__name__}")
        print(f"错误信息: {str(e)}")
        print("\n详细堆栈跟踪:")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_step0_step1()
    sys.exit(0 if success else 1)