#!/usr/bin/env python3
"""
批量路径规划测试脚本

演示如何使用 BatchPathPlanner 进行批量路径规划
"""

from utils.batch_path_planning import BatchPathPlanner


def test_example_1():
    """测试1: 使用 YAML 配置文件批量添加任务（共享占用地图）"""
    print("\n" + "="*80)
    print("测试1: 从 YAML 配置批量添加任务（共享占用地图）")
    print("="*80)

    planner = BatchPathPlanner()

    # 定义多组起终点
    goal_pairs = [
        ([1.0, 1.0], [8.0, 8.0]),
        ([2.0, 2.0], [7.0, 7.0]),
        ([1.5, 1.5], [6.0, 8.0]),
        ([3.0, 1.0], [6.0, 9.0]),
        ([1.0, 3.0], [9.0, 7.0]),
        ([5.0, 5.0], [2.0, 2.0]),
        ([8.0, 1.0], [1.0, 8.0]),
    ]

    planner.add_tasks_from_yaml_assets(
        yaml_path="config/navigation/navigation_assets_fbh.yaml",
        goal_pairs=goal_pairs
    )

    # 执行规划
    results = planner.run_batch(parallel=False, verbose=True)

    # 保存结果
    output_dir = "outputs/batch_planning_test1"
    planner.save_results(output_dir=output_dir)
    planner.generate_summary_report(output_dir=output_dir)
    planner.visualize_paths(output_dir=output_dir, max_paths=5)
    planner.export_to_csv(output_path=f"{output_dir}/results.csv")

    return planner


def test_example_2():
    """测试2: 手动添加多个任务（每个任务独立占用地图）"""
    print("\n" + "="*80)
    print("测试2: 手动添加任务（独立占用地图）")
    print("="*80)

    planner = BatchPathPlanner()

    # 手动添加多个任务
    tasks_config = [
        {
            "task_id": "fbh_scene_001",
            "start": [1.0, 1.0],
            "end": [8.0, 8.0],
            "barrier_image_path": "data/navigation_scenes/barrier_image_level5_1.png",
            "x_bounds": [0.0, 10.0],
            "y_bounds": [0.0, 10.0],
            "offset_radius": 0.3
        },
        {
            "task_id": "fbh_scene_002",
            "start": [2.0, 2.0],
            "end": [7.0, 7.0],
            "barrier_image_path": "data/navigation_scenes/barrier_image_level5_1.png",
            "x_bounds": [0.0, 10.0],
            "y_bounds": [0.0, 10.0],
            "offset_radius": 0.3
        },
    ]

    for task_config in tasks_config:
        planner.add_task(**task_config)

    # 执行规划
    results = planner.run_batch(parallel=False, verbose=True)

    # 保存结果
    output_dir = "outputs/batch_planning_test2"
    planner.save_results(output_dir=output_dir)
    planner.generate_summary_report(output_dir=output_dir)

    return planner


def test_example_3():
    """测试3: 从 JSON 配置文件加载任务"""
    print("\n" + "="*80)
    print("测试3: 从 JSON 配置文件加载任务")
    print("="*80)

    # 从配置文件加载
    planner = BatchPathPlanner("config/batch_tasks_example.json")

    # 执行规划
    results = planner.run_batch(parallel=False, verbose=True)

    # 保存结果
    output_dir = "outputs/batch_planning_test3"
    planner.save_results(output_dir=output_dir)
    planner.generate_summary_report(output_dir=output_dir)
    planner.visualize_paths(output_dir=output_dir, max_paths=10)

    return planner


def test_example_4():
    """测试4: 并行处理模式"""
    print("\n" + "="*80)
    print("测试4: 并行处理模式")
    print("="*80)

    planner = BatchPathPlanner()

    # 添加更多任务以展示并行效果
    goal_pairs = []
    for i in range(20):
        start_x = 1.0 + (i % 5) * 0.5
        start_y = 1.0 + (i // 5) * 0.5
        end_x = 8.0 - (i % 5) * 0.5
        end_y = 8.0 - (i // 5) * 0.5
        goal_pairs.append([[start_x, start_y], [end_x, end_y]])

    planner.add_tasks_from_yaml_assets(
        yaml_path="config/navigation/navigation_assets_fbh.yaml",
        goal_pairs=goal_pairs
    )

    # 并行执行
    results = planner.run_batch(parallel=True, max_workers=4, verbose=True)

    # 保存结果
    output_dir = "outputs/batch_planning_test4_parallel"
    planner.save_results(output_dir=output_dir)
    planner.generate_summary_report(output_dir=output_dir)

    return planner


if __name__ == "__main__":
    print("\n" + "="*80)
    print("批量路径规划测试套件")
    print("="*80)

    # 选择要运行的测试
    import sys

    if len(sys.argv) > 1:
        test_num = int(sys.argv[1])
    else:
        print("\n请选择要运行的测试:")
        print("1. YAML 配置批量添加（共享占用地图）")
        print("2. 手动添加任务（独立占用地图）")
        print("3. 从 JSON 配置文件加载")
        print("4. 并行处理模式")
        print("5. 运行所有测试")
        print("\n默认运行测试1...\n")
        test_num = 1

    if test_num == 1:
        test_example_1()
    elif test_num == 2:
        test_example_2()
    elif test_num == 3:
        test_example_3()
    elif test_num == 4:
        test_example_4()
    elif test_num == 5:
        test_example_1()
        test_example_2()
        test_example_3()
        test_example_4()
    else:
        print(f"未知的测试编号: {test_num}")

    print("\n" + "="*80)
    print("测试完成！")
    print("="*80 + "\n")
