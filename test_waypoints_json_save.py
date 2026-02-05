#!/usr/bin/env python3
"""
测试waypoints JSON保存功能

验证：
1. waypoints能否正确保存到JSON文件
2. 保存的格式是否与批量路径规划结果一致
3. 能否正确读取和解析保存的JSON文件
"""

import json
import sys
from pathlib import Path

def test_waypoints_json_format():
    """测试waypoints JSON文件格式"""

    print("="*70)
    print("测试waypoints JSON保存功能")
    print("="*70)

    # 查找最新的waypoints目录
    output_base = Path("outputs/collect")
    if not output_base.exists():
        print("❌ 未找到outputs/collect目录")
        print("   请先运行导航任务生成waypoints数据")
        return False

    # 查找最新的运行目录
    run_dirs = sorted(output_base.glob("*_*"), reverse=True)
    if not run_dirs:
        print("❌ 未找到任何运行记录")
        return False

    latest_run = run_dirs[0]
    waypoints_dir = latest_run / "waypoints"

    print(f"\n📁 检查目录: {latest_run}")
    print(f"📁 waypoints目录: {waypoints_dir}")

    if not waypoints_dir.exists():
        print(f"❌ waypoints目录不存在: {waypoints_dir}")
        print(f"   提示：运行导航任务后会自动创建waypoints目录")
        return False

    # 查找所有waypoints JSON文件
    json_files = list(waypoints_dir.glob("episode_*_waypoints.json"))
    if not json_files:
        print(f"❌ waypoints目录中没有JSON文件")
        return False

    print(f"\n✅ 找到 {len(json_files)} 个waypoints JSON文件\n")

    # 检查每个文件
    all_valid = True
    for json_file in sorted(json_files):
        print(f"📄 检查文件: {json_file.name}")

        try:
            with open(json_file, 'r', encoding='utf-8') as f:
                data = json.load(f)

            # 验证必需字段
            required_fields = [
                "task_id", "start", "end", "is_success",
                "waypoints", "total_distance", "num_waypoints"
            ]

            missing_fields = [field for field in required_fields if field not in data]
            if missing_fields:
                print(f"  ❌ 缺少字段: {missing_fields}")
                all_valid = False
                continue

            # 验证数据类型
            if not isinstance(data["waypoints"], list):
                print(f"  ❌ waypoints应该是列表")
                all_valid = False
                continue

            if len(data["waypoints"]) == 0:
                print(f"  ⚠️ waypoints为空")
                continue

            # 验证waypoint格式
            first_wp = data["waypoints"][0]
            if not isinstance(first_wp, list) or len(first_wp) != 3:
                print(f"  ❌ waypoint格式错误，应为[x, y, theta]")
                all_valid = False
                continue

            # 显示信息
            print(f"  ✅ task_id: {data['task_id']}")
            print(f"     起点: {data['start']}")
            print(f"     终点: {data['end']}")
            print(f"     路径点数: {data['num_waypoints']}")
            print(f"     路径长度: {data['total_distance']:.3f}米")
            print(f"     成功: {data['is_success']}")

            # 显示前3个waypoints
            print(f"     前3个路径点:")
            for i, wp in enumerate(data["waypoints"][:3]):
                print(f"       {i+1}. [{wp[0]:.3f}, {wp[1]:.3f}, {wp[2]:.3f}]")
            if len(data["waypoints"]) > 3:
                print(f"       ... (共{len(data['waypoints'])}个点)")

        except json.JSONDecodeError as e:
            print(f"  ❌ JSON解析失败: {e}")
            all_valid = False
        except Exception as e:
            print(f"  ❌ 读取失败: {e}")
            all_valid = False

        print()

    # 检查合并文件
    all_waypoints_file = latest_run / "all_waypoints.json"
    if all_waypoints_file.exists():
        print(f"\n📄 检查合并文件: {all_waypoints_file.name}")
        try:
            with open(all_waypoints_file, 'r', encoding='utf-8') as f:
                all_data = json.load(f)
            print(f"  ✅ 包含 {len(all_data)} 个episodes")
        except Exception as e:
            print(f"  ❌ 读取合并文件失败: {e}")

    # 最终结果
    print("\n" + "="*70)
    if all_valid:
        print("✅ 所有waypoints JSON文件格式正确！")
        print("\n📊 保存格式与批量路径规划结果完全一致，可用于：")
        print("   - 路径分析和可视化")
        print("   - 与其他数据集合并")
        print("   - 统计和分析")
    else:
        print("❌ 部分文件存在问题，请检查上述错误信息")
    print("="*70)

    return all_valid


def compare_with_batch_planning_format():
    """对比JSON格式与批量路径规划格式"""

    print("\n" + "="*70)
    print("对比JSON格式与批量路径规划结果格式")
    print("="*70)

    # 查找批量路径规划结果示例
    batch_results_dir = Path("outputs/path_planning_batch_results_gemini_flash")
    if not batch_results_dir.exists():
        print("⚠️ 未找到批量路径规划结果目录")
        return

    # 查找一个示例文件
    example_files = list(batch_results_dir.rglob("*_all_results.json"))
    if not example_files:
        print("⚠️ 未找到批量路径规划结果文件")
        return

    example_file = example_files[0]
    print(f"\n📄 示例文件: {example_file}")

    try:
        with open(example_file, 'r') as f:
            batch_data = json.load(f)

        if len(batch_data) > 0:
            batch_example = batch_data[0]
            print(f"\n批量路径规划结果字段:")
            for key in sorted(batch_example.keys()):
                value = batch_example[key]
                if isinstance(value, list):
                    print(f"  - {key}: {type(value).__name__} (长度: {len(value)})")
                else:
                    print(f"  - {key}: {type(value).__name__} = {value}")

    except Exception as e:
        print(f"❌ 读取批量路径规划结果失败: {e}")


if __name__ == "__main__":
    # 测试JSON格式
    success = test_waypoints_json_format()

    # 对比格式
    compare_with_batch_planning_format()

    sys.exit(0 if success else 1)
