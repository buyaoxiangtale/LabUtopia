#!/usr/bin/env python3
"""运行碰撞分析 - 批量处理版本"""
import sys
import os
import re
from pathlib import Path
from typing import List, Tuple

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent))

from utils.collision_analyzer import CollisionAnalyzer


def find_scene_files(scene_name: str, layout_base_dir: str, assets_dir: str) -> Tuple[str, str]:
    """
    查找场景文件和assets文件

    Args:
        scene_name: 场景名称（例如：Hydrolysis_of_Nitrile_to_Amide_20260112_202101）
        layout_base_dir: layout基础目录
        assets_dir: assets目录

    Returns:
        (scene_json_path, assets_json_path)
    """
    # 去掉日期后缀，找到基础场景名
    # 例如: Hydrolysis_of_Nitrile_to_Amide_20260112_202101 -> Hydrolysis_of_Nitrile_to_Amide
    base_name = re.sub(r'_20\d{6}_\d{6}$', '', scene_name)

    # 查找assets文件
    assets_json = Path(assets_dir) / "assets_annotated.json"
    if not assets_json.exists():
        return None, None

    # 尝试在多个位置查找场景文件
    search_paths = [
        Path(layout_base_dir),  # layout_new目录
        Path(assets_dir),        # gemini-3-flash-preview目录
    ]

    scene_json = None
    for search_path in search_paths:
        # 匹配所有可能的日期后缀
        scene_dirs = list(search_path.glob(f"*{base_name}_20*"))

        if scene_dirs:
            # 使用第一个匹配的目录
            scene_dir = scene_dirs[0]
            scene_json = scene_dir / f"{base_name}_room_isaacsim.json"

            if scene_json.exists():
                return str(scene_json), str(assets_json)

    return None, None


def batch_process_collision_analysis(
    results_base_dir: str,
    layout_base_dir: str,
    assets_dir: str,
    output_base_dir: str,
    offset_radius: float = 0.3
):
    """
    批量处理所有失败任务的碰撞分析

    Args:
        results_base_dir: 批量结果基础目录
        layout_base_dir: layout基础目录
        assets_dir: assets目录
        output_base_dir: 输出基础目录
        offset_radius: 膨胀半径
    """
    results_path = Path(results_base_dir)
    output_path = Path(output_base_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    # 查找所有失败任务文件
    failed_task_files = list(results_path.rglob("*_failed_tasks.json"))

    if not failed_task_files:
        print(f"未找到任何失败任务文件")
        return

    print("="*80)
    print("批量碰撞分析工具")
    print("="*80)
    print(f"结果目录: {results_base_dir}")
    print(f"Layout目录: {layout_base_dir}")
    print(f"Assets目录: {assets_dir}")
    print(f"输出目录: {output_base_dir}")
    print(f"膨胀半径: {offset_radius} 米")
    print(f"找到失败任务文件: {len(failed_task_files)} 个")
    print("="*80)

    success_count = 0
    failed_count = 0
    no_failed_tasks_count = 0

    for failed_tasks_json in failed_task_files:
        print(f"\n{'-'*80}")
        scene_name = failed_tasks_json.stem.replace("_failed_tasks", "")
        print(f"处理场景: {scene_name}")

        # 查找场景文件和assets文件
        scene_json, assets_json = find_scene_files(scene_name, layout_base_dir, assets_dir)

        if not scene_json or not assets_json:
            print(f"  ✗ 无法找到场景文件或assets文件")
            failed_count += 1
            continue

        print(f"  场景文件: {Path(scene_json).parent.name}")
        print(f"  Assets文件: {Path(assets_json).name}")

        # 检查失败任务文件是否为空
        import json
        with open(failed_tasks_json, 'r', encoding='utf-8') as f:
            failed_tasks = json.load(f)

        if len(failed_tasks) == 0:
            print(f"  ℹ 没有失败任务，跳过")
            no_failed_tasks_count += 1
            continue

        print(f"  失败任务数: {len(failed_tasks)}")

        try:
            # 创建分析器
            analyzer = CollisionAnalyzer(scene_json, assets_json, offset_radius)

            # 创建场景输出目录
            scene_output_dir = output_path / scene_name
            scene_output_dir.mkdir(parents=True, exist_ok=True)

            # 分析失败任务
            analysis = analyzer.analyze_failed_task(str(failed_tasks_json))

            # 生成文本报告
            report_file = scene_output_dir / "diagnostic_report.txt"
            with open(report_file, 'w', encoding='utf-8') as f:
                f.write("="*80 + "\n")
                f.write(f"场景: {scene_name}\n")
                f.write("="*80 + "\n\n")

                for task in analysis['failed_tasks']:
                    f.write(f"\n任务: {task['task_id']}\n")
                    f.write(f"  起点: ({task['start_position'][0]:.3f}, {task['start_position'][1]:.3f})\n")
                    f.write(f"  终点: ({task['end_position'][0]:.3f}, {task['end_position'][1]:.3f})\n")
                    f.write(f"  失败原因: {task['failure_reason']}\n\n")

                    report = task['collision_report']

                    # 起点阻隔分析
                    if report['start_blocked']:
                        f.write(f"  ✗ 起点被阻隔！\n")
                        for obj_info in report['blocking_objects']['start']:
                            distance = obj_info['distance']
                            distance_text = f"{abs(distance):.2f}m (穿透)" if distance < 0 else f"{distance:.2f}m"
                            f.write(f"    • 阻隔物体: {obj_info['id']}\n")
                            f.write(f"      - 物体中心: ({obj_info['position'][0]:.2f}, {obj_info['position'][1]:.2f})\n")
                            if 'bbox_edge' in obj_info:
                                f.write(f"      - 膨胀边界框: {obj_info['bbox_edge']}\n")
                            f.write(f"      - 距离边界框边缘: {distance_text}\n")
                            if 'category' in obj_info:
                                f.write(f"      - 类型: {obj_info['category']}\n")
                            if 'size' in obj_info:
                                size = obj_info['size']
                                f.write(f"      - 原始尺寸: 长={size['long']:.2f}m, 宽={size['short']:.2f}m\n")
                    else:
                        f.write(f"  ✓ 起点畅通\n")

                    # 终点阻隔分析
                    if report['end_blocked']:
                        f.write(f"  ✗ 终点被阻隔！\n")
                        for obj_info in report['blocking_objects']['end']:
                            distance = obj_info['distance']
                            distance_text = f"{abs(distance):.2f}m (穿透)" if distance < 0 else f"{distance:.2f}m"
                            f.write(f"    • 阻隔物体: {obj_info['id']}\n")
                            f.write(f"      - 物体中心: ({obj_info['position'][0]:.2f}, {obj_info['position'][1]:.2f})\n")
                            if 'bbox_edge' in obj_info:
                                f.write(f"      - 膨胀边界框: {obj_info['bbox_edge']}\n")
                            f.write(f"      - 距离边界框边缘: {distance_text}\n")
                            if 'category' in obj_info:
                                f.write(f"      - 类型: {obj_info['category']}\n")
                            if 'size' in obj_info:
                                size = obj_info['size']
                                f.write(f"      - 原始尺寸: 长={size['long']:.2f}m, 宽={size['short']:.2f}m\n")
                    else:
                        f.write(f"  ✓ 终点畅通\n")

                    f.write("\n")

                f.write("\n生成可视化...\n")

            # 生成可视化图像
            output_image = scene_output_dir / "collision_analysis.png"
            analyzer.visualize_collisions(str(failed_tasks_json), None, str(output_image))

            print(f"  ✓ 分析完成")
            print(f"    报告: {report_file}")
            print(f"    可视化: {output_image}")
            success_count += 1

        except Exception as e:
            print(f"  ✗ 分析失败: {e}")
            import traceback
            traceback.print_exc()
            failed_count += 1

    # 打印汇总
    print("\n" + "="*80)
    print("批量处理完成")
    print("="*80)
    print(f"成功处理: {success_count} 个场景")
    print(f"处理失败: {failed_count} 个场景")
    print(f"无失败任务: {no_failed_tasks_count} 个场景")
    print(f"总计: {len(failed_task_files)} 个场景")
    print(f"输出目录: {output_base_dir}")


if __name__ == "__main__":
    # 配置
    RESULTS_BASE_DIR = "/home/pjlab/fbh/LabUtopia/outputs/path_planning_batch_results_gemini_flash/run_2026-01-14_14-00-11"
    LAYOUT_BASE_DIR = "/home/pjlab/fbh/LabUtopia/roomlayout/layout_new"
    ASSETS_DIR = "/home/pjlab/fbh/LabUtopia/gemini-3-flash-preview"
    OUTPUT_BASE_DIR = "/home/pjlab/fbh/LabUtopia/outputs/path_planning_batch_results_gemini_flash/run_2026-01-14_14-00-11/collision_analysis"
    OFFSET_RADIUS = 0.3

    # 批量处理
    batch_process_collision_analysis(
        results_base_dir=RESULTS_BASE_DIR,
        layout_base_dir=LAYOUT_BASE_DIR,
        assets_dir=ASSETS_DIR,
        output_base_dir=OUTPUT_BASE_DIR,
        offset_radius=OFFSET_RADIUS
    )
