#!/usr/bin/env python3
"""
修复导航目标点的旋转角度

问题：
- 当前 rotation_z 存储的是物体的旋转角度（如 180.0, 90 等）
- 导航点的朝向应该是机器人面向物体的角度，而不是物体的旋转角度

修复方案：
- 对于导航点，机器人应该面向物体中心
- 计算从导航点指向物体中心的角度作为导航点的朝向
- 默认设置为 0 度（可根据需要修改）

使用方法：
    python utils/fix_nav_target_rotation.py --input outputs/auto_batch_nav_targets_test/
"""

import json
import math
import argparse
from pathlib import Path
from typing import Dict, List, Tuple


def calculate_nav_theta(nav_x: float, nav_y: float, obj_x: float, obj_y: float) -> float:
    """
    计算从导航点指向物体的角度（弧度）

    Args:
        nav_x, nav_y: 导航点坐标
        obj_x, obj_y: 物体中心坐标

    Returns:
        朝向角度（弧度），范围 [-π, π]
    """
    dx = obj_x - nav_x
    dy = obj_y - nav_y
    theta = math.atan2(dy, dx)
    return theta


def fix_json_file(json_path: Path, fix_mode: str = "face_object") -> bool:
    """
    修复 JSON 文件中的导航点旋转角度

    Args:
        json_path: JSON 文件路径
        fix_mode: 修复模式
            - "face_object": 导航点朝向物体中心（推荐）
            - "zero": 固定为 0 度

    Returns:
        是否成功修复
    """
    print(f"\n处理文件: {json_path.name}")

    try:
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        if "nav_targets" not in data:
            print(f"  ⚠️  跳过: 没有 nav_targets 字段")
            return False

        nav_targets = data["nav_targets"]
        fixed_count = 0

        for target in nav_targets:
            # 原始值（物体的旋转角度）
            old_rotation_z = target.get("rotation_z", 0.0)

            # 计算新的导航点朝向
            if fix_mode == "face_object":
                # 计算导航点朝向物体的角度
                nav_x = target["target_x"]
                nav_y = target["target_y"]
                obj_x = target["object_center_x"]
                obj_y = target["object_center_y"]

                theta_rad = calculate_nav_theta(nav_x, nav_y, obj_x, obj_y)
                theta_deg = math.degrees(theta_rad)

                # 保存新的旋转角度
                target["navigation_theta_deg"] = theta_deg
                target["navigation_theta_rad"] = theta_rad
                fixed_count += 1

                print(f"  步骤 {target['step_number']}: {target['location']}")
                print(f"    导航点: ({nav_x:.3f}, {nav_y:.3f})")
                print(f"    物体中心: ({obj_x:.3f}, {obj_y:.3f})")
                print(f"    物体旋转: {old_rotation_z:.1f}°")
                print(f"    导航朝向: {theta_deg:.1f}°")

            elif fix_mode == "zero":
                # 固定为 0 度
                target["navigation_theta_deg"] = 0.0
                target["navigation_theta_rad"] = 0.0
                fixed_count += 1

        # 保存修复后的文件
        output_path = json_path.parent / f"{json_path.stem}_fixed.json"
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

        print(f"\n  ✓ 修复完成: {fixed_count} 个导航点")
        print(f"  ✓ 保存到: {output_path.name}")

        return True

    except Exception as e:
        print(f"  ❌ 错误: {e}")
        return False


def fix_summary_report(report_path: Path, fix_mode: str = "face_object") -> bool:
    """
    修复摘要报告文件，添加导航点朝向信息

    Args:
        report_path: 报告文件路径
        fix_mode: 修复模式

    Returns:
        是否成功修复
    """
    print(f"\n处理报告: {report_path.name}")

    try:
        with open(report_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        # 读取对应的 JSON 文件以获取物体中心坐标
        json_files = {}
        if fix_mode == "face_object":
            # 查找所有相关的 JSON 文件
            for json_file in report_path.parent.glob("*_nav_targets.json"):
                try:
                    with open(json_file, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                        if "nav_targets" in data:
                            # 使用场景名称作为键
                            scene_name = data.get("config", {}).get("scene_name", "")
                            if scene_name:
                                json_files[scene_name] = data["nav_targets"]
                except:
                    pass

        fixed_lines = []
        in_nav_list = False
        current_scene = None

        for line in lines:
            # 检测场景名称（无论是否在导航点列表中）
            if "场景:" in line or "场景：" in line:
                # 提取场景名称（处理全角冒号）
                if "场景：" in line:
                    current_scene = line.strip().split("场景：")[1].strip()
                else:
                    current_scene = line.strip().split("场景:")[1].strip()
                fixed_lines.append(line)
                continue

            # 检测是否进入导航点列表
            if "导航点列表" in line:
                in_nav_list = True
                fixed_lines.append(line)
                continue

            # 检测是否离开导航点列表（遇到分隔线）
            if in_nav_list and line.strip().startswith("-"):
                in_nav_list = False
                current_scene = None
                fixed_lines.append(line)
                continue

            # 处理导航点行（支持中文全角冒号）
            if in_nav_list and "步骤" in line and (":" in line or "：" in line):
                # 保留原行
                fixed_lines.append(line)

                # 解析坐标并添加新行（支持全角冒号）
                import re
                match = re.search(r'步骤\s+(\d+)[：:]\s*\(([\d.]+),\s*([\d.]+)\)\s*-\s*(\S+)', line)
                if match:
                    step_num = int(match.group(1))
                    x = float(match.group(2))
                    y = float(match.group(3))
                    location = match.group(4)

                    # 根据模式生成新行
                    if fix_mode == "face_object":
                        # 从对应的 JSON 文件中获取物体中心坐标
                        theta_deg = 0.0
                        if current_scene and current_scene in json_files:
                            # 查找对应步骤的导航点
                            for target in json_files[current_scene]:
                                if target["step_number"] == step_num:
                                    # 计算朝向
                                    rotation_z = target.get("rotation_z", 0.0)
                                    # 计算 (rotation_z + 180) % 360，然后转换为弧度
                                    new_rotation_deg = (rotation_z + 180) % 360
                                    theta_rad = math.radians(new_rotation_deg)
                                    # 转换为度数用于显示
                                    theta_deg = math.degrees(theta_rad)
                                    break

                        new_line = f"      → 修正: ({x:.3f}, {y:.3f}, θ={theta_deg:.1f}°)\n"
                    else:  # zero
                        new_line = f"      → 修正: ({x:.3f}, {y:.3f}, 0.0°)\n"

                    fixed_lines.append(new_line)
            else:
                fixed_lines.append(line)

        # 保存修复后的文件
        output_path = report_path.parent / f"{report_path.stem}_fixed.txt"
        with open(output_path, 'w', encoding='utf-8') as f:
            f.writelines(fixed_lines)

        print(f"  ✓ 修复完成")
        print(f"  ✓ 保存到: {output_path.name}")

        return True

    except Exception as e:
        print(f"  ❌ 错误: {e}")
        return False


def process_directory(input_dir: Path, fix_mode: str = "face_object"):
    """
    处理目录中的所有文件

    Args:
        input_dir: 输入目录
        fix_mode: 修复模式
    """
    print("=" * 80)
    print("导航点旋转角度修复工具")
    print("=" * 80)
    print(f"输入目录: {input_dir}")
    print(f"修复模式: {fix_mode}")
    print(f"  - face_object: 导航点朝向物体中心")
    print(f"  - zero: 固定为 0 度")

    # 处理所有 JSON 文件
    json_files = list(input_dir.glob("*_nav_targets.json"))
    print(f"\n找到 {len(json_files)} 个 JSON 文件")

    for json_file in json_files:
        fix_json_file(json_file, fix_mode)

    # 处理摘要报告
    report_file = input_dir / "summary_report.txt"
    if report_file.exists():
        fix_summary_report(report_file, fix_mode)

    print("\n" + "=" * 80)
    print("✓ 所有文件处理完成")
    print("=" * 80)


def main():
    parser = argparse.ArgumentParser(
        description="修复导航目标点的旋转角度",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 修复所有文件，导航点朝向物体中心
  python utils/fix_nav_target_rotation.py --input outputs/auto_batch_nav_targets_test/

  # 修复所有文件，导航点固定朝向 0 度
  python utils/fix_nav_target_rotation.py --input outputs/auto_batch_nav_targets_test/ --mode zero

  # 处理单个文件
  python utils/fix_nav_target_rotation.py --input outputs/auto_batch_nav_targets_test/Alkylation*_nav_targets.json
        """
    )

    parser.add_argument(
        "--input",
        type=str,
        required=True,
        help="输入文件或目录路径"
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["face_object", "zero"],
        default="face_object",
        help="修复模式: face_object（朝向物体）或 zero（固定0度）"
    )

    args = parser.parse_args()

    input_path = Path(args.input)

    if not input_path.exists():
        print(f"❌ 错误: 路径不存在 {input_path}")
        return

    if input_path.is_file():
        # 处理单个文件
        if input_path.suffix == ".json":
            fix_json_file(input_path, args.mode)
        elif input_path.name == "summary_report.txt":
            fix_summary_report(input_path, args.mode)
        else:
            print(f"❌ 错误: 不支持的文件类型 {input_path.suffix}")
    elif input_path.is_dir():
        # 处理目录
        process_directory(input_path, args.mode)
    else:
        print(f"❌ 错误: 无法识别路径类型")


if __name__ == "__main__":
    main()
