#!/usr/bin/env python3
"""
提取导航目标点并生成 JSON/YAML 格式的 goal_pairs 文件
从 nav_goal_targets_demo_12_17.py 的输出中提取导航目标点（X, Y, theta）
并生成类似 level5_Navigation_smooth_12_23.yaml 中 goal_pairs 格式的输出
"""

import json
import math
import yaml
import argparse
from pathlib import Path

# 导入原有脚本的函数
import sys
sys.path.insert(0, str(Path(__file__).parent))
from nav_goal_targets_demo_12_17 import (
    _norm, load_json, build_location_to_platform,
    build_bbox_lookup, get_platform_info, calculate_nav_target,
    ROOT, PROTOCOL, ROOM_ASSETS, ASSET_LIB, OFFSET_RADIUS
)

def calculate_robot_theta(rz_deg):
    """
    根据物体旋转角度计算机器人朝向角度
    规则：270度对应的物体旋转 -> 机器人朝向90度（1.57弧度）
    转换公式：robot_theta_deg = (object_rz_deg + 180) % 360
    然后转换为弧度
    
    参数:
        rz_deg: 物体的旋转角度（度）
    返回:
        robot_theta: 机器人朝向角度（弧度）
    """
    # 物体旋转270度 -> 机器人朝向90度
    # 物体旋转0度 -> 机器人朝向180度
    # 物体旋转90度 -> 机器人朝向270度
    # 转换公式：robot_theta_deg = (object_rz_deg + 180) % 360
    robot_theta_deg = (rz_deg + 180) % 360
    robot_theta = math.radians(robot_theta_deg)
    return robot_theta

def extract_nav_goals(protocol_data, room_assets_data, asset_lib_data, offset_radius=0.6):
    """
    提取所有步骤的导航目标点
    
    返回:
        nav_points: 列表，每个元素包含 [x, y, theta]
    """
    # 构建查找表
    bbox_lookup = build_bbox_lookup(asset_lib_data)
    location_mapping = build_location_to_platform(protocol_data, room_assets_data)
    
    steps = protocol_data.get("procedure", [])
    nav_points = []
    
    for step in steps:
        step_idx = step.get("step_number")
        loc = step.get("location", "BENCH")
        
        # 获取匹配的房间对象 ID
        matched_id = location_mapping.get(loc)
        
        # 在布局中找到该对象实例
        target_instance = None
        for obj in room_assets_data.get("objects", []):
            if _norm(obj.get("id")) == _norm(matched_id):
                target_instance = obj
                break
        
        if target_instance:
            target = calculate_nav_target(target_instance, offset_radius, bbox_lookup)
            if target:
                info = target["info"]
                x = target["x"]
                y = target["y"]
                rz_deg = info["rz"]
                
                # 计算机器人朝向角度
                theta = calculate_robot_theta(rz_deg)
                
                nav_points.append({
                    "step": step_idx,
                    "location": loc,
                    "point": [x, y, theta],
                    "object_rz_deg": rz_deg,
                    "robot_theta_deg": math.degrees(theta)
                })
                print(f"步骤 {step_idx}: 位置={loc}, 导航点=[{x:.3f}, {y:.3f}, {theta:.3f}], "
                      f"物体旋转={rz_deg}°, 机器人朝向={math.degrees(theta):.1f}°")
            else:
                print(f"警告: 步骤 {step_idx} ({loc}) - 无法在资产库中找到该物体的几何尺寸。")
        else:
            print(f"错误: 步骤 {step_idx} ({loc}) - 无法在房间布局中定位该设备。")
    
    return nav_points

def generate_goal_pairs(nav_points):
    """
    生成连续的路径对（goal_pairs格式）
    
    参数:
        nav_points: 导航点列表，每个元素包含 "point": [x, y, theta]
    返回:
        goal_pairs: 路径对列表，格式为 [{"start": [x1,y1,theta1], "end": [x2,y2,theta2]}, ...]
    """
    goal_pairs = []
    
    if len(nav_points) < 2:
        print("警告: 导航点数量少于2个，无法生成路径对")
        return goal_pairs
    
    for i in range(len(nav_points) - 1):
        start_point = nav_points[i]["point"]
        end_point = nav_points[i + 1]["point"]
        
        goal_pairs.append({
            "start": start_point,
            "end": end_point
        })
        
        print(f"路径对 {i+1}: start={start_point}, end={end_point}")
    
    return goal_pairs

def main():
    parser = argparse.ArgumentParser(description='提取导航目标点并生成 JSON/YAML 格式')
    parser.add_argument('--protocol', type=str, default=str(PROTOCOL),
                       help='协议 JSON 文件路径')
    parser.add_argument('--room-assets', type=str, default=str(ROOM_ASSETS),
                       help='房间资产 JSON 文件路径')
    parser.add_argument('--asset-lib', type=str, default=str(ASSET_LIB),
                       help='资产库 JSON 文件路径')
    parser.add_argument('--offset-radius', type=float, default=OFFSET_RADIUS,
                       help='机器人半径（默认：0.6）')
    parser.add_argument('--output', type=str, default=None,
                       help='输出文件路径（默认：goal_pairs.json）')
    parser.add_argument('--format', type=str, choices=['json', 'yaml'], default='json',
                       help='输出格式（json 或 yaml）')
    
    args = parser.parse_args()
    
    # 加载数据
    protocol_data = load_json(Path(args.protocol))
    room_assets_data = load_json(Path(args.room_assets))
    asset_lib_data = load_json(Path(args.asset_lib))
    
    print("=" * 80)
    print("提取导航目标点")
    print("=" * 80)
    
    # 提取导航点
    nav_points = extract_nav_goals(protocol_data, room_assets_data, asset_lib_data, args.offset_radius)
    
    if not nav_points:
        print("错误: 未能提取到任何导航点")
        return
    
    print("\n" + "=" * 80)
    print("生成路径对")
    print("=" * 80)
    
    # 生成路径对
    goal_pairs = generate_goal_pairs(nav_points)
    
    if not goal_pairs:
        print("错误: 未能生成路径对")
        return
    
    # 准备输出数据
    output_data = {
        "goal_pairs": goal_pairs
    }
    
    # 确定输出文件路径
    if args.output:
        output_path = Path(args.output)
    else:
        output_path = Path(__file__).parent / f"goal_pairs.{args.format}"
    
    # 保存文件
    print("\n" + "=" * 80)
    print(f"保存到文件: {output_path}")
    print("=" * 80)
    
    if args.format == 'json':
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(output_data, f, indent=2, ensure_ascii=False)
    else:  # yaml
        with open(output_path, 'w', encoding='utf-8') as f:
            yaml.dump(output_data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    
    print(f"成功生成 {len(goal_pairs)} 个路径对")
    print(f"文件已保存到: {output_path}")

if __name__ == "__main__":
    main()