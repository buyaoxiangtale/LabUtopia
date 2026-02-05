#!/usr/bin/env python3
"""
Waypoints JSON保存工具

用于将导航任务的waypoints保存为JSON格式的文件，与批量路径规划结果保持一致的格式。

保存格式：
{
    "task_id": "episode_0000",
    "start": [x, y],
    "end": [x, y],
    "is_success": true,
    "waypoints": [[x, y, theta], ...],
    "total_distance": 0.0,
    "num_waypoints": 0
}
"""

import json
import numpy as np
from pathlib import Path
from typing import List, Dict, Any, Optional
from datetime import datetime


def calculate_path_distance(waypoints: List[List[float]]) -> float:
    """
    计算路径总长度

    Args:
        waypoints: 路径点列表 [[x, y, theta], ...]

    Returns:
        float: 路径总长度（米）
    """
    if len(waypoints) < 2:
        return 0.0

    total_distance = 0.0
    for i in range(len(waypoints) - 1):
        x1, y1, _ = waypoints[i]
        x2, y2, _ = waypoints[i + 1]
        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        total_distance += distance

    return total_distance


class WaypointsJSONSaver:
    """Waypoints JSON保存器"""

    def __init__(self, output_dir: str):
        """
        初始化保存器

        Args:
            output_dir: 输出目录路径
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # 创建waypoints子目录
        self.waypoints_dir = self.output_dir / "waypoints"
        self.waypoints_dir.mkdir(parents=True, exist_ok=True)

        self.episode_count = 0

    def save_waypoints(
        self,
        waypoints: List[List[float]],
        start: List[float],
        end: List[float],
        episode_num: Optional[int] = None,
        is_success: bool = True,
        metadata: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        保存waypoints到JSON文件

        Args:
            waypoints: 路径点列表 [[x, y, theta], ...]
            start: 起点坐标 [x, y] 或 [x, y, z]
            end: 终点坐标 [x, y] 或 [x, y, z]
            episode_num: episode编号，如果为None则使用内部计数器
            is_success: 是否成功
            metadata: 额外的元数据信息

        Returns:
            str: 保存的文件路径
        """
        # 确定episode编号
        if episode_num is None:
            episode_num = self.episode_count
            self.episode_count += 1

        # 提取起点和终点（只取x, y）
        start_xy = [float(start[0]), float(start[1])]
        end_xy = [float(end[0]), float(end[1])]

        # 计算路径总长度
        total_distance = calculate_path_distance(waypoints)

        # 构建结果字典（与批量路径规划结果格式一致）
        result = {
            "task_id": f"episode_{episode_num:04d}",
            "start": start_xy,
            "end": end_xy,
            "is_success": is_success,
            "waypoints": [[float(x), float(y), float(theta)] for x, y, theta in waypoints],
            "total_distance": round(total_distance, 6),
            "num_waypoints": len(waypoints)
        }

        # 添加元数据（如果有）
        if metadata:
            result["metadata"] = metadata

        # 保存到JSON文件
        output_filename = f"episode_{episode_num:04d}_waypoints.json"
        output_path = self.waypoints_dir / output_filename

        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(result, f, indent=2, ensure_ascii=False)

        print(f"✓ Waypoints已保存到JSON: {output_path}")
        print(f"  起点: {start_xy}")
        print(f"  终点: {end_xy}")
        print(f"  路径点数量: {len(waypoints)}")
        print(f"  路径总长度: {total_distance:.3f} 米")

        return str(output_path)

    def save_failed_waypoints(
        self,
        start: List[float],
        end: List[float],
        failure_reason: str,
        episode_num: Optional[int] = None,
        metadata: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        保存失败的waypoints信息

        Args:
            start: 起点坐标 [x, y] 或 [x, y, z]
            end: 终点坐标 [x, y] 或 [x, y, z]
            failure_reason: 失败原因
            episode_num: episode编号
            metadata: 额外的元数据信息

        Returns:
            str: 保存的文件路径
        """
        # 确定episode编号
        if episode_num is None:
            episode_num = self.episode_count
            self.episode_count += 1

        # 提取起点和终点
        start_xy = [float(start[0]), float(start[1])]
        end_xy = [float(end[0]), float(end[1])]

        # 构建失败结果字典
        result = {
            "task_id": f"episode_{episode_num:04d}",
            "start": start_xy,
            "end": end_xy,
            "is_success": False,
            "failure_reason": failure_reason,
            "waypoints": [],
            "total_distance": 0.0,
            "num_waypoints": 0
        }

        # 添加元数据（如果有）
        if metadata:
            result["metadata"] = metadata

        # 保存到JSON文件
        output_filename = f"episode_{episode_num:04d}_waypoints.json"
        output_path = self.waypoints_dir / output_filename

        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(result, f, indent=2, ensure_ascii=False)

        print(f"✗ 失败信息已保存到JSON: {output_path}")
        print(f"  起点: {start_xy}")
        print(f"  终点: {end_xy}")
        print(f"  失败原因: {failure_reason}")

        return str(output_path)

    def merge_all_waypoints(self, output_filename: str = "all_waypoints.json") -> str:
        """
        将所有单独的waypoints文件合并为一个总的JSON文件

        Args:
            output_filename: 输出文件名

        Returns:
            str: 合并后的文件路径
        """
        all_results = []

        # 读取所有waypoints JSON文件
        for json_file in sorted(self.waypoints_dir.glob("episode_*_waypoints.json")):
            with open(json_file, 'r', encoding='utf-8') as f:
                result = json.load(f)
                all_results.append(result)

        # 保存合并后的文件
        output_path = self.output_dir / output_filename
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(all_results, f, indent=2, ensure_ascii=False)

        print(f"✓ 所有waypoints已合并: {output_path}")
        print(f"  总共 {len(all_results)} 个episodes")

        return str(output_path)


# 便捷函数
def save_waypoints_to_json(
    waypoints: List[List[float]],
    start: List[float],
    end: List[float],
    output_dir: str,
    episode_num: int = 0,
    is_success: bool = True
) -> str:
    """
    便捷函数：保存waypoints到JSON文件

    Args:
        waypoints: 路径点列表 [[x, y, theta], ...]
        start: 起点坐标 [x, y] 或 [x, y, z]
        end: 终点坐标 [x, y] 或 [x, y, z]
        output_dir: 输出目录
        episode_num: episode编号
        is_success: 是否成功

    Returns:
        str: 保存的文件路径
    """
    saver = WaypointsJSONSaver(output_dir)
    return saver.save_waypoints(waypoints, start, end, episode_num, is_success)


if __name__ == "__main__":
    # 测试代码
    import tempfile

    # 创建临时目录
    with tempfile.TemporaryDirectory() as tmpdir:
        print(f"测试WaypointsJSONSaver...")

        # 创建保存器
        saver = WaypointsJSONSaver(tmpdir)

        # 测试数据
        test_waypoints = [
            [1.0, 2.0, 0.0],
            [1.5, 2.5, 0.78],
            [2.0, 3.0, 0.78],
            [2.5, 3.5, 1.57]
        ]
        test_start = [1.0, 2.0, 0.0]
        test_end = [2.5, 3.5, 1.57]

        # 保存waypoints
        path = saver.save_waypoints(
            waypoints=test_waypoints,
            start=test_start,
            end=test_end,
            episode_num=0,
            is_success=True
        )

        print(f"\n保存路径: {path}")

        # 读取并显示保存的内容
        with open(path, 'r') as f:
            saved_data = json.load(f)
            print(f"\n保存的内容:")
            print(json.dumps(saved_data, indent=2, ensure_ascii=False))

        # 测试合并功能
        merged_path = saver.merge_all_waypoints()
        print(f"\n合并路径: {merged_path}")
