#!/usr/bin/env python3
"""
自动化场景导航点批量生成工具

功能特点：
1. 自动扫描指定目录下的所有实验场景
2. 自动识别每个场景的关键文件（room_isaacsim.json 和 protocol*.json）
3. 使用共享的 assets_annotated.json
4. 批量生成所有场景的导航目标点
5. 自动转换为 goal_pairs 格式
6. 去除重复的导航点
7. 每个场景单独输出 goal_pairs YAML 文件
8. 生成详细的统计报告

使用场景：
- layout_new/ 目录结构（每个实验有独立的子目录）
- 其他类似的目录结构

输出格式：
- goal_pairs YAML 文件（主要输出）
- nav_targets JSON 文件（调试用）
- 统计报告（summary_report.txt）
"""

import json
import math
import re
import yaml
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, asdict


@dataclass
class SceneConfig:
    """场景配置"""
    scene_name: str           # 场景名称
    scene_dir: Path           # 场景目录（包含所有文件）
    room_assets_file: str     # 房间资产文件名
    protocol_file: str        # 协议文件名
    asset_lib_file: Path      # 资产库文件路径（共享）
    offset_radius: float = 0.6  # 机器人半径


@dataclass
class NavTarget:
    """导航目标点"""
    step_number: int
    location: str
    matched_object: str
    target_x: float
    target_y: float
    offset_dx: float
    offset_dy: float
    bbox_width: float
    bbox_depth: float
    object_center_x: float
    object_center_y: float
    rotation_z: float


class AutoSceneScanner:
    """自动场景扫描器"""

    def __init__(self, root_dir: Path, verbose: bool = True):
        """
        初始化扫描器

        Args:
            root_dir: 根目录（如 layout_new/）
            verbose: 是否显示详细输出
        """
        self.root_dir = Path(root_dir)
        self.verbose = verbose
        self.scenes = []

    def _print(self, msg: str):
        """打印输出"""
        if self.verbose:
            print(msg)

    def find_shared_asset_lib(self) -> Optional[Path]:
        """
        查找共享的资产库文件
        查找 assets_annotated.json
        """
        asset_lib_path = self.root_dir / "assets_annotated.json"

        if asset_lib_path.exists():
            self._print(f"✓ 找到共享资产库: {asset_lib_path}")
            return asset_lib_path

        self._print(f"⚠️  警告: 未找到共享资产库 assets_annotated.json")
        return None

    def is_scene_directory(self, dir_path: Path) -> bool:
        """
        判断是否为场景目录
        场景目录应包含 _room_isaacsim.json 和 protocol*.json
        """
        if not dir_path.is_dir():
            return False

        # 查找 room_isaacsim.json
        room_files = list(dir_path.glob("*_room_isaacsim.json"))
        if not room_files:
            return False

        # 查找 protocol*.json
        protocol_files = list(dir_path.glob("protocol*.json"))
        if not protocol_files:
            return False

        return True

    def identify_scene_files(self, scene_dir: Path) -> Optional[Dict[str, str]]:
        """
        识别场景目录中的关键文件
        返回: {"room_assets_file": ..., "protocol_file": ..., "scene_name": ...}
        """
        # 查找 room_isaacsim.json（应该只有一个）
        room_files = list(scene_dir.glob("*_room_isaacsim.json"))
        if len(room_files) != 1:
            self._print(f"  ⚠️  警告: {scene_dir.name} 中找到 {len(room_files)} 个 room_isaacsim.json")
            if room_files:
                self._print(f"     使用: {room_files[0].name}")
            else:
                return None

        # 查找 protocol*.json（应该只有一个）
        protocol_files = list(scene_dir.glob("protocol*.json"))
        if len(protocol_files) != 1:
            self._print(f"  ⚠️  警告: {scene_dir.name} 中找到 {len(protocol_files)} 个 protocol*.json")
            if protocol_files:
                self._print(f"     使用: {protocol_files[0].name}")
            else:
                return None

        # 生成场景名称（从目录名提取）
        scene_name = scene_dir.name

        return {
            "room_assets_file": room_files[0].name,
            "protocol_file": protocol_files[0].name,
            "scene_name": scene_name
        }

    def scan_all_scenes(self) -> List[SceneConfig]:
        """
        扫描所有场景目录
        返回场景配置列表
        """
        print(f"\n{'='*80}")
        print(f"自动扫描场景目录")
        print(f"{'='*80}")
        print(f"根目录: {self.root_dir}\n")

        # 1. 查找共享资产库
        asset_lib_path = self.find_shared_asset_lib()
        if not asset_lib_path:
            print("❌ 错误: 未找到共享资产库，无法继续")
            return []

        # 2. 扫描所有子目录
        subdirs = [d for d in self.root_dir.iterdir() if d.is_dir() and not d.name.startswith('.')]
        print(f"找到 {len(subdirs)} 个子目录\n")

        # 3. 识别场景目录
        scene_configs = []

        for subdir in sorted(subdirs):
            self._print(f"检查: {subdir.name}")

            if not self.is_scene_directory(subdir):
                self._print(f"  跳过（不是场景目录）")
                continue

            # 识别关键文件
            files_info = self.identify_scene_files(subdir)

            if not files_info:
                self._print(f"  跳过（无法识别关键文件）")
                continue

            # 创建场景配置
            scene_config = SceneConfig(
                scene_name=files_info["scene_name"],
                scene_dir=subdir,
                room_assets_file=files_info["room_assets_file"],
                protocol_file=files_info["protocol_file"],
                asset_lib_file=asset_lib_path,
                offset_radius=0.6
            )

            scene_configs.append(scene_config)
            self._print(f"  ✓ 识别为场景: {files_info['scene_name']}")
            self._print(f"    - 房间文件: {files_info['room_assets_file']}")
            self._print(f"    - 协议文件: {files_info['protocol_file']}")

        print(f"\n{'='*80}")
        print(f"扫描完成: 共识别 {len(scene_configs)} 个场景")
        print(f"{'='*80}\n")

        self.scenes = scene_configs
        return scene_configs


def calculate_robot_theta(rotation_z_deg: float) -> float:
    """
    根据物体旋转角度计算机器人朝向角度（弧度）

    规则：
    - 物体旋转270度 -> 机器人朝向90度
    - 物体旋转0度 -> 机器人朝向180度
    - 物体旋转90度 -> 机器人朝向270度

    转换公式：robot_theta_deg = (object_rz_deg + 180) % 360

    Args:
        rotation_z_deg: 物体的旋转角度（度）

    Returns:
        机器人朝向角度（弧度）
    """
    robot_theta_deg = (rotation_z_deg + 180) % 360
    robot_theta_rad = math.radians(robot_theta_deg)
    return robot_theta_rad


def convert_nav_targets_to_goal_pairs(nav_targets: List[NavTarget]) -> List[Dict]:
    """
    将导航目标点列表转换为 goal_pairs 格式

    Args:
        nav_targets: 导航目标点列表

    Returns:
        goal_pairs 列表
    """
    goal_pairs = []

    for i in range(len(nav_targets) - 1):
        current = nav_targets[i]
        next_target = nav_targets[i + 1]

        # 计算机器人朝向角度
        current_theta = calculate_robot_theta(current.rotation_z)
        next_theta = calculate_robot_theta(next_target.rotation_z)

        goal_pairs.append({
            "start": [current.target_x, current.target_y, current_theta],
            "end": [next_target.target_x, next_target.target_y, next_theta]
        })

    return goal_pairs


def deduplicate_goal_pairs(goal_pairs: List[Dict], tolerance: float = 0.001) -> List[Dict]:
    """
    去除无效的 goal_pairs（起点和终点相同的路径段）

    保留所有起点和终点不同的 goal_pairs，即使它们是连续的

    Args:
        goal_pairs: goal_pairs 列表
        tolerance: 坐标容差（用于判断是否相同）

    Returns:
        去重后的 goal_pairs 列表
    """
    if not goal_pairs:
        return []

    deduplicated = []

    for current in goal_pairs:
        curr_start = current["start"]
        curr_end = current["end"]

        # 检查 start 和 end 是否相同（无效路径段）
        start_end_same = (
            abs(curr_start[0] - curr_end[0]) < tolerance and
            abs(curr_start[1] - curr_end[1]) < tolerance and
            abs(curr_start[2] - curr_end[2]) < tolerance
        )

        # 只保留起点和终点不同的路径段
        if not start_end_same:
            deduplicated.append(current)
        # else: start == end，跳过这个无效的 goal_pair

    return deduplicated


class MultiSceneNavGenerator:
    """多场景导航点生成器"""

    def __init__(self, scenes: List[SceneConfig], verbose: bool = True):
        """
        初始化生成器

        Args:
            scenes: 场景配置列表
            verbose: 是否显示详细输出
        """
        self.scenes = scenes
        self.verbose = verbose
        self.results = {}

    def load_json(self, path: Path) -> dict:
        """加载 JSON 文件"""
        if not path.exists():
            self._print(f"❌ 错误: 找不到文件 {path}")
            return {}
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)

    def _print(self, msg: str, scene_name: str = ""):
        """打印输出"""
        if self.verbose:
            if scene_name:
                print(f"[{scene_name}] {msg}")
            else:
                print(msg)

    @staticmethod
    def _norm(s: str) -> str:
        """规范化字符串：去除下划线/空格/横线，转小写"""
        if not s:
            return ""
        return str(s).lower().replace(" ", "").replace("-", "").replace("_", "")

    def build_bbox_lookup(self, asset_lib: dict) -> Dict[str, Tuple[float, float]]:
        """
        从 assets_annotated.json 构建 bbox 查找表
        返回: normalized_name -> (half_x, half_y)
        """
        lookup = {}
        assets_list = asset_lib.get("assets", [])

        for a in assets_list:
            asset_id = a.get("id", "")
            name = a.get("name", "")

            # 从 geometry.bbox 获取尺寸
            geometry = a.get("geometry", {})
            bbox = geometry.get("bbox", {})

            if not bbox:
                continue

            short_side = bbox.get("short", 0)
            long_side = bbox.get("long", 0)

            # half_x 使用 short/2, half_y 使用 long/2
            half_tuple = (short_side / 2.0, long_side / 2.0)

            if asset_id:
                lookup[self._norm(asset_id)] = half_tuple
            if name:
                lookup[self._norm(name)] = half_tuple

        return lookup

    def find_object_by_id(self, obj_id: str, objects: list) -> Optional[dict]:
        """在 objects 列表中查找指定 id 的对象"""
        norm_id = self._norm(obj_id)

        # 精确匹配
        for obj in objects:
            obj_id_real = obj.get("id", "")
            if self._norm(obj_id_real) == norm_id:
                return obj

        # 模糊匹配
        for obj in objects:
            obj_id_real = obj.get("id", "")
            if self._norm(obj_id_real).startswith(norm_id) or norm_id.startswith(self._norm(obj_id_real)):
                return obj

        return None

    def get_bbox_area(self, half_x: float, half_y: float) -> float:
        """计算 bbox 面积"""
        return (half_x * 2) * (half_y * 2)

    def get_object_bbox(self, obj: dict, bbox_lookup: dict) -> Optional[Tuple[float, float]]:
        """获取对象的 bbox (half_x, half_y)"""
        obj_id = obj.get("id") or obj.get("assetId", "")
        if not obj_id:
            return None

        # 先尝试精确匹配
        norm_id = self._norm(obj_id)
        bbox = bbox_lookup.get(norm_id)

        if bbox:
            return bbox

        # 尝试前缀匹配
        for key, value in bbox_lookup.items():
            if key.startswith(norm_id) or norm_id.startswith(key):
                return value

        return None

    def process_nonfloor_objects(
        self,
        room_assets: dict,
        asset_lib: dict,
        scene_name: str
    ) -> dict:
        """
        处理所有 initial_location 不在 floor 上的对象
        返回映射关系: 原对象id -> 更大的对象id
        """
        objects = room_assets.get("objects", [])
        bbox_lookup = self.build_bbox_lookup(asset_lib)

        result = {}
        processed_count = 0

        for obj in objects:
            obj_id = obj.get("id", "")
            initial_loc = obj.get("initial_location", "")

            # 只处理不在 floor 上的对象
            if not initial_loc or initial_loc.lower() == "floor":
                continue

            # 查找父对象
            parent_obj = self.find_object_by_id(initial_loc, objects)

            if not parent_obj:
                self._print(f"  ⚠️  警告: 找不到父对象 '{initial_loc}' (对于对象 '{obj_id}')", scene_name)
                result[obj_id] = obj_id
                continue

            # 获取两个对象的 bbox
            child_bbox = self.get_object_bbox(obj, bbox_lookup)
            parent_bbox = self.get_object_bbox(parent_obj, bbox_lookup)

            if not child_bbox or not parent_bbox:
                self._print(f"  ⚠️  警告: 无法获取 bbox (对象 '{obj_id}' 或 父对象 '{initial_loc}')", scene_name)
                result[obj_id] = obj_id
                continue

            # 计算面积
            child_area = self.get_bbox_area(child_bbox[0], child_bbox[1])
            parent_area = self.get_bbox_area(parent_bbox[0], parent_bbox[1])

            # 选择更大的
            if parent_area >= child_area:
                larger_id = parent_obj.get("id", "")
                result[obj_id] = larger_id
                self._print(f"  ✓ {obj_id} (面积: {child_area:.4f}) -> {larger_id} (面积: {parent_area:.4f}) [使用父对象]", scene_name)
            else:
                result[obj_id] = obj_id
                self._print(f"  ✓ {obj_id} (面积: {child_area:.4f}) [使用原对象，父对象面积: {parent_area:.4f}]", scene_name)

            processed_count += 1

        self._print(f"\n处理完成: 共处理 {processed_count} 个非 floor 对象", scene_name)
        return result

    def build_location_to_platform(
        self,
        protocol_data: dict,
        room_assets: dict,
        processed_mapping: dict,
        scene_name: str
    ) -> dict:
        """
        构建 location -> platform 映射
        使用处理后的映射更新原始映射
        """
        # 特殊映射
        special_mapping = {
            "labbench": "ExperimentalPlatform",
            "bench": "ExperimentalPlatform",
            "hood": "FumeHood",
            "validation_platform": "ValidationPlatform"
        }

        locations = set()
        for step in protocol_data.get("procedure", []):
            loc = step.get("location")
            if loc:
                locations.add(loc)

        platform_ids = [obj.get("id", "") for obj in room_assets.get("objects", [])]

        result = {}
        for loc in locations:
            norm_loc = self._norm(loc)

            # 1. 先检查特殊映射
            if norm_loc in special_mapping:
                result[loc] = special_mapping[norm_loc]
                continue

            # 2. 规范化匹配
            matched = None
            for pid in platform_ids:
                if self._norm(pid) == norm_loc:
                    matched = pid
                    break

            # 3. 模糊匹配
            if not matched:
                for pid in platform_ids:
                    if self._norm(pid).startswith(norm_loc) or norm_loc.startswith(self._norm(pid)):
                        matched = pid
                        break

            result[loc] = matched if matched else loc

        # 使用 processed_mapping 更新
        updated = {}
        for protocol_loc, platform_id in result.items():
            new_platform_id = processed_mapping.get(platform_id, platform_id)
            updated[protocol_loc] = new_platform_id

            if new_platform_id != platform_id:
                self._print(f"  更新映射: {protocol_loc}: {platform_id} -> {new_platform_id}", scene_name)

        return updated

    def get_platform_info(
        self,
        platform: dict,
        bbox_lookup: dict
    ) -> Optional[dict]:
        """获取物体的中心、旋转、半尺寸及完整尺寸"""
        pos = platform.get("position", {})
        ox, oy = pos.get("x"), pos.get("y")

        obj_id = platform.get("id") or platform.get("assetId")
        half = bbox_lookup.get(self._norm(obj_id))

        if ox is None or oy is None or not half:
            return None

        hx, hy = half
        return {
            "center": (ox, oy),
            "half_x": hx,
            "half_y": hy,
            "width": hx * 2,
            "depth": hy * 2,
            "rz": platform.get("rotation", {}).get("z", 0)
        }

    def calculate_nav_target(
        self,
        platform: dict,
        offset_radius: float,
        bbox_lookup: dict
    ) -> Optional[dict]:
        """
        计算导航目标点
        基于 rotation.z 和 bbox 计算机器人的停靠位置
        """
        info = self.get_platform_info(platform, bbox_lookup)
        if not info:
            return None

        cx, cy = info["center"]
        hy = info["half_y"]
        rz_deg = info["rz"]

        # 机器人停靠距离
        dist = hy + offset_radius
        rad = math.radians(rz_deg)

        # 投影计算 (0deg=+Y, 270deg=+X)
        dx_raw = -dist * math.sin(rad)
        dy_raw = dist * math.cos(rad)

        tx_raw = cx + dx_raw
        ty_raw = cy + dy_raw

        # 向上取整逻辑
        precision = 1000.0
        tx = math.ceil(tx_raw * precision) / precision
        ty = math.ceil(ty_raw * precision) / precision

        dx = math.ceil(dx_raw * precision) / precision
        dy = math.ceil(dy_raw * precision) / precision

        return {
            "x": tx,
            "y": ty,
            "dx": dx,
            "dy": dy,
            "info": info
        }

    def process_single_scene(self, scene: SceneConfig) -> List[NavTarget]:
        """处理单个场景"""
        scene_name = scene.scene_name
        self._print(f"\n{'='*80}", scene_name)
        self._print(f"开始处理场景: {scene_name}", scene_name)
        self._print(f"{'='*80}", scene_name)

        # 1. 加载文件
        room_assets_path = scene.scene_dir / scene.room_assets_file
        protocol_path = scene.scene_dir / scene.protocol_file
        asset_lib_path = scene.asset_lib_file

        room_assets = self.load_json(room_assets_path)
        protocol_data = self.load_json(protocol_path)
        asset_lib = self.load_json(asset_lib_path)

        if not room_assets or not protocol_data or not asset_lib:
            self._print(f"❌ 错误: 无法加载必要的数据文件", scene_name)
            return []

        self._print(f"✓ 已加载 room_assets: {room_assets_path.name}", scene_name)
        self._print(f"✓ 已加载 protocol: {protocol_path.name}", scene_name)
        self._print(f"✓ 已加载 asset_lib: {asset_lib_path.name}", scene_name)
        self._print(f"✓ 机器人半径: {scene.offset_radius}m\n", scene_name)

        # 2. 处理非 floor 对象
        self._print(f"步骤 1/3: 处理非 floor 对象...", scene_name)
        self._print(f"{'-'*80}", scene_name)
        processed_mapping = self.process_nonfloor_objects(room_assets, asset_lib, scene_name)

        # 3. 构建 location 映射
        self._print(f"\n步骤 2/3: 构建 location 映射...", scene_name)
        self._print(f"{'-'*80}", scene_name)
        location_mapping = self.build_location_to_platform(
            protocol_data,
            room_assets,
            processed_mapping,
            scene_name
        )

        # 4. 生成导航目标点
        self._print(f"\n步骤 3/3: 生成导航目标点...", scene_name)
        self._print(f"{'-'*80}", scene_name)

        bbox_lookup = self.build_bbox_lookup(asset_lib)
        nav_targets = []

        steps = protocol_data.get("procedure", [])

        for step in steps:
            step_idx = step.get("step_number")
            loc = step.get("location", "BENCH")

            # 获取匹配的房间对象 ID
            matched_id = location_mapping.get(loc)

            # 在布局中找到该对象实例
            target_instance = None
            for obj in room_assets.get("objects", []):
                if self._norm(obj.get("id")) == self._norm(matched_id):
                    target_instance = obj
                    break

            self._print(f"\n步骤 {step_idx}: {step.get('description')[:55]}...", scene_name)
            self._print(f"  [Protocol位置]: {loc} -> [匹配实例]: {matched_id}", scene_name)

            if target_instance:
                target = self.calculate_nav_target(target_instance, scene.offset_radius, bbox_lookup)
                if target:
                    info = target["info"]
                    self._print(f"  [边界尺寸]: 宽度(W)={info['width']:.2f}m, 深度(D)={info['depth']:.2f}m", scene_name)
                    self._print(f"  [对象中心]: ({info['center'][0]:.2f}, {info['center'][1]:.2f}), 旋转: {info['rz']}°", scene_name)
                    self._print(f"  [导航目标]: X={target['x']:.3f}, Y={target['y']:.3f} (偏移量: dx={target['dx']:.2f}, dy={target['dy']:.2f})", scene_name)

                    # 保存到结果
                    nav_target = NavTarget(
                        step_number=step_idx,
                        location=loc,
                        matched_object=matched_id,
                        target_x=target['x'],
                        target_y=target['y'],
                        offset_dx=target['dx'],
                        offset_dy=target['dy'],
                        bbox_width=info['width'],
                        bbox_depth=info['depth'],
                        object_center_x=info['center'][0],
                        object_center_y=info['center'][1],
                        rotation_z=info['rz']
                    )
                    nav_targets.append(nav_target)
                else:
                    self._print(f"  [警告]: 无法在资产库中找到该物体的几何尺寸。", scene_name)
            else:
                self._print(f"  [错误]: 无法在房间布局中定位该设备。", scene_name)

        self._print(f"\n{'='*80}", scene_name)
        self._print(f"✓ 场景 {scene_name} 处理完成: 生成 {len(nav_targets)} 个导航目标点", scene_name)
        self._print(f"{'='*80}", scene_name)

        return nav_targets

    def process_all_scenes(self):
        """处理所有场景"""
        print(f"\n{'='*80}")
        print(f"多场景导航点批量生成")
        print(f"{'='*80}")
        print(f"场景数量: {len(self.scenes)}\n")

        all_results = {}

        for scene in self.scenes:
            nav_targets = self.process_single_scene(scene)

            # 转换为 goal_pairs 格式
            goal_pairs = convert_nav_targets_to_goal_pairs(nav_targets)

            # 去除重复的点
            deduplicated_pairs = deduplicate_goal_pairs(goal_pairs)

            all_results[scene.scene_name] = {
                "config": {
                    "scene_name": scene.scene_name,
                    "scene_dir": str(scene.scene_dir),
                    "room_assets_file": scene.room_assets_file,
                    "protocol_file": scene.protocol_file,
                    "asset_lib_file": str(scene.asset_lib_file),
                    "offset_radius": scene.offset_radius
                },
                "nav_targets": [asdict(nt) for nt in nav_targets],
                "num_targets": len(nav_targets),
                "goal_pairs": goal_pairs,
                "goal_pairs_deduplicated": deduplicated_pairs,
                "num_goal_pairs": len(deduplicated_pairs)
            }

            self._print(f"✓ 转换为 goal_pairs: {len(goal_pairs)} 个", scene.scene_name)
            self._print(f"✓ 去重后: {len(deduplicated_pairs)} 个", scene.scene_name)

        self.results = all_results
        return all_results

    def save_results(self, output_dir: Path):
        """保存所有结果"""
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        # 1. 保存完整结果 JSON（保留用于调试）
        output_file = output_dir / "batch_nav_targets.json"
        with open(output_file, "w", encoding="utf-8") as f:
            json.dump(self.results, f, indent=2, ensure_ascii=False)

        print(f"\n✓ 完整结果已保存到: {output_file}")

        # 2. 为每个场景保存单独的 nav_targets JSON 文件（保留用于调试）
        for scene_name, result in self.results.items():
            scene_file = output_dir / f"{scene_name}_nav_targets.json"
            with open(scene_file, "w", encoding="utf-8") as f:
                json.dump(result, f, indent=2, ensure_ascii=False)

        # 3. 为每个场景保存单独的 goal_pairs YAML 文件（主要输出）
        goal_pairs_dir = output_dir / "goal_pairs"
        goal_pairs_dir.mkdir(parents=True, exist_ok=True)

        for scene_name, result in self.results.items():
            # 提取去重后的 goal_pairs
            goal_pairs = result.get("goal_pairs_deduplicated", [])

            # 构建输出数据
            output_data = {
                "goal_pairs": goal_pairs,
                "metadata": {
                    "scene_name": scene_name,
                    "num_targets": result["num_targets"],
                    "num_goal_pairs": len(goal_pairs),
                    "config": result["config"]
                }
            }

            # 保存为 YAML 文件
            # 清理场景名称，移除无效的文件名字符
            safe_scene_name = scene_name.replace("/", "_").replace("\\", "_")
            yaml_file = goal_pairs_dir / f"{safe_scene_name}_goal_pairs.yaml"

            with open(yaml_file, "w", encoding="utf-8") as f:
                yaml.dump(output_data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

            print(f"✓ 场景 {scene_name} goal_pairs 已保存到: {yaml_file}")

        # 4. 生成统计报告
        self.generate_summary_report(output_dir)

        print(f"\n✓ 所有 goal_pairs 文件已保存到目录: {goal_pairs_dir}")

    def generate_summary_report(self, output_dir: Path):
        """生成统计报告"""
        report_file = output_dir / "summary_report.txt"

        with open(report_file, "w", encoding="utf-8") as f:
            f.write("="*80 + "\n")
            f.write("多场景导航点生成统计报告\n")
            f.write("="*80 + "\n\n")

            f.write(f"总场景数: {len(self.results)}\n\n")

            for scene_name, result in self.results.items():
                f.write(f"场景: {scene_name}\n")
                f.write(f"  导航目标点数量: {result['num_targets']}\n")
                f.write(f"  Goal Pairs 数量: {result['num_goal_pairs']}\n")
                f.write(f"  配置:\n")
                config = result['config']
                f.write(f"    - 场景目录: {config['scene_dir']}\n")
                f.write(f"    - 房间资产文件: {config['room_assets_file']}\n")
                f.write(f"    - 协议文件: {config['protocol_file']}\n")
                f.write(f"    - 资产库文件: {config['asset_lib_file']}\n")
                f.write(f"    - 机器人半径: {config['offset_radius']}m\n")
                f.write("\n")

                # 列出所有导航点
                if result['nav_targets']:
                    f.write(f"  导航点列表:\n")
                    for nt in result['nav_targets']:
                        f.write(f"    步骤 {nt['step_number']}: ({nt['target_x']:.3f}, {nt['target_y']:.3f}) - {nt['location']}\n")
                    f.write("\n")

                # 列出 goal_pairs
                if result.get('goal_pairs_deduplicated'):
                    f.write(f"  Goal Pairs 列表 (去重后):\n")
                    for i, gp in enumerate(result['goal_pairs_deduplicated'], 1):
                        start = gp['start']
                        end = gp['end']
                        f.write(f"    {i}. [{start[0]:.3f}, {start[1]:.3f}, {start[2]:.2f}] -> [{end[0]:.3f}, {end[1]:.3f}, {end[2]:.2f}]\n")
                    f.write("\n")

                f.write("-"*80 + "\n\n")

        print(f"✓ 统计报告已保存到: {report_file}")


# ==================== 使用示例 ====================

def main():
    """主函数：自动处理 layout_new 目录"""
    import sys

    # 配置参数
    DEFAULT_ROOT_DIR = Path("/home/pjlab/fbh/LabUtopia/roomlayout/layout_new")
    DEFAULT_OUTPUT_DIR = Path("/home/pjlab/fbh/LabUtopia/outputs/auto_batch_nav_targets")

    # 解析命令行参数
    if len(sys.argv) > 1:
        root_dir = Path(sys.argv[1])
    else:
        root_dir = DEFAULT_ROOT_DIR

    if len(sys.argv) > 2:
        output_dir = Path(sys.argv[2])
    else:
        output_dir = DEFAULT_OUTPUT_DIR

    # 1. 自动扫描场景
    print("\n" + "="*80)
    print("自动化场景导航点批量生成工具")
    print("="*80)

    scanner = AutoSceneScanner(root_dir, verbose=True)
    scene_configs = scanner.scan_all_scenes()

    if not scene_configs:
        print("\n❌ 错误: 未找到任何场景，请检查目录结构")
        return

    # 2. 批量生成导航点
    generator = MultiSceneNavGenerator(scene_configs, verbose=True)
    results = generator.process_all_scenes()

    # 3. 保存结果
    generator.save_results(output_dir)

    print("\n" + "="*80)
    print("✓ 全部处理完成！")
    print("="*80)


if __name__ == "__main__":
    main()
