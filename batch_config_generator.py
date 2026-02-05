#!/usr/bin/env python3
"""
批量配置文件生成脚本
根据USD库文件结构，批量生成navigation配置和level5主配置文件
"""

import os
import yaml
import argparse
from pathlib import Path
from typing import List, Dict, Any


class ConfigGenerator:
    """配置文件生成器"""

    def __init__(self,
                 usd_library_dir: str,
                 navigation_template_path: str,
                 main_config_template_path: str,
                 output_dir: str):
        """
        初始化配置生成器

        Args:
            usd_library_dir: USD库文件根目录
            navigation_template_path: navigation配置模板路径
            main_config_template_path: 主配置模板路径
            output_dir: 输出目录
        """
        self.usd_library_dir = Path(usd_library_dir)
        self.navigation_template_path = Path(navigation_template_path)
        self.main_config_template_path = Path(main_config_template_path)
        self.output_dir = Path(output_dir)

        # 创建输出目录
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # 加载模板
        self.navigation_template = self._load_yaml(self.navigation_template_path)
        self.main_config_template = self._load_yaml(self.main_config_template_path)

    def _load_yaml(self, yaml_path: Path) -> Dict[str, Any]:
        """加载YAML文件"""
        with open(yaml_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)

    def _save_yaml(self, data: Dict[str, Any], output_path: Path):
        """保存YAML文件"""
        with open(output_path, 'w', encoding='utf-8') as f:
            yaml.dump(data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)

    def scan_usd_scenes(self) -> List[Dict[str, str]]:
        """
        扫描USD库目录，查找所有场景

        Returns:
            场景列表，每个元素包含 {name, usd_path, relative_path, barrier_image_path}
        """
        scenes = []

        for scene_dir in sorted(self.usd_library_dir.iterdir()):
            if not scene_dir.is_dir():
                continue

            # 查找目录中的.usd文件
            usd_files = list(scene_dir.glob("*.usd"))

            if not usd_files:
                print(f"警告: 目录 {scene_dir.name} 中没有找到.usd文件")
                continue

            # 取第一个.usd文件（通常每个场景只有一个）
            usd_file = usd_files[0]

            # 提取场景名称（去除_room_isaacsim等后缀）
            scene_name = usd_file.stem.replace('_room_isaacsim', '')

            # 查找场景目录中的PNG文件（用于barrier_image_path）
            png_files = list(scene_dir.glob("*.png"))
            barrier_image_path = None
            if png_files:
                # 使用找到的第一个PNG文件
                # 路径格式: 相对于项目根目录的完整路径（与usd_path保持一致）
                png_file = png_files[0]
                barrier_image_path = str(png_file.relative_to(self.usd_library_dir.parent))
            else:
                print(f"警告: 目录 {scene_dir.name} 中没有找到PNG文件")

            scenes.append({
                'name': scene_name,
                'usd_filename': usd_file.name,
                'usd_path': str(usd_file.relative_to(self.usd_library_dir.parent)),
                'scene_dir': scene_dir.name,
                'barrier_image_path': barrier_image_path
            })

        return scenes

    def generate_navigation_config(self, scene: Dict[str, str], output_path: Path):
        """
        为单个场景生成navigation配置文件

        Args:
            scene: 场景信息字典
            output_path: 输出文件路径
        """
        # 复制模板
        config = self.navigation_template.copy()

        # 更新场景信息
        config['assets'][0]['name'] = scene['name']
        config['assets'][0]['scene_asset_path'] = f"{scene['scene_dir']}/{scene['usd_filename']}"

        # 设置 barrier_image_path（如果找到了PNG文件）
        if scene.get('barrier_image_path'):
            config['assets'][0]['barrier_image_path'] = scene['barrier_image_path']
            print(f"  设置 barrier_image_path: {scene['barrier_image_path']}")
        else:
            print(f"  警告: 未找到PNG文件，使用默认barrier_image_path")

        # 设置 offset_radius 为 0.6m
        config['assets'][0]['offset_radius'] = 0.6
        print(f"  设置 offset_radius: 0.6m")

        # 保存文件
        self._save_yaml(config, output_path)
        print(f"生成navigation配置: {output_path}")

    def generate_main_config(self, scene: Dict[str, str],
                             nav_config_path: str,
                             output_path: Path,
                             custom_params: Dict[str, Any] = None,
                             force_single_zero_goal: bool = False):
        """
        为单个场景生成主配置文件

        Args:
            scene: 场景信息字典
            nav_config_path: navigation配置文件路径
            output_path: 输出文件路径
            custom_params: 自定义参数覆盖
            force_single_zero_goal: 是否强制使用单个[0,0,0] goal_pair
        """
        # 复制模板
        config = self.main_config_template.copy()

        # 更新基础信息
        config['name'] = f"level5_Navigation_{scene['name']}"
        config['usd_path'] = scene['usd_path']
        config['task']['navigation_config_path'] = nav_config_path

        # 强制使用单个[0,0,0] goal_pair
        if force_single_zero_goal:
            config['task']['goal_pairs'] = [
                {
                    'start': [0.0, 0.0, 0.0],
                    'end': [0.0, 0.0, 0.0]
                }
            ]
            print(f"  ⚠️  强制设置 goal_pairs 为单个 [0,0,0]")

        # 应用自定义参数
        if custom_params:
            for key, value in custom_params.items():
                keys = key.split('.')
                current = config
                for k in keys[:-1]:
                    if k not in current:
                        current[k] = {}
                    current = current[k]
                current[keys[-1]] = value

        # 保存文件
        self._save_yaml(config, output_path)
        print(f"生成主配置: {output_path}")

    def batch_generate(self,
                       navigation_output_dir: str = None,
                       main_config_output_dir: str = None,
                       custom_params: Dict[str, Any] = None,
                       prefix: str = "",
                       force_single_zero_goal: bool = False):
        """
        批量生成配置文件

        Args:
            navigation_output_dir: navigation配置输出目录（默认为output_dir/navigation）
            main_config_output_dir: 主配置输出目录（默认为output_dir/main）
            custom_params: 自定义参数覆盖
            prefix: 配置文件名前缀
            force_single_zero_goal: 是否强制使用单个[0,0,0] goal_pair
        """
        # 设置输出目录
        if navigation_output_dir is None:
            navigation_output_dir = self.output_dir / "navigation"
        else:
            navigation_output_dir = Path(navigation_output_dir)

        if main_config_output_dir is None:
            main_config_output_dir = self.output_dir / "main"
        else:
            main_config_output_dir = Path(main_config_output_dir)

        navigation_output_dir.mkdir(parents=True, exist_ok=True)
        main_config_output_dir.mkdir(parents=True, exist_ok=True)

        # 扫描场景
        scenes = self.scan_usd_scenes()
        print(f"\n找到 {len(scenes)} 个场景\n")

        # 为每个场景生成配置
        for scene in scenes:
            # 生成navigation配置
            nav_config_name = f"{prefix}navigation_assets_{scene['name']}.yaml"
            nav_config_path = navigation_output_dir / nav_config_name
            self.generate_navigation_config(scene, nav_config_path)

            # 生成主配置
            main_config_name = f"{prefix}level5_Navigation_{scene['name']}.yaml"
            main_config_path = main_config_output_dir / main_config_name
            self.generate_main_config(
                scene,
                str(nav_config_path),
                main_config_path,
                custom_params,
                force_single_zero_goal
            )

        print(f"\n批量生成完成！")
        print(f"navigation配置目录: {navigation_output_dir}")
        print(f"主配置目录: {main_config_output_dir}")


def main():
    parser = argparse.ArgumentParser(description='批量生成配置文件')
    parser.add_argument('--usd-lib', type=str, required=True,
                        help='USD库文件根目录')
    parser.add_argument('--nav-template', type=str,
                        default='config/navigation/navigation_assets_fbh2.yaml',
                        help='navigation配置模板路径')
    parser.add_argument('--main-template', type=str,
                        default='config/level5_Navigation_parquet_1_20.yaml',
                        help='主配置模板路径')
    parser.add_argument('--output-dir', type=str,
                        default='generated_configs',
                        help='输出目录')
    parser.add_argument('--nav-output-dir', type=str,
                        help='navigation配置输出目录（默认为output_dir/navigation）')
    parser.add_argument('--main-output-dir', type=str,
                        help='主配置输出目录（默认为output_dir/main）')
    parser.add_argument('--prefix', type=str, default='',
                        help='配置文件名前缀')
    parser.add_argument('--max-episodes', type=int,
                        help='最大episode数量')
    parser.add_argument('--max-steps', type=int,
                        help='最大步数')
    parser.add_argument('--force-single-zero-goal', action='store_true',
                        help='强制所有goal_pairs只保留一个[0,0,0]')

    args = parser.parse_args()

    # 构建自定义参数
    custom_params = {}
    if args.max_episodes:
        custom_params['max_episodes'] = args.max_episodes
    if args.max_steps:
        custom_params['task.max_steps'] = args.max_steps

    # 创建生成器
    generator = ConfigGenerator(
        usd_library_dir=args.usd_lib,
        navigation_template_path=args.nav_template,
        main_config_template_path=args.main_template,
        output_dir=args.output_dir
    )

    # 批量生成
    generator.batch_generate(
        navigation_output_dir=args.nav_output_dir,
        main_config_output_dir=args.main_output_dir,
        custom_params=custom_params if custom_params else None,
        prefix=args.prefix,
        force_single_zero_goal=args.force_single_zero_goal
    )


if __name__ == '__main__':
    main()
