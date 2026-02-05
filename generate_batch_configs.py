#!/usr/bin/env python3
"""
批量生成配置文件脚本
为 gemini_3_flash_preview_1_16 文件夹下的每个子文件夹生成对应的配置文件
"""

import os
import yaml
from pathlib import Path
import re


def find_usd_file(folder_path):
    """在文件夹中查找 USD 文件"""
    usd_files = list(Path(folder_path).glob("*_room_isaacsim.usd"))
    if usd_files:
        return usd_files[0]
    return None


def extract_task_name(folder_name):
    """从文件夹名中提取任务名称（去掉日期后缀）"""
    # 去掉 _20260112_202101 这样的日期后缀
    pattern = r'_202\d+_\d+$'
    task_name = re.sub(pattern, '', folder_name)
    return task_name


def generate_config_files(use_common_nav_config=True, common_nav_config_path=None):
    """
    批量生成配置文件
    
    Args:
        use_common_nav_config: 是否使用通用的 navigation 配置文件
        common_nav_config_path: 通用的 navigation 配置文件路径（如果 use_common_nav_config=True）
    """
    # 路径配置
    base_dir = Path("/home/pjlab/fbh/LabUtopia")
    template_config_path = base_dir / "config" / "level5_Navigation_parquet.yaml"
    source_folder = base_dir / "gemini_3_flash_preview_1_16"
    output_config_dir = base_dir / "config"
    
    # 读取模板配置文件（保持原始格式）
    with open(template_config_path, 'r', encoding='utf-8') as f:
        template_content = f.read()
        template_config = yaml.safe_load(template_content)
    
    # 设置默认的通用 navigation 配置文件路径
    if use_common_nav_config and common_nav_config_path is None:
        common_nav_config_path = "config/navigation/navigation_assets_1_4.yaml"
    
    # 获取所有子文件夹
    subfolders = [d for d in source_folder.iterdir() if d.is_dir()]
    subfolders.sort()
    
    print(f"找到 {len(subfolders)} 个子文件夹")
    if use_common_nav_config:
        print(f"使用通用 Navigation 配置: {common_nav_config_path}")
    else:
        print("为每个场景生成独立的 Navigation 配置")
    
    generated_configs = []
    
    for folder in subfolders:
        folder_name = folder.name
        
        # 查找 USD 文件
        usd_file = find_usd_file(folder)
        if not usd_file:
            print(f"警告: 在 {folder_name} 中未找到 USD 文件，跳过")
            continue
        
        # 提取任务名称
        task_name = extract_task_name(folder_name)
        
        # 构建相对路径（相对于项目根目录）
        usd_relative_path = usd_file.relative_to(base_dir)
        
        # 创建新的配置内容（基于模板内容）
        new_content = template_content
        
        # 修改配置名称
        config_name = f"level5_Navigation_parquet_{task_name}"
        new_content = re.sub(r'^name:\s*.*$', f'name: {config_name}', new_content, flags=re.MULTILINE)
        
        # 修改 USD 路径（使用引号保持格式）
        usd_path_str = str(usd_relative_path).replace('\\', '/')
        new_content = re.sub(
            r'^usd_path:\s*".*"$',
            f'usd_path: "{usd_path_str}"',
            new_content,
            flags=re.MULTILINE
        )
        
        # 修改 navigation_config_path
        if use_common_nav_config:
            nav_config_path = common_nav_config_path
        else:
            nav_config_name = f"navigation_assets_{task_name}.yaml"
            nav_config_path = f"config/navigation/{nav_config_name}"
        
        new_content = re.sub(
            r'^(\s+)navigation_config_path:\s*".*"$',
            f'\\1navigation_config_path: "{nav_config_path}"',
            new_content,
            flags=re.MULTILINE
        )
        
        # 生成输出文件名
        output_filename = f"{config_name}.yaml"
        output_path = output_config_dir / output_filename
        
        # 保存配置文件（保持原始格式）
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(new_content)
        
        generated_configs.append({
            'folder': folder_name,
            'task_name': task_name,
            'usd_path': str(usd_relative_path),
            'config_file': output_filename,
            'nav_config_path': nav_config_path
        })
        
        print(f"✓ 已生成: {output_filename}")
        print(f"  USD路径: {usd_relative_path}")
        print(f"  Navigation配置: {nav_config_path}")
    
    # 打印总结
    print(f"\n总共生成了 {len(generated_configs)} 个配置文件")
    
    return generated_configs


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='批量生成配置文件')
    parser.add_argument('--use-common-nav', action='store_true', default=True,
                        help='使用通用的 navigation 配置文件（默认：True）')
    parser.add_argument('--nav-config-path', type=str, default=None,
                        help='通用的 navigation 配置文件路径（默认：config/navigation/navigation_assets_1_4.yaml）')
    parser.add_argument('--use-scene-specific-nav', action='store_false', dest='use_common_nav',
                        help='为每个场景生成独立的 navigation 配置文件')
    
    args = parser.parse_args()
    
    try:
        generated_configs = generate_config_files(
            use_common_nav_config=args.use_common_nav,
            common_nav_config_path=args.nav_config_path
        )
        print("\n批量生成完成！")
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()

