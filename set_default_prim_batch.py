import os
import argparse
from pxr import Usd, Sdf

def set_default_prim(usd_path):
    try:
        stage = Usd.Stage.Open(usd_path)
        if not stage:
            print(f"Error: Could not open stage {usd_path}")
            return False

        # 查找名为 "World" 或 "world" 的 prim
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = stage.GetPrimAtPath("/world")
        
        if not world_prim.IsValid():
            print(f"Warning: Could not find '/World' or '/world' prim in {usd_path}")
            # 列出根 prim 供参考
            try:
                print(f"  Root prims: {[p.GetName() for p in stage.GetPseudoRoot().GetChildren()]}")
            except:
                pass
            return False

        # 检查当前的 default prim
        current_default = stage.GetDefaultPrim()
        if current_default and current_default.GetPath() == world_prim.GetPath():
            print(f"Skipping {os.path.basename(usd_path)}: Default prim is already set to {current_default.GetName()}")
            return True

        # 设置 default prim
        stage.SetDefaultPrim(world_prim)
        stage.GetRootLayer().Save()
        print(f"Success: Set default prim to '{world_prim.GetName()}' for {os.path.basename(usd_path)}")
        return True

    except Exception as e:
        print(f"Error processing {usd_path}: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Batch set default prim to '/World' for USD files.")
    parser.add_argument("root_dir", help="Root directory containing USD files")
    args = parser.parse_args()

    root_dir = args.root_dir
    if not os.path.isdir(root_dir):
        print(f"Error: Directory {root_dir} does not exist.")
        return

    count = 0
    success_count = 0
    
    print(f"Scanning directory: {root_dir} ...")
    
    for dirpath, dirnames, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename.endswith(".usd") or filename.endswith(".usda") or filename.endswith(".usdc"):
                # 忽略备份文件
                if ".backup" in filename:
                    continue
                    
                usd_path = os.path.join(dirpath, filename)
                if set_default_prim(usd_path):
                    success_count += 1
                count += 1

    print(f"\nFinished. Processed {count} files.")

if __name__ == "__main__":
    main()

