# 自动化场景导航点批量生成工具使用指南

## 概述

`auto_batch_scene_nav.py` 是一个自动化工具，能够：
- 自动扫描指定目录（如 `layout_new/`）下的所有实验场景
- 自动识别每个场景中的关键文件（`*_room_isaacsim.json` 和 `protocol*.json`）
- 使用共享的 `assets_annotated.json` 资产库
- 批量生成所有场景的导航目标点
- 生成详细的统计报告

## 目录结构要求

工具期望的目录结构：

```
layout_new/                          # 根目录
├── assets_annotated.json            # 共享资产库（必需）
├── Scene_1_timestamp/               # 实验场景目录
│   ├── Experiment_room_isaacsim.json  # 房间布局文件（必需）
│   ├── protocol_Experiment_*.json     # 协议文件（必需）
│   └── [其他文件...]
├── Scene_2_timestamp/
│   ├── Experiment_room_isaacsim.json
│   ├── protocol_Experiment_*.json
│   └── [其他文件...]
└── ...
```

### 文件识别规则

1. **房间布局文件**：匹配 `*_room_isaacsim.json` 模式
2. **协议文件**：匹配 `protocol*.json` 模式
3. **资产库文件**：在根目录查找 `assets_annotated.json`

## 使用方法

### 基本用法

```bash
# 使用默认路径
python3 utils/auto_batch_scene_nav.py

# 指定输入目录
python3 utils/auto_batch_scene_nav.py /path/to/your/layout_new

# 指定输入和输出目录
python3 utils/auto_batch_scene_nav.py /path/to/input /path/to/output
```

### 默认路径

- **输入目录**: `/home/pjlab/fbh/LabUtopia/roomlayout/layout_new`
- **输出目录**: `/home/pjlab/fbh/LabUtopia/outputs/auto_batch_nav_targets`

## 输出结果

运行后会在输出目录生成以下文件：

```
outputs/auto_batch_nav_targets/
├── batch_nav_targets.json                           # 完整结果（包含所有场景）
├── Scene_1_timestamp_nav_targets.json               # 各场景的独立结果
├── Scene_2_timestamp_nav_targets.json
├── ...
└── summary_report.txt                               # 统计报告
```

### 输出文件说明

#### 1. `batch_nav_targets.json`
包含所有场景的完整结果，结构如下：

```json
{
  "场景名称": {
    "config": {
      "scene_name": "场景名称",
      "scene_dir": "场景目录路径",
      "room_assets_file": "房间资产文件名",
      "protocol_file": "协议文件名",
      "asset_lib_file": "资产库文件路径",
      "offset_radius": 0.6
    },
    "nav_targets": [
      {
        "step_number": 1,
        "location": "ReagentCabinet",
        "matched_object": "ReagentCabinet",
        "target_x": 2.200,
        "target_y": 6.810,
        "offset_dx": 0.000,
        "offset_dy": -2.100,
        "bbox_width": 0.910,
        "bbox_depth": 0.865,
        "object_center_x": 2.200,
        "object_center_y": 8.910,
        "rotation_z": 0.0
      },
      ...
    ],
    "num_targets": 9
  },
  ...
}
```

#### 2. `{Scene_Name}_nav_targets.json`
每个场景的独立结果文件，内容与上述结构相同。

#### 3. `summary_report.txt`
文本格式的统计报告，包含：

- 总场景数统计
- 每个场景的配置信息
- 每个场景的导航点列表
- 每个导航点的坐标和位置信息

## 工作流程

工具的处理流程：

```
1. 扫描根目录
   └─> 查找共享资产库 assets_annotated.json

2. 遍历子目录
   └─> 识别场景目录（包含 *_room_isaacsim.json 和 protocol*.json）
       └─> 自动提取关键文件名

3. 处理每个场景
   └─> 加载房间资产、协议、资产库
       ├─> 处理非 floor 对象（选择更大的父对象）
       ├─> 构建 location -> platform 映射
       └─> 生成导航目标点

4. 保存结果
   ├─> 生成完整 JSON 文件
   ├─> 为每个场景生成独立 JSON 文件
   └─> 生成统计报告
```

## 处理逻辑

### 1. 非 Floor 对象处理

对于 `initial_location` 不在 "floor" 上的对象：
- 查找其父对象
- 比较对象和父对象的 bbox 面积
- 选择面积更大的对象作为导航目标

**示例**：
- `EthylAcetoacetate` (面积 0.01m²) → `ReagentCabinet` (面积 0.79m²)
- `HeatingPlate` (面积 0.046m²) → `ExperimentalPlatform` (面积 4.16m²)

### 2. Location 映射

Protocol 中的 location 映射到实际场景对象：

```
protocol location          实际对象
├─ labbench/bench      →  ExperimentalPlatform
├─ hood                 →  FumeHood
├─ validation_platform  →  ValidationPlatform
└─ 其他                  →  精确/模糊匹配
```

### 3. 导航点计算

基于对象的以下信息计算机器人停靠位置：
- 对象中心坐标
- 旋转角度 (rotation.z)
- bbox 尺寸
- 机器人半径 (offset_radius = 0.6m)

**公式**：
```
停靠距离 = bbox_half_y + 机器人半径
偏移量 dx = -停靠距离 × sin(rotation_z)
偏移量 dy = 停靠距离 × cos(rotation_z)
目标位置 = (对象中心 x + dx, 对象中心 y + dy)
```

## 测试结果

在 `layout_new/` 目录上的测试结果：

```
✓ 成功识别 13 个实验场景
✓ 每个场景生成 8-11 个导航目标点
✓ 所有结果文件正常生成
```

### 处理的场景示例

1. Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726
2. Basic_Methanolysis_of_Acetate_Ester_20251229_003725
3. Boc_Deprotection_of_Hydrazine_Derivative_20251230_151033
4. Chlorination_of_Phenylphosphonic_Acid_with_Oxalyl__20251230_151033
5. ...（共 13 个）

## 与原版 `batch_scene_nav_targets.py` 的区别

| 特性 | batch_scene_nav_targets.py | auto_batch_scene_nav.py |
|------|---------------------------|-------------------------|
| 目录结构 | 每个场景独立目录，资产库在各自目录 | 共享资产库在根目录 |
| 文件识别 | 手动指定文件名 | 自动识别 `*_room_isaacsim.json` 和 `protocol*.json` |
| 配置方式 | 手动编写 SceneConfig | 自动扫描生成 SceneConfig |
| 适用场景 | 少量场景，文件位置分散 | 批量处理 `layout_new/` 类似结构 |

## 使用建议

### 何时使用 `auto_batch_scene_nav.py`

- 目录结构符合 `layout_new/` 模式
- 有多个实验场景需要批量处理
- 使用共享的 `assets_annotated.json`
- 文件命名遵循规范模式

### 何时使用 `batch_scene_nav_targets.py`

- 每个场景有独立的资产库文件
- 文件命名不规范或需要特殊配置
- 需要手动控制每个场景的参数
- 场景数量较少

## 常见问题

### Q1: 提示"未找到共享资产库"

**原因**: 根目录下没有 `assets_annotated.json`

**解决**:
- 确保在正确的根目录下运行
- 检查 `assets_annotated.json` 是否存在

### Q2: 某个场景未被识别

**原因**: 场景目录缺少必需的文件

**解决**:
- 确保目录中有 `*_room_isaacsim.json`
- 确保目录中有 `protocol*.json`
- 检查文件名是否匹配模式

### Q3: 导航点数量不符合预期

**原因**: 可能存在对象映射失败

**解决**:
- 查看详细输出，检查哪些步骤的导航点生成失败
- 检查资产的 bbox 信息是否完整
- 检查 protocol 中的 location 是否能匹配到实际对象

## 命令行参数

```bash
python3 utils/auto_batch_scene_nav.py [root_dir] [output_dir]

参数说明:
  root_dir    - 场景根目录（默认: roomlayout/layout_new/）
  output_dir  - 输出目录（默认: outputs/auto_batch_nav_targets/）
```

## 示例输出

```
================================================================================
自动扫描场景目录
================================================================================
根目录: /home/pjlab/fbh/LabUtopia/roomlayout/layout_new

✓ 找到共享资产库: /home/pjlab/fbh/LabUtopia/roomlayout/layout_new/assets_annotated.json
找到 13 个子目录

检查: Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726
  ✓ 识别为场景: Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_20251229_003726
    - 房间文件: Alkylation_of_Ethyl_Acetoacetate_with_Bis_4-fluoro_room_isaacsim.json
    - 协议文件: protocol_Alkylation_of_Ethyl_Acetoaceta_20251215_102129.json
...

================================================================================
扫描完成: 共识别 13 个场景
================================================================================

[开始处理各场景...]

================================================================================
✓ 全部处理完成！
================================================================================
```

## 技术细节

### AutoSceneScanner 类

负责自动扫描和识别场景：

```python
scanner = AutoSceneScanner(root_dir, verbose=True)
scene_configs = scanner.scan_all_scenes()
```

**主要方法**:
- `find_shared_asset_lib()`: 查找共享资产库
- `is_scene_directory()`: 判断是否为场景目录
- `identify_scene_files()`: 识别场景中的关键文件

### MultiSceneNavGenerator 类

负责批量生成导航点：

```python
generator = MultiSceneNavGenerator(scene_configs, verbose=True)
results = generator.process_all_scenes()
generator.save_results(output_dir)
```

**主要方法**:
- `process_single_scene()`: 处理单个场景
- `process_all_scenes()`: 处理所有场景
- `save_results()`: 保存结果文件
- `generate_summary_report()`: 生成统计报告

## 扩展和定制

如需定制行为，可以修改以下参数：

### 1. 机器人半径

在 `SceneConfig` 中修改 `offset_radius`（默认 0.6m）:

```python
scene_config = SceneConfig(
    ...
    offset_radius=0.8  # 改为 0.8m
)
```

### 2. 特殊映射规则

在 `build_location_to_platform()` 方法中修改 `special_mapping`:

```python
special_mapping = {
    "labbench": "ExperimentalPlatform",
    "bench": "ExperimentalPlatform",
    "hood": "FumeHood",
    "validation_platform": "ValidationPlatform",
    # 添加新的映射
}
```

### 3. 文件识别模式

在 `identify_scene_files()` 方法中修改 glob 模式：

```python
room_files = list(scene_dir.glob("*_room_isaacsim.json"))  # 修改此模式
protocol_files = list(scene_dir.glob("protocol*.json"))     # 修改此模式
```

## 性能

在 `layout_new/` 目录（13 个场景）上的性能：

- 扫描时间: < 1 秒
- 处理时间: 约 2-5 秒（取决于场景复杂度）
- 总时间: < 10 秒

## 总结

`auto_batch_scene_nav.py` 提供了一个自动化的解决方案，特别适合处理 `layout_new/` 这样结构化的目录。通过自动识别文件模式，大大减少了手动配置的工作量。
