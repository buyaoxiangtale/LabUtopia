# 多场景导航点批量生成工具使用指南

## 📋 概述

`roomlayout/batch_scene_nav_targets.py` 提供了批量处理多个场景导航点生成的功能，解决了原 `process_nonfloor_objects.py` 只能处理单个场景的限制。

### 主要功能

- ✅ 支持批量处理多个场景
- ✅ 每个场景配备独立的假设位置文件、资产文件、protocol 文件
- ✅ 自动处理非 floor 对象映射（继承原 `process_nonfloor_objects.py` 的功能）
- ✅ 生成导航目标点并导出 JSON
- ✅ 生成统计报告

---

## 🚀 快速开始

### 方式1: 处理单个场景

```python
from roomlayout.batch_scene_nav_targets import MultiSceneNavGenerator, SceneConfig
from pathlib import Path

# 定义场景配置
scene = SceneConfig(
    scene_name="12_17_alkylation",
    scene_dir=Path("/path/to/roomlayout/12_17"),
    room_assets_file="Alkylation_of_Ethyl_Acetoacetate_with_Bis(4-fluoro_room_isaacsim.json",
    protocol_file="protocol_Alkylation_of_Ethyl_Acetoaceta_20251215_102129.json",
    asset_lib_file="assets_annotated.json",
    offset_radius=0.6  # 机器人半径
)

# 创建生成器
generator = MultiSceneNavGenerator([scene], verbose=True)

# 处理所有场景
results = generator.process_all_scenes()

# 保存结果
generator.save_results(Path("outputs/batch_nav_targets"))
```

### 方式2: 处理多个场景

```python
# 定义多个场景
scenes = [
    SceneConfig(
        scene_name="12_17_alkylation",
        scene_dir=Path("/path/to/roomlayout/12_17"),
        room_assets_file="Alkylation_of_Ethyl_Acetoacetate_with_Bis(4-fluoro_room_isaacsim.json",
        protocol_file="protocol_Alkylation_of_Ethyl_Acetoaceta_20251215_102129.json",
        asset_lib_file="assets_annotated.json",
        offset_radius=0.6
    ),
    SceneConfig(
        scene_name="12_18_crystallization",
        scene_dir=Path("/path/to/roomlayout/12_18"),
        room_assets_file="Crystallization_room_isaacsim.json",
        protocol_file="protocol_Crystallization_20251215_103000.json",
        asset_lib_file="assets_annotated.json",
        offset_radius=0.6
    ),
]

# 批量处理
generator = MultiSceneNavGenerator(scenes, verbose=True)
results = generator.process_all_scenes()
generator.save_results(Path("outputs/batch_nav_targets"))
```

### 方式3: 从配置文件加载

**配置文件格式** (`config/batch_scenes_config_example.json`):

```json
{
  "description": "多场景批量导航点生成配置文件",
  "output_dir": "outputs/batch_nav_scenes",

  "scenes": [
    {
      "scene_name": "12_17_alkylation",
      "scene_dir": "/home/pjlab/fbh/LabUtopia/roomlayout/12_17",
      "room_assets_file": "Alkylation_of_Ethyl_Acetoacetate_with_Bis(4-fluoro_room_isaacsim.json",
      "protocol_file": "protocol_Alkylation_of_Ethyl_Acetoaceta_20251215_102129.json",
      "asset_lib_file": "assets_annotated.json",
      "offset_radius": 0.6
    },
    {
      "scene_name": "sodium_acetate_preparation",
      "scene_dir": "/home/pjlab/fbh/LabUtopia/roomlayout",
      "room_assets_file": "Preparation_of_Sodium_Acetate,_Crystallization,_and_Simple_Distillation_of_Residual_Liquor_room_isaacsim.json",
      "protocol_file": "protocol_Preparation_of_Sodium_Acetate,_20251111_165059.json",
      "asset_lib_file": "12_17/assets_annotated.json",
      "offset_radius": 0.6
    }
  ]
}
```

**使用代码**:

```python
import json
from pathlib import Path
from roomlayout.batch_scene_nav_targets import MultiSceneNavGenerator, SceneConfig

# 加载配置文件
with open("config/batch_scenes_config_example.json", "r") as f:
    config_data = json.load(f)

# 创建场景配置列表
scenes = []
for scene_cfg in config_data.get("scenes", []):
    scene = SceneConfig(
        scene_name=scene_cfg["scene_name"],
        scene_dir=Path(scene_cfg["scene_dir"]),
        room_assets_file=scene_cfg["room_assets_file"],
        protocol_file=scene_cfg["protocol_file"],
        asset_lib_file=scene_cfg.get("asset_lib_file", "assets_annotated.json"),
        offset_radius=scene_cfg.get("offset_radius", 0.6)
    )
    scenes.append(scene)

# 批量处理
generator = MultiSceneNavGenerator(scenes, verbose=True)
results = generator.process_all_scenes()
generator.save_results(Path(config_data.get("output_dir", "outputs/batch_nav_targets")))
```

---

## 📂 文件组织结构

### 场景目录结构

每个场景的文件应该组织在同一目录下：

```
roomlayout/
├── 12_17/                              # 场景 1 目录
│   ├── Alkylation_of_Ethyl_Acetoacetate_with_Bis(4-fluoro_room_isaacsim.json  # 资产文件
│   ├── protocol_Alkylation_of_Ethyl_Acetoaceta_20251215_102129.json            # 协议文件
│   └── assets_annotated.json           # 资产库文件（可选，可以是共享的）
│
├── 12_18/                              # 场景 2 目录
│   ├── Crystallization_room_isaacsim.json
│   ├── protocol_Crystallization_20251215_103000.json
│   └── assets_annotated.json
│
└── assets_annotated.json               # 共享资产库文件（可选）
```

### 输出目录结构

批量处理后的输出文件：

```
outputs/batch_nav_targets/
├── batch_nav_targets.json              # 完整结果（包含所有场景）
├── summary_report.txt                  # 统计报告
├── 12_17_alkylation_nav_targets.json   # 场景 1 的独立结果
├── 12_18_crystallization_nav_targets.json  # 场景 2 的独立结果
└── ...
```

---

## 📊 输出结果格式

### 1. 完整结果 JSON (`batch_nav_targets.json`)

```json
{
  "12_17_alkylation": {
    "config": {
      "scene_name": "12_17_alkylation",
      "scene_dir": "/path/to/roomlayout/12_17",
      "room_assets_file": "...",
      "protocol_file": "...",
      "asset_lib_file": "assets_annotated.json",
      "offset_radius": 0.6
    },
    "nav_targets": [
      {
        "step_number": 1,
        "location": "FUMEHOOD",
        "matched_object": "FumeHood",
        "target_x": 2.456,
        "target_y": 7.891,
        "offset_dx": -0.6,
        "offset_dy": 0.0,
        "bbox_width": 1.2,
        "bbox_depth": 0.8,
        "object_center_x": 2.5,
        "object_center_y": 7.5,
        "rotation_z": 0.0
      },
      ...
    ],
    "num_targets": 15
  },
  ...
}
```

### 2. 统计报告 (`summary_report.txt`)

```
================================================================================
多场景导航点生成统计报告
================================================================================

总场景数: 2

场景: 12_17_alkylation
  导航点数量: 15
  配置:
    - 场景目录: /path/to/roomlayout/12_17
    - 房间资产文件: Alkylation_of_Ethyl_Acetoacetate_with_Bis(4-fluoro_room_isaacsim.json
    - 协议文件: protocol_Alkylation_of_Ethyl_Acetoaceta_20251215_102129.json
    - 资产库文件: assets_annotated.json
    - 机器人半径: 0.6m

  导航点列表:
    步骤 1: (2.456, 7.891) - FUMEHOOD
    步骤 2: (5.123, 4.567) - BENCH
    ...

--------------------------------------------------------------------------------
```

---

## 🔧 配置参数说明

### SceneConfig 参数

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `scene_name` | str | ✅ | 场景名称（用于标识） |
| `scene_dir` | Path | ✅ | 场景目录路径（包含所有文件） |
| `room_assets_file` | str | ✅ | 房间资产文件名（假设位置文件） |
| `protocol_file` | str | ✅ | 协议文件名 |
| `asset_lib_file` | str | ❌ | 资产库文件名（默认: `assets_annotated.json`） |
| `offset_radius` | float | ❌ | 机器人半径（默认: `0.6`） |

---

## 🧪 测试脚本

提供了完整的测试脚本 `test_batch_scene_nav.py`：

```bash
# 运行所有测试
python test_batch_scene_nav.py 5

# 运行单个测试
python test_batch_scene_nav.py 1  # 单个场景
python test_batch_scene_nav.py 2  # 多个场景
python test_batch_scene_nav.py 3  # 从配置文件加载
python test_batch_scene_nav.py 4  # 快速演示
```

---

## 📖 核心功能说明

### 1. 非 Floor 对象处理

继承自 `process_nonfloor_objects.py`，处理初始位置不在 floor 上的对象：

```python
# 对于 initial_location != "floor" 的对象
# 1. 查找父对象
# 2. 获取两者的 bbox
# 3. 比较面积
# 4. 选择更大的对象作为导航目标
```

**示例**：
- 烧杯的 `initial_location` = "ExperimentalPlatform"
- 烧杯的 bbox: 0.1m × 0.1m (面积: 0.01m²)
- 实验台的 bbox: 1.2m × 0.8m (面积: 0.96m²)
- **结果**: 使用实验台作为导航目标

### 2. Location 映射

自动建立 protocol location → room_assets 对象的映射：

```python
# 特殊映射（硬编码）
SPECIAL_MAPPING = {
    "labbench": "ExperimentalPlatform",
    "bench": "ExperimentalPlatform",
    "hood": "FumeHood",
    "validation_platform": "ValidationPlatform"
}

# 规范化匹配（自动）
# 去除下划线/空格/横线，转小写
# 例如: "FumeHood_01" -> " fumehood"
```

### 3. 导航点计算

基于对象的旋转和 bbox 计算导航目标点：

```python
# 机器人停靠距离 = 物体半深度 + 机器人半径
dist = hy + offset_radius

# 基于 rotation.z 的投影计算
dx = -dist * sin(rz)
dy = dist * cos(rz)

# 目标点
tx = cx + dx
ty = cy + dy
```

---

## 💡 使用建议

1. **场景组织**：
   - 每个场景的文件放在独立目录下
   - 资产库文件可以共享（放在父目录）

2. **配置文件**：
   - 对于大量场景，建议使用配置文件
   - 方便批量管理和版本控制

3. **机器人半径**：
   - 根据实际机器人尺寸设置 `offset_radius`
   - 建议值: 0.3 ~ 0.8 米

4. **错误处理**：
   - 如果某个对象找不到 bbox，会跳过并记录警告
   - 检查输出日志中的警告信息

---

## 🐛 常见问题

### Q1: 找不到资产库文件？

**A**: 检查以下几点：
1. 文件路径是否正确（相对于 `scene_dir`）
2. 文件名是否正确（区分大小写）
3. 文件格式是否正确（JSON 格式）

### Q2: 某些对象无法匹配？

**A**:
1. 检查 `protocol` 中的 `location` 名称
2. 检查 `room_assets` 中的对象 `id`
3. 尝试添加特殊映射（`SPECIAL_MAPPING`）

### Q3: 导航点位置不合理？

**A**:
1. 检查对象的 `rotation.z` 是否正确
2. 检查 bbox 尺寸是否正确
3. 调整 `offset_radius` 参数

### Q4: 如何处理多个协议文件对应同一个场景？

**A**: 创建多个 SceneConfig，使用相同的 `scene_dir` 和 `room_assets_file`：

```python
scenes = [
    SceneConfig(
        scene_name="12_17_protocol1",
        scene_dir=Path("/path/to/12_17"),
        room_assets_file="room_assets.json",
        protocol_file="protocol1.json",  # 不同的协议文件
        asset_lib_file="assets_annotated.json"
    ),
    SceneConfig(
        scene_name="12_17_protocol2",
        scene_dir=Path("/path/to/12_17"),
        room_assets_file="room_assets.json",  # 相同的资产文件
        protocol_file="protocol2.json",  # 不同的协议文件
        asset_lib_file="assets_annotated.json"
    ),
]
```

---

## 📚 相关文件

- `roomlayout/batch_scene_nav_targets.py` - 多场景导航点批量生成主程序
- `roomlayout/process_nonfloor_objects.py` - 单场景非 floor 对象处理
- `roomlayout/12_17/nav_goal_targets_demo_12_17.py` - 单场景导航点生成示例
- `test_batch_scene_nav.py` - 测试脚本
- `config/batch_scenes_config_example.json` - 配置文件示例

---

## 📝 更新日志

- **2024-01-03**: 初始版本
  - 支持多场景批量处理
  - 继承非 floor 对象处理功能
  - 生成详细统计报告
  - 支持配置文件
