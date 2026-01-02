# 实验平台物品导航目标点生成工具使用指南

## 📋 功能概述

这个工具可以为实验平台上的物品自动生成导航目标点，使得机器人能够：
- 在实验平台外找到合适的停靠位置
- 面向目标物品
- 满足安全距离要求

## 🚀 快速开始

### 1. 列出所有在实验平台上的物品

```bash
python utils/find_nav_target_for_item.py --list
```

**输出示例**:
```
在实验平台上的物品 (9 个):
  1. HeatingPlate     位置: {'x': 4.1, 'y': 4.2, 'z': 0.8}
  2. RoundBottomFlask 位置: {'x': 3.8, 'y': 4.2, 'z': 0.8}
  3. Thermometer      位置: {'x': 3.8, 'y': 4.35, 'z': 0.8}
  ...
```

### 2. 为指定物品生成导航目标点

```bash
python utils/find_nav_target_for_item.py --item Beaker --output outputs/beaker_nav.json
```

**输出示例**:
```
导航目标点信息:
  位置: X=4.825, Y=6.600
  朝向: θ=1.326 弧度 (76.0°)

物品信息:
  ID: Beaker
  位置: (4.400, 4.900)

使用示例代码:
goal_pairs:
  - start: [4.825, 6.600, 75.96]
    end: [4.400, 4.900, 0.0]
```

### 3. 验证导航点是否在障碍物外

```bash
python utils/find_nav_target_for_item.py --item Beaker --verify
```

## 📖 核心算法

### 导航点计算策略

1. **找到物品相对平台中心的位置**
   ```
   dx = item_x - platform_center_x
   dy = item_y - platform_center_y
   ```

2. **计算从平台中心到物品的方向**
   ```
   angle_to_item = atan2(dy, dx)
   ```

3. **在平台边缘外生成导航点**
   ```
   platform_radius = sqrt(half_x^2 + half_y^2)
   nav_dist = platform_radius + offset_radius
   nav_x = platform_center_x + nav_dist * cos(angle_to_item)
   nav_y = platform_center_y + nav_dist * sin(angle_to_item)
   ```

4. **机器人朝向物品**
   ```
   robot_theta = angle_to_item  # 面向物品
   ```

### 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--offset-radius` | 0.6 米 | 机器人半径 + 安全距离 |
| `--nav-cfg` | `config/navigation/navigation_assets_12_18.yaml` | 导航配置文件 |

## 📝 命令行参数

| 参数 | 说明 | 示例 |
|------|------|------|
| `--item` | 物品ID（必需） | `--item Beaker` |
| `--list` | 列出所有在平台上的物品 | `--list` |
| `--output` | 输出JSON文件路径 | `--output output.json` |
| `--verify` | 验证导航点是否在障碍物外 | `--verify` |
| `--offset-radius` | 导航偏移半径（米） | `--offset-radius 0.8` |
| `--room-layout` | 房间布局JSON文件 | `--room-layout path/to/file.json` |
| `--asset-lib` | 资产库JSON文件 | `--asset-lib path/to/assets.json` |

## 💡 使用场景

### 场景1：在导航任务中使用

生成的导航目标点可以直接添加到任务配置中：

```yaml
# config/level5_Navigation_*.yaml

task:
  goal_pairs:
    # 从工具生成的导航点
    - start: [4.825, 6.600, 75.96]  # 平台外，面向 Beaker
      end: [4.400, 4.900, 0.0]       # Beaker 位置

    # 其他导航任务
    - start: [2.444, 3.386, -149.04]
      end: [3.800, 4.200, 0.0]
```

### 场景2：批量生成多个物品的导航点

```bash
# 为多个物品生成导航点
python utils/find_nav_target_for_item.py --item Beaker --output beaker.json
python utils/find_nav_target_for_item.py --item RoundBottomFlask --output flask.json
python utils/find_nav_target_for_item.py --item Pipette --output pipette.json
```

### 场景3：自定义偏移半径

如果需要更大的安全距离：

```bash
python utils/find_nav_target_for_item.py --item Beaker --offset-radius 1.0
```

## 📊 输出文件格式

工具生成的JSON文件包含完整的导航信息：

```json
{
  "item": {
    "id": "Beaker",
    "position": {"x": 4.4, "y": 4.9}
  },
  "platform": {
    "id": "LabBench",
    "center": {"x": 4.3, "y": 4.5},
    "half_size": {"x": 0.76, "y": 1.37},
    "rotation": 180
  },
  "nav_target": {
    "position": {"x": 4.825, "y": 6.600},
    "rotation": {"z": 75.96}
  },
  "offset_info": {
    "distance_from_platform_edge": 0.6,
    "direction_to_item": 75.96
  }
}
```

## 🔧 工作原理

### 1. 物品识别

工具会查找 `initial_location == "experimental_platform"` 的物品，例如：

```json
{
  "id": "Beaker",
  "position": {"x": 4.4, "y": 4.9, "z": 0.8},
  "rotation": {"x": 0, "y": 0, "z": 0.0},
  "initial_location": "experimental_platform"
}
```

### 2. 平台信息

工具会找到实验平台（如 `LabBench`）并获取其尺寸信息：

- 从 `assets_annotated.json` 获取平台的 `bbox`
- 平台ID映射：`LabBench` → `ExperimentalPlatform`

### 3. 导航点计算

工具会：
1. 计算物品相对于平台中心的方向
2. 在平台边缘外 `offset_radius` 处生成导航点
3. 设置机器人朝向为面向物品的角度

### 4. 障碍物验证（可选）

使用 `--verify` 选项时，工具会：
- 加载导航网格（障碍物地图）
- 检查导航点是否在自由空间中
- 提供详细的验证结果

## ⚠️ 注意事项

1. **平台ID映射**
   - 房间布局中的 `LabBench` 对应资产库中的 `ExperimentalPlatform`
   - 工具已内置常见映射关系

2. **物品重复ID**
   - 如果有多个同名物品（如两个 `Beaker`），工具会找到第一个

3. **偏移半径**
   - 默认值 0.6 米适合大多数情况
   - 如果物品靠近平台边缘，可能需要增大偏移半径

4. **导航验证**
   - 验证功能需要导航配置文件（包含障碍物地图路径）
   - 确保配置文件路径正确

## 🎯 示例：完整工作流

```bash
# 1. 查看所有可用物品
python utils/find_nav_target_for_item.py --list

# 2. 为 Beaker 生成导航点
python utils/find_nav_target_for_item.py \
    --item Beaker \
    --output outputs/beaker_nav.json \
    --verify

# 3. 将生成的 goal_pair 添加到任务配置
# 编辑 config/level5_Navigation_*.yaml:
task:
  goal_pairs:
    - start: [4.825, 6.600, 75.96]  # 从工具输出复制
      end: [4.400, 4.900, 0.0]

# 4. 测试导航任务
python run_nav_with_video.py --config config/level5_Navigation_*.yaml
```

## 🔗 相关文件

| 文件 | 说明 |
|------|------|
| `utils/find_nav_target_for_item.py` | 主脚本 |
| `roomlayout/12_17/Alkylation_*_room_isaacsim.json` | 房间布局 |
| `roomlayout/12_17/assets_annotated.json` | 资产库 |
| `config/navigation/navigation_assets_12_18.yaml` | 导航配置 |
| `roomlayout/12_17/nav_goal_targets_demo_12_17.py` | 参考实现 |

## 📞 故障排除

### 问题1：找不到物品

```
❌ 无法在实验平台上找到物品: XXX
```

**解决方法**：
- 使用 `--list` 查看所有可用物品
- 检查物品ID拼写（区分大小写）

### 问题2：无法获取平台边界信息

```
❌ 无法获取平台边界信息
```

**解决方法**：
- 检查 `assets_annotated.json` 是否包含平台信息
- 确认平台ID映射正确（如 `LabBench` → `ExperimentalPlatform`）

### 问题3：导航点在障碍物上

```
✗ 导航点在障碍物上（网格坐标: XX, YY）
```

**解决方法**：
- 增加 `--offset-radius` 值
- 手动微调导航点位置

## ✅ 成功示例

```bash
$ python utils/find_nav_target_for_item.py --item Beaker

✓ 目标物品: Beaker
  位置: {'x': 4.4, 'y': 4.9, 'z': 0.8}

导航目标点信息:
  位置: X=4.825, Y=6.600
  朝向: θ=1.326 弧度 (76.0°)

使用示例代码:
goal_pairs:
  - start: [4.825, 6.600, 75.96]
    end: [4.400, 4.900, 0.0]
```
