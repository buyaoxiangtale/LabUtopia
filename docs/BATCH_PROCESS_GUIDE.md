# 批量路径处理工具使用指南

## 📋 功能概述

`utils/batch_process_paths.py` 是一个批量路径处理工具，可以：
- 批量处理同一文件夹下的多个路径JSON文件
- 提取路径统计信息（长度、时间、效率等）
- 支持排序、过滤、导出CSV/JSON报告

## 🚀 快速开始

### 1. 处理所有路径文件

```bash
python utils/batch_process_paths.py --input outputs/
```

**输出示例**：
```
📁 在 outputs 中找到 4 个文件

✓ level5_goal_pairs_paths.json [0] - 长度: 5.48m, 点数: 110, 时间: 545s
✓ level5_goal_pairs_paths.json [1] - 长度: 1.21m, 点数: 25, 时间: 120s
✓ level5_goal_pairs_paths.json [2] - 长度: 8.90m, 点数: 178, 时间: 885s

统计摘要:
  总路径数: 3
  路径长度总计: 15.59m
  平均长度: 5.20m
  预计总时间: 1550秒 (25.8分钟)
```

### 2. 只处理匹配的文件

```bash
# 只处理包含 "goal" 的文件
python utils/batch_process_paths.py --input outputs/ --pattern "*goal*.json"

# 只处理特定前缀的文件
python utils/batch_process_paths.py --input outputs/ --pattern "level5_*.json"
```

### 3. 按指标排序

```bash
# 按路径长度排序（降序）
python utils/batch_process_paths.py --input outputs/ --sort-by distance

# 按预计时间排序
python utils/batch_process_paths.py --input outputs/ --sort-by time

# 按路径点数排序
python utils/batch_process_paths.py --input outputs/ --sort-by waypoints

# 按路径效率排序（直线/实际，升序）
python utils/batch_process_paths.py --input outputs/ --sort-by efficiency
```

### 4. 显示前N条路径

```bash
# 显示前10条最长路径
python utils/batch_process_paths.py --input outputs/ --sort-by distance --top 10

# 显示前5条最短时间路径
python utils/batch_process_paths.py --input outputs/ --sort-by time --top 5
```

### 5. 导出报告

```bash
# 导出为CSV
python utils/batch_process_paths.py --input outputs/ --output report.csv

# 导出为JSON
python utils/batch_process_paths.py --input outputs/ --output report.json

# 只显示统计摘要，不显示详细列表
python utils/batch_process_paths.py --input outputs/ --summary-only
```

## 📊 统计指标说明

### 基础指标

| 指标 | 说明 | 示例值 |
|------|------|--------|
| `total_distance` | 实际路径长度（米） | 5.48m |
| `straight_distance` | 直线距离（米） | 4.32m |
| `efficiency` | 路径效率（直线/实际） | 0.788 (78.8%) |
| `num_waypoints` | 路径点数量 | 110 |
| `num_segments` | 路径段数量 | 109 |
| `total_time` | 预计完成时间（秒） | 545s |
| `avg_velocity` | 平均速度（m/s） | 0.0101 |

### 路径效率

- **高效率 (>0.8)**: 路径接近直线，很少绕行
- **中等效率 (0.5-0.8)**: 路径有一些绕行，避开障碍物
- **低效率 (<0.5)**: 路径需要大量绕行，障碍物密集

## 📝 输出格式

### CSV格式

```csv
Source File,Path Index,Start (x,y),End (x,y),Total Distance (m),Straight Distance (m),Efficiency,Waypoints,Segments,Total Time (s),Avg Velocity (m/s)
level5_goal_pairs_paths.json,0,"(1.92, 4.50)","(6.00, 5.90)",5.480,4.317,0.788,110,109,545.0,0.0101
```

### JSON格式

```json
{
  "summary": {
    "total_paths": 3,
    "statistics": {
      "total_distance": 15.59,
      "total_time": 1550.0,
      "avg_distance": 5.20,
      "avg_time": 516.67,
      "avg_efficiency": 0.699
    }
  },
  "paths": [
    {
      "source_file": "level5_goal_pairs_paths.json",
      "path_index": 0,
      "start": [1.916, 4.5],
      "end": [6.0, 5.899],
      "total_distance": 5.48,
      ...
    }
  ]
}
```

## 🎯 使用场景

### 场景1：分析所有路径文件

```bash
# 处理outputs文件夹下的所有路径文件
python utils/batch_process_paths.py --input outputs/ --output outputs/all_paths_summary.csv
```

**输出**：
- 控制台显示统计摘要
- CSV文件包含每条路径的详细信息
- 可在Excel中打开进行进一步分析

### 场景2：找出最长/最短的路径

```bash
# 找出最长的5条路径
python utils/batch_process_paths.py --input outputs/ --sort-by distance --top 5

# 找出最短的5条路径（手动查看输出的排序结果）
```

### 场景3：分析路径效率

```bash
# 按效率排序，找出最直和最绕的路径
python utils/batch_process_paths.py --input outputs/ --sort-by efficiency --top 10
```

### 场景4：时间估算

```bash
# 计算所有路径的总时间
python utils/batch_process_paths.py --input outputs/ --summary-only

# 输出会显示：
# 预计时间 (秒):
#   总计: 1550 (25.8 分钟)
```

### 场景5：生成报告

```bash
# 生成完整报告（CSV格式）
python utils/batch_process_paths.py \
    --input outputs/ \
    --pattern "*goal_pairs*.json" \
    --sort-by distance \
    --output outputs/paths_report.csv

# 在Excel/Google Sheets中打开分析
```

## 📖 命令行参数

| 参数 | 说明 | 默认值 | 示例 |
|------|------|--------|------|
| `--input` | 输入目录路径 | `outputs/` | `--input data/paths/` |
| `--pattern` | 文件匹配模式 | `*.json` | `--pattern "*nav*.json"` |
| `--sort-by` | 排序字段 | 无 | `--sort-by distance` |
| `--output` | 输出文件路径 | 无 | `--output report.csv` |
| `--top` | 显示前N条路径 | 5 | `--top 10` |
| `--summary-only` | 只显示统计摘要 | False | `--summary-only` |

## 💡 高级用法

### 组合多个参数

```bash
# 处理特定文件，按时间排序，显示前10条，导出CSV
python utils/batch_process_paths.py \
    --input outputs/ \
    --pattern "*goal*.json" \
    --sort-by time \
    --top 10 \
    --output outputs/fastest_paths.csv
```

### 过滤特定路径

```bash
# 只处理 goal_pairs 相关的路径
python utils/batch_process_paths.py --input outputs/ --pattern "*goal_pairs*.json"

# 只处理导航目标文件
python utils/batch_process_paths.py --input outputs/ --pattern "*_nav_target.json"
```

### 批量分析多个文件夹

```bash
# 创建一个简单的shell脚本
for dir in outputs/*/; do
    python utils/batch_process_paths.py --input "$dir" --output "${dir}summary.csv"
done
```

## 🔍 示例输出解读

### 控制台输出

```
🏆 前 3 条路径（按 distance 排序）:
--------------------------------------------------------------------------------

1. level5_goal_pairs_paths.json [2]
   起点: (4.90, 6.00)
   终点: (5.13, 2.50)
   长度: 8.90m, 时间: 885s, 点数: 178

2. level5_goal_pairs_paths.json [0]
   起点: (1.92, 4.50)
   终点: (6.00, 5.90)
   长度: 5.48m, 时间: 545s, 点数: 110

3. level5_goal_pairs_paths.json [1]
   起点: (6.00, 5.90)
   终点: (4.90, 6.00)
   长度: 1.21m, 时间: 120s, 点数: 25
```

**解读**：
- 第1条：最长路径（8.90m），需要178个路径点，预计14.75分钟
- 第2条：中等长度，路径效率较高（直线距离4.32m，实际5.48m）
- 第3条：最短路径，只有25个路径点，只需2分钟

### 统计摘要

```
路径效率 (直线/实际):
  平均: 0.699
  最高: 0.915
  最低: 0.394
```

**解读**：
- 平均路径效率约70%，说明大部分路径需要绕行
- 最高效率91.5%，接近直线（第3条路径）
- 最低效率39.4%，需要大量绕行（第3条路径）

## 🛠️ 工作原理

### 1. 文件扫描

```
input_dir/
  ├── file1.json  ✓ 匹配 *.json
  ├── file2.txt  ✗ 不匹配
  └── file3.json  ✓ 匹配
```

### 2. 数据提取

```python
# 从每条路径提取：
- start, end（起终点）
- total_distance（路径长度）
- straight_distance（直线距离）
- efficiency（效率 = 直线/实际）
- num_waypoints（路径点数）
- total_time（预计时间）
- avg_velocity（平均速度）
```

### 3. 统计分析

```python
# 计算统计量：
- 总计、平均、中位数
- 最大值、最小值
- 标准差
```

### 4. 排序与输出

```python
# 按指定字段排序
# 显示前N条
# 导出CSV/JSON
```

## ⚠️ 注意事项

1. **文件格式**
   - 只支持JSON格式的路径文件
   - 文件必须包含 `total_distance`, `start`, `end` 等字段

2. **性能**
   - 对于大量文件（>100），处理可能需要几秒钟
   - 建议使用 `--pattern` 过滤以减少处理时间

3. **内存使用**
   - 所有路径数据会加载到内存中
   - 对于超大型数据集，建议分批处理

4. **输出文件**
   - CSV文件可以用Excel/Google Sheets打开
   - JSON文件保留完整数据，便于编程处理

## 🔗 相关工具

| 工具 | 用途 |
|------|------|
| `path_planning_precompute.py` | 预计算路径（生成这些文件） |
| `find_nav_target_for_item.py` | 为物品生成导航点 |
| `test_goal_pairs_paths.py` | 测试特定配置的路径 |

## 📞 常见问题

### Q1: 为什么有些路径长度为0？

**A**: 这些是物品导航目标文件（`beaker_nav_target.json`），不包含路径信息，只有导航点坐标。

### Q2: 如何只处理有效路径？

**A**: 使用过滤参数：
```bash
python utils/batch_process_paths.py \
    --input outputs/ \
    --pattern "*goal_pairs*.json"  # 只处理包含路径数据的文件
```

### Q3: CSV文件在Excel中显示乱码？

**A**: 使用UTF-8编码打开Excel：
1. 打开Excel
2. 数据 → 从文本/CSV导入
3. 选择UTF-8编码

### Q4: 如何按效率从高到低排序？

**A**: 工具默认按降序排序（大到小），对于效率来说，高效率在前：
```bash
python utils/batch_process_paths.py --input outputs/ --sort-by efficiency
```

## ✅ 总结

这个批量处理工具可以帮助你：
- ✅ 快速分析大量路径文件
- ✅ 找出最长/最短/最快/最慢的路径
- ✅ 生成可用于报告的CSV/JSON文件
- ✅ 统计路径质量指标（效率、速度等）
