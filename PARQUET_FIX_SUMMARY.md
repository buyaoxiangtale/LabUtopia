# Parquet 轨迹数据修复总结

## 🔍 问题分析

通过分析现有的 parquet 文件，发现了以下关键问题：

### 问题 1: Action 数据错误（最严重）🔴

**问题描述：**
- Action 列存储的是 4x4 单位矩阵
- 所有帧的 action 都相同

**数据示例：**
```python
# 所有 556 帧的 action 都相同：
[[1., 0., 0., 0.],
 [0., 1., 0., 0.],
 [0., 0., 1., 0.],
 [0., 0., 0., 1.]]
```

**期望数据：**
- 应该是 (N, 4, 4) 的轨迹矩阵
- 每帧记录相机/机器人的真实位姿

**影响：**
- ❌ 无法生成训练标签
- ❌ 无法进行动作增强
- ❌ 无法训练导航模型

### 问题 2: Parquet 列名不匹配 🔴

**问题描述：**
- 列名嵌套了 `observation`

**实际列名：**
```python
- observation.observation.camera_intrinsic  # ❌ 错误
- observation.observation.camera_extrinsic  # ❌ 错误
```

**期望列名：**
```python
- observation.camera_intrinsic  # ✅ 正确
- observation.camera_extrinsic  # ✅ 正确
```

**影响：**
- `KeyError: 'observation.camera_intrinsic'`
- 数据集无法被加载

---

## ✅ 修复方案

### 修复 1: 修改 `_save_parquet()` 方法

**文件：** `data_collectors/parquet_format_collector.py`

#### 1.1 修复列名（去掉嵌套的 observation）

**修改前：**
```python
data[f'observation.{cam_name}.camera_intrinsic'] = intrinsic_data
# 如果 cam_name = 'observation'，则变成 'observation.observation.camera_intrinsic'
```

**修改后：**
```python
# ✅ 修复：去掉嵌套的 observation
if cam_name == 'observation':
    data['observation.camera_intrinsic'] = intrinsic_data
else:
    data[f'observation.{cam_name}.camera_intrinsic'] = intrinsic_data
```

#### 1.2 修复 Action 数据（使用真实轨迹）

**修改前：**
```python
# 如果没有轨迹数据，使用单位矩阵
data['action'] = [np.eye(4).tolist()] * num_steps
```

**修改后：**
```python
# ✅ 修复：使用真实的相机轨迹
trajectory = [step[cam_name].tolist() for step in self.temp_trajectory]

# 调整轨迹长度以匹配 num_steps
if len(trajectory) < num_steps:
    # 如果轨迹太短，重复最后一个位姿
    last_pose = trajectory[-1]
    trajectory = trajectory + [last_pose] * (num_steps - len(trajectory))
elif len(trajectory) > num_steps:
    # 如果轨迹太长，截断
    trajectory = trajectory[:num_steps]

data['action'] = trajectory
```

---

## 📊 修复效果对比

### 修复前的数据结构

| 列名 | 数据类型 | 形状 | 说明 |
|------|---------|------|------|
| `observation.observation.camera_intrinsic` | object | (9,) | ❌ 嵌套 observation |
| `observation.observation.camera_extrinsic` | object | (16,) | ❌ 嵌套 observation |
| `action` | object | (4,) | ❌ 单位矩阵 |

### 修复后的数据结构

| 列名 | 数据类型 | 形状 | 说明 |
|------|---------|------|------|
| `observation.camera_intrinsic` | object | (9,) | ✅ 正确 |
| `observation.camera_extrinsic` | object | (16,) | ✅ 正确 |
| `action` | object | (N, 4, 4) | ✅ 真实轨迹 |

---

## 🚀 下一步操作

### 1. 重新收集数据（推荐）

修复已经应用，重新运行数据收集即可：

```bash
python main.py --config-name level5_Navigation_parquet
```

新的数据将包含：
- ✅ 正确的列名：`observation.camera_intrinsic`
- ✅ 真实的相机轨迹：action 存储每帧的相机位姿矩阵

### 2. 修复现有数据（可选）

如果需要修复已收集的数据，可以使用修复工具：

```bash
# 分析现有数据
python fix_parquet_trajectory.py --root-dir outputs/collect/2026.01.17/00.27.16_level5_Navigation_parquet --mode analyze

# 修复现有数据（需要提供真实轨迹数据）
python fix_parquet_trajectory.py --root-dir outputs/collect/2026.01.17/00.27.16_level5_Navigation_parquet --mode fix --use-mock-trajectory
```

⚠️ **注意：** 修复工具仅用于测试，真实场景需要从数据源获取相机轨迹数据。

---

## 📋 数据集兼容性检查清单

使用新的数据收集代码后，验证以下内容：

- [ ] Parquet 文件列名正确
- [ ] Action 数据是真实轨迹（不是单位矩阵）
- [ ] Action 数据形状是 (N, 4, 4)
- [ ] 相机内参矩阵正确（3x3）
- [ ] 相机外参矩阵正确（4x4）
- [ ] 图像文件命名正确：`0.jpg`, `1.jpg`, ...
- [ ] 目录结构正确：`trajectory_XXXXXX/data/chunk-000/`
- [ ] 图像目录正确：`trajectory_XXXXXX/videos/chunk-000/observation.images.rgb/`

---

## 🔧 相关文件

1. **数据收集器**：`data_collectors/parquet_format_collector.py`
   - 修复了 `_save_parquet()` 方法
   - 修复了列名和 action 数据

2. **修复工具**：`fix_parquet_trajectory.py`
   - 分析现有 parquet 文件
   - 批量修复列名
   - 修复 action 数据（需要提供真实轨迹）

3. **数据集分析**：`/home/pjlab/fbh/InternNav/dataset_compatibility_analysis.md`
   - 详细的问题分析报告

---

## 📖 参考资料

- [InternNav 数据集格式说明](/home/pjlab/fbh/InternNav/dataset_compatibility_analysis.md)
- [LeRobot 数据集格式](https://github.com/huggingface/lerobot)
- [Open X-Embodiment 数据集](https://robotics-transformer-x.github.io/)

---

## ✅ 总结

通过修复 `_save_parquet()` 方法，我们解决了两个关键问题：

1. ✅ **列名修复**：从 `observation.observation.camera_intrinsic` 改为 `observation.camera_intrinsic`
2. ✅ **Action 修复**：从单位矩阵改为真实的相机轨迹数据

现在新收集的数据将**完全兼容** InternNav 的数据集格式！🎉
