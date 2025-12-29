# 导航速度信息集成指南

## 📋 概述

本指南介绍如何在真实的LabUtopia导航场景中集成和使用路径段速度信息功能。

## 🚀 快速开始

### 1. 使用新任务类

确保你的配置文件使用 `navigation_new_cp` 任务类型：

```yaml
# config/level5_Navigation_velocity_demo.yaml
task_type: "navigation_new_cp"  # 支持速度信息
controller_type: "navigation_new"
```

### 2. 配置速度显示

在任务配置中启用速度信息显示：

```yaml
task:
  show_velocity_info: true  # 显示详细速度信息
  max_linear_speed: 0.02    # 最大线速度 (m/s)
  position_threshold: 0.08   # 位置阈值 (m)
```

### 3. 运行导航任务

```bash
# 使用新的配置文件运行
python main.py --config config/level5_Navigation_velocity_demo.yaml
```

## 📊 访问速度信息

### 在任务状态中

任务的 `step()` 方法现在返回包含速度信息的状态：

```python
state = task.step()

# 访问所有路径段速度信息
segment_velocities = state['segment_velocities']

# 访问当前段速度信息
current_segment = state['current_segment_velocity']
```

### 速度信息结构

#### 路径段速度信息 (`segment_velocities`)
```python
{
    "segment_idx": 0,           # 段索引
    "start_point": [x1, y1],    # 起点坐标
    "end_point": [x2, y2],      # 终点坐标
    "distance": 1.0,            # 段长度 (m)
    "direction": 0.0,           # 方向角度 (rad)
    "direction_deg": 0.0,       # 方向角度 (度)
    "velocity_magnitude": 0.02, # 速度大小 (m/s)
    "velocity_vector": [vx, vy] # 速度向量 (m/s)
}
```

#### 当前段速度信息 (`current_segment_velocity`)
与上面相同结构，表示机器人当前正在执行的路径段。

## 🎮 在控制器中使用

### 基本访问

```python
def step(self, state):
    # 访问速度信息
    if 'segment_velocities' in state:
        velocities = state['segment_velocities']
        print(f"路径有 {len(velocities)} 个段")

    if 'current_segment_velocity' in state:
        current = state['current_segment_velocity']
        speed = current['velocity_magnitude']
        direction = current['direction_deg']
        print(f"当前速度: {speed*100:.1f}cm/s, 方向: {direction:.1f}°")
```

### 高级应用：速度自适应控制

```python
def step(self, state):
    # 根据当前段速度调整控制参数
    if 'current_segment_velocity' in state:
        current_seg = state['current_segment_velocity']

        # 距离越短，控制更保守
        if current_seg['distance'] < 0.5:
            # 减小PID增益，避免震荡
            self.k_p_linear = 1.0  # 降低增益
        else:
            # 距离较长，可以更快
            self.k_p_linear = 2.5  # 正常增益

        # 转弯角度大时，减慢速度
        if abs(current_seg['direction_deg']) > 45:
            self.max_linear_speed = 0.01  # 减慢速度
        else:
            self.max_linear_speed = 0.02  # 正常速度
```

## 🔍 调试和监控

### 控制台输出

运行时会自动显示：
- 📍 路径规划时的速度统计
- 🚀 每个路径段的详细信息
- 📍 当前段的速度状态

### 自定义监控

```python
def step(self, state):
    # 记录速度历史用于分析
    if hasattr(self, 'speed_history') == False:
        self.speed_history = []

    if 'current_segment_velocity' in state:
        current = state['current_segment_velocity']
        self.speed_history.append({
            'frame': self.frame_count,
            'speed': current['velocity_magnitude'],
            'direction': current['direction_deg']
        })

        # 每100帧打印一次统计
        if self.frame_count % 100 == 0:
            avg_speed = sum(h['speed'] for h in self.speed_history[-100:]) / 100
            print(f"最近100帧平均速度: {avg_speed*100:.1f}cm/s")
```

## 📈 应用场景

### 1. 强化学习训练

```python
def get_reward(self, state):
    if 'current_segment_velocity' in state:
        current = state['current_segment_velocity']

        # 奖励：沿着路径方向前进
        reward = current['velocity_magnitude'] * 10

        # 惩罚：偏离路径方向
        angle_error = abs(current['direction'] - robot_heading)
        reward -= angle_error * 0.1

        return reward
```

### 2. 路径优化

```python
def optimize_path(self, original_path):
    """根据速度信息优化路径"""
    velocities = self._calculate_segment_velocities()

    # 找出速度太慢的段
    slow_segments = [v for v in velocities if v['velocity_magnitude'] < 0.01]

    # 重新规划这些段
    for segment in slow_segments:
        # 简化路径或调整角度
        pass

    return optimized_path
```

### 3. 安全监控

```python
def safety_check(self, state):
    """基于速度信息的安全检查"""
    if 'current_segment_velocity' in state:
        current = state['current_segment_velocity']

        # 检查速度是否过高
        if current['velocity_magnitude'] > self.max_safe_speed:
            print("⚠️ 速度过高，降低速度")
            return False

        # 检查是否接近障碍物
        if self._check_obstacle_ahead():
            print("⚠️ 前方有障碍物，减速")
            return False

    return True
```

## 🛠️ 故障排除

### 问题：没有速度信息

**检查：**
1. 确认使用 `navigation_new_cp` 任务类型
2. 确认路径已成功规划
3. 检查任务的 `step()` 方法是否被正确调用

### 问题：速度显示不正确

**检查：**
1. 确认 `_calculate_segment_velocities()` 方法正常工作
2. 检查路径点坐标是否正确
3. 验证速度计算公式是否正确

### 问题：性能影响

**优化：**
1. 减少显示频率（每50帧显示一次）
2. 只在调试时启用详细输出
3. 缓存计算结果避免重复计算

## 📚 相关文件

- `tasks/navigation_task_new_cp_cp.py` - 任务实现
- `controllers/navigation_controller_test_speed.py` - 控制器示例
- `config/level5_Navigation_velocity_demo.yaml` - 配置示例






