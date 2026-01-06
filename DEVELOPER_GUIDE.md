# LabUtopia 开发者指南

> 本文档面向开发者，提供扩展和修改 LabUtopia 的详细指南

## 目录

- [开发环境设置](#开发环境设置)
- [代码结构理解](#代码结构理解)
- [添加新任务](#添加新任务)
- [添加新机器人](#添加新机器人)
- [添加新策略模型](#添加新策略模型)
- [调试技巧](#调试技巧)
- [最佳实践](#最佳实践)
- [常见问题解决](#常见问题解决)

---

## 开发环境设置

### 1. 环境变量配置

推荐在项目根目录创建 `.env` 文件：

```bash
# Isaac Sim 路径
ISAACSIM_PATH=/path/to/isaac-sim

# Python 路径
PYTHONPATH=$ISAACSIM_PATH:$PYTHONPATH

# GPU 设置
CUDA_VISIBLE_DEVICES=0
```

### 2. VS Code 配置

运行以下命令生成 VS Code 设置：

```bash
python -m isaacsim --generate-vscode-settings
```

推荐的 VS Code 扩展：
- Python
- Pylance
- YAML
- GitLens

### 3. 开发工作流

```bash
# 1. 创建功能分支
git checkout -b feature/my-new-feature

# 2. 开发和测试
python main.py --config-name level1_pick --headless

# 3. 提交更改
git add .
git commit -m "Add my new feature"

# 4. 推送到远程
git push origin feature/my-new-feature
```

---

## 代码结构理解

### 核心概念

LabUtopia 采用模块化设计，主要包含以下核心组件：

```
┌─────────────────────────────────────────────┐
│                  main.py                     │  主入口
└──────────────────┬──────────────────────────┘
                   │
        ┌──────────┴──────────┐
        ▼                     ▼
┌──────────────┐      ┌──────────────┐
│  World       │      │   Robot      │
│ (Isaac Sim)  │      │  Factory     │
└──────────────┘      └──────────────┘
        │                     │
        └──────────┬──────────┘
                   ▼
          ┌────────────────┐
          │  Task Factory  │
          └────────┬───────┘
                   ▼
          ┌────────────────┐
          │     Task       │
          │  (任务逻辑)     │
          └────────┬───────┘
                   ▼
          ┌────────────────┐
          │  Controller    │
          │  (控制逻辑)     │
          └────────┬───────┘
                   ▼
          ┌────────────────┐
          │  Robot Action  │
          └────────────────┘
```

### 数据流向

```
场景状态 → Task.step() → State Dict → Controller.step() → Action → Robot
                                        ↓
                                  Data Collector
                                        ↓
                                      Dataset
```

---

## 添加新任务

### 步骤 1: 创建任务类

在 `tasks/` 目录创建新文件 `my_new_task.py`：

```python
from tasks.base_task import BaseTask
from typing import Dict, Any
import numpy as np

class MyNewTask(BaseTask):
    """
    我的自定义任务

    任务描述：简要描述这个任务做什么
    """

    def __init__(self, cfg, world, stage, robot):
        """
        初始化任务

        Args:
            cfg: 任务配置
            world: 仿真世界实例
            stage: USD 舞台
            robot: 机器人实例
        """
        super().__init__(cfg, world, stage, robot)

        # 自定义初始化
        self.my_custom_param = getattr(cfg.task, 'custom_param', 100)

        # 设置对象和材质
        self.setup_my_objects()

    def setup_my_objects(self):
        """设置自定义对象"""
        # 从配置读取对象配置
        if hasattr(self.cfg.task, 'my_objects'):
            self.my_objects = self.cfg.task.my_objects
        else:
            self.my_objects = []

    def reset(self):
        """重置任务状态"""
        super().reset()

        # 自定义重置逻辑
        # 例如：随机化对象位置
        for obj_config in self.my_objects:
            self.randomize_object_position(
                obj_config['path'],
                obj_config['position_range']
            )

    def step(self) -> Dict[str, Any]:
        """
        执行任务的一个步骤

        Returns:
            Dict[str, Any]: 包含观测数据的字典
        """
        self.frame_idx += 1

        # 获取机器人状态
        joint_positions = self.robot.get_joint_positions()

        # 构建状态字典
        state = self.get_basic_state_info(
            joint_positions=joint_positions,
            object_path=self.object_path if hasattr(self, 'object_path') else None,
            additional_info={
                'custom_info': 'my_custom_value'
            }
        )

        # 检查任务完成条件
        if self.check_task_complete():
            self.reset_needed = True

        return state

    def check_task_complete(self) -> bool:
        """检查任务是否完成"""
        # 实现你的完成条件检查
        # 例如：检查物体是否到达目标位置
        return False
```

### 步骤 2: 创建控制器类

在 `controllers/` 目录创建 `my_new_controller.py`：

```python
from controllers.base_controller import BaseController
from typing import Dict, Any, Tuple
import numpy as np

class MyNewController(BaseController):
    """
    我的自定义控制器
    """

    def __init__(self, cfg, robot):
        """
        初始化控制器

        Args:
            cfg: 控制器配置
            robot: 机器人实例
        """
        super().__init__(cfg, robot)

        # 自定义参数
        self.my_threshold = getattr(cfg.controller, 'threshold', 0.01)

        # 控制状态
        self.phase = "phase1"  # phase1, phase2, etc.

    def step(self, state: Dict[str, Any]) -> Tuple[np.ndarray, bool, bool]:
        """
        执行一步控制

        Args:
            state: 当前状态字典

        Returns:
            Tuple[action, done, is_success]:
                - action: 机器人动作
                - done: 是否完成
                - is_success: 是否成功
        """
        if self.mode == "collect":
            return self._collect_step(state)
        else:
            return self._infer_step(state)

    def _collect_step(self, state: Dict[str, Any]) -> Tuple[np.ndarray, bool, bool]:
        """数据收集模式的控制逻辑"""
        # 记录数据
        self.data_collector.add_step(
            state=state['camera_data'],
            action=state['joint_positions']
        )

        # 实现你的控制逻辑
        action = self._compute_action(state)

        # 检查完成条件
        done, is_success = self._check_completion(state)

        if done:
            # 保存回合数据
            self.data_collector.save_episode(
                language_instruction=self.get_language_instruction()
            )

        return action, done, is_success

    def _infer_step(self, state: Dict[str, Any]) -> Tuple[np.ndarray, bool, bool]:
        """推理模式的控制逻辑"""
        # 准备观测
        obs = self._prepare_observation(state)

        # 使用推理引擎预测动作
        action = self.inference_engine.predict(obs)

        # 检查完成条件
        done, is_success = self._check_completion(state)

        return action, done, is_success

    def _compute_action(self, state: Dict[str, Any]) -> np.ndarray:
        """计算控制动作"""
        # 实现你的动作计算逻辑
        # 可以使用 RMPFlow 控制器
        target_pos = np.array([0.5, 0.0, 0.5])
        current_pos = state['object_position'] if 'object_position' in state else None

        if current_pos is not None:
            # 计算目标动作
            action = self.rmp_controller.compute_robot_command(
                target_position=target_pos,
                target_orientation=None
            )
            return action

        return None

    def _check_completion(self, state: Dict[str, Any]) -> Tuple[bool, bool]:
        """检查任务是否完成"""
        # 实现你的完成条件检查
        is_success = False

        # 示例：检查物体是否在目标位置
        if 'object_position' in state:
            target = np.array([0.5, 0.0, 0.5])
            current = state['object_position']
            distance = np.linalg.norm(current - target)

            if distance < self.my_threshold:
                self.check_success_counter += 1
                if self.check_success_counter >= self.REQUIRED_SUCCESS_STEPS:
                    is_success = True
            else:
                self.check_success_counter = 0

        done = is_success or self.frame_idx > self.cfg.task.max_steps

        return done, is_success

    def _prepare_observation(self, state: Dict[str, Any]) -> Dict[str, Any]:
        """准备推理所需的观测数据"""
        obs = {}

        # 添加图像数据
        for key, value in state['camera_data'].items():
            obs[key] = value

        # 添加其他状态
        obs['joint_positions'] = state['joint_positions']
        obs['gripper_position'] = state['gripper_position']

        return obs

    def get_language_instruction(self) -> str:
        """获取语言指令"""
        return self.language_instruction or "Perform my custom task"
```

### 步骤 3: 创建配置文件

在 `config/` 目录创建 `level1_my_new_task.yaml`：

```yaml
# 基础配置
name: level1_my_new_task
task_type: "my_new_task"
controller_type: "my_new_controller"
mode: "collect"  # 或 "infer"

# 场景配置
usd_path: "assets/chemistry_lab/my_task/scene.usd"

# 任务参数
task:
  max_steps: 1000

  # 自定义参数
  custom_param: 100

  # 对象配置
  obj_paths:
    - path: "/World/my_object"
      position_range:
        x: [0.2, 0.3]
        y: [-0.1, 0.1]
        z: [0.85, 0.85]

  # 可选：材质配置
  material_paths:
    - path: "/World/my_object"
      materials:
        - "/World/Looks/Material1"
        - "/World/Looks/Material2"
      test_materials:
        - "/World/Looks/TestMaterial"

# 相机配置
cameras_names: ["camera_1", "camera_2"]
cameras:
  - prim_path: "/World/Camera1"
    name: "camera_1"
    translation: [2, 0, 2]
    resolution: [256, 256]
    focal_length: 6
    orientation: [0.61237, 0.35355, 0.35355, 0.61237]
    image_type: "rgb"  # 可选: rgb, depth, pointcloud, rgb+depth, segmentation
    clipping_range: [0.1, 10.0]

  - prim_path: "/World/Camera2"
    name: "camera_2"
    translation: [-2, 0, 2]
    resolution: [256, 256]
    focal_length: 6
    orientation: [-0.61237, 0.35355, -0.35355, 0.61237]
    image_type: "rgb"

# 机器人配置
robot:
  type: "franka"  # franka, ridgebase, fetch
  position: [-0.4, -0, 0.71]

# 控制器配置
controller:
  threshold: 0.01

# 数据收集配置
collector:
  type: "default"
  compression: null  # 或 "zstd", "lz4"

# 运行配置
max_episodes: 100

# 输出目录
multi_run:
  run_dir: "outputs/collect/${now:%Y.%m.%d}/${now:%H.%M.%S}_${name}"
```

### 步骤 4: 注册到工厂

#### 注册任务

在 `factories/task_factory.py` 中添加：

```python
from tasks.my_new_task import MyNewTask

def create_task(task_type, cfg, world, stage, robot):
    task_classes = {
        # ... 现有任务 ...
        "my_new_task": MyNewTask,
    }

    if task_type not in task_classes:
        raise ValueError(f"Unknown task type: {task_type}")

    return task_classes[task_type](cfg, world, stage, robot)
```

#### 注册控制器

在 `factories/controller_factory.py` 中添加：

```python
from controllers.my_new_controller import MyNewController

def create_controller(controller_type, cfg, robot):
    controller_classes = {
        # ... 现有控制器 ...
        "my_new_controller": MyNewController,
    }

    if controller_type not in controller_classes:
        raise ValueError(f"Unknown controller type: {controller_type}")

    return controller_classes[controller_type](cfg, robot)
```

### 步骤 5: 测试新任务

```bash
# 运行新任务
python main.py --config-name level1_my_new_task

# 无头模式测试
python main.py --config-name level1_my_new_task --headless

# 禁用视频加速测试
python main.py --config-name level1_my_new_task --no-video
```

---

## 添加新机器人

### 步骤 1: 创建机器人配置

在 `robots/` 目录创建你的机器人文件夹：

```
robots/
└── my_robot/
    ├── robot_descriptor.yaml
    └── usd/
        └── my_robot.usd
```

`robot_descriptor.yaml` 示例：

```yaml
robot_name: "my_robot"
robot_type: "articulation"

# USD 文件路径
usd_path: "robots/my_robot/usd/my_robot.usd"

# 机器人默认位置
default_position: [0, 0, 0]

# 关节配置
joints:
  - "joint1"
  - "joint2"
  - "joint3"

# 夹爪配置
end_effector: "gripper_joint"
```

### 步骤 2: 创建机器人控制器

在 `controllers/robot_controllers/` 创建控制器：

```python
from controllers.robot_controllers.trajectory_controller import TrajectoryController

class MyRobotController(TrajectoryController):
    """自定义机器人控制器"""

    def __init__(self, name, robot_articulation, **kwargs):
        super().__init__(name, robot_articulation, **kwargs)

        # 自定义初始化
        self.dof = self.robot_articulation.num_dof

    def compute_robot_command(self, target_position, target_orientation=None):
        """计算机器人命令"""
        # 实现你的控制逻辑
        pass
```

### 步骤 3: 注册机器人

在 `factories/robot_factory.py` 中添加：

```python
def create_robot(robot_type, position):
    if robot_type == "franka":
        from robots.franka import Franka
        robot = Franka(prim_path="/World/Franka", name="franka")
        robot.world_pose = (position, [0, 0, 0, 1])

    elif robot_type == "my_robot":
        from robots.my_robot import MyRobot
        robot = MyRobot(prim_path="/World/MyRobot", name="my_robot")
        robot.world_pose = (position, [0, 0, 0, 1])

    else:
        raise ValueError(f"Unknown robot type: {robot_type}")

    return robot
```

### 步骤 4: 测试新机器人

创建配置文件并测试：

```yaml
robot:
  type: "my_robot"
  position: [0, 0, 0]
```

```bash
python main.py --config-name test_my_robot
```

---

## 添加新策略模型

### 步骤 1: 定义模型架构

在 `policy/model/` 创建你的模型文件夹：

```python
# policy/model/my_model/my_model.py
import torch
import torch.nn as nn

class MyModel(nn.Module):
    """自定义策略模型"""

    def __init__(self, config):
        super().__init__()

        # 模型参数
        self.obs_dim = config.obs_dim
        self.action_dim = config.action_dim
        self.hidden_dim = config.hidden_dim

        # 网络结构
        self.encoder = self._build_encoder()
        self.decoder = self._build_decoder()

    def _build_encoder(self):
        """构建编码器"""
        return nn.Sequential(
            nn.Linear(self.obs_dim, self.hidden_dim),
            nn.ReLU(),
            nn.Linear(self.hidden_dim, self.hidden_dim),
            nn.ReLU()
        )

    def _build_decoder(self):
        """构建解码器"""
        return nn.Sequential(
            nn.Linear(self.hidden_dim, self.hidden_dim),
            nn.ReLU(),
            nn.Linear(self.hidden_dim, self.action_dim)
        )

    def forward(self, obs):
        """前向传播"""
        features = self.encoder(obs)
        action = self.decoder(features)
        return action

    def get_action(self, obs):
        """推理时获取动作"""
        with torch.no_grad():
            action = self.forward(obs)
        return action.cpu().numpy()
```

### 步骤 2: 创建策略类

在 `policy/policy/` 创建策略文件：

```python
# policy/policy/my_model_policy.py
from policy.policy.base_image_policy import BaseImagePolicy
from policy.model.my_model.my_model import MyModel
import torch

class MyModelPolicy(BaseImagePolicy):
    """自定义策略"""

    def __init__(self, config, device):
        super().__init__(config, device)

        # 初始化模型
        self.model = MyModel(config.model).to(device)

        # 优化器
        self.optimizer = torch.optim.Adam(
            self.model.parameters(),
            lr=config.training.lr
        )

    def forward(self, obs):
        """前向传播"""
        return self.model(obs)

    def compute_loss(self, batch):
        """计算损失"""
        # 实现你的损失函数
        pred_action = self.model(batch['obs'])
        loss = torch.nn.functional.mse_loss(pred_action, batch['action'])
        return loss

    def update(self, batch):
        """更新模型"""
        self.optimizer.zero_grad()
        loss = self.compute_loss(batch)
        loss.backward()
        self.optimizer.step()
        return loss.item()

    def get_action(self, obs):
        """获取动作"""
        return self.model.get_action(obs)
```

### 步骤 3: 创建训练工作空间

在 `policy/workspace/` 创建训练文件：

```python
# policy/workspace/train_my_model_workspace.py
from policy.workspace.base_workspace import BaseWorkspace

class TrainMyModelWorkspace(BaseWorkspace):
    """自定义模型训练工作空间"""

    def __init__(self, config):
        super().__init__(config)

        # 初始化策略
        from policy.policy.my_model_policy import MyModelPolicy
        self.policy = MyModelPolicy(
            config=self.config,
            device=self.device
        )

    def train_step(self, batch):
        """训练步骤"""
        loss = self.policy.update(batch)
        return {"loss": loss}

    def val_step(self, batch):
        """验证步骤"""
        with torch.no_grad():
            loss = self.policy.compute_loss(batch)
        return {"val_loss": loss.item()}
```

### 步骤 4: 创建配置文件

在 `policy/config/` 创建训练配置：

```yaml
# policy/config/train_my_model_workspace.yaml

defaults:
  - task: my_model_task
  - override hydra/launcher: basic

# 工作空间配置
workspace:
  _target_: policy.workspace.train_my_model_workspace.TrainMyModelWorkspace

# 策略配置
policy:
  _target_: policy.policy.my_model_policy.MyModelPolicy
  device: "cuda:0"

  # 模型配置
  model:
    obs_dim: 256
    action_dim: 7
    hidden_dim: 256

# 训练配置
training:
  device: "cuda:0"
  seed: 42
  num_epochs: 5000
  lr: 1.0e-4
  batch_size: 64
  checkpoint_every: 50
  val_every: 10

# 数据加载器配置
dataloader:
  batch_size: 64
  num_workers: 4
  shuffle: true

# 优化器配置
optimizer:
  _target_: torch.optim.Adam
  lr: 1.0e-4
```

创建任务配置：

```yaml
# policy/config/task/my_model_task.yaml

# 形状元数据
shape_meta:
  # 观测形状
  obs:
    camera_1_rgb:
      type: "rgb"
      shape: [3, 256, 256]
    camera_2_rgb:
      type: "rgb"
      shape: [3, 256, 256]
    joint_positions:
      type: "low_dim"
      shape: [7]

  # 动作形状
  action:
    shape: [7]

# 数据集配置
dataset:
  _target_: policy.dataset.base_dataset.BaseDataset
  dataset_path: "path/to/your/dataset"

# 数据键映射
data_keys:
  # 观测键
  obs:
    camera_1_rgb: "camera_1_rgb"
    camera_2_rgb: "camera_2_rgb"
    joint_positions: "joint_positions"

  # 动作键
  action: "action"
```

### 步骤 5: 训练模型

```bash
python train.py --config-name train_my_model_workspace
```

---

## 调试技巧

### 1. 启用详细日志

```python
import logging

# 设置日志级别
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# 在代码中添加日志
logger.debug("Debug information")
logger.info("Information")
logger.warning("Warning")
logger.error("Error")
```

### 2. 使用调试模式

```bash
# 使用 Python 调试器
python -m pdb main.py --config-name level1_pick

# 在 VS Code 中设置断点
# 点击行号左侧设置断点，然后按 F5 开始调试
```

### 3. 可视化调试

```python
# 在控制器中添加可视化
def step(self, state):
    # 可视化对象位置
    if 'object_position' in state:
        pos = state['object_position']
        print(f"Object position: {pos}")

    # 可视化相机图像
    if 'camera_display' in state:
        import cv2
        for name, img in state['camera_display'].items():
            cv2.imshow(name, img)
        cv2.waitKey(1)
```

### 4. 单元测试

在 `tests/` 目录创建测试文件：

```python
# tests/test_my_new_task.py
import unittest
from tasks.my_new_task import MyNewTask

class TestMyNewTask(unittest.TestCase):
    def setUp(self):
        """设置测试环境"""
        self.cfg = self._load_test_config()
        # 初始化任务...

    def test_task_initialization(self):
        """测试任务初始化"""
        self.assertIsNotNone(self.task)

    def test_task_step(self):
        """测试任务步骤"""
        state = self.task.step()
        self.assertIsNotNone(state)

    def _load_test_config(self):
        """加载测试配置"""
        # 返回测试配置...
        pass

if __name__ == '__main__':
    unittest.main()
```

运行测试：

```bash
python -m unittest tests/test_my_new_task.py
```

### 5. 性能分析

```python
import cProfile
import pstats

# 性能分析
profiler = cProfile.Profile()
profiler.enable()

# 运行你的代码
# ...

profiler.disable()

# 打印结果
stats = pstats.Stats(profiler)
stats.sort_stats('cumtime')
stats.print_stats(10)
```

---

## 最佳实践

### 1. 代码风格

遵循 PEP 8 规范：

```python
# 好的命名
def calculate_object_distance(obj1_pos, obj2_pos):
    """计算两个对象之间的距离"""
    return np.linalg.norm(obj1_pos - obj2_pos)

# 不好的命名
def calc_dist(p1, p2):
    pass
```

### 2. 文档字符串

使用 Google 风格的文档字符串：

```python
def my_function(param1, param2):
    """
    函数的简短描述

    更详细的描述可以在这里写多行。

    Args:
        param1 (int): 参数1的描述
        param2 (str): 参数2的描述

    Returns:
        bool: 返回值的描述

    Raises:
        ValueError: 什么时候抛出异常

    Examples:
        >>> my_function(1, "test")
        True
    """
    pass
```

### 3. 配置管理

使用 Hydra 配置管理：

```yaml
# config/base_config.yaml
defaults:
  - robot: franka
  - task: pick

# 可以覆盖默认配置
robot:
  type: franka

# 使用继承
defaults:
  - base_config
  - override robot: ridgebase
```

### 4. 错误处理

```python
def safe_operation():
    """安全操作示例"""
    try:
        result = risky_operation()
        return result
    except ValueError as e:
        logger.error(f"Value error: {e}")
        return None
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        raise
    finally:
        cleanup()
```

### 5. 资源管理

```python
# 使用上下文管理器
with open('file.txt', 'r') as f:
    content = f.read()

# 确保资源释放
def close_resources():
    if hasattr(self, 'data_collector'):
        self.data_collector.close()
    if hasattr(self, 'robot'):
        self.robot.cleanup()
```

### 6. 版本控制

```bash
# .gitignore 示例
__pycache__/
*.pyc
outputs/
*.usd
.venv/
.env

# 提交前检查
git diff --check
```

---

## 常见问题解决

### 1. Isaac Sim 相关问题

**问题:** Isaac Sim 无法启动

**解决方案:**
```bash
# 检查 Isaac Sim 路径
echo $ISAACSIM_PATH

# 重新安装 Isaac Sim
pip uninstall isaacsim
pip install isaacsim[all]==5.1.0 --extra-index-url https://pypi.nvidia.com
```

**问题:** GPU 渲染问题

**解决方案:**
```bash
# 设置环境变量
export ISAACSIM_GPU_ENABLED=1

# 或使用 CPU 后端
python main.py --backend numpy
```

### 2. 内存问题

**问题:** 内存不足

**解决方案:**
```python
# 减少批处理大小
batch_size: 32  # 从 64 减少

# 减少图像分辨率
resolution: [128, 128]  # 从 [256, 256] 减少

# 启用压缩
compression: "zstd"
```

### 3. 导航问题

**问题:** 机器人无法到达目标

**解决方案:**
```python
# 检查导航网格
# 确保场景文件包含有效的导航网格

# 调整容差
tolerance: 0.1  # 增大容差

# 检查路径规划
from utils.a_star import AStarPlanner
planner = AStarPlanner(grid_size, obstacles)
path = planner.plan_path(start, goal)
print(f"Path length: {len(path)}")
```

### 4. 数据收集问题

**问题:** 数据未保存

**解决方案:**
```python
# 确保调用 save_episode
self.data_collector.save_episode()

# 检查目录权限
import os
os.makedirs(save_dir, exist_ok=True)

# 检查磁盘空间
df -h
```

### 5. 训练问题

**问题:** 训练损失不下降

**解决方案:**
```python
# 检查学习率
lr: 1.0e-4  # 尝试不同的学习率

# 检查数据归一化
from policy.common.normalize_util import Normalizer
normalizer = Normalizer(mode="normal")
normalized_data = normalizer.normalize(data)

# 检查梯度
for name, param in model.named_parameters():
    if param.grad is not None:
        print(f"{name}: {param.grad.norm()}")
```

---

## 贡献指南

### 代码审查清单

提交代码前检查：

- [ ] 代码符合 PEP 8 规范
- [ ] 添加了文档字符串
- [ ] 添加了单元测试
- [ ] 更新了相关文档
- [ ] 通过了所有现有测试
- [ ] 代码有适当的错误处理
- [ ] 配置文件正确

### Pull Request 模板

```markdown
## 描述
简要描述这个 PR 做了什么

## 变更类型
- [ ] 新功能
- [ ] Bug 修复
- [ ] 文档更新
- [ ] 重构

## 测试
描述如何测试这些变更

## 相关 Issue
关闭 #issue_number

## 检查清单
- [ ] 我已阅读贡献指南
- [ ] 我的代码遵循此项目的风格指南
- [ ] 我已对我的代码进行了审查
- [ ] 我已添加了文档（如需要）
- [ ] 我的更改不产生新的警告
- [ ] 我已添加了测试（如需要）
```

---

## 资源链接

- [Isaac Sim 文档](https://docs.omniverse.nvidia.com/isaacsim/)
- [Hydra 文档](https://hydra.cc/)
- [PyTorch 文档](https://pytorch.org/docs/)
- [项目 GitHub](https://github.com/Rui-li023/LabUtopia)

---

**最后更新:** 2025-01-04
