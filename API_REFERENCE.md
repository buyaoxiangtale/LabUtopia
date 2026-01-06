# LabUtopia API 参考文档

> 本文档提供 LabUtopia 核心模块的详细 API 说明

## 目录

- [任务模块 API](#任务模块-api)
- [控制器模块 API](#控制器模块-api)
- [数据收集器 API](#数据收集器-api)
- [推理引擎 API](#推理引擎-api)
- [工具函数 API](#工具函数-api)
- [工厂模块 API](#工厂模块-api)

---

## 任务模块 API

### BaseTask

所有任务类的基类，定义了任务的基本接口和通用功能。

**位置:** `tasks/base_task.py`

#### 类属性

| 属性 | 类型 | 说明 |
|------|------|------|
| `cfg` | OmegaConf | 任务配置对象 |
| `world` | World | Isaac Sim 仿真世界实例 |
| `stage` | Usd.Stage | USD 场景舞台 |
| `robot` | Robot | 机器人实例 |
| `cameras` | List[Camera] | 相机列表 |
| `reset_needed` | bool | 是否需要重置标志 |
| `frame_idx` | int | 当前帧索引 |
| `object_utils` | ObjectUtils | 对象工具实例 |
| `obj_configs` | List[Dict] | 对象配置列表 |
| `material_config` | Dict | 材质配置 |
| `current_obj_idx` | int | 当前对象索引 |
| `current_material_idx` | int | 当前材质索引 |

#### 方法

##### `__init__(cfg, world, stage, robot)`

初始化任务。

**参数:**
- `cfg` (OmegaConf): 任务配置
- `world` (World): 仿真世界实例
- `stage` (Usd.Stage): USD 舞台
- `robot` (Robot): 机器人实例

**示例:**
```python
task = MyTask(cfg, world, stage, robot)
```

##### `reset()`

重置任务状态和仿真世界。

**返回值:** None

**示例:**
```python
task.reset()
```

##### `step() -> Dict[str, Any]`

执行任务的一个步骤（抽象方法，需子类实现）。

**返回值:**
- `Dict[str, Any]`: 包含任务状态信息的字典

**示例:**
```python
state = task.step()
```

##### `get_task_info() -> Dict[str, Any]`

获取任务相关信息。

**返回值:**
```python
{
    "frame_idx": int,      # 当前帧索引
    "reset_needed": bool   # 是否需要重置
}
```

##### `need_reset() -> bool`

检查任务是否需要重置。

**返回值:**
- `bool`: True 表示需要重置

##### `on_task_complete(success: bool)`

处理任务完成逻辑，更新对象和材质索引。

**参数:**
- `success` (bool): 任务是否成功完成

##### `setup_cameras()`

设置任务所需的相机。

**功能:**
- 根据配置创建相机实例
- 设置相机参数（位置、分辨率、焦距等）
- 初始化图像类型（rgb、depth、pointcloud、segmentation等）

**支持的图像类型:**
- `rgb`: RGB 图像
- `depth`: 深度图像
- `pointcloud`: 点云数据
- `segmentation`: 实例分割
- `semantic_pointcloud`: 语义点云
- 组合类型: `rgb+depth`

##### `setup_objects()`

设置仿真世界中的对象。

**功能:**
- 从配置中读取对象路径和位置范围
- 构建对象配置列表

##### `setup_materials()`

设置对象的材质。

**功能:**
- 加载材质配置
- 区分训练材质和测试材质（OOD场景）

##### `get_camera_data() -> Tuple[Dict, Dict]`

获取相机数据。

**返回值:**
```python
(
    {
        "camera_1_rgb": np.ndarray,    # 记录用数据
        "camera_1_depth": np.ndarray,
        ...
    },
    {
        "camera_1": np.ndarray,        # 显示用数据
        "camera_2": np.ndarray,
        ...
    }
)
```

##### `apply_material_to_object(target_path: str, material_idx: int = None)`

应用材质到指定对象。

**参数:**
- `target_path` (str): 对象的 USD 路径
- `material_idx` (int, optional): 材质索引，默认使用当前材质索引

**示例:**
```python
task.apply_material_to_object("/World/bottle01", material_idx=0)
```

##### `randomize_object_position(obj_path: str, position_range: Dict) -> np.ndarray`

根据位置范围随机化对象位置。

**参数:**
- `obj_path` (str): 对象路径
- `position_range` (Dict): 位置范围字典
  ```python
  {
      'x': [min, max],
      'y': [min, max],
      'z': [min, max]
  }
  ```

**返回值:**
- `np.ndarray`: 随机生成的位置 [x, y, z]

**示例:**
```python
position = task.randomize_object_position(
    "/World/bottle01",
    {'x': [0.2, 0.3], 'y': [-0.1, 0.1], 'z': [0.85, 0.85]}
)
```

##### `place_objects_with_visibility_management(current_obj_idx: int, far_distance: float = 10.0) -> str`

放置对象并管理可见性，将非当前对象移动到远处。

**参数:**
- `current_obj_idx` (int): 当前对象索引
- `far_distance` (float): 远处距离，默认 10.0

**返回值:**
- `str`: 当前对象的路径

##### `get_basic_state_info(joint_positions=None, object_path=None, target_path=None, additional_info=None) -> Dict[str, Any]`

获取包含通用数据的基本状态信息。

**参数:**
- `joint_positions` (np.ndarray, optional): 机器人关节位置
- `object_path` (str, optional): 操作对象路径
- `target_path` (str, optional): 目标对象路径
- `additional_info` (Dict, optional): 额外状态信息

**返回值:**
```python
{
    'joint_positions': np.ndarray,      # 关节位置
    'camera_data': Dict,                # 相机数据
    'camera_display': Dict,             # 相机显示数据
    'done': bool,                       # 是否完成
    'gripper_position': float,          # 夹爪位置
    'object_position': np.ndarray,      # 对象位置 (如果提供 object_path)
    'object_size': np.ndarray,          # 对象尺寸
    'object_path': str,                 # 对象路径
    'object_name': str,                 # 对象名称
    'target_position': np.ndarray,      # 目标位置 (如果提供 target_path)
    'target_size': np.ndarray,          # 目标尺寸
    'target_path': str,                 # 目标路径
    'target_name': str,                 # 目标名称
    ...                                 # additional_info 中的内容
}
```

##### `check_frame_limits(max_steps: int = None) -> bool`

检查帧限制，超限时设置重置标志。

**参数:**
- `max_steps` (int, optional): 最大步数，默认使用配置中的值

**返回值:**
- `bool`: True 表示应该继续执行

##### `update_object_and_material_indices(success: bool)`

更新对象和材质索引。

**参数:**
- `success` (bool): 任务是否成功完成

---

### SingleObjectTask

单物体任务基类，继承自 BaseTask。

**位置:** `tasks/single_object_task.py`

#### 额外属性

| 属性 | 类型 | 说明 |
|------|------|------|
| `object_path` | str | 当前操作对象的路径 |

#### 使用示例

```python
from tasks.single_object_task import SingleObjectTask

class PickTask(SingleObjectTask):
    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)

    def step(self):
        # 实现抓取逻辑
        state = self.get_basic_state_info(
            object_path=self.object_path
        )
        return state
```

---

### DualObjectTask

双物体任务基类，继承自 BaseTask。

**位置:** `tasks/dual_object_task.py`

#### 额外属性

| 属性 | 类型 | 说明 |
|------|------|------|
| `object_path` | str | 第一个对象路径 |
| `target_path` | str | 第二个对象（目标）路径 |

#### 使用示例

```python
from tasks.dual_object_task import DualObjectTask

class PickPlaceTask(DualObjectTask):
    def __init__(self, cfg, world, stage, robot):
        super().__init__(cfg, world, stage, robot)

    def step(self):
        # 实现抓取-放置逻辑
        state = self.get_basic_state_info(
            object_path=self.object_path,
            target_path=self.target_path
        )
        return state
```

---

## 控制器模块 API

### BaseController

所有控制器的基类，定义了控制器的基本接口。

**位置:** `controllers/base_controller.py`

#### 类属性

| 属性 | 类型 | 说明 |
|------|------|------|
| `cfg` | OmegaConf | 控制器配置 |
| `robot` | Robot | 机器人实例 |
| `object_utils` | ObjectUtils | 对象工具实例 |
| `mode` | str | 运行模式 ("collect" 或 "infer") |
| `data_collector` | DataCollector | 数据收集器 (collect 模式) |
| `inference_engine` | InferenceEngine | 推理引擎 (infer 模式) |
| `rmp_controller` | RMPFlowController | RMPFlow 运动控制器 |
| `gripper_control` | Gripper | 夹爪控制器 |
| `reset_needed` | bool | 重置标志 |
| `_episode_num` | int | 当前回合数 |
| `success_count` | int | 成功次数 |
| `check_success_counter` | int | 成功检查计数器 |
| `REQUIRED_SUCCESS_STEPS` | int | 所需成功步数 (默认60) |

#### 方法

##### `__init__(cfg, robot, use_default_config=True)`

初始化控制器。

**参数:**
- `cfg` (OmegaConf): 配置对象
- `robot` (Robot): 机器人实例
- `use_default_config` (bool): 是否使用默认配置

##### `step(state: Dict[str, Any]) -> Tuple[Any, bool, bool]`

执行一步控制（抽象方法，需子类实现）。

**参数:**
- `state` (Dict): 当前状态字典

**返回值:**
```python
(
    action,      # 控制动作
    done,        # 回合是否结束
    is_success   # 是否成功完成任务
)
```

##### `reset()`

重置控制器状态。

**功能:**
- 增加回合计数
- 计算并打印成功率
- 重置成功检查计数器
- 清除数据收集器缓存（collect 模式）

##### `close()`

清理控制器使用的资源。

**功能:**
- 关闭数据收集器（collect 模式）

##### `need_reset() -> bool`

检查控制器是否需要重置。

**返回值:**
- `bool`: True 表示需要重置

##### `episode_num() -> int`

获取当前回合数。

**返回值:**
- `int`: 当前回合数

##### `is_success()`

获取上次任务是否成功。

**返回值:**
- `bool`: 上次任务的成功状态

##### `language_instruction` (property)

获取/设置当前任务的语言指令。

**使用示例:**
```python
# 获取语言指令
instruction = controller.language_instruction

# 设置语言指令
controller.language_instruction = "Pick up the red bottle"
```

---

### 原子动作控制器

#### PickController

执行抓取动作的控制器。

**位置:** `controllers/atomic_actions/pick_controller.py`

**主要参数:**
- `pick_height`: 抓取高度偏移
- `pre_grasp_dist`: 预抓取距离
- `grasp_force`: 抓取力

**使用示例:**
```python
from controllers.atomic_actions.pick_controller import PickController

picker = PickController(cfg, robot)
action, done, success = picker.step(state)
```

#### PlaceController

执行放置动作的控制器。

**位置:** `controllers/atomic_actions/place_controller.py`

#### PourController

执行倾倒动作的控制器。

**位置:** `controllers/atomic_actions/pour_controller.py`

**额外参数:**
- `pour_angle`: 倾倒角度
- `pour_duration`: 倾倒持续时间

#### StirController

执行搅拌动作的控制器。

**位置:** `controllers/atomic_actions/stir_controller.py`

**额外参数:**
- `stir_radius`: 搅拌半径
- `stir_speed`: 搅拌速度
- `stir_rotations`: 搅拌圈数

---

### 任务级控制器

#### PickPlaceController

组合抓取和放置动作的控制器。

**位置:** `controllers/pickplace_controller.py`

**状态流程:**
1. 移动到预抓取位置
2. 执行抓取
3. 移动到预放置位置
4. 执行放置

#### CleanBeakerController

清洗烧杯任务的控制器。

**位置:** `controllers/cleanbeaker_controller.py`

**动作序列:**
1. 抓取烧杯
2. 移动到水龙头
3. 打开水龙头
4. 装水
5. 关闭水龙头
6. 摇晃烧杯
7. 倒掉水
8. 重复清洗步骤

---

### 导航控制器

#### NavigationController

基础导航控制器。

**位置:** `controllers/navigation_controller.py`

**主要功能:**
- 路径跟踪
- 避障
- 目标点导航

**参数:**
- `path`: 导航路径点列表
- `tolerance`: 目标容差
- `max_linear_velocity`: 最大线速度
- `max_angular_velocity`: 最大角速度

#### NavigationControllerNew

新版导航控制器，改进的路径跟踪算法。

**位置:** `controllers/navigation_controller_new.py`

**改进:**
- 更平滑的转向
- 更好的速度控制
- 减少抖动

#### NavigationControllerSmooth

带速度集成的平滑导航控制器。

**位置:** `controllers/navigation_controller_smooth_12_16.py`

**特点:**
- 速度积分
- 平滑加速/减速
- 减少位置抖动

---

## 数据收集器 API

### BaseDataCollector

数据收集器基类。

**位置:** `data_collectors/data_collector.py`

#### 方法

##### `__init__(camera_configs, save_dir, max_episodes, compression=None)`

初始化数据收集器。

**参数:**
- `camera_configs` (List): 相机配置列表
- `save_dir` (str): 保存目录
- `max_episodes` (int): 最大回合数
- `compression`: 压缩设置

##### `add_step(state, action)`

添加一步数据。

**参数:**
- `state` (Dict): 状态数据
- `action` (np.ndarray): 动作数据

##### `save_episode(language_instruction=None)`

保存当前回合数据。

**参数:**
- `language_instruction` (str, optional): 语言指令

##### `clear_cache()`

清除缓存。

##### `close()`

关闭数据收集器并保存所有数据。

---

### PickDataCollector

抓取任务专用数据收集器。

**位置:** `data_collectors/pick_data_collector.py`

**额外功能:**
- 记录抓取成功/失败
- 记录对象位置变化
- 特定的数据格式

---

## 推理引擎 API

### BaseInferenceEngine

推理引擎基类。

**位置:** `controllers/inference_engines/base_inference_engine.py`

#### 方法

##### `predict(obs: Dict[str, Any]) -> np.ndarray`

根据观测预测动作。

**参数:**
- `obs` (Dict): 观测数据

**返回值:**
- `np.ndarray`: 预测的动作

##### `reset()`

重置推理引擎状态。

##### `close()`

关闭推理引擎。

---

### LocalModelInferenceEngine

本地模型推理引擎。

**位置:** `controllers/inference_engines/local_model_inference_engine.py`

#### 初始化参数

- `policy_model_path`: 模型检查点路径
- `policy_config_path`: 配置文件路径
- `normalizer_path`: 归一化器路径
- `device`: 计算设备

#### 使用示例

```python
from controllers.inference_engines.local_model_inference_engine import LocalModelInferenceEngine

engine = LocalModelInferenceEngine(
    policy_model_path="checkpoints/model.ckpt",
    policy_config_path="config.yaml",
    normalizer_path="normalize.ckpt",
    device="cuda:0"
)
action = engine.predict(obs)
```

---

### RemoteInferenceEngine

远程模型推理引擎，通过 WebSocket 连接到远程服务器。

**位置:** `controllers/inference_engines/remote_inference_engine.py`

#### 初始化参数

- `host`: 服务器地址
- `port`: 服务器端口
- `n_obs_steps`: 观测步数
- `timeout`: 超时时间
- `max_retries`: 最大重试次数

#### 使用示例

```python
from controllers.inference_engines.remote_inference_engine import RemoteInferenceEngine

engine = RemoteInferenceEngine(
    host="192.168.1.100",
    port=8080,
    n_obs_steps=2,
    timeout=30,
    max_retries=3
)
action = engine.predict(obs)
```

---

### InferenceEngineFactory

推理引擎工厂类。

**位置:** `controllers/inference_engines/inference_engine_factory.py`

#### 方法

##### `create_inference_engine(cfg, trajectory_controller)`

根据配置创建推理引擎。

**参数:**
- `cfg`: 配置对象
- `trajectory_controller`: 轨迹控制器

**返回值:**
- `BaseInferenceEngine`: 推理引擎实例

**使用示例:**
```python
from controllers.inference_engines.inference_engine_factory import InferenceEngineFactory

engine = InferenceEngineFactory.create_inference_engine(cfg, trajectory_controller)
```

---

## 工具函数 API

### ObjectUtils

对象操作工具类。

**位置:** `utils/object_utils.py`

#### 方法

##### `get_instance() -> ObjectUtils`

获取 ObjectUtils 单例实例。

**返回值:**
- `ObjectUtils`: 工具实例

##### `set_object_position(object_path: str, position: np.ndarray)`

设置对象位置。

**参数:**
- `object_path` (str): 对象路径
- `position` (np.ndarray): 位置 [x, y, z]

##### `get_geometry_center(object_path: str) -> np.ndarray`

获取对象几何中心。

**参数:**
- `object_path` (str): 对象路径

**返回值:**
- `np.ndarray`: 几何中心坐标

##### `get_object_size(object_path: str) -> np.ndarray`

获取对象尺寸。

**参数:**
- `object_path` (str): 对象路径

**返回值:**
- `np.ndarray`: 尺寸 [length, width, height]

##### `set_object_rotation(object_path: str, rotation: np.ndarray)`

设置对象旋转。

**参数:**
- `object_path` (str): 对象路径
- `rotation` (np.ndarray): 旋转 [x, y, z, w]

---

### CameraUtils

相机工具类。

**位置:** `utils/camera_utils.py`

#### 方法

##### `process_camera_image(camera: Camera, image_type: str) -> Tuple[Dict, np.ndarray]`

处理相机图像。

**参数:**
- `camera` (Camera): 相机实例
- `image_type` (str): 图像类型

**返回值:**
```python
(
    {                        # 记录数据
        "rgb": np.ndarray,
        "depth": np.ndarray,
        ...
    },
    np.ndarray              # 显示数据
)
```

---

### AStarPlanner

A* 路径规划器。

**位置:** `utils/a_star.py`

#### 方法

##### `__init__(grid_size: Tuple[int, int], obstacles: List[Tuple])`

初始化路径规划器。

**参数:**
- `grid_size` (Tuple): 网格尺寸 (width, height)
- `obstacles` (List): 障碍物列表

##### `plan_path(start: Tuple, goal: Tuple) -> List[Tuple]`

规划从起点到终点的路径。

**参数:**
- `start` (Tuple): 起点坐标 (x, y)
- `goal` (Tuple): 终点坐标 (x, y)

**返回值:**
- `List[Tuple]`: 路径点列表

**使用示例:**
```python
from utils.a_star import AStarPlanner

planner = AStarPlanner(
    grid_size=(100, 100),
    obstacles=[(20, 30), (25, 35)]
)
path = planner.plan_path(start=(10, 10), goal=(80, 80))
```

---

## 工厂模块 API

### RobotFactory

机器人工厂。

**位置:** `factories/robot_factory.py`

#### 方法

##### `create_robot(robot_type: str, position: np.ndarray) -> Robot`

创建机器人实例。

**参数:**
- `robot_type` (str): 机器人类型 ("franka", "ridgebase", "fetch")
- `position` (np.ndarray): 机器人位置

**返回值:**
- `Robot`: 机器人实例

**使用示例:**
```python
from factories.robot_factory import create_robot

robot = create_robot(
    robot_type="franka",
    position=np.array([-0.4, 0, 0.71])
)
```

---

### TaskFactory

任务工厂。

**位置:** `factories/task_factory.py`

#### 方法

##### `create_task(task_type: str, cfg, world, stage, robot) -> BaseTask`

创建任务实例。

**参数:**
- `task_type` (str): 任务类型
- `cfg`: 配置对象
- `world`: 仿真世界
- `stage`: USD 舞台
- `robot`: 机器人实例

**返回值:**
- `BaseTask`: 任务实例

**使用示例:**
```python
from factories.task_factory import create_task

task = create_task(
    task_type="pick",
    cfg=cfg,
    world=world,
    stage=stage,
    robot=robot
)
```

---

### ControllerFactory

控制器工厂。

**位置:** `factories/controller_factory.py`

#### 方法

##### `create_controller(controller_type: str, cfg, robot) -> BaseController`

创建控制器实例。

**参数:**
- `controller_type` (str): 控制器类型
- `cfg`: 配置对象
- `robot`: 机器人实例

**返回值:**
- `BaseController`: 控制器实例

**使用示例:**
```python
from factories.controller_factory import create_controller

controller = create_controller(
    controller_type="pick",
    cfg=cfg,
    robot=robot
)
```

---

### CollectorFactory

数据收集器工厂。

**位置:** `factories/collector_factory.py`

#### 方法

##### `create_collector(collector_type: str, camera_configs, save_dir, max_episodes, compression)`

创建数据收集器实例。

**参数:**
- `collector_type` (str): 收集器类型
- `camera_configs`: 相机配置
- `save_dir` (str): 保存目录
- `max_episodes` (int): 最大回合数
- `compression`: 压缩设置

**返回值:**
- `DataCollector`: 数据收集器实例

---

## 使用示例

### 完整的数据收集流程

```python
from factories.robot_factory import create_robot
from factories.task_factory import create_task
from factories.controller_factory import create_controller

# 创建机器人
robot = create_robot(
    robot_type="franka",
    position=np.array([-0.4, 0, 0.71])
)

# 创建任务
task = create_task(
    task_type="pick",
    cfg=cfg,
    world=world,
    stage=stage,
    robot=robot
)

# 创建控制器
controller = create_controller(
    controller_type="pick",
    cfg=cfg,
    robot=robot
)

# 运行仿真
task.reset()
while simulation_app.is_running():
    world.step(render=True)

    if task.need_reset() or controller.need_reset():
        controller.reset()
        task.reset()
        continue

    state = task.step()
    if state is None:
        continue

    action, done, is_success = controller.step(state)
    if action is not None:
        robot.get_articulation_controller().apply_action(action)

    if done:
        task.on_task_complete(is_success)
```

### 完整的推理流程

```python
# 修改配置为推理模式
cfg.mode = "infer"
cfg.infer.policy_model_path = "outputs/train/.../checkpoints/latest.ckpt"
cfg.infer.policy_config_path = "outputs/train/.../.hydra/config.yaml"
cfg.infer.normalizer_path = "outputs/train/.../checkpoints/normalize.ckpt"

# 创建控制器（会自动初始化推理引擎）
controller = create_controller(
    controller_type="pick",
    cfg=cfg,
    robot=robot
)

# 运行推理
while simulation_app.is_running():
    state = task.step()
    action, done, success = controller.step(state)
    robot.get_articulation_controller().apply_action(action)
```

---

**最后更新:** 2025-01-04
