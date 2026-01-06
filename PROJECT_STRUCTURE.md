# LabUtopia 项目结构完整说明

> 本文档提供 LabUtopia 项目的完整目录结构和文件说明

## 目录

- [项目概述](#项目概述)
- [目录结构](#目录结构)
- [核心模块说明](#核心模块说明)
- [根目录文件](#根目录文件)
- [配置文件说明](#配置文件说明)
- [工具脚本说明](#工具脚本说明)

---

## 项目概述

LabUtopia 是一个高保真科学实验仿真环境和分层基准测试框架，用于训练和评估具身智能体在实验室场景中的操作能力。

**主要特性:**
- 基于 NVIDIA Isaac Sim 5.1 的高保真物理仿真
- 分层任务体系 (Level 1-5)，从基础操作到复杂导航
- 支持多种机器人平台 (Franka, Ridgebase-Franka, Fetch)
- 完整的数据收集、训练和推理流程
- A* 路径规划和导航系统

---

## 目录结构

```
LabUtopia/
├── assets/                          # 资源文件目录
│   ├── chemistry_lab/               # 化学实验室场景资源
│   │   ├── pick_task/              # 抓取任务场景
│   │   ├── pour_task/              # 倾倒任务场景
│   │   └── ...                     # 其他任务场景
│   ├── fetch/                       # Fetch 机器人资源
│   │   └── fetch_descriptor.yaml   # Fetch 机器人描述文件
│   ├── navigation/                  # 导航任务资源
│   │   ├── grids/                  # 网格配置
│   │   └── nav_assets/             # 导航资产
│   └── robots/                      # 机器人模型
│       ├── franka/                 # Franka 机器人
│       └── ridgebase/              # Ridgebase 机器人
│
├── config/                          # 配置文件目录
│   ├── level1_*.yaml               # Level 1 基础任务配置
│   ├── level2_*.yaml               # Level 2 组合任务配置
│   ├── level3_*.yaml               # Level 3 泛化任务配置
│   ├── level4_*.yaml               # Level 4 长序列任务配置
│   └── navigation/                 # 导航任务配置
│       └── navigation_assets.yaml
│
├── controllers/                     # 控制器实现
│   ├── atomic_actions/             # 原子动作控制器
│   │   ├── pick_controller.py     # 抓取动作
│   │   ├── place_controller.py    # 放置动作
│   │   ├── pour_controller.py     # 倾倒动作
│   │   ├── shake_controller.py    # 摇晃动作
│   │   ├── stir_controller.py     # 搅拌动作
│   │   ├── press_controller.py    # 按压动作
│   │   ├── open_controller.py     # 打开动作
│   │   ├── close_controller.py    # 关闭动作
│   │   └── move_controller.py     # 移动动作
│   │
│   ├── inference_engines/          # 推理引擎
│   │   ├── base_inference_engine.py           # 基础推理引擎
│   │   ├── local_model_inference_engine.py    # 本地模型推理
│   │   ├── remote_inference_engine.py         # 远程推理引擎
│   │   └── inference_engine_factory.py        # 推理引擎工厂
│   │
│   ├── robot_controllers/          # 机器人控制器
│   │   ├── trajectory_controller.py           # 轨迹控制器
│   │   ├── grapper_manager.py                 # 夹爪管理器
│   │   └── ridgebase/                        # Ridgebase 控制器
│   │       ├── ridgebase_controller.py       # 基础控制器
│   │       ├── ridgebase_controller_new.py   # 新版控制器
│   │       └── ridgebase_controller_smooth.py # 平滑控制器
│   │
│   ├── base_controller.py          # 基础控制器类
│   ├── pick_controller.py          # 抓取任务控制器
│   ├── place_controller.py         # 放置任务控制器
│   ├── pour_controller.py          # 倾倒任务控制器
│   ├── pickplace_controller.py     # 抓取-放置组合控制器
│   ├── pickpour_controller.py      # 抓取-倾倒组合控制器
│   ├── shake_controller.py         # 摇晃任务控制器
│   ├── stir_controller.py          # 搅拌任务控制器
│   ├── press_controller.py         # 按压任务控制器
│   ├── open_controller.py          # 打开任务控制器
│   ├── close_controller.py         # 关闭任务控制器
│   ├── openclose_controller.py     # 开关组合控制器
│   ├── shakebeaker_controller.py   # 摇晃烧杯控制器
│   ├── stirglassrod_controller.py  # 玻璃棒搅拌控制器
│   ├── cleanbeaker_controller.py   # 清洗烧杯控制器
│   ├── cleanbeaker7policy_controller.py  # 7策略清洗控制器
│   ├── device_operate_controller.py # 设备操作控制器
│   ├── opentransportpour_controller.py  # 开启-运输-倾倒控制器
│   ├── LiquidMixing_controller.py  # 液体混合控制器
│   ├── mobile_pick_controller.py   # 移动抓取控制器
│   └── navigation_controller_*.py  # 导航控制器系列
│       ├── navigation_controller.py              # 基础导航控制器
│       ├── navigation_controller_new.py          # 新版导航控制器
│       ├── navigation_controller_smooth_12_16.py # 平滑导航控制器
│       └── navigation_controller_test_speed.py  # 测试速度控制器
│
├── tasks/                           # 任务定义
│   ├── base_task.py               # 基础任务类
│   ├── single_object_task.py      # 单物体任务基类
│   ├── dual_object_task.py        # 双物体任务基类
│   ├── pick_task.py               # 抓取任务
│   ├── place_task.py              # 放置任务
│   ├── pickplace_task.py          # 抓取-放置任务
│   ├── pickpour_task.py           # 抓取-倾倒任务
│   ├── pour_task.py               # 倾倒任务 (缺失但被引用)
│   ├── shake_task.py              # 摇晃任务
│   ├── stir_task.py               # 搅拌任务
│   ├── press_task.py              # 按压任务
│   ├── placepress_task.py         # 放置-按压任务
│   ├── open_task.py               # 打开任务
│   ├── close_task.py              # 关闭任务
│   ├── shakebeaker_task.py        # 摇晃烧杯任务 (缺失但被引用)
│   ├── cleanbeaker_task.py        # 清洗烧杯任务
│   ├── device_operate_task.py     # 设备操作任务
│   ├── opentransportpour_task.py  # 开启-运输-倾倒任务
│   ├── LiquidMixing_task.py       # 液体混合任务
│   ├── mobile_pick_task.py        # 移动抓取任务
│   └── navigation_task_*.py       # 导航任务系列
│       ├── navigation_task.py              # 基础导航任务
│       ├── navigation_task_new.py          # 新版导航任务
│       ├── navigation_task_new_cp.py       # 新版检查点导航任务
│       ├── navigation_task_new_cp_cp.py    # 新版检查点导航任务v2
│       ├── navigation_task_with_path_info.py  # 带路径信息的导航任务
│       └── navigation_task_test_weizi.py  # 位置测试导航任务
│
├── data_collectors/                # 数据收集器
│   ├── data_collector.py          # 基础数据收集器
│   ├── mock_collector.py          # 模拟数据收集器
│   └── pick_data_collector.py     # 抓取任务数据收集器
│
├── factories/                       # 工厂类
│   ├── robot_factory.py           # 机器人工厂
│   ├── task_factory.py            # 任务工厂
│   ├── controller_factory.py      # 控制器工厂
│   └── collector_factory.py       # 收集器工厂
│
├── policy/                          # 策略模型
│   ├── config/                    # 配置文件
│   │   ├── train_*.yaml          # 训练配置
│   │   └── task/                 # 任务配置
│   │       ├── act.yaml          # ACT 任务配置
│   │       ├── dp.yaml           # Diffusion Policy 任务配置
│   │       ├── nav_*.yaml        # 导航任务配置
│   │       └── ...
│   │
│   ├── common/                    # 通用工具
│   │   ├── checkpoint_util.py    # 检查点工具
│   │   ├── json_logger.py        # JSON 日志
│   │   ├── pytorch_util.py       # PyTorch 工具
│   │   ├── normalize_util.py     # 归一化工具
│   │   └── robomimic_config_util.py  # Robomimic 配置工具
│   │
│   ├── dataset/                   # 数据集类
│   │   ├── base_dataset.py       # 基础数据集
│   │   ├── act_image_dataset.py  # ACT 图像数据集
│   │   ├── dp_image_dataset.py   # DP 图像数据集
│   │   ├── act_nav_dataset.py    # ACT 导航数据集
│   │   └── nav_dataset.py        # 导航数据集
│   │
│   ├── model/                     # 模型定义
│   │   ├── act/                  # ACT 模型
│   │   │   ├── detr_vae.py       # DETR VAE
│   │   │   ├── transformer.py    # Transformer
│   │   │   ├── backbone.py       # 骨干网络
│   │   │   └── position_encoding.py  # 位置编码
│   │   ├── diffusion/            # Diffusion Policy 模型
│   │   │   ├── conditional_unet1d.py  # 条件 UNet 1D
│   │   │   ├── conv1d_components.py   # 1D 卷积组件
│   │   │   ├── transformer_for_diffusion.py  # Transformer
│   │   │   ├── positional_embedding.py    # 位置嵌入
│   │   │   ├── mask_generator.py          # 掩码生成器
│   │   │   └── ema_model.py               # EMA 模型
│   │   ├── vision/               # 视觉模型
│   │   │   ├── multi_image_obs_encoder.py  # 多图像观测编码器
│   │   │   ├── model_getter.py             # 模型获取器
│   │   │   └── crop_randomizer.py          # 裁剪随机化
│   │   └── common/               # 通用模型组件
│   │       ├── normalizer.py     # 归一化器
│   │       ├── lr_scheduler.py   # 学习率调度器
│   │       └── rotation_transformer.py  # 旋转变换器
│   │
│   ├── policy/                   # 策略实现
│   │   ├── base_image_policy.py         # 基础图像策略
│   │   ├── act_image_policy.py          # ACT 图像策略
│   │   └── diffusion_unet_image_policy.py  # Diffusion UNet 图像策略
│   │
│   ├── env_runner/               # 环境运行器
│   │   └── base_image_runner.py  # 基础图像运行器
│   │
│   ├── workspace/                # 训练工作空间
│   │   ├── base_workspace.py                    # 基础工作空间
│   │   ├── lightning_workspace.py                # Lightning 工作空间
│   │   ├── train_act_image_workspace.py         # ACT 训练
│   │   ├── train_act_image_workspace_lightning.py  # ACT Lightning 训练
│   │   ├── train_diffusion_unet_image_workspace.py  # Diffusion 训练
│   │   └── train_diffusion_unet_image_workspace_lightning.py  # Diffusion Lightning 训练
│   │
│   └── codecs/                   # 编解码器
│       └── imagecodecs_numcodecs.py
│
├── robots/                         # 机器人配置
│   └── franka/                   # Franka 机器人
│       └── rmpflow/             # RMPFlow 配置
│           ├── franka_rmpflow_common.yaml
│           └── robot_descriptor.yaml
│
├── roomlayout/                    # 场景布局 (USD 文件)
│   ├── chemistry_lab/           # 化学实验室布局
│   └── navigation/              # 导航场景布局
│
├── utils/                         # 工具函数
│   ├── object_utils.py          # 对象工具
│   ├── camera_utils.py          # 相机工具
│   ├── task_utils.py            # 任务工具
│   ├── Material_utils.py        # 材质工具
│   ├── angle.py                 # 角度计算
│   ├── merge_datasets.py        # 数据集合并
│   ├── a_star.py                # A* 路径规划算法
│   ├── path_planning_precompute.py      # 路径规划预计算
│   ├── batch_path_planning.py            # 批量路径规划
│   ├── batch_process_paths.py             # 批量处理路径
│   ├── find_nav_target_for_item.py       # 查找导航目标
│   ├── fix_nav_target_rotation.py        # 修复导航目标旋转
│   ├── batch_process_nav_targets.py      # 批量处理导航目标
│   └── auto_batch_scene_nav.py           # 自动批量场景导航
│
├── tests/                         # 测试代码
│   ├── test_config_files.py     # 配置文件测试
│   └── test_single_config.py    # 单配置测试
│
├── scripts/                       # 辅助脚本
│   ├── requirements_conversion.txt
│   ├── add_language_instructions.py    # 添加语言指令
│   ├── convert_labsim_data_to_lerobot.py  # 数据格式转换
│   └── merge_dataset.py          # 合并数据集
│
├── packages/                      # 外部包
│   └── openpi-client/           # OpenPI 客户端
│       └── src/openpi_client/
│           ├── __init__.py
│           ├── base_policy.py
│           ├── websocket_client_policy.py
│           ├── action_chunk_broker.py
│           ├── image_tools.py
│           ├── image_tools_test.py
│           ├── msgpack_numpy.py
│           ├── msgpack_numpy_test.py
│           └── runtime/
│               ├── runtime.py
│               ├── environment.py
│               ├── agent.py
│               ├── subscriber.py
│               └── agents/
│                   └── policy_agent.py
│
├── docs/                          # 文档目录
│   ├── PATH_INFO_GUIDE.md              # 路径信息指南
│   ├── ITEM_NAV_TARGET_GUIDE.md        # 物品导航目标指南
│   ├── BATCH_PROCESS_GUIDE.md          # 批量处理指南
│   ├── batch_path_planning_guide.md    # 批量路径规划指南
│   ├── batch_scene_nav_guide.md        # 批量场景导航指南
│   ├── AUTO_BATCH_SCENE_NAV_GUIDE.md   # 自动批量场景导航指南
│   └── FIX_NAV_TARGET_ROTATION_GUIDE.md # 修复导航目标旋转指南
│
├── images/                        # 图片资源
│
├── outputs/                       # 输出目录
│   ├── collect/                  # 数据收集输出
│   ├── train/                    # 训练输出
│   ├── infer/                    # 推理输出
│   └── nav_runs/                 # 导航运行输出
│
├── .git/                          # Git 仓库
├── .gitattributes
├── .gitignore
│
├── main.py                        # 主入口文件
├── train.py                       # 训练脚本
├── train-muilt.py                # 多任务训练脚本
├── requirements.txt               # Python 依赖
├── LICENSE                        # MIT 许可证
├── README.md                      # 英文说明文档
├── README_CN.md                   # 中文说明文档
│
├── controller_comparison.md              # 控制器对比文档
├── README_velocity_integration.md       # 速度集成说明文档
│
├── demo_ridgebase_astar_nav.py          # Ridgebase A* 导航演示
├── demo_ridgebase_astar_nav_backup.py   # Ridgebase A* 导航演示备份
├── demo_ridgebase_astar_nav_save.py     # Ridgebase A* 导航演示保存版
├── nav_goal_demo.py                     # 导航目标演示
├── run_nav_with_video.py                # 带视频的导航运行
│
├── test_speed_simple.py                 # 简单速度测试
├── test_navigation_speed.py             # 导航速度测试
├── test_navigation_velocity.py          # 导航速度测试
├── test_pose_config.py                  # 位姿配置测试
├── test_goal_pairs_paths.py             # 目标对路径测试
├── test_batch_path_planning.py          # 批量路径规划测试
└── test_batch_scene_nav.py              # 批量场景导航测试
```

---

## 核心模块说明

### 1. 主入口模块

#### `main.py`
**功能:** 项目主入口文件

**主要参数:**
- `--backend`: 计算后端 (numpy/gpu)
- `--headless`: 无头模式运行
- `--no-video`: 禁用视频
- `--config-name`: 配置文件名
- `--config-dir`: 配置目录

**使用示例:**
```bash
# 基础使用
python main.py --config-name level1_pick

# GPU 加速 + 无头模式
python main.py --config-name level5_Navigation --backend gpu --headless

# 禁用视频输出
python main.py --config-name level1_pour --no-video
```

#### `train.py`
**功能:** 模型训练脚本

**使用示例:**
```bash
# Diffusion Policy 训练
python train.py --config-name train_diffusion_unet_image_workspace

# ACT 训练
python train.py --config-name train_act_image_workspace
```

### 2. 控制器模块 (controllers/)

#### 基础控制器架构
- `base_controller.py`: 所有控制器的基类，定义通用接口
- `inference_engines/`: 推理引擎，支持本地和远程推理

#### 原子动作控制器 (atomic_actions/)
这些是基础的、可复用的动作单元：

| 文件 | 功能 | 主要方法 |
|------|------|----------|
| `pick_controller.py` | 抓取物体 | `pick()` |
| `place_controller.py` | 放置物体 | `place()` |
| `pour_controller.py` | 倾倒液体 | `pour()` |
| `shake_controller.py` | 摇晃容器 | `shake()` |
| `stir_controller.py` | 搅拌液体 | `stir()` |
| `press_controller.py` | 按压物体 | `press()` |
| `open_controller.py` | 打开门/抽屉 | `open()` |
| `close_controller.py` | 关闭门/抽屉 | `close()` |
| `move_controller.py` | 移动机械臂 | `move()` |
| `pressZ_controller.py` | Z轴按压 | `pressZ()` |

#### 任务级控制器
这些控制器组合多个原子动作完成复杂任务：

| 文件 | 任务类型 | 组合动作 | Level |
|------|----------|----------|-------|
| `pick_controller.py` | 抓取 | pick | 1 |
| `place_controller.py` | 放置 | place | 1 |
| `pickplace_controller.py` | 抓取+放置 | pick + place | 2 |
| `pickpour_controller.py` | 抓取+倾倒 | pick + pour | 2 |
| `shakebeaker_controller.py` | 摇晃烧杯 | pick + shake + place | 2 |
| `stirglassrod_controller.py` | 玻璃棒搅拌 | pick + stir + place | 2 |
| `cleanbeaker_controller.py` | 清洗烧杯 | 多动作序列 | 4 |
| `device_operate_controller.py` | 设备操作 | 多动作序列 | 4 |
| `opentransportpour_controller.py` | 开启-运输-倾倒 | open + move + pour | 4 |
| `LiquidMixing_controller.py` | 液体混合 | 多动作序列 | 4 |

#### 导航控制器系列

| 文件 | 特点 | 适用场景 |
|------|------|----------|
| `navigation_controller.py` | 基础导航 | 简单点对点导航 |
| `navigation_controller_new.py` | 新版导航 | 改进的路径跟踪 |
| `navigation_controller_smooth_12_16.py` | 平滑导航 | 减少抖动的平滑运动 |
| `navigation_controller_test_speed.py` | 速度测试 | 性能测试 |

### 3. 任务模块 (tasks/)

#### 任务层次结构

```
base_task.py (基础任务类)
    ├── single_object_task.py (单物体任务)
    │   ├── pick_task.py
    │   ├── place_task.py
    │   ├── pour_task.py (缺失)
    │   ├── shake_task.py
    │   ├── stir_task.py
    │   └── press_task.py
    │
    └── dual_object_task.py (双物体任务)
        ├── pickplace_task.py
        ├── pickpour_task.py
        ├── placepress_task.py
        ├── open_task.py
        └── close_task.py
```

#### 导航任务系列
- `navigation_task.py`: 基础导航任务
- `navigation_task_new.py`: 新版导航任务
- `navigation_task_new_cp.py`: 带检查点的新版导航
- `navigation_task_with_path_info.py`: 带路径信息的导航

### 4. 策略模块 (policy/)

#### 支持的模型架构

**1. ACT (Action Chunking with Transformers)**
- 位置: `policy/model/act/`
- 文件: `detr_vae.py`, `transformer.py`, `backbone.py`
- 训练配置: `policy/config/train_act_image_workspace.yaml`

**2. Diffusion Policy**
- 位置: `policy/model/diffusion/`
- 文件: `conditional_unet1d.py`, `transformer_for_diffusion.py`
- 训练配置: `policy/config/train_diffusion_unet_image_workspace.yaml`

#### 数据集类型
- `act_image_dataset.py`: ACT 图像数据集
- `dp_image_dataset.py`: Diffusion Policy 图像数据集
- `act_nav_dataset.py`: ACT 导航数据集
- `nav_dataset.py`: 导航数据集

### 5. 工具模块 (utils/)

#### 路径规划工具
| 文件 | 功能 |
|------|------|
| `a_star.py` | A* 路径规划算法实现 |
| `path_planning_precompute.py` | 路径预计算 |
| `batch_path_planning.py` | 批量路径规划 |
| `batch_process_paths.py` | 批量路径处理 |

#### 导航目标工具
| 文件 | 功能 |
|------|------|
| `find_nav_target_for_item.py` | 为物品查找导航目标 |
| `fix_nav_target_rotation.py` | 修复导航目标旋转 |
| `batch_process_nav_targets.py` | 批量处理导航目标 |
| `auto_batch_scene_nav.py` | 自动批量场景导航 |

#### 通用工具
| 文件 | 功能 |
|------|------|
| `object_utils.py` | 对象操作工具 |
| `camera_utils.py` | 相机设置和图像获取 |
| `Material_utils.py` | 材质设置 |
| `angle.py` | 角度计算 |
| `task_utils.py` | 任务工具函数 |
| `merge_datasets.py` | 数据集合并 |

### 6. 配置文件 (config/)

#### Level 1: 基础任务
```
level1_pick.yaml          - 抓取单个物体
level1_place.yaml         - 放置物体
level1_pour.yaml          - 倾倒液体
level1_shake.yaml         - 摇晃容器
level1_stir.yaml          - 搅拌液体
level1_press.yaml         - 按压物体
level1_open_door.yaml     - 打开门
level1_open_drawer.yaml   - 打开抽屉
level1_close_door.yaml    - 关闭门
level1_close_drawer.yaml  - 关闭抽屉
```

#### Level 2: 组合任务
```
level2_ShakeBeaker.yaml        - 摇晃烧杯
level2_StirGlassrod.yaml       - 玻璃棒搅拌
level2_PourLiquid.yaml         - 倾倒液体
level2_TransportBeaker.yaml    - 运输烧杯
level2_HeatLiquid.yaml         - 加热液体
level2_openclose.yaml          - 开关组合
```

#### Level 3: 泛化任务
```
level3_PourLiquid.yaml         - 复杂倾倒
level3_HeatLiquid.yaml         - 复杂加热
level3_TransportBeaker.yaml    - 复杂运输
level3_pick.yaml               - 复杂抓取
level3_press.yaml              - 复杂按压
level3_open.yaml               - 复杂打开
```

#### Level 4: 长序列任务
```
level4_CleanBeaker.yaml            - 清洗烧杯
level4_CleanBeaker7Policy.yaml     - 7策略清洗烧杯
level4_DeviceOperation.yaml        - 设备操作
level4_LiquidMixing.yaml           - 液体混合
level4_OpenTransportPour.yaml      - 开启-运输-倾倒
```

---

## 根目录文件

### 核心文件

| 文件 | 类型 | 功能 |
|------|------|------|
| `main.py` | Python | 主入口，运行仿真任务 |
| `train.py` | Python | 训练脚本 |
| `train-muilt.py` | Python | 多任务训练脚本 |
| `requirements.txt` | Text | Python 依赖列表 |
| `LICENSE` | Text | MIT 许可证 |
| `README.md` | Markdown | 英文项目说明 |
| `README_CN.md` | Markdown | 中文项目说明 |

### 演示脚本

| 文件 | 功能 |
|------|------|
| `demo_ridgebase_astar_nav.py` | Ridgebase 机器人 A* 导航演示 |
| `demo_ridgebase_astar_nav_backup.py` | 演示脚本备份 |
| `demo_ridgebase_astar_nav_save.py` | 演示脚本保存版 |
| `nav_goal_demo.py` | 导航目标演示 |
| `run_nav_with_video.py` | 带视频录制的导航运行 |

### 测试脚本

| 文件 | 功能 |
|------|------|
| `test_speed_simple.py` | 简单速度测试 |
| `test_navigation_speed.py` | 导航速度测试 |
| `test_navigation_velocity.py` | 导航速度测试 |
| `test_pose_config.py` | 位姿配置测试 |
| `test_goal_pairs_paths.py` | 目标对路径测试 |
| `test_batch_path_planning.py` | 批量路径规划测试 |
| `test_batch_scene_nav.py` | 批量场景导航测试 |

### 辅助脚本

| 文件 | 功能 |
|------|------|
| `extract_nav_goals_to_json.py` | 提取导航目标到 JSON |

### 文档文件

| 文件 | 内容 |
|------|------|
| `controller_comparison.md` | 控制器性能对比 |
| `README_velocity_integration.md` | 速度集成说明 |

---

## 配置文件结构

每个配置文件包含以下主要部分：

```yaml
# 基础配置
name: task_name                    # 任务名称
task_type: "pick"                 # 任务类型
controller_type: "pick"           # 控制器类型
mode: "collect"                   # 模式: collect/infer

# 场景配置
usd_path: "path/to/scene.usd"     # USD 场景文件路径

# 任务参数
task:
  max_steps: 1000                 # 最大步数
  obj_paths:                      # 对象路径配置
    - path: "/World/object_name"
      position_range:             # 位置范围
        x: [min, max]
        y: [min, max]
        z: [min, max]

# 相机配置
cameras_names: ["camera_1", "camera_2"]
cameras:
  - prim_path: "/World/Camera1"
    name: "camera_1"
    translation: [x, y, z]
    resolution: [width, height]
    focal_length: f
    orientation: [x, y, z, w]
    image_type: "rgb"            # rgb/depth/point

# 机器人配置
robot:
  type: "franka"                 # 机器人类型
  position: [x, y, z]            # 机器人位置

# 数据收集配置
collector:
  type: "default"
  compression: null

max_episodes: 100                # 最大回合数

# 推理配置 (仅在 infer 模式下)
infer:
  obs_names: {...}
  policy_model_path: "path/to/model.ckpt"
  policy_config_path: "path/to/config.yaml"
  normalizer_path: "path/to/normalize.ckpt"
  type: "local"                 # local/remote
  host: "server_ip"             # 远程服务器
  port: port_number
  n_obs_steps: 2
  timeout: 30
```

---

## 工具脚本说明

### 路径规划脚本

#### `utils/a_star.py`
**功能:** A* 路径规划算法核心实现

**主要类:**
- `AStarPlanner`: A* 路径规划器

**使用场景:**
- 导航任务中的路径计算
- 避障路径规划

#### `utils/path_planning_precompute.py`
**功能:** 预计算场景中的路径

**输出:** 保存预计算的路径数据

#### `utils/batch_path_planning.py`
**功能:** 批量处理多个路径规划请求

**用途:**
- 多场景路径规划
- 批量导航目标处理

### 导航目标处理脚本

#### `utils/find_nav_target_for_item.py`
**功能:** 为场景中的物品自动查找导航目标点

**输入:** 物品路径
**输出:** 导航目标位置和旋转

#### `utils/fix_nav_target_rotation.py`
**功能:** 修复导航目标的旋转角度

**用途:**
- 确保机器人正确朝向目标
- 优化导航路径

#### `utils/batch_process_nav_targets.py`
**功能:** 批量处理导航目标

**用途:**
- 批量生成导航目标
- 处理多个场景的导航配置

#### `utils/auto_batch_scene_nav.py`
**功能:** 自动批量场景导航

**功能:**
- 自动遍历多个场景
- 执行导航任务
- 生成测试报告

### 数据处理脚本

#### `scripts/convert_labsim_data_to_lerobot.py`
**功能:** 将 LabUtopia 格式数据转换为 LeRobot 格式

**参数:**
- `--data_dir`: 数据目录
- `--num_processes`: 处理进程数
- `--fps`: 帧率
- `--repo_name`: 仓库名称

**使用示例:**
```bash
python scripts/convert_labsim_data_to_lerobot.py \
    --data_dir outputs/collect/xxx/dataset \
    --num_processes 8 \
    --fps 60 \
    --repo_name labutopia/level3-pick
```

#### `scripts/merge_dataset.py`
**功能:** 合并多个数据集

**用途:**
- 合并多次数据收集的结果
- 创建统一的数据集

#### `utils/merge_datasets.py`
**功能:** 数据集合并工具函数

---

## 工作流程

### 数据收集流程

1. **选择配置文件**
   ```bash
   # 在 config/ 目录中选择合适的任务配置
   config/level1_pick.yaml
   ```

2. **修改配置参数**
   - 设置 `max_episodes`
   - 配置相机参数
   - 设置对象位置范围

3. **运行数据收集**
   ```bash
   python main.py --config-name level1_pick
   ```

4. **查看输出**
   ```
   outputs/collect/
   └── 2025.12.05/
       └── 13.07.19_Level1_pour/
           ├── config.yaml
           ├── dataset/
           └── video/
   ```

### 训练流程

1. **准备数据集**
   - 确保数据已收集在 `outputs/collect/` 目录

2. **修改训练配置**
   ```yaml
   # policy/config/train_diffusion_unet_image_workspace.yaml
   dataset_path: "path/to/your/dataset"
   ```

3. **运行训练**
   ```bash
   python train.py --config-name train_diffusion_unet_image_workspace
   ```

4. **查看训练输出**
   ```
   outputs/train/
   └── 2025.03.25/
       └── 12.43.59_train_diffusion_unet_image_pick/
           ├── .hydra/
           ├── checkpoints/
           │   ├── latest.ckpt
           │   └── normalize.ckpt
           └── logs/
   ```

### 推理流程

1. **修改配置为推理模式**
   ```yaml
   mode: "infer"
   ```

2. **配置推理参数**
   ```yaml
   infer:
     policy_model_path: "outputs/train/.../checkpoints/latest.ckpt"
     policy_config_path: "outputs/train/.../.hydra/config.yaml"
     normalizer_path: "outputs/train/.../checkpoints/normalize.ckpt"
   ```

3. **运行推理**
   ```bash
   python main.py --config-name level1_pick
   ```

4. **查看推理结果**
   ```
   outputs/infer/
   └── 2025.03.25/
       └── 15.30.00_infer_pick/
           ├── config.yaml
           └── video/
   ```

---

## 开发指南

### 添加新任务

1. **创建任务类** (`tasks/new_task.py`)
   ```python
   from tasks.base_task import BaseTask

   class NewTask(BaseTask):
       def __init__(self, cfg, world, stage, robot):
           super().__init__(cfg, world, stage, robot)
           # 任务初始化

       def step(self):
           # 任务步骤逻辑
           pass
   ```

2. **创建控制器** (`controllers/new_controller.py`)
   ```python
   from controllers.base_controller import BaseController

   class NewController(BaseController):
       def __init__(self, cfg, robot):
           super().__init__(cfg, robot)
           # 控制器初始化

       def step(self, state):
           # 控制逻辑
           pass
   ```

3. **创建配置文件** (`config/level1_new_task.yaml`)
   ```yaml
   name: level1_new_task
   task_type: "new_task"
   controller_type: "new_controller"
   mode: "collect"
   # ... 其他配置
   ```

4. **注册到工厂** (`factories/task_factory.py`, `factories/controller_factory.py`)

### 添加新机器人

1. **在 `robots/` 目录添加机器人配置**
2. **在 `factories/robot_factory.py` 注册机器人**
3. **创建对应的控制器** (`controllers/robot_controllers/`)

---

## 常见问题

### Q1: 如何切换机器人类型？
修改配置文件中的 `robot.type` 字段：
```yaml
robot:
  type: "franka"        # 或 "ridgebase", "fetch"
  position: [x, y, z]
```

### Q2: 如何修改相机设置？
在配置文件中修改 `cameras` 部分：
```yaml
cameras:
  - resolution: [256, 256]    # 修改分辨率
    image_type: "rgb"         # 或 "depth", "point", "rgb+depth"
```

### Q3: 如何使用远程推理？
配置 `infer.type` 为 "remote"：
```yaml
infer:
  type: "remote"
  host: "server_ip"
  port: 8080
```

### Q4: 导航任务如何配置？
使用 Level 5 导航配置，确保：
1. 场景文件包含导航网格
2. 配置导航目标和路径
3. 使用 Ridgebase 或 Fetch 机器人

---

## 版本历史

- **v1.0** - 初始版本，基础任务实现
- **v2.0** - 添加 Diffusion Policy 支持
- **v3.0** - 实现导航功能和 A* 路径规划
- **v4.0** - 添加 Ridgebase 机器人支持
- **v5.0** - 导航平滑优化，减少抖动

---

## 联系方式

- 项目主页: https://rui-li023.github.io/labutopia-site/
- 论文: https://arxiv.org/abs/2505.22634
- 数据集: https://huggingface.co/datasets/Ruinwalker/Labutopia-Dataset

---

**最后更新:** 2025-01-04
