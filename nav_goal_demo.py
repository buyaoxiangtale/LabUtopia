"""
Navigation demo with user-specified start/end points.
Provides a callable function `run_navigation(start_point, end_point, ...)`
that reuses the existing A* planner and Ridgebase controller.
"""

import os
import numpy as np
from isaacsim import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from omni.isaac.core.robots import Robot

from controllers.robot_controllers.ridgebase.ridgebase_controller import RidgebaseController
from utils.a_star import plan_navigation_path
import yaml

def load_assets(cfg_path):
    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f)
    return cfg["assets"]


def build_world():
    world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene")
    return world


def load_scene(nav_scene):
    add_reference_to_stage(
        usd_path=os.path.abspath(nav_scene["scene_asset_path"]),
        prim_path="/World",
    )


def load_robot():
    robot_path = "/World/Ridgebase"
    add_reference_to_stage(
        usd_path=os.path.abspath("assets/robots/ridgeback_franka.usd"),
        prim_path=robot_path,
    )
    robot = Robot(
        prim_path=robot_path,
        name="ridgebase",
        position=np.array([0.0, 0.0, 0.0]),
    )
    return robot


def compute_waypoints(path_real):
    waypoints = []
    for i, (x, y, _) in enumerate(path_real):
        if i < len(path_real) - 1:
            nx, ny, _ = path_real[i + 1]
            theta = np.arctan2(ny - y, nx - x)
        else:
            theta = waypoints[-1][2] if waypoints else 0.0
        waypoints.append([x, y, theta])
    return waypoints


def run_navigation(
    start_point,
    end_point,
    assets_cfg="config/navigation/navigation_assets.yaml",
    scene_index=0,
    headless=False,
):
    """
    以函数形式运行导航。
    Args:
        start_point (list[float]): [x, y] 起点，需在场景边界内且不落在障碍物上
        end_point (list[float]): [x, y] 终点，同上
        assets_cfg (str): 导航资产配置路径
        scene_index (int): 使用的场景索引
        headless (bool): 是否无头运行
    Returns:
        bool: 是否成功到达终点
    """

    simulation_app = SimulationApp({"headless": headless})
    world = build_world()

    assets = load_assets(assets_cfg)
    nav_scene = assets[scene_index]

    # 加载场景与机器人
    load_scene(nav_scene)
    robot = load_robot()

    controller = RidgebaseController(
        robot_articulation=robot,
        max_linear_speed=0.04,
        max_angular_speed=1.5,
        position_threshold=0.08,
        angle_threshold=0.1,
    )

    # 规划路径
    task_info = {"asset": nav_scene, "start": start_point, "end": end_point}
    path_result = plan_navigation_path(task_info)
    if path_result is None:
        print("❌ 路径规划失败，请检查起终点是否在可行走区域或障碍物未被膨胀挡住。")
        simulation_app.close()
        return False
    path_real, _ = path_result
    waypoints = compute_waypoints(path_real)
    controller.set_waypoints(waypoints)

    # 设置初始位姿
    robot.set_world_pose(
        position=np.array([start_point[0], start_point[1], 0.0]),
        orientation=euler_angles_to_quat(np.array([0.0, 0.0, waypoints[0][2]])),
    )

    world.reset()
    robot_initialized = False
    reset_need = False

    print(
        f"✅ 路径规划成功: 起点 {start_point}, 终点 {end_point}, "
        f"路径点数 {len(waypoints)}, 预计距离 "
        f"{np.linalg.norm(np.array(end_point) - np.array(start_point)):.2f} m"
    )

    while simulation_app.is_running():
        world.step(render=True)

        if world.is_stopped():
            reset_need = True
            robot_initialized = False

        if world.is_playing():
            if not robot_initialized:
                robot.initialize()
                robot_initialized = True

            if reset_need:
                world.reset()
                robot.initialize()
                controller.set_waypoints(waypoints)
                reset_need = False

            pos, ori = robot.get_world_pose()
            yaw = quat_to_euler_angles(ori, extrinsic=False)[2]
            current_pose = np.array([pos[0], pos[1], yaw])

            action, done = controller.get_action(current_pose)
            if action is not None:
                robot.apply_action(action)

            if done or controller.is_path_complete():
                final_pos = robot.get_world_pose()[0]
                print(f"🎉 抵达终点: [{final_pos[0]:.2f}, {final_pos[1]:.2f}]")
                simulation_app.close()
                return True

    simulation_app.close()
    return False


if __name__ == "__main__":
    # 默认示例参数（可自行修改）
    DEFAULT_START = [4.3, 6.1]
    DEFAULT_END = [4.3,1.53]
    run_navigation(
        start_point=DEFAULT_START,
        end_point=DEFAULT_END,
        assets_cfg="config/navigation/navigation_assets.yaml",
        scene_index=0,
        headless=False,
    )
