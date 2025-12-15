"""
导航示例脚本：指定起点/终点并保存视频（viewport 截屏）。
使用导航资产 config/navigation/navigation_assets_fbh2.yaml，A* 规划 + Ridgebase 控制。

说明：
- 修改 START/END 以设定导航路径；或设为 None 让脚本随机选点。
- 视频保存在 outputs/nav_runs/{timestamp}/video.mp4。
- 默认 headless=False 显示窗口；如需无头将 HEADLESS 改 True。
"""

import os
import json
import time
from pathlib import Path

import numpy as np
import yaml
import cv2
from isaacsim import SimulationApp

# ===== 可调参数 =====
HEADLESS = False  # 需录视频请保持 False
START_POINT = [4.3, 6.1]   # None 则随机
END_POINT = [4.3, 1.53]    # None 则随机
NAV_CFG = Path("config/navigation/navigation_assets_fbh2.yaml")
FPS = 30.0
# ==================

simulation_app = SimulationApp({"headless": HEADLESS})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
import omni.usd
# 视口截帧接口，兼容不同版本
try:
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport
except ImportError:
    from omni.kit.viewport.utility import get_active_viewport
    capture_viewport = None
try:
    from omni.kit.viewport.utility import capture_viewport_to_image  # 旧版本接口
except Exception:
    capture_viewport_to_image = None

from controllers.robot_controllers.ridgebase.ridgebase_controller import RidgebaseController
from utils.a_star import plan_navigation_path, real_to_grid, load_grid

ROOT = Path(__file__).resolve().parent


def load_assets_config(config_path):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config["assets"]


def generate_random_points(x_bounds, y_bounds, grid, attempts=200):
    W = len(grid[0])
    H = len(grid)
    for _ in range(attempts):
        sx = np.random.uniform(x_bounds[0], x_bounds[1])
        sy = np.random.uniform(y_bounds[0], y_bounds[1])
        ex = np.random.uniform(x_bounds[0], x_bounds[1])
        ey = np.random.uniform(y_bounds[0], y_bounds[1])
        i_s, j_s = real_to_grid(sx, sy, x_bounds, y_bounds, (W, H))
        i_e, j_e = real_to_grid(ex, ey, x_bounds, y_bounds, (W, H))
        if grid[i_s][j_s] == 0 and grid[i_e][j_e] == 0:
            return [sx, sy], [ex, ey]
    raise RuntimeError("Failed to find valid random start/end")


def validate_points(start_point, end_point, x_bounds, y_bounds, grid):
    W = len(grid[0])
    H = len(grid)
    i_s, j_s = real_to_grid(start_point[0], start_point[1], x_bounds, y_bounds, (W, H))
    i_e, j_e = real_to_grid(end_point[0], end_point[1], x_bounds, y_bounds, (W, H))
    if not (x_bounds[0] <= start_point[0] <= x_bounds[1] and y_bounds[0] <= start_point[1] <= y_bounds[1]):
        return False, "start out of bounds"
    if not (x_bounds[0] <= end_point[0] <= x_bounds[1] and y_bounds[0] <= end_point[1] <= y_bounds[1]):
        return False, "end out of bounds"
    if grid[i_s][j_s] != 0:
        return False, "start on obstacle"
    if grid[i_e][j_e] != 0:
        return False, "end on obstacle"
    return True, "ok"


def ensure_run_dir():
    ts = time.strftime("%Y.%m.%d_%H.%M.%S")
    run_dir = ROOT / "outputs" / "nav_runs_video" / ts
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def main():
    if HEADLESS:
        print("⚠️ HEADLESS=True 时无法截取视口，视频不会生成。请将 HEADLESS=False。")
    run_dir = ensure_run_dir()
    assets = load_assets_config(NAV_CFG)
    nav_scene = assets[0]

    world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene")
    robot_path = "/World/Ridgebase"
    add_reference_to_stage(usd_path="assets/robots/ridgeback_franka.usd", prim_path=robot_path)
    robot = Robot(prim_path=robot_path, name="ridgebase", position=np.array([0.0, 0.0, 0.0]))
    controller = RidgebaseController(
        robot_articulation=robot,
        max_linear_speed=0.04,
        max_angular_speed=1.5,
        position_threshold=0.08,
        angle_threshold=0.1,
    )

    scene_path = os.path.abspath(nav_scene["scene_asset_path"])
    add_reference_to_stage(usd_path=scene_path, prim_path="/World")
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/World").IsValid():
        print("❌ Scene load failed"); simulation_app.close(); return

    barrier_path = nav_scene["barrier_image_path"]
    grid, W, H = load_grid(barrier_path)

    # start/end
    s, e = START_POINT, END_POINT
    if s is None or e is None:
        s, e = generate_random_points(nav_scene["x_bounds"], nav_scene["y_bounds"], grid)
    ok, msg = validate_points(s, e, nav_scene["x_bounds"], nav_scene["y_bounds"], grid)
    if not ok:
        print(f"❌ {msg}"); simulation_app.close(); return

    task_info = {"asset": nav_scene, "start": s, "end": e}
    res = plan_navigation_path(task_info)
    if not res:
        print("❌ Path planning failed"); simulation_app.close(); return
    path_real, _ = res

    waypoints = []
    for i in range(len(path_real)):
        x, y, r = path_real[i]
        if i < len(path_real) - 1:
            nx, ny, _ = path_real[i + 1]
            theta = np.arctan2(ny - y, nx - x)
        else:
            theta = waypoints[-1][2] if waypoints else 0.0
        waypoints.append([x, y, theta])
    controller.set_waypoints(waypoints)
    robot.set_world_pose(
        position=np.array([s[0], s[1], 0.0]),
        orientation=euler_angles_to_quat(np.array([0.0, 0.0, waypoints[0][2]])),
    )

    # video writer
    video_path = run_dir / "video.mp4"
    writer = None

    warn_no_capture = False

    def capture_frame():
        nonlocal writer, warn_no_capture
        vp = get_active_viewport()
        if vp is None:
            if not warn_no_capture:
                print("⚠️ 未找到活动 viewport，无法录制视频。请在 GUI 模式下运行。")
                warn_no_capture = True
            return
        img = None
        # 依次尝试新版与旧版接口
        if capture_viewport:
            try:
                img = capture_viewport(vp)
            except Exception:
                img = None
        if img is None and capture_viewport_to_image:
            try:
                img = capture_viewport_to_image(vp)
            except Exception:
                img = None
        if img is None:
            if not warn_no_capture:
                print("⚠️ 无可用的 viewport 截帧接口，视频将不会生成。")
                warn_no_capture = True
            return
        # img: HxWx4 (RGBA)
        frame = np.array(img, copy=False)
        frame = frame[:, :, :3]  # drop alpha
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if writer is None:
            h, w, _ = frame.shape
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(str(video_path), fourcc, FPS, (w, h))
        writer.write(frame)

    robot_initialized = False
    world.reset()

    print(f"Scene: {scene_path}")
    print(f"Barrier: {barrier_path}")
    print(f"Start: {s}, End: {e}")
    print(f"Video: {video_path}")

    while simulation_app.is_running():
        world.step(render=True)
        capture_frame()

        if world.is_stopped():
            break

        if world.is_playing():
            if not robot_initialized:
                robot.initialize()
                robot_initialized = True

            pos, ori = robot.get_world_pose()
            yaw = quat_to_euler_angles(ori, extrinsic=False)[2]
            current_pose = np.array([pos[0], pos[1], yaw])
            action, done = controller.get_action(current_pose)
            if action is not None:
                robot.apply_action(action)
            if done or controller.is_path_complete():
                final_pos = robot.get_world_pose()[0]
                print(f"✅ Navigation completed. Final: [{final_pos[0]:.2f}, {final_pos[1]:.2f}]")
                break

    if writer:
        writer.release()
    simulation_app.close()
    # 保存简单 run.json
    summary = {
        "scene": scene_path,
        "barrier": barrier_path,
        "start": s,
        "end": e,
        "waypoints": len(waypoints),
        "video": str(video_path),
    }
    (run_dir / "run.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"Run saved to {run_dir}")


if __name__ == "__main__":
    main()
