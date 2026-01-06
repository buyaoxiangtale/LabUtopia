"""
Navigation demo with saving run summary.
- Loads navigation scene and barrier from config/navigation/navigation_assets_fbh2.yaml
- Supports manual or random start/end
- Saves a run summary (scene, start/end, waypoints count, success) to
  outputs/nav_runs/{timestamp}/run.json and copies the config there.
"""

import os
import json
import time
from pathlib import Path

import numpy as np
import yaml
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from isaacsim.core.api.robots.robot import Robot
import omni.usd

from controllers.robot_controllers.ridgebase.ridgebase_controller import RidgebaseController
from utils.a_star import plan_navigation_path, real_to_grid, load_grid

ROOT = Path(__file__).resolve().parent
NAV_CFG = ROOT / "config/navigation/navigation_assets_fbh2.yaml"


def load_assets_config(config_path):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config["assets"]


def generate_random_points(x_bounds, y_bounds, grid, attempts=100):
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
    raise RuntimeError("Failed to find valid start/end in free space")


def validate_points(start_point, end_point, x_bounds, y_bounds, grid):
    W = len(grid[0])
    H = len(grid)
    i_s, j_s = real_to_grid(start_point[0], start_point[1], x_bounds, y_bounds, (W, H))
    i_e, j_e = real_to_grid(end_point[0], end_point[1], x_bounds, y_bounds, (W, H))
    if not (x_bounds[0] <= start_point[0] <= x_bounds[1] and y_bounds[0] <= start_point[1] <= y_bounds[1]):
        return False, "Start point out of bounds"
    if not (x_bounds[0] <= end_point[0] <= x_bounds[1] and y_bounds[0] <= end_point[1] <= y_bounds[1]):
        return False, "End point out of bounds"
    if grid[i_s][j_s] != 0:
        return False, "Start on obstacle"
    if grid[i_e][j_e] != 0:
        return False, "End on obstacle"
    return True, "ok"


def ensure_run_dir():
    ts = time.strftime("%Y.%m.%d_%H.%M.%S")
    run_dir = ROOT / "outputs" / "nav_runs" / ts
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def save_summary(run_dir, data, cfg_path):
    (run_dir / "run.json").write_text(json.dumps(data, indent=2), encoding="utf-8")
    # copy config for reference
    cfg_target = run_dir / Path(cfg_path).name
    if os.path.exists(cfg_path):
        with open(cfg_path, "rb") as src, open(cfg_target, "wb") as dst:
            dst.write(src.read())


def run_navigation(start_point=None, end_point=None, auto_loop=False):
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
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        print("❌ Scene load failed")
        simulation_app.close()
        return False

    barrier_path = nav_scene["barrier_image_path"]
    grid, W, H = load_grid(barrier_path)

    use_manual = start_point is not None and end_point is not None

    summary = {
        "scene": scene_path,
        "barrier": barrier_path,
        "runs": [],
    }

    def setup(nav_start=None, nav_end=None):
        nonlocal summary
        try:
            if nav_start is None or nav_end is None:
                nav_start, nav_end = generate_random_points(nav_scene["x_bounds"], nav_scene["y_bounds"], grid)
                mode = "random"
            else:
                mode = "manual"
                ok, msg = validate_points(nav_start, nav_end, nav_scene["x_bounds"], nav_scene["y_bounds"], grid)
                if not ok:
                    print(f"❌ {msg}")
                    return None, None, None
            task_info = {"asset": nav_scene, "start": nav_start, "end": nav_end}
            result = plan_navigation_path(task_info)
            if not result:
                print("❌ Path planning failed")
                return None, None, None
            path_real, path_grid = result
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
                position=np.array([nav_start[0], nav_start[1], 0.0]),
                orientation=euler_angles_to_quat(np.array([0.0, 0.0, waypoints[0][2]])),
            )
            return nav_start, nav_end, waypoints
        except Exception as e:
            print(f"❌ Exception in setup: {e}")
            return None, None, None

    s, e, wps = setup(start_point, end_point)
    if s is None:
        simulation_app.close()
        return False

    reset_need = False
    robot_initialized = False
    world.reset()

    def record_run(s, e, wps, status):
        summary["runs"].append(
            {
                "start": s,
                "end": e,
                "waypoints": len(wps) if wps else 0,
                "status": status,
            }
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
                if use_manual and not auto_loop:
                    s, e, wps = setup(start_point, end_point)
                else:
                    s, e, wps = setup(None, None)
                if s is None:
                    record_run(start_point, end_point, wps, "failed")
                    break
                reset_need = False

            pos, ori = robot.get_world_pose()
            yaw = quat_to_euler_angles(ori, extrinsic=False)[2]
            current_pose = np.array([pos[0], pos[1], yaw])
            action, done = controller.get_action(current_pose)
            if action is not None:
                robot.apply_action(action)
            if done or controller.is_path_complete():
                final_pos = robot.get_world_pose()[0]
                print(f"\n✅ Navigation completed! Final: [{final_pos[0]:.2f}, {final_pos[1]:.2f}]")
                record_run(s, e, wps, "success")
                if use_manual and not auto_loop:
                    break
                reset_need = True

    save_summary(run_dir, summary, NAV_CFG)
    print(f"Run summary saved to: {run_dir}")
    simulation_app.close()
    return True


def main():
    # 默认使用 manual 点，可自行修改
    USE_MANUAL_POINTS = True
    MANUAL_START_POINT = [4.3, 6.1]
    MANUAL_END_POINT = [4.3, 1.53]
    AUTO_LOOP = False

    if USE_MANUAL_POINTS:
        run_navigation(start_point=MANUAL_START_POINT, end_point=MANUAL_END_POINT, auto_loop=AUTO_LOOP)
    else:
        run_navigation(auto_loop=AUTO_LOOP)


if __name__ == "__main__":
    main()
