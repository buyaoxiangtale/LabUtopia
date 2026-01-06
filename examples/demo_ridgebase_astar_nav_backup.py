import os
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
import numpy as np
import yaml
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage

from controllers.robot_controllers.ridgebase.ridgebase_controller import RidgebaseController
from utils.a_star import plan_navigation_path, real_to_grid, load_grid

def load_assets_config(config_path):
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config['assets']

def generate_random_points(x_bounds, y_bounds, grid, attempts=100):
    """Generate random start and end points in free space"""
    W = len(grid[0])
    H = len(grid)
    
    for _ in range(attempts):
        start_x = np.random.uniform(x_bounds[0], x_bounds[1])
        start_y = np.random.uniform(y_bounds[0], y_bounds[1])
        end_x = np.random.uniform(x_bounds[0], x_bounds[1])
        end_y = np.random.uniform(y_bounds[0], y_bounds[1])
        
        i_start, j_start = real_to_grid(
            start_x, start_y, x_bounds, y_bounds, (W, H)
        )
        i_end, j_end = real_to_grid(
            end_x, end_y, x_bounds, y_bounds, (W, H)
        )
        
        if grid[i_start][j_start] == 0 and grid[i_end][j_end] == 0:
            return [start_x, start_y], [end_x, end_y]
            
    raise RuntimeError("Failed to find valid start and end points")

def validate_points(start_point, end_point, x_bounds, y_bounds, grid):
    """Validate if start and end points are in free space
    
    Args:
        start_point: [x, y] 起点坐标
        end_point: [x, y] 终点坐标
        x_bounds: [min_x, max_x] x轴边界
        y_bounds: [min_y, max_y] y轴边界
        grid: 障碍物网格
        
    Returns:
        tuple: (is_valid, message) 验证结果和消息
    """
    W = len(grid[0])
    H = len(grid)
    
    i_start, j_start = real_to_grid(
        start_point[0], start_point[1], x_bounds, y_bounds, (W, H)
    )
    i_end, j_end = real_to_grid(
        end_point[0], end_point[1], x_bounds, y_bounds, (W, H)
    )
    
    if not (x_bounds[0] <= start_point[0] <= x_bounds[1] and 
            y_bounds[0] <= start_point[1] <= y_bounds[1]):
        return False, "Start point is out of scene bounds"
    
    if not (x_bounds[0] <= end_point[0] <= x_bounds[1] and 
            y_bounds[0] <= end_point[1] <= y_bounds[1]):
        return False, "End point is out of scene bounds"
    
    if grid[i_start][j_start] != 0:
        return False, "Start point is on an obstacle"
    
    if grid[i_end][j_end] != 0:
        return False, "End point is on an obstacle"
    
    return True, "Validation passed"


def run_navigation(start_point=None, end_point=None, auto_loop=True, output_pixel_path=True):
    """Run navigation task with specified or random start/end points

    This is the main API function for running navigation tasks.

    Args:
        start_point: [x, y] 起点坐标，如果为 None 则随机生成
        end_point: [x, y] 终点坐标，如果为 None 则随机生成
        auto_loop: bool, 导航完成后是否自动开始新任务（随机点模式）
        output_pixel_path: bool, 是否输出路径中每个像素点的坐标

    Returns:
        bool: 导航任务是否成功启动

    Example:
        # 使用指定的起点和终点
        run_navigation(start_point=[-3.0, 0.5], end_point=[5.0, 2.0])

        # 使用随机生成的起点和终点
        run_navigation()

        # 指定起点，随机终点
        run_navigation(start_point=[-3.0, 0.5])

        # 禁用像素路径输出
        run_navigation(start_point=[-3.0, 0.5], end_point=[5.0, 2.0], output_pixel_path=False)
    """
    world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene")
    
    robot_path = "/World/Ridgebase"
    add_reference_to_stage(
        usd_path="assets/robots/ridgeback_franka.usd",
        prim_path=robot_path
    )
    
    from isaacsim.core.api.robots.robot import Robot
    robot = Robot(
        prim_path=robot_path,
        name="ridgebase",
        position=np.array([0.0, 0.0, 0.0])
    )
    
    controller = RidgebaseController(
        robot_articulation=robot,
        max_linear_speed=0.02,
        max_angular_speed=1.5,
        position_threshold=0.08,
        angle_threshold=0.1
    )
    #需要修改，根据不同的场景，加载不同的资产配置
    
    assets = load_assets_config("config/navigation/navigation_assets_fbh2.yaml")
    nav_scene = assets[0]  

    # 加载场景 USD
    scene_path = os.path.abspath(nav_scene['scene_asset_path'])
    if not os.path.exists(scene_path):
        print(f"❌ Error: Scene file not found: {scene_path}")
        simulation_app.close()
        return False
    print(f"✅ Loading scene: {scene_path}")
    print(f"   prim_path: /World")

    add_reference_to_stage(
        usd_path=scene_path, 
        prim_path="/World"
    )
    
    # 检查场景是否成功加载
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    world_prim = stage.GetPrimAtPath("/World")
    if world_prim.IsValid():
        print(f"   ✅ Scene loaded successfully!")
        # 列出 /World 下的子节点
        children = world_prim.GetChildren()
        print(f"   Children under /World: {len(children)}")
        for i, child in enumerate(children[:10]):  # 只显示前10个
            print(f"      - {child.GetPath()}")
        if len(children) > 10:
            print(f"      ... and {len(children) - 10} more")
    else:
        print(f"   ❌ Failed to load scene! /World prim is invalid")
    
    # 加载障碍物图
    barrier_path = nav_scene['barrier_image_path']
    if not os.path.exists(barrier_path):
        print(f"❌ Error: Barrier image not found: {barrier_path}")
        simulation_app.close()
        return False
    print(f"✅ Loading barrier image: {barrier_path}")
    grid, W, H = load_grid(barrier_path)
    
    # 打印障碍物图信息
    total_cells = W * H
    free_cells = sum(row.count(0) for row in grid)
    obstacle_cells = total_cells - free_cells
    print(f"   Image size: {W} x {H} pixels")
    print(f"   Free cells: {free_cells} ({free_cells/total_cells*100:.1f}%)")
    print(f"   Obstacle cells: {obstacle_cells} ({obstacle_cells/total_cells*100:.1f}%)")
    
    # 判断是否使用手动指定的点
    use_manual_points = start_point is not None and end_point is not None
    
    # 保存初始指定的点（用于重置时）
    initial_start_point = start_point
    initial_end_point = end_point
    
    print("=" * 60)
    print("Isaac Sim Navigation Demo")
    print("=" * 60)
    print(f"Scene bounds: x={nav_scene['x_bounds']}, y={nav_scene['y_bounds']}")
    print(f"Mode: {'Manual points' if use_manual_points else 'Random points'}")
    if use_manual_points:
        print(f"Start point: {start_point}")
        print(f"End point: {end_point}")
    print(f"Auto loop: {auto_loop}")
    print("=" * 60)
    print("\nInstructions:")
    print("1. Wait for Isaac Sim window to open")
    print("2. Click the play button at the bottom to start simulation")
    print("3. Robot will automatically navigate to the destination")
    print("4. Use mouse to rotate, pan, zoom the view")
    print("=" * 60)
    print()
    
    def setup_navigation(nav_start=None, nav_end=None):
        """Setup navigation task with specified or random points
        
        Args:
            nav_start: [x, y] 起点坐标，如果为 None 则随机生成
            nav_end: [x, y] 终点坐标，如果为 None 则随机生成
            
        Returns:
            bool: 是否成功设置导航任务
        """
        path_result = None
        attempts = 0
        max_attempts = 1 if (nav_start is not None and nav_end is not None) else 100
        
        while path_result is None and attempts < max_attempts:
            attempts += 1
            try:
                # 根据参数选择使用指定或随机生成的起点和终点
                if nav_start is not None and nav_end is not None:
                    current_start = nav_start
                    current_end = nav_end
                    
                    # 验证指定的点是否有效
                    is_valid, message = validate_points(
                        current_start, current_end,
                        nav_scene['x_bounds'],
                        nav_scene['y_bounds'],
                        grid
                    )
                    if not is_valid:
                        print(f"Error: {message}")
                        print(f"Start: {current_start}, End: {current_end}")
                        print(f"Scene bounds: x={nav_scene['x_bounds']}, y={nav_scene['y_bounds']}")
                        raise RuntimeError(message)
                    print(f"Using manual points - Start: {current_start}, End: {current_end}")
                else:
                    current_start, current_end = generate_random_points(
                        nav_scene['x_bounds'],
                        nav_scene['y_bounds'],
                        grid
                    )
                    print(f"Using random points - Start: {current_start}, End: {current_end}")
                
                task_info = {
                    "asset": nav_scene,
                    "start": current_start,
                    "end": current_end
                }
                
                path_result = plan_navigation_path(task_info)
                if path_result is not None:
                    merged_path_real, _ = path_result
                    print(f"\n✅ Path planning successful!")
                    print(f"   Start: [{current_start[0]:.2f}, {current_start[1]:.2f}]")
                    print(f"   End: [{current_end[0]:.2f}, {current_end[1]:.2f}]")
                    print(f"   Waypoints: {len(merged_path_real)}")
                    print(f"   Estimated distance: {np.sqrt((current_end[0]-current_start[0])**2 + (current_end[1]-current_start[1])**2):.2f} m")
                    
                    waypoints = []
                    for i in range(len(merged_path_real)):
                        x, y, r = merged_path_real[i]

                        if i < len(merged_path_real) - 1:
                            next_x, next_y, _ = merged_path_real[i + 1]
                            theta = np.arctan2(next_y - y, next_x - x)
                        else:
                            theta = waypoints[-1][2] if waypoints else 0.0
                        waypoints.append([x, y, theta])

                    # 输出路径中每个像素点（如果启用）
                    if output_pixel_path:
                        print(f"\n📍 Path pixel coordinates (row, col):")
                        pixel_path = []
                        for i, (x, y, r) in enumerate(merged_path_real):
                            pixel_row, pixel_col = real_to_grid(
                                x, y,
                                nav_scene['x_bounds'],
                                nav_scene['y_bounds'],
                                (W, H)
                            )
                            pixel_path.append((pixel_row, pixel_col))
                            print(f"   Waypoint {i+1:2d}: world=({x:6.2f}, {y:6.2f}) -> pixel=({pixel_row:3d}, {pixel_col:3d})")

                        print(f"   Total path pixels: {len(pixel_path)}")

                        # 可选：保存像素路径到文件
                        try:
                            import json
                            pixel_path_data = {
                                "start_world": current_start,
                                "end_world": current_end,
                                "start_pixel": pixel_path[0] if pixel_path else None,
                                "end_pixel": pixel_path[-1] if pixel_path else None,
                                "pixel_path": pixel_path,
                                "world_path": [[float(x), float(y), float(r)] for x, y, r in merged_path_real],
                                "grid_size": [H, W],  # 注意：grid[H][W]，H是行数，W是列数
                                "scene_bounds": {
                                    "x_bounds": nav_scene['x_bounds'],
                                    "y_bounds": nav_scene['y_bounds']
                                }
                            }

                            output_file = f"navigation_path_pixels_{len(pixel_path)}points.json"
                            with open(output_file, 'w') as f:
                                json.dump(pixel_path_data, f, indent=2)
                            print(f"   💾 Pixel path saved to: {output_file}")

                        except Exception as e:
                            print(f"   ⚠️  Failed to save pixel path: {e}")
                    
                    controller.set_waypoints(waypoints)
                    
                    initial_position = np.array([current_start[0], current_start[1], 0.0])
                    initial_orientation = np.array([0.0, 0.0, waypoints[0][2]])  

                    from isaacsim.core.utils.rotations import euler_angles_to_quat
                    initial_rotation = euler_angles_to_quat(initial_orientation)
                    
                    robot.set_world_pose(position=initial_position)
                    return True
                else:
                    if nav_start is not None and nav_end is not None:
                        print(f"\n❌ Path planning failed: Cannot reach from {current_start} to {current_end}")
                        print(f"   Please check if points are in traversable area")
                        return False
                    
            except RuntimeError as e:
                print(f"Error: {e}")
                return False
        
        if attempts >= max_attempts:
            print(f"\n❌ Path planning failed after {max_attempts} attempts")
        return False

    # 初始化导航
    if not setup_navigation(start_point, end_point):
        simulation_app.close()
        return False
        
    reset_need = False
    robot_initialized = False
    task_completed = False
    world.reset()
    
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
                
                # 根据模式决定重置时使用的点
                if use_manual_points and not auto_loop:
                    # 手动模式且不自动循环，使用原始指定的点
                    if not setup_navigation(initial_start_point, initial_end_point):
                        simulation_app.close()
                        break
                else:
                    # 随机模式或自动循环模式，使用随机点
                    if not setup_navigation(None, None):
                        simulation_app.close()
                        break

                reset_need = False
                
            position, orientation = robot.get_world_pose()
            from isaacsim.core.utils.rotations import quat_to_euler_angles
            euler_angles = quat_to_euler_angles(orientation, extrinsic=False)
            current_pose = np.array([position[0], position[1], euler_angles[2]])
            
            action, done = controller.get_action(current_pose)
            
            if action is not None:
                robot.apply_action(action)
            
            if done or controller.is_path_complete():
                final_position = robot.get_world_pose()[0]
                print(f"\n✅ Navigation completed!")
                print(f"   Final position: [{final_position[0]:.2f}, {final_position[1]:.2f}]")
                
                if use_manual_points and not auto_loop:
                    # 手动模式且不自动循环，任务完成后退出
                    print("   Task finished. Closing simulation...")
                    task_completed = True
                    simulation_app.close()
                    break
                else:
                    print("   Starting new navigation task...\n")
                    reset_need = True
    
    simulation_app.close()
    return task_completed or True


def main():
    """Main function with configurable navigation mode

    Configure the following options:
    - USE_MANUAL_POINTS: True to use manual points, False for random
    - MANUAL_START_POINT: Start coordinates [x, y]
    - MANUAL_END_POINT: End coordinates [x, y]
    - AUTO_LOOP: True to auto-start new task after completion
    - OUTPUT_PIXEL_PATH: True to output path pixel coordinates to console and file
    """
    # ========== Configuration Options ==========
    # Set to True to use manually specified start and end points
    # Set to False to use randomly generated start and end points
    USE_MANUAL_POINTS = True

    # Manually specified start and end points (only used when USE_MANUAL_POINTS=True)
    # Coordinates must be within scene bounds and not on obstacles
    # Scene bounds: x=[-7.925, 9.175], y=[-1.525, 4.175]

    # Path found! Start: [-3.616787721733812, -0.5491800431903086], End: [8.346586494370905, 1.3575864445663188]
    # Navigation completed! Resetting new navigation task...
    # Path found! Start: [6.971586473272797, -0.11371377176966146], End: [1.8200373483185643, 1.9644897802529644]
    # Navigation completed! Resetting new navigation task...
    # Path found! Start: [-6.687120187801245, 0.3711462282850533], End: [0.5845890419860886, 2.0521614770933994]
    MANUAL_START_POINT =[4.3 ,6.1]  # [x, y]
    MANUAL_END_POINT = [4.3, 1.53]     # [x, y]

    # Auto loop: start new navigation task after completion
    # Only effective when USE_MANUAL_POINTS=False or set to True
    AUTO_LOOP = True

    # Output pixel path: save path pixel coordinates to console and JSON file
    OUTPUT_PIXEL_PATH = True
    # ============================================

    if USE_MANUAL_POINTS:
        run_navigation(
            start_point=MANUAL_START_POINT,
            end_point=MANUAL_END_POINT,
            auto_loop=AUTO_LOOP,
            output_pixel_path=OUTPUT_PIXEL_PATH
        )
    else:
        run_navigation(
            auto_loop=AUTO_LOOP,
            output_pixel_path=OUTPUT_PIXEL_PATH
        )


if __name__ == "__main__":
    main()



