import heapq
from PIL import Image
import copy
import matplotlib.pyplot as plt
import numpy as np
from typing import Optional, Tuple, List

def a_star(grid, start, end, diagonal_cost=1.414):
    """A* pathfinding algorithm with diagonal movement cost."""
    open_heap = []
    heapq.heappush(open_heap, (0, *start))
    came_from = {}
    g_scores = {start: 0}
    f_scores = {start: heuristic(start, end)}
    
    while open_heap:
        current_f, cx, cy = heapq.heappop(open_heap)
        if (cx, cy) == end:
            return reconstruct_path(came_from, end)
        
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),]:
            nx, ny = cx + dx, cy + dy
            if (
                not (0 <= nx < len(grid) and 0 <= ny < len(grid[0]))
                or grid[nx][ny] != 0
            ):
                continue
            
            move_cost = diagonal_cost if dx != 0 and dy != 0 else 1
            tentative_g = g_scores[(cx, cy)] + move_cost
            
            if tentative_g < g_scores.get((nx, ny), float("inf")):
                came_from[(nx, ny)] = (cx, cy)
                g_scores[(nx, ny)] = tentative_g
                f = tentative_g + heuristic((nx, ny), end)
                heapq.heappush(open_heap, (f, nx, ny))
    
    return None

def heuristic(a, b):
    """Manhattan distance heuristic."""
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return dx + dy

def reconstruct_path(came_from, end):
    path = [end]
    while path[-1] in came_from:
        path.append(came_from[path[-1]])
    return path[::-1]

def load_grid(image_path):
    """Convert image to binary grid representation."""
    with Image.open(image_path) as img:
        W, H = img.size
        return [
            [
                [0 if is_white(img.getpixel((j, i))) else 1 for j in range(W)]
                for i in range(H)
            ],
            W,
            H,
        ]

def is_white(pixel):
    """Determine if pixel represents white space."""
    if isinstance(pixel, int):
        return pixel == 255
    channels = pixel[:3]
    return channels == (255, 255, 255)

def inflate_obstacles(grid: np.ndarray, radius: int) -> np.ndarray:
    """Expand obstacles by robot radius."""
    from scipy.ndimage import binary_dilation
    struct = np.ones((2 * radius + 1, 2 * radius + 1))
    return binary_dilation(grid, structure=struct).astype(np.int32)

def real_to_grid(x, y, x_bounds, y_bounds, grid_size):
    """Convert real coordinates to grid indices."""
    x_min, x_max = x_bounds
    y_min, y_max = y_bounds
    W, H = grid_size
    return (
        min(max(int((y_max - y) / (y_max - y_min) * H), 0), H - 1),
        min(max(int((x - x_min) / (x_max - x_min) * W), 0), W - 1),
    )

def grid_to_real(i, j, x_bounds, y_bounds, grid_size):
    """Convert grid indices to real coordinates."""
    x_min, x_max = x_bounds
    y_min, y_max = y_bounds
    W, H = grid_size
    return (
        x_min + (j + 0.5) * (x_max - x_min) / W,
        y_max - (i + 0.5) * (y_max - y_min) / H,
    )

def calculate_path_distance(path: List[List[float]]) -> float:
    """
    Calculate the total distance of a path by summing Euclidean distances between consecutive points.
    
    Args:
        path: List of path points, each point is [x, y, ...] or [x, y]
    
    Returns:
        float: Total path distance in meters
    """
    if len(path) < 2:
        return 0.0
    
    total_distance = 0.0
    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i + 1]
        
        # 确保坐标格式一致
        if len(p1) == 2:
            x1, y1 = p1[0], p1[1]
        else:
            x1, y1, z1 = p1[0], p1[1], p1[2]
            x2, y2 = p2[0], p2[1] if len(p2) >= 2 else p2[1]
            
        dx = x2 - x1
        dy = y2 - y1
        total_distance += np.sqrt(dx**2 + dy**2)
    
    return total_distance

def visualize_failure(grid, start, end, start_on_obstacle=False, end_on_obstacle=False, output_path=None):
    """
    可视化路径规划失败情况
    
    Args:
        grid: 二值网格 (0=可通行, 1=障碍物)，可以是列表或numpy数组
        start: 起点坐标 [i, j]
        end: 终点坐标 [i, j]
        start_on_obstacle: 起点是否在障碍物上
        end_on_obstacle: 终点是否在障碍物上
        output_path: 保存路径 (可选)
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    
    # 确保 grid 是 numpy 数组
    if not isinstance(grid, np.ndarray):
        grid = np.array(grid)
    
    fig, ax = plt.subplots(figsize=(12, 12))
    
    # 显示网格 (0=白色可通行, 1=黑色障碍物)
    ax.imshow(grid, cmap='binary', origin='upper', extent=[0, grid.shape[1], grid.shape[0], 0])
    
    # 标记起点
    start_y, start_x = start[0], start[1]
    color1 = 'lime' if not start_on_obstacle else 'yellow'
    facecolor1 = 'lime' if not start_on_obstacle else 'yellow'
    edgecolor1 = 'green' if not start_on_obstacle else 'orange'
    circle1 = patches.Circle((start_x, start_y), radius=3, linewidth=2, 
                           edgecolor=edgecolor1, facecolor=facecolor1, label='Start')
    ax.add_patch(circle1)
    if start_on_obstacle:
        ax.text(start_x, start_y + 5, '✗ Start on obstacle!', color='red', fontsize=12, 
               bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # 标记终点
    end_y, end_x = end[0], end[1]
    color2 = 'orange' if not end_on_obstacle else 'yellow'
    facecolor2 = 'orange' if not end_on_obstacle else 'yellow'
    edgecolor2 = 'red' if not end_on_obstacle else 'orange'
    circle2 = patches.Circle((end_x, end_y), radius=3, linewidth=2, 
                           edgecolor=edgecolor2, facecolor=facecolor2, label='End')
    ax.add_patch(circle2)
    if end_on_obstacle:
        ax.text(end_x, end_y + 5, '✗ End on obstacle!', color='red', fontsize=12, 
               bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # 标题和说明
    title_parts = []
    if start_on_obstacle and end_on_obstacle:
        title_parts.append("Both on obstacle!")
    elif start_on_obstacle:
        title_parts.append("Start on obstacle!")
    elif end_on_obstacle:
        title_parts.append("End on obstacle!")
    else:
        title_parts.append("No direct obstacle")
    
    ax.set_title(f"Path Finding Failed - {' | '.join(title_parts)}\nGreen=Start, Red=End, Lime/Yellow=OK", fontsize=12, weight='bold')
    
    # 添加说明
    info_text = "Possible reasons:\n"
    info_text += "1. Start/End points inside obstacles (blocked)\n"
    info_text += "2. Path completely blocked by obstacles\n"
    info_text += "3. No valid path through maze\n"
    
    ax.text(len(grid[0])//2, len(grid)//2 - 2, info_text, color='black', fontsize=10,
           bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.3),
           verticalalignment='top')
    
    ax.set_xlim(-1, len(grid[0]))
    ax.set_ylim(len(grid), -1)
    ax.set_xlabel('X (pixels)')
    ax.set_ylabel('Y (pixels)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"失败可视化已保存到: {output_path}")
    
    plt.close()

def save_path_image(grid_path, path, save_path=None):

    plt.figure(figsize=(10, 10))

    plt.imshow(grid_path, cmap='binary')
    
    if path:
        
        path_i, path_j = zip(*path)
        plt.plot(path_j, path_i, 'r-', linewidth=2, label='Path')
        plt.plot(path_j[0], path_i[0], 'go', markersize=10, label='Start')
        plt.plot(path_j[-1], path_i[-1], 'bo', markersize=10, label='End')
    
    plt.legend()
    
    if save_path:
        plt.savefig(save_path)
        plt.close()
    else:
        plt.show()

def visualize_pathfinding(grid, start, end, path_grid=None, output_path=None):
    """
    可视化路径规划过程
    
    Args:
        grid: 二值网格 (0=可通行, 1=障碍物)
        start: 起点坐标 [i, j]
        end: 终点坐标 [i, j]
        path_grid: 网格路径 (可选)
        output_path: 保存路径 (可选)
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    
    # 确保 grid 是 numpy 数组
    if not isinstance(grid, np.ndarray):
        grid = np.array(grid)
    
    fig, ax = plt.subplots(figsize=(12, 12))
    
    # 显示网格 (0=白色可通行, 1=黑色障碍物)
    ax.imshow(grid, cmap='binary', origin='upper', extent=[0, grid.shape[1], grid.shape[0], 0])
    
    # 标记起点 (绿色圆点)
    start_y, start_x = start[0], start[1]
    circle1 = patches.Circle((start_x, start_y), radius=3, linewidth=2, edgecolor='green', facecolor='lime', label='Start')
    ax.add_patch(circle1)
    
    # 标记终点 (红色圆点)
    end_y, end_x = end[0], end[1]
    circle2 = patches.Circle((end_x, end_y), radius=3, linewidth=2, edgecolor='red', facecolor='orange', label='End')
    ax.add_patch(circle2)
    
    # 检查起点是否在障碍物上
    if grid[start_y][start_x] == 1:
        ax.text(start_x, start_y + 5, '✗ Start on obstacle!', color='red', fontsize=12, 
               bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # 检查终点是否在障碍物上
    if grid[end_y][end_x] == 1:
        ax.text(end_x, end_y + 5, '✗ End on obstacle!', color='red', fontsize=12, 
               bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # 如果有路径，绘制路径
    if path_grid:
        # 将路径分为 X 和 Y 坐标
        path_x = [p[1] for p in path_grid]
        path_y = [p[0] for p in path_grid]
        
        # 绘制路径 (蓝色线)
        ax.plot(path_x, path_y, 'b-', linewidth=2, markersize=0, label='Path')
        
        # 标记路径点
        ax.scatter(path_x, path_y, c='blue', s=20, alpha=0.5)
        
        # 在每个路径点上标注序号
        for idx, (px, py) in enumerate(zip(path_x, path_y)):
            if idx % 5 == 0 or idx == len(path_grid) - 1:  # 每5个点标注一次
                ax.text(px, py, str(idx), color='white', fontsize=8, 
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='black', alpha=0.6))
    else:
        ax.text(len(grid[0])//2, len(grid)//2, '✗ NO PATH FOUND!', color='red', fontsize=20,
               ha='center', va='center', weight='bold',
               bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.9))
    
    ax.set_xlim(-1, len(grid[0]))
    ax.set_ylim(len(grid), -1)
    ax.set_xlabel('X (pixels)')
    ax.set_ylabel('Y (pixels)')
    ax.set_title(f'Path Finding: Start {start} → End {end}\nGreen=Start, Red=End, Black=Obstacle, Blue=Path', fontsize=12)
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 保存图像
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"可视化结果已保存到: {output_path}")
    
    plt.close()

def visualize_pathfinding(grid, start, end, path_grid=None, output_path=None):
    """
    可视化路径规划过程
    
    Args:
        grid: 二值网格 (0=可通行, 1=障碍物)
        start: 起点坐标 [i, j]
        end: 终点坐标 [i, j]
        path_grid: 网格路径 (可选)
        output_path: 保存路径 (可选)
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    
    # 确保 grid 是 numpy 数组
    if not isinstance(grid, np.ndarray):
        grid = np.array(grid)
    
    fig, ax = plt.subplots(figsize=(12, 12))
    
    # 显示网格 (0=白色可通行, 1=黑色障碍物)
    ax.imshow(grid, cmap='binary', origin='upper', extent=[0, grid.shape[1], grid.shape[0], 0])
    
    # 标记起点 (绿色圆点)
    start_y, start_x = start[0], start[1]
    circle1 = patches.Circle((start_x, start_y), radius=3, linewidth=2, edgecolor='green', facecolor='lime', label='Start')
    ax.add_patch(circle1)
    
    # 标记终点 (红色圆点)
    end_y, end_x = end[0], end[1]
    circle2 = patches.Circle((end_x, end_y), radius=3, linewidth=2, edgecolor='red', facecolor='orange', label='End')
    ax.add_patch(circle2)
    
    # 检查起点是否在障碍物上
    if grid[start_y][start_x] == 1:
        ax.text(start_x, start_y + 5, '✗ Start on obstacle!', color='red', fontsize=12, 
               bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # 检查终点是否在障碍物上
    if grid[end_y][end_x] == 1:
        ax.text(end_x, end_y + 5, '✗ End on obstacle!', color='red', fontsize=12,
               bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # 如果有路径，绘制路径
    if path_grid:
        # 将路径分为 X 和 Y 坐标
        path_x = [p[1] for p in path_grid]
        path_y = [p[0] for p in path_grid]
        
        # 绘制路径 (蓝色线)
        ax.plot(path_x, path_y, 'b-', linewidth=2, markersize=0, label='Path')
        
        # 标记路径点
        ax.scatter(path_x, path_y, c='blue', s=20, alpha=0.5)
        
        # 在每个路径点上标注序号
        for idx, (px, py) in enumerate(zip(path_x, path_y)):
            if idx % 5 == 0 or idx == len(path_grid) - 1:  # 每5个点标注一次
                ax.text(px, py, str(idx), color='white', fontsize=8, 
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='black', alpha=0.6))
    else:
        ax.text(len(grid[0])//2, len(grid)//2, '✗ NO PATH FOUND!', color='red', fontsize=20,
               ha='center', va='center', weight='bold',
               bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.9))
    
    ax.set_xlim(-1, len(grid[0]))
    ax.set_ylim(len(grid), -1)
    ax.set_xlabel('X (pixels)')
    ax.set_ylabel('Y (pixels)')
    ax.set_title(f'Path Finding: Start {start} → End {end}\nGreen=Start, Red=End, Black=Obstacle, Blue=Path', fontsize=12)
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 保存图像
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"可视化结果已保存到: {output_path}")
    
    plt.close()

def plan_navigation_path(task_info: dict) -> Optional[Tuple[List[List[float]], List[List[int]], float]]:
    """
    Plan navigation path using A* algorithm.
    
    Args:
        task_info: Dictionary containing:
            - 'asset': Navigation scene configuration with 'barrier_image_path', 'x_bounds', 'y_bounds', 'offset_radius'
            - 'start': [x, y] starting position
            - 'end': [x, y] ending position
    
    Returns:
        Optional[Tuple]: (real_path, path_grid, total_distance) if path found, None otherwise
            - real_path: List of path points in real coordinates [[x, y, 0.0], ...]
            - path_grid: List of path points in grid coordinates [[i, j], ...]
            - total_distance: Total path distance in meters (float)
    """
    grid, W, H = load_grid(task_info['asset']['barrier_image_path'])
    
    x_bounds = task_info['asset']['x_bounds']
    y_bounds = task_info['asset']['y_bounds']
    meters_per_pixel_x = (x_bounds[1] - x_bounds[0]) / W
    meters_per_pixel_y = (y_bounds[1] - y_bounds[0]) / H
    meters_per_pixel = min(meters_per_pixel_x, meters_per_pixel_y)
    radius_pixels = int(task_info['asset']['offset_radius'] / meters_per_pixel)
    
    inflated_grid = inflate_obstacles(grid, radius_pixels)
    start = real_to_grid(
        task_info['start'][0], task_info['start'][1],
        x_bounds, y_bounds, (W, H)
    )
    end = real_to_grid(
        task_info['end'][0], task_info['end'][1],
        x_bounds, y_bounds, (W, H)
    )
    
    if inflated_grid[start[0]][start[1]] == 1 or inflated_grid[end[0]][end[1]] == 1:
        print(f"❌ Start or end point is on obstacle!")
        print(f"   Start grid: {start}, value: {inflated_grid[start[0]][start[1]]}")
        print(f"   End grid: {end}, value: {inflated_grid[end[0]][end[1]]}")
        
        # 生成失败可视化
        debug_path = None
        return None, debug_path, 0.0
    
    path_grid = a_star(inflated_grid, start, end)
    if not path_grid:
        print("❌ No path found between start and end!")
        # 生成失败可视化
        debug_path = None
        return None, debug_path, 0.0
    
    print(f"✓ Path found with {len(path_grid)} nodes")
    real_path = []
    for point in path_grid:
        real_x, real_y = grid_to_real(
            point[0], point[1],
            x_bounds, y_bounds, (W, H)
        )
        real_path.append([real_x, real_y, 0.0])
    
    # Calculate total path distance
    total_distance = calculate_path_distance(real_path)
    
    return real_path, path_grid, total_distance
