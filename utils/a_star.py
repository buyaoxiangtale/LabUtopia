import heapq
from PIL import Image
import copy
import matplotlib.pyplot as plt
import numpy as np
from typing import Optional, Tuple, List


def astar(grid, start, end, max_iterations=100000):
    """A* pathfinding algorithm with Manhattan heuristic and diagonal movement.
    
    Args:
        grid: 2D grid where 0 is free space and 1 is obstacle
        start: Starting position (x, y)
        end: End position (x, y)
        max_iterations: Maximum number of nodes to explore to prevent infinite loops
    
    Returns:
        List of positions from start to end, or None if no path found
    """
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1),]
                #  (-1, -1), (-1, 1), (1, -1), (1, 1)]
    open_heap = []
    heapq.heappush(open_heap, (0, *start))
    came_from = {}
    g_scores = {start: 0}
    f_scores = {start: heuristic(start, end)}
    closed = set()  # 已扩展过的节点，避免重复扩展导致堆爆炸
    iterations = 0  # 迭代计数器，防止无限循环

    while open_heap:
        iterations += 1
        if iterations > max_iterations:
            print(f"A* search exceeded maximum iterations ({max_iterations}), aborting.")
            return None
            
        current_f, cx, cy = heapq.heappop(open_heap)
        
        # 跳过已经处理过的节点（避免处理堆中的重复项）
        if (cx, cy) in closed:
            continue
            
        if (cx, cy) == end:
            return reconstruct_path(came_from, end)
            
        closed.add((cx, cy))

        for dx, dy in directions:
            nx, ny = cx + dx, cy + dy
            
            # 边界检查和障碍物检查
            if (
                not (0 <= nx < len(grid) and 0 <= ny < len(grid[0]))
                or grid[nx][ny] != 0
            ):
                continue

            # 跳过已经处理过的节点，避免重复加入堆
            if (nx, ny) in closed:
                continue

            move_cost = 1.414 if dx != 0 and dy != 0 else 1
            tentative_g = g_scores[(cx, cy)] + move_cost
            
            # 只有当找到更好的路径时才更新并加入堆
            if tentative_g < g_scores.get((nx, ny), float("inf")):
                came_from[(nx, ny)] = (cx, cy)
                g_scores[(nx, ny)] = tentative_g
                f = tentative_g + heuristic((nx, ny), end)
                f_scores[(nx, ny)] = f
                heapq.heappush(open_heap, (f, nx, ny))

    return None


def heuristic(a, b):
    """Modified heuristic using diagonal distance."""
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    

    return max(dx, dy) + (1.414 - 1) * min(dx, dy)


def reconstruct_path(came_from, end):
    path = [end]
    while path[-1] in came_from:
        path.append(came_from[path[-1]])
    return path[::-1]


def load_grid(image_path):
    """Convert image to binary grid representation."""
    with Image.open(image_path) as img:
        W, H = img.size
        return (
            [
                [0 if is_white(img.getpixel((j, i))) else 1 for j in range(W)]
                for i in range(H)
            ],
            W,
            H,
        )


def is_white(pixel):
    """Determine if pixel represents white space."""
    if isinstance(pixel, int):
        return pixel == 255
    channels = pixel[:3]
    return channels == (255, 255, 255)


def inflate_obstacles(grid: np.ndarray, radius: int) -> np.ndarray:
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
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        total_distance += np.sqrt(dx**2 + dy**2)
    
    return total_distance


def save_path_image(grid_path, path, save_path=None):

    plt.figure(figsize=(10, 10))

    plt.imshow(grid_path, cmap='binary')
    
    if path:
        
        path_i, path_j = zip(*path)
        plt.plot(path_j, path_i, 'r-', linewidth=2, label='Path')
        plt.plot(path_j[0], path_i[0], 'go', markersize=10, label='Start')
        plt.plot(path_j[-1], path_i[-1], 'bo', markersize=10, label='End')
    
    plt.legend()
    plt.grid(True)
    
    try:
        if save_path:
            plt.savefig(save_path)
        else:
            plt.show()
    finally:
        plt.close()  # 避免 figure 常驻内存导致泄漏


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
        return None

    path_grid = astar(inflated_grid, start, end)
    if not path_grid:
        print("No path found.")
        return None

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
