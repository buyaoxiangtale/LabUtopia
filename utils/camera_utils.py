import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def get_camera_intrinsic(camera) -> np.ndarray:
    """
    获取相机内参矩阵 (3x3)

    Args:
        camera: Isaac Sim Camera 实例

    Returns:
        np.ndarray: 相机内参矩阵
            [[fx, 0,  cx],
             [0,  fy, cy],
             [0,  0,  1 ]]
    """
    return camera.get_intrinsics_matrix()


def get_camera_extrinsic(camera, robot) -> np.ndarray:
    """
    获取相机外参矩阵 (4x4)

    外参矩阵表示相机相对于机器人基座的变换。
    计算公式: camera_extrinsic = robot_world^{-1} @ camera_world

    Args:
        camera: Isaac Sim Camera 实例
        robot: 机器人实例

    Returns:
        np.ndarray: 相机外参矩阵 (4x4)
    """
    # 获取相机在世界坐标系中的位姿
    cam_pos, cam_quat = camera.get_world_pose()

    # 获取机器人在世界坐标系中的位姿
    robot_pos, robot_quat = robot.get_world_pose()

    # 转换为变换矩阵
    cam_matrix = pose_to_matrix(cam_pos, cam_quat)
    robot_matrix = pose_to_matrix(robot_pos, robot_quat)

    # 相对外参 = 机器人世界位姿的逆 @ 相机世界位姿
    extrinsic = np.linalg.inv(robot_matrix) @ cam_matrix

    return extrinsic


def get_camera_trajectory_matrix(camera) -> np.ndarray:
    """
    获取相机在世界坐标系中的位姿矩阵 (4x4)

    Args:
        camera: Isaac Sim Camera 实例

    Returns:
        np.ndarray: 相机世界坐标位姿矩阵 (4x4)
    """
    pos, quat = camera.get_world_pose()
    return pose_to_matrix(pos, quat)


def pose_to_matrix(position, quaternion) -> np.ndarray:
    """
    将位置和四元数转换为4x4齐次变换矩阵

    Args:
        position: 位置 [x, y, z]
        quaternion: 四元数 [w, x, y, z] (Isaac Sim格式)

    Returns:
        np.ndarray: 4x4齐次变换矩阵
    """
    # Isaac Sim 使用 [w, x, y, z] 格式
    # 转换为旋转矩阵
    rotation = Rotation.from_quat([quaternion[1], quaternion[2], quaternion[3], quaternion[0]]).as_matrix()

    # 构建齐次矩阵
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = position

    return matrix


def string_to_color(category, num_colors=256):
    hash_value = hash(category) % num_colors
    color = [(hash_value * 37) % 256, (hash_value * 59) % 256, (hash_value * 73) % 256]
    return tuple(color)

def create_color_map(id_to_labels):
    color_map = {}
    for id_str, label in id_to_labels.items():
        color_map[id_str] = string_to_color(label)
    return color_map

def process_single_type(camera, image_type):
    if image_type == "rgb":
        rgb_img = camera.get_rgb()
        img_for_record = np.transpose(rgb_img, (2, 0, 1))
        return img_for_record, img_for_record
    elif image_type == "pointcloud":
        rgb_img = camera.get_rgb()
        img_for_display = np.transpose(rgb_img, (2, 0, 1))
        pointcloud = camera.get_pointcloud()
        if pointcloud is not None:
            pointcloud_for_record = pointcloud.astype(np.float32)
            return pointcloud_for_record, img_for_display
        else:
            print("Warning: Point cloud data not available.")
            return None, None
    elif image_type == "depth":
        # Isaac Sim depth 通常是以“米”为单位的 float（H, W）
        # 这里“记录数据”不做 min/max 归一化，直接存原始深度（更利于训练/后处理）
        # “显示数据”再做固定范围映射，避免单帧 max==min 导致全黑/全蓝
        depth_m = camera.get_depth()
        if depth_m is not None:
            depth_m = depth_m.astype(np.float32)
            depth_for_record = depth_m[:, :, np.newaxis]          # H, W, 1
            depth_for_record = np.transpose(depth_for_record, (2, 0, 1))  # 1, H, W

            # display: 将 [near, far] 映射到 0..255，再做伪彩色
            # 若相机裁剪范围可用，优先用它；否则用常见默认值
            near, far = 0.1, 10.0
            try:
                # 某些 Camera 实现支持 get_clipping_range()
                cr = camera.get_clipping_range()
                if cr is not None and len(cr) == 2:
                    near, far = float(cr[0]), float(cr[1])
            except Exception:
                pass

            # 处理无效值：<=0 或 非有限数 视为 far
            valid = np.isfinite(depth_m) & (depth_m > 0)
            depth_vis_m = depth_m.copy()
            depth_vis_m[~valid] = far

            depth_u8 = ((np.clip(depth_vis_m, near, far) - near) / max(far - near, 1e-6) * 255.0).astype(np.uint8)
            depth_for_display = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)
            return depth_for_record, depth_for_display
        else:
            print("Warning: Depth data not available.")
            return None, None
    elif image_type == "segmentation":
        frame_dict = camera.get_current_frame()
        instance_id_seg = frame_dict.get('instance_segmentation')
        if instance_id_seg is not None:
            seg = instance_id_seg['data']
            if seg is not None:
                seg_for_record = seg.astype(np.uint8)
                id_to_labels = instance_id_seg.get('info', {}).get('idToLabels', {})
                color_map = create_color_map(id_to_labels)
                seg_for_display = np.zeros((seg_for_record.shape[0], seg_for_record.shape[1], 3), dtype=np.uint8) # H * W * C
                for id_value in np.unique(seg_for_record):
                    if str(id_value) in color_map:
                        seg_for_display[seg_for_record == id_value] = color_map[str(id_value)]
                seg_for_record = seg_for_record[:, :, np.newaxis]
                seg_for_record = np.transpose(seg_for_record, (2, 0, 1)) # C * W * H
                seg_for_display = np.transpose(seg_for_display, (2, 0, 1)) # C * W * H
            return seg_for_record, seg_for_display
        else:
            print("Warning: Segmentation data not available.")
            return None, None
    elif image_type == "point":
        frame_dict = camera.get_current_frame()
        rgb_img = camera.get_rgb()
        img_for_display = rgb_img[..., ::-1].copy()  # Make a copy to modify
        instance_id_seg = frame_dict.get('instance_segmentation')
        if instance_id_seg is not None:
            seg = instance_id_seg['data']
            if seg is not None:
                seg_for_process = seg.astype(np.uint8)
                id_to_labels = instance_id_seg.get('info', {}).get('idToLabels', {})
                for id_value in np.unique(seg_for_process):
                    if str(id_value) in id_to_labels and id_value != 0 and id_value != 1:
                        mask = (seg_for_process == id_value)
                        y, x = np.where(mask)
                        if len(y) > 0 and len(x) > 0:
                            center_y, center_x = int(np.mean(y)), int(np.mean(x))
                            cv2.circle(img_for_display, (center_x, center_y), 6, (255, 100, 100), -1)
                img_for_display_rgb = img_for_display[..., ::-1]  # BGR to RGB
                img_for_record = np.transpose(img_for_display_rgb, (2, 0, 1))
                return img_for_record, img_for_display
            else:
                print("Warning: Segmentation data not available.")
                return None, None
        else:
            print("Warning: Instance segmentation not available.")
            return None, None
    else:
        return None, None

def process_camera_image(camera, image_type):
    """
    Process camera image with support for combined types (e.g., 'rgb+pointcloud')
    
    Args:
        camera: Camera instance
        image_type: String indicating the type(s) of image data to process
                   Can be single type or combined types with '+' (e.g., 'rgb+pointcloud')
    
    Returns:
        tuple: (record_data, display_data) where each can be single item or dict
    """
    record, display = process_single_type(camera, "rgb")
    if '+' in image_type:
        types = image_type.split('+')
        record_dict = {}
        for t in types:
            record, _ = process_single_type(camera, t)
            if record is not None:
                record_dict[t] = record
        return record_dict, display
    else:
        return process_single_type(camera, image_type)