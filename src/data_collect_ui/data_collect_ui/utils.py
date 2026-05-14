"""Utility functions and shared configurations for data collect UI."""
import math
import os
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory

try:
    from PySide6.QtGui import QImage, QPixmap
except ImportError:
    from PyQt5.QtGui import QImage, QPixmap

from cv_bridge import CvBridge
from sensor_msgs.msg import PointField


# ROS Topic and Service Constants
DATA_COLLECT_ACTIVATE = "/data_collect_activate"
DATA_COLLECT_DEACTIVATE = "/data_collect_deactivate"
DATA_COLLECT_SET_TASK = "/data_collect_set_task"
START_FIX_SCAN = "/start_fix_scan"
STOP_FIX_SCAN = "/stop_fix_scan"
IMAGE_TOPIC = "/image_topic"
POINT_CLOUD_TOPIC = "/tcp_cloud_raw"
RELOAD_CAMERA_3D_CONFIG = "/reload_camera_3d_config"

# File and Path Constants
DEFAULT_DATA_ROOT = "data"
DEFAULT_CAMERA_TCP_YAML = "config/cameratcp.yaml"
DEFAULT_FANUC_SO_FILE = "lib/libFanucRobot.so"
MAX_PREVIEW_POINTS = 500_000

# Settings and Camera Configuration Schemas
SETTINGS_SCHEMA = [
    {
        "title": "2D相机",
        "node": "camera_node",
        "fields": [
            ("trigger_mode", "触发模式", "int", 0, 10, 2),
            ("strobe_polarity", "频闪极性", "int", 0, 1, 0),
            ("saturation", "饱和度", "int", 0, 255, 64),
            ("gamma", "Gamma", "int", 0, 300, 106),
            ("exposure_time", "曝光时间", "double", 0.001, 100000.0, 4.3),
            ("analog_gain", "模拟增益", "int", 0, 255, 64),
            ("frame_rate", "发布频率", "double", 1.0, 240.0, 60.0),
        ],
    },
    {
        "title": "3D相机",
        "node": "camera_driver_3d",
        "fields": [
            ("cfg", "配置文件", "text", None, None, DEFAULT_CAMERA_TCP_YAML),
            ("publish_tf", "发布TF", "bool", None, None, True),
        ],
    },
    {
        "title": "Fanuc机器人",
        "node": "robot_driver_fanuc",
        "fields": [
            ("so_file_path", "动态库路径", "text", None, None, DEFAULT_FANUC_SO_FILE),
            ("robot_ip", "机器人IP", "text", None, None, "10.16.140.114"),
            ("robot_port", "机器人端口", "int", 1, 65535, 60008),
            ("target_register_index", "目标寄存器", "int", 0, 9999, 100),
        ],
    },
    {
        "title": "数据采集",
        "node": "data_collect_node",
        "fields": [
            ("save_dir_root", "保存根目录", "text", None, None, DEFAULT_DATA_ROOT),
            ("image_save_interval", "图像保存间隔", "int", 1, 100000, 12),
            ("image_log_save_interval", "图像日志间隔", "int", 1, 100000, 3),
            ("height_log_save_interval", "高度日志间隔", "int", 1, 100000, 4),
            ("fix_scan_interval", "点云保存间隔", "int", 1, 100000, 6),
            ("auto_save_flag", "自动采集", "int_bool", None, None, 0),
            ("target_register_index", "目标寄存器", "int", 0, 9999, 100),
        ],
    },
]

CAMERA_TCP_SCHEMA = [
    {
        "title": "3D相机位姿",
        "section": "camera",
        "fields": [
            ("x", "X", "double6", -10.0, 10.0, -0.0240785),
            ("y", "Y", "double6", -10.0, 10.0, 0.183801),
            ("z", "Z", "double6", -10.0, 10.0, 0.495747),
            ("rx", "RX", "double6", -6.283185, 6.283185, -3.00009),
            ("ry", "RY", "double6", -6.283185, 6.283185, 0.0952431),
            ("rz", "RZ", "double6", -6.283185, 6.283185, 2.80206),
        ],
    },
    {
        "title": "工具位姿",
        "section": "tool",
        "fields": [
            ("x", "X", "double6", -10.0, 10.0, 0.0),
            ("y", "Y", "double6", -10.0, 10.0, 0.0),
            ("z", "Z", "double6", -10.0, 10.0, 0.0),
            ("rx", "RX", "double6", -6.283185, 6.283185, 0.0),
            ("ry", "RY", "double6", -6.283185, 6.283185, 0.0),
            ("rz", "RZ", "double6", -6.283185, 6.283185, 0.0),
        ],
    },
    {
        "title": "裁剪范围",
        "section": None,
        "fields": [
            ("y_min", "Y最小", "double6", -10.0, 10.0, -0.015),
            ("y_max", "Y最大", "double6", -10.0, 10.0, 0.015),
            ("z_min", "Z最小", "double6", -10.0, 10.0, -0.035),
            ("z_max", "Z最大", "double6", -10.0, 10.0, 0.015),
            ("y_min_f", "Y精细最小", "double6", -10.0, 10.0, -0.002),
            ("y_max_f", "Y精细最大", "double6", -10.0, 10.0, 0.007),
        ],
    },
    {
        "title": "检测阈值",
        "section": None,
        "fields": [
            ("percentile_low", "低分位", "double3", 0.0, 1.0, 0.333),
            ("percentile_high", "高分位", "double3", 0.0, 1.0, 0.666),
            ("number_points_threshold", "点数阈值", "int", 0, 1000000, 25),
            ("no_detection_count_threshold", "无检测计数", "int", 0, 1000000, 3),
        ],
    },
]


def default_nodemanage_yaml():
    """Get the path to nodemanage.yaml configuration file."""
    env_path = os.environ.get("WELD_UI_NODEMANAGE_YAML")
    if env_path:
        return env_path

    candidates = []
    try:
        candidates.append(str(Path(get_package_share_directory("data_collect_bringup")) / "config" / "nodemanage.yaml"))
    except Exception:
        pass
    candidates.extend([
        str(Path("src/data_collect_bringup/config/nodemanage.yaml")),
        str(Path("src/config/nodemanage.yaml")),
        os.environ.get("AUTOCOVER_NODEMANAGE_YAML", ""),
        "/etc/weld_data_collect/nodemanage.yaml",
    ])
    for candidate in candidates:
        if candidate and Path(candidate).exists():
            return candidate
    return str(Path("src/data_collect_bringup/config/nodemanage.yaml"))


def qimage_format(name):
    """Get QImage format by name."""
    fmt = getattr(QImage, "Format", QImage)
    return getattr(fmt, name)


def qt_enum(owner, enum_group, enum_name):
    """Get Qt enum value by group and name."""
    group = getattr(owner, enum_group, None)
    if group is not None and hasattr(group, enum_name):
        return getattr(group, enum_name)
    return getattr(owner, enum_name)


def image_msg_to_qimage(msg):
    """Convert ROS Image message to QImage."""
    encoding = msg.encoding.lower()
    data = bytes(msg.data)

    if encoding in ("rgb8", "bgr8"):
        fmt_name = "Format_RGB888" if encoding == "rgb8" else "Format_BGR888"
        image = QImage(data, msg.width, msg.height, msg.step, qimage_format(fmt_name))
        return image.copy()

    if encoding in ("mono8", "8uc1"):
        image = QImage(data, msg.width, msg.height, msg.step, qimage_format("Format_Grayscale8"))
        return image.copy()

    return None


def _point_field_dtype(field):
    """Get numpy dtype and size for a PointField."""
    mapping = {
        PointField.INT8: ("i1", 1),
        PointField.UINT8: ("u1", 1),
        PointField.INT16: ("<i2", 2),
        PointField.UINT16: ("<u2", 2),
        PointField.INT32: ("<i4", 4),
        PointField.UINT32: ("<u4", 4),
        PointField.FLOAT32: ("<f4", 4),
        PointField.FLOAT64: ("<f8", 8),
    }
    return mapping.get(field.datatype)


def pointcloud2_msg_to_xyz(msg, max_points=MAX_PREVIEW_POINTS):
    """Convert ROS PointCloud2 message to numpy array of (x, y, z) coordinates."""
    if msg.is_bigendian:
        return None, "PointCloud2 big-endian 数据暂不支持"
    fields = {field.name: field for field in msg.fields}
    required = [fields.get(name) for name in ("x", "y", "z")]
    if any(field is None for field in required):
        return None, "PointCloud2 缺少 x/y/z 字段"
    if any(field.datatype != PointField.FLOAT32 for field in required):
        return None, "PointCloud2 的 x/y/z 字段必须是 float32"
    if msg.point_step <= 0:
        return None, "PointCloud2 point_step 无效"

    dtype_fields = []
    cursor = 0
    ordered_fields = sorted(msg.fields, key=lambda field: field.offset)
    for index, field in enumerate(ordered_fields):
        if field.offset > cursor:
            dtype_fields.append((f"_pad_{index}", f"u1", field.offset - cursor))
            cursor = field.offset
        field_dtype = _point_field_dtype(field)
        if field_dtype is None:
            continue
        dtype_name, size = field_dtype
        count = max(1, field.count)
        if count == 1:
            dtype_fields.append((field.name, dtype_name))
        else:
            dtype_fields.append((field.name, dtype_name, (count,)))
        cursor = field.offset + size * count
    if msg.point_step > cursor:
        dtype_fields.append(("_pad_end", "u1", msg.point_step - cursor))

    try:
        dtype = np.dtype(dtype_fields)
        dtype = np.dtype({"names": dtype.names, "formats": [dtype.fields[name][0] for name in dtype.names],
                          "offsets": [dtype.fields[name][1] for name in dtype.names],
                          "itemsize": msg.point_step})
        raw = memoryview(msg.data)
        point_count = len(raw) // msg.point_step
        if msg.width and msg.height:
            point_count = min(point_count, int(msg.width) * int(msg.height))
        if point_count <= 0:
            return np.empty((0, 3), dtype=np.float32), ""
        cloud = np.frombuffer(raw, dtype=dtype, count=point_count)
        xyz = np.empty((point_count, 3), dtype=np.float32)
        xyz[:, 0] = cloud["x"]
        xyz[:, 1] = cloud["y"]
        xyz[:, 2] = cloud["z"]
    except Exception as exc:
        return None, f"PointCloud2 解析失败：{exc}"

    valid = np.isfinite(xyz).all(axis=1)
    if not bool(np.all(valid)):
        xyz = xyz[valid]
    if xyz.shape[0] > max_points:
        stride = int(math.ceil(xyz.shape[0] / max_points))
        xyz = xyz[::stride][:max_points]
    return np.ascontiguousarray(xyz, dtype=np.float32), ""
