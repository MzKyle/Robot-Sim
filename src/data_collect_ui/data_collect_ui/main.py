import json
import os
import queue
import sys
import time
import zipfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

try:
    from PySide6.QtCore import QThread, Qt, QTimer, Signal, QUrl, QSize
    from PySide6.QtGui import QDesktopServices, QImage, QPixmap
    from PySide6.QtWidgets import (
        QAbstractItemView,
        QApplication,
        QCheckBox,
        QDoubleSpinBox,
        QFileDialog,
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QHeaderView,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QScrollArea,
        QSizePolicy,
        QSpinBox,
        QStyle,
        QTableWidget,
        QTableWidgetItem,
        QTabWidget,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
except ImportError:
    try:
        from PyQt5.QtCore import QThread, Qt, QTimer, QUrl, QSize, pyqtSignal as Signal
        from PyQt5.QtGui import QDesktopServices, QImage, QPixmap
        from PyQt5.QtWidgets import (
            QAbstractItemView,
            QApplication,
            QCheckBox,
            QDoubleSpinBox,
            QFileDialog,
            QFormLayout,
            QGridLayout,
            QGroupBox,
            QHBoxLayout,
            QHeaderView,
            QLabel,
            QLineEdit,
            QMainWindow,
            QMessageBox,
            QPushButton,
            QScrollArea,
            QSizePolicy,
            QSpinBox,
            QStyle,
            QTableWidget,
            QTableWidgetItem,
            QTabWidget,
            QTextEdit,
            QVBoxLayout,
            QWidget,
        )
    except ImportError as exc:
        print("缺少 Qt Python 绑定，无法启动桌面界面。", file=sys.stderr)
        print("安装示例：sudo apt install python3-pyqt5", file=sys.stderr)
        print("或安装 PySide6：python3 -m pip install --user PySide6", file=sys.stderr)
        print(f"原始错误：{exc}", file=sys.stderr)
        raise SystemExit(2)

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from std_srvs.srv import Trigger
from weld_interface.msg import DataCollectStatus, FanucRobotInfo
from weld_interface.srv import SetCollectionTask


DATA_COLLECT_ACTIVATE = "/data_collect_activate"
DATA_COLLECT_DEACTIVATE = "/data_collect_deactivate"
DATA_COLLECT_SET_TASK = "/data_collect_set_task"
START_FIX_SCAN = "/start_fix_scan"
STOP_FIX_SCAN = "/stop_fix_scan"
IMAGE_TOPIC = "/image_topic"
DEFAULT_DATA_ROOT = "data"
DEFAULT_CAMERA_TCP_YAML = "config/cameratcp.yaml"
DEFAULT_FANUC_SO_FILE = "lib/libFanucRobot.so"
RELOAD_CAMERA_3D_CONFIG = "/reload_camera_3d_config"


def default_nodemanage_yaml():
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


def qimage_format(name):
    fmt = getattr(QImage, "Format", QImage)
    return getattr(fmt, name)


def qt_enum(owner, enum_group, enum_name):
    group = getattr(owner, enum_group, None)
    if group is not None and hasattr(group, enum_name):
        return getattr(group, enum_name)
    return getattr(owner, enum_name)


def image_msg_to_qimage(msg):
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


class RosBridge(QThread):
    status_received = Signal(dict)
    fanuc_received = Signal(dict)
    image_received = Signal(object)
    services_received = Signal(dict)
    log_received = Signal(str)
    ros_state_changed = Signal(bool)

    def __init__(self):
        super().__init__()
        self._commands = queue.Queue()
        self._running = True
        self._node = None
        self._clients = {}
        self._last_service_emit = 0.0
        self._last_image_emit = 0.0

    def stop(self):
        self._running = False
        self._commands.put(("stop", None))

    def call_service(self, service_name):
        self._commands.put(("call_empty_service", service_name))

    def set_task(self, task):
        self._commands.put(("set_task", task))

    def set_parameters(self, node_name, parameters):
        self._commands.put(("set_parameters", {
            "node_name": node_name,
            "parameters": parameters,
        }))

    def run(self):
        rclpy.init(args=None)
        self._node = Node("data_collect_ui")
        self._clients = {
            DATA_COLLECT_ACTIVATE: self._node.create_client(Empty, DATA_COLLECT_ACTIVATE),
            DATA_COLLECT_DEACTIVATE: self._node.create_client(Empty, DATA_COLLECT_DEACTIVATE),
            START_FIX_SCAN: self._node.create_client(Empty, START_FIX_SCAN),
            STOP_FIX_SCAN: self._node.create_client(Empty, STOP_FIX_SCAN),
            DATA_COLLECT_SET_TASK: self._node.create_client(SetCollectionTask, DATA_COLLECT_SET_TASK),
            RELOAD_CAMERA_3D_CONFIG: self._node.create_client(Trigger, RELOAD_CAMERA_3D_CONFIG),
            "/camera_node/set_parameters": self._node.create_client(SetParameters, "/camera_node/set_parameters"),
            "/camera_driver_3d/set_parameters": self._node.create_client(SetParameters, "/camera_driver_3d/set_parameters"),
            "/data_collect_node/set_parameters": self._node.create_client(SetParameters, "/data_collect_node/set_parameters"),
            "/robot_driver_fanuc/set_parameters": self._node.create_client(SetParameters, "/robot_driver_fanuc/set_parameters"),
        }
        self._node.create_subscription(
            DataCollectStatus,
            "/data_collect_status",
            self._on_status,
            10,
        )
        self._node.create_subscription(
            FanucRobotInfo,
            "/fanuc_robot_info",
            self._on_fanuc,
            10,
        )
        self._node.create_subscription(
            Image,
            IMAGE_TOPIC,
            self._on_image,
            1,
        )
        self.ros_state_changed.emit(True)
        self.log_received.emit("ROS 通信线程已启动")

        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.1)
                self._process_commands()
                self._emit_service_state_periodically()
        finally:
            if self._node is not None:
                self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self.ros_state_changed.emit(False)

    def _process_commands(self):
        while True:
            try:
                command, payload = self._commands.get_nowait()
            except queue.Empty:
                break

            if command == "stop":
                return
            if command == "call_empty_service":
                self._call_empty_service(payload)
            if command == "set_task":
                self._call_set_task(payload)
            if command == "set_parameters":
                self._call_set_parameters(payload)

    def _call_empty_service(self, service_name):
        client = self._clients.get(service_name)
        if client is None:
            self.log_received.emit(f"未知服务：{service_name}")
            return

        if not client.service_is_ready():
            self.log_received.emit(f"服务不可用：{service_name}")
            return

        future = client.call_async(Empty.Request())
        future.add_done_callback(lambda done: self._on_service_done(service_name, done))
        self.log_received.emit(f"已发送服务请求：{service_name}")

    def _call_set_parameters(self, payload):
        node_name = payload["node_name"]
        service_name = f"/{node_name}/set_parameters"
        client = self._clients.get(service_name)
        if client is None or not client.service_is_ready():
            self.log_received.emit(f"参数服务不可用：{service_name}")
            return

        request = SetParameters.Request()
        request.parameters = [
            self._make_parameter(name, value)
            for name, value in payload["parameters"].items()
        ]
        future = client.call_async(request)
        future.add_done_callback(lambda done: self._on_set_parameters_done(node_name, done))
        names = ", ".join(payload["parameters"].keys())
        self.log_received.emit(f"已发送实时参数：{node_name} ({names})")

    def _make_parameter(self, name, value):
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue()
        if isinstance(value, bool):
            parameter.value.type = ParameterType.PARAMETER_BOOL
            parameter.value.bool_value = value
        elif isinstance(value, int):
            parameter.value.type = ParameterType.PARAMETER_INTEGER
            parameter.value.integer_value = value
        elif isinstance(value, float):
            parameter.value.type = ParameterType.PARAMETER_DOUBLE
            parameter.value.double_value = value
        else:
            parameter.value.type = ParameterType.PARAMETER_STRING
            parameter.value.string_value = str(value)
        return parameter

    def _on_set_parameters_done(self, node_name, future):
        try:
            response = future.result()
        except Exception as exc:
            self.log_received.emit(f"实时参数应用失败：{node_name}，{exc}")
            return
        failed = [result.reason for result in response.results if not result.successful]
        if failed:
            self.log_received.emit(f"实时参数被拒绝：{node_name}，{'; '.join(failed)}")
        else:
            self.log_received.emit(f"实时参数已生效：{node_name}")

    def _call_set_task(self, task):
        client = self._clients.get(DATA_COLLECT_SET_TASK)
        if client is None or not client.service_is_ready():
            self.log_received.emit(f"服务不可用：{DATA_COLLECT_SET_TASK}")
            return

        request = SetCollectionTask.Request()
        request.task_id = task["task_id"]
        request.workpiece_id = task["workpiece_id"]
        request.weld_seam_id = task["weld_seam_id"]
        request.operator_name = task["operator_name"]
        request.shift = task["shift"]
        request.notes = task["notes"]
        future = client.call_async(request)
        future.add_done_callback(lambda done: self._on_service_done(DATA_COLLECT_SET_TASK, done))
        self.log_received.emit("已发送采集任务信息")

    def _on_service_done(self, service_name, future):
        try:
            response = future.result()
        except Exception as exc:
            self.log_received.emit(f"服务调用失败：{service_name}，{exc}")
            return
        message = getattr(response, "message", "")
        if message:
            self.log_received.emit(f"服务调用完成：{service_name}，{message}")
        else:
            self.log_received.emit(f"服务调用完成：{service_name}")

    def _emit_service_state_periodically(self):
        now = time.monotonic()
        if now - self._last_service_emit < 1.0:
            return
        self._last_service_emit = now
        self.services_received.emit({
            name: client.service_is_ready()
            for name, client in self._clients.items()
        })

    def _on_status(self, msg):
        self.status_received.emit({
            "running": msg.running,
            "auto_save": msg.auto_save,
            "current_save_dir": msg.current_save_dir,
            "target_register_index": msg.target_register_index,
            "target_register_value": msg.target_register_value,
            "has_target_register_value": msg.has_target_register_value,
            "image_count": msg.image_count,
            "image_log_count": msg.image_log_count,
            "height_log_count": msg.height_log_count,
            "point_cloud_count": msg.point_cloud_count,
            "tool_pose_count": msg.tool_pose_count,
            "estimated_line_count": msg.estimated_line_count,
            "fanuc_info_count": msg.fanuc_info_count,
            "task_id": msg.task_id,
            "workpiece_id": msg.workpiece_id,
            "weld_seam_id": msg.weld_seam_id,
            "operator_name": msg.operator_name,
            "shift": msg.shift,
            "notes": msg.notes,
            "last_error": msg.last_error,
        })

    def _on_fanuc(self, msg):
        self.fanuc_received.emit({
            "main_pgm": msg.main_pgm,
            "cur_pgm": msg.cur_pgm,
            "cur_seq": msg.cur_seq,
            "ncstatus": msg.ncstatus,
            "mode": msg.mode,
            "voltage1": msg.voltage1,
            "current1": msg.current1,
            "wire_speed1": msg.wire_speed1,
            "weld_detect1": msg.weld_detect1,
            "voltage2": msg.voltage2,
            "current2": msg.current2,
            "wire_speed2": msg.wire_speed2,
            "weld_detect2": msg.weld_detect2,
            "alarm": msg.alarm,
            "alarm_msg": msg.alarm_msg,
            "emg": msg.emg,
            "override": msg.override,
            "weld_enable": msg.weld_enable,
        })

    def _on_image(self, msg):
        now = time.monotonic()
        if now - self._last_image_emit < 0.15:
            return
        self._last_image_emit = now

        image = image_msg_to_qimage(msg)
        if image is not None:
            self.image_received.emit(image)


class StatusPill(QLabel):
    def __init__(self, text="未知"):
        super().__init__(text)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumWidth(76)
        self.setMinimumHeight(26)
        self.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
        self.setProperty("state", "unknown")

    def set_state(self, text, state):
        self.setText(text)
        self.setProperty("state", state)
        self.style().unpolish(self)
        self.style().polish(self)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("焊接数据采集操作台")
        self.resize(1280, 820)
        self._current_save_dir = ""
        self._last_status_time = 0.0
        self._last_fanuc_time = 0.0
        self._last_image_time = 0.0
        self._last_image = None
        self._labels = {}
        self._service_buttons = {}
        self._history_records = []
        self._task_inputs = {}
        self._setting_widgets = {}
        self._camera_tcp_widgets = {}
        self._settings_data = {}
        self._camera_tcp_data = {}
        self._loading_settings = False
        self._responsive_mode = None
        self._status_items = []
        self._operation_buttons = []
        self._preview_metrics = []
        self._settings_panels = []
        self._camera_tcp_panels = []
        self._history_buttons = []
        self._settings_buttons = []
        self._collection_layout = None
        self._collection_left = None
        self._fanuc_panel = None
        self._operation_buttons_layout = None
        self._preview_metrics_layout = None
        self._settings_body_layout = None
        self._camera_tcp_body_layout = None
        self._history_buttons_layout = None
        self._settings_buttons_layout = None
        self._status_grid = None
        self._history_toolbar = None
        self._settings_toolbar = None
        self._tab_scrolls = []
        self._settings_save_timer = QTimer(self)
        self._settings_save_timer.setSingleShot(True)
        self._settings_save_timer.timeout.connect(self._apply_pending_settings)
        self._ros = RosBridge()
        self._build_ui()
        self._connect_ros()
        self._load_settings()
        self._ros.start()

        self._heartbeat_timer = QTimer(self)
        self._heartbeat_timer.timeout.connect(self._refresh_heartbeat)
        self._heartbeat_timer.start(1000)

    def closeEvent(self, event):
        self._ros.stop()
        self._ros.wait(2000)
        super().closeEvent(event)

    def _build_ui(self):
        self.setMinimumSize(760, 560)
        root = QWidget()
        root.setObjectName("appRoot")
        layout = QVBoxLayout(root)
        layout.setContentsMargins(18, 18, 18, 16)
        layout.setSpacing(14)

        layout.addWidget(self._build_top_bar())

        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.setElideMode(Qt.ElideRight)
        self.tabs.addTab(self._build_collection_tab(), "采集操作")
        self.tabs.addTab(self._build_preview_tab(), "实时预览")
        self.tabs.addTab(self._build_history_tab(), "历史数据")
        self.tabs.addTab(self._build_settings_tab(), "参数设置")
        layout.addWidget(self.tabs, 1)

        layout.addWidget(self._build_log_panel())
        self.setCentralWidget(root)
        self._apply_style()
        self._apply_responsive_layouts(force=True)

    def _build_top_bar(self):
        bar = QWidget()
        bar.setObjectName("topBar")
        layout = QVBoxLayout(bar)
        layout.setContentsMargins(18, 14, 18, 14)
        layout.setSpacing(12)

        title = QLabel("焊接数据采集操作台")
        title.setObjectName("title")
        subtitle = QLabel("采集状态、任务信息、实时预览与历史数据")
        subtitle.setObjectName("subtitle")

        title_box = QVBoxLayout()
        title_box.setSpacing(3)
        title_box.addWidget(title)
        title_box.addWidget(subtitle)

        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        header.addLayout(title_box, 1)
        layout.addLayout(header)

        self.ros_pill = StatusPill()
        self.collect_pill = StatusPill()
        self.fanuc_pill = StatusPill()
        self.image_pill = StatusPill()

        status_strip = QWidget()
        status_strip.setObjectName("statusStrip")
        self._status_grid = QGridLayout(status_strip)
        self._status_grid.setContentsMargins(0, 0, 0, 0)
        self._status_grid.setHorizontalSpacing(10)
        self._status_grid.setVerticalSpacing(8)
        for caption, pill, accent in (
            ("ROS", self.ros_pill, "blue"),
            ("采集", self.collect_pill, "green"),
            ("Fanuc", self.fanuc_pill, "orange"),
            ("图像", self.image_pill, "purple"),
        ):
            item = self._make_status_item(caption, pill, accent)
            self._status_items.append(item)
        layout.addWidget(status_strip)
        return bar

    def _make_status_item(self, text, pill, accent):
        item = QWidget()
        item.setObjectName("statusItem")
        item.setProperty("accent", accent)
        layout = QHBoxLayout(item)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(8)
        layout.addWidget(self._make_status_caption(text))
        layout.addWidget(pill, 1)
        return item

    def _make_status_caption(self, text):
        label = QLabel(text)
        label.setObjectName("statusCaption")
        return label

    def _build_collection_tab(self):
        scroll, page = self._make_tab_scroll("collectionScroll")
        page.setObjectName("tabPage")
        layout = QGridLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)
        self._collection_layout = layout

        left = QWidget()
        self._collection_left = left
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(14)
        left_layout.addWidget(self._build_collection_panel())
        left_layout.addWidget(self._build_task_panel())
        left_layout.addWidget(self._build_collection_status_panel(), 1)

        self._fanuc_panel = self._build_fanuc_panel()
        return scroll

    def _make_tab_scroll(self, object_name):
        scroll = QScrollArea()
        scroll.setObjectName(object_name)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(qt_enum(Qt, "ScrollBarPolicy", "ScrollBarAlwaysOff"))
        scroll.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        scroll.setMinimumSize(0, 0)
        body = QWidget()
        body.setMinimumWidth(0)
        body.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        scroll.setWidget(body)
        self._tab_scrolls.append(scroll)
        return scroll, body

    def _build_collection_panel(self):
        panel = QGroupBox("操作控制")
        panel.setProperty("accent", "blue")
        panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        layout = QVBoxLayout(panel)
        layout.setSpacing(12)

        buttons = QGridLayout()
        buttons.setHorizontalSpacing(10)
        buttons.setVerticalSpacing(10)
        self._operation_buttons_layout = buttons
        self.start_collect_btn = self._make_service_button(
            "启动采集",
            DATA_COLLECT_ACTIVATE,
            role="primary",
            icon_name="SP_MediaPlay",
        )
        self.stop_collect_btn = self._make_service_button(
            "停止采集",
            DATA_COLLECT_DEACTIVATE,
            role="danger",
            icon_name="SP_MediaStop",
        )
        self.start_scan_btn = self._make_service_button(
            "开始3D扫描",
            START_FIX_SCAN,
            role="success",
            icon_name="SP_ComputerIcon",
        )
        self.stop_scan_btn = self._make_service_button(
            "停止3D扫描",
            STOP_FIX_SCAN,
            role="warning",
            icon_name="SP_MediaStop",
        )
        self.open_dir_btn = QPushButton("打开保存目录")
        self._decorate_button(self.open_dir_btn, "secondary", "SP_DirOpenIcon")
        self.open_dir_btn.clicked.connect(self._open_current_dir)
        self._operation_buttons = [
            self.start_collect_btn,
            self.stop_collect_btn,
            self.start_scan_btn,
            self.stop_scan_btn,
            self.open_dir_btn,
        ]
        layout.addLayout(buttons)
        layout.addStretch(1)
        return panel

    def _build_collection_status_panel(self):
        panel = QGroupBox("采集概览")
        panel.setProperty("accent", "green")
        grid = QGridLayout()
        grid.setColumnStretch(1, 1)
        grid.setHorizontalSpacing(18)
        grid.setVerticalSpacing(9)
        rows = [
            ("保存目录", "current_save_dir"),
            ("自动采集", "auto_save"),
            ("目标寄存器", "target_register"),
            ("当前任务", "current_task"),
            ("图像数量", "image_count"),
            ("点云数量", "point_cloud_count"),
            ("工具位姿", "tool_pose_count"),
            ("Fanuc记录", "fanuc_info_count"),
            ("图像日志", "image_log_count"),
            ("高度日志", "height_log_count"),
            ("直线日志", "estimated_line_count"),
            ("最近错误", "last_error"),
        ]
        for row, (name, key) in enumerate(rows):
            self._add_value_row(grid, row, name, key)

        QVBoxLayout(panel).addLayout(grid)
        return panel

    def _build_task_panel(self):
        panel = QGroupBox("采集任务")
        panel.setProperty("accent", "purple")
        panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        form = QFormLayout(panel)
        form.setFieldGrowthPolicy(qt_enum(QFormLayout, "FieldGrowthPolicy", "AllNonFixedFieldsGrow"))
        form.setHorizontalSpacing(14)
        form.setVerticalSpacing(10)
        fields = [
            ("任务号", "task_id"),
            ("工件号", "workpiece_id"),
            ("焊道号", "weld_seam_id"),
            ("操作员", "operator_name"),
            ("班次", "shift"),
        ]
        for label, key in fields:
            edit = QLineEdit()
            edit.setPlaceholderText(label)
            form.addRow(label, edit)
            self._task_inputs[key] = edit

        notes = QTextEdit()
        notes.setPlaceholderText("备注")
        notes.setFixedHeight(68)
        form.addRow("备注", notes)
        self._task_inputs["notes"] = notes

        self.save_task_btn = QPushButton("保存任务信息")
        self._decorate_button(self.save_task_btn, "primary", "SP_DialogSaveButton")
        self.save_task_btn.setEnabled(False)
        self.save_task_btn.clicked.connect(self._save_task)
        form.addRow("", self.save_task_btn)
        return panel

    def _build_fanuc_panel(self):
        panel = QGroupBox("Fanuc 状态")
        panel.setProperty("accent", "orange")
        layout = QGridLayout(panel)
        layout.setColumnStretch(1, 1)
        layout.setHorizontalSpacing(14)
        layout.setVerticalSpacing(9)
        rows = [
            ("主程序", "main_pgm"),
            ("当前程序", "cur_pgm"),
            ("当前步号", "cur_seq"),
            ("NC状态", "ncstatus"),
            ("模式", "mode"),
            ("电压1", "voltage1"),
            ("电流1", "current1"),
            ("送丝1", "wire_speed1"),
            ("检测1", "weld_detect1"),
            ("电压2", "voltage2"),
            ("电流2", "current2"),
            ("送丝2", "wire_speed2"),
            ("检测2", "weld_detect2"),
            ("报警", "alarm"),
            ("急停", "emg"),
            ("倍率", "override"),
            ("焊接使能", "weld_enable"),
            ("报警信息", "alarm_msg"),
        ]
        for row, (name, key) in enumerate(rows):
            self._add_value_row(layout, row, name, f"fanuc_{key}")
        return panel

    def _add_value_row(self, grid, row, name, key):
        label = QLabel(name)
        label.setObjectName("fieldLabel")
        value = QLabel("-")
        value.setObjectName("fieldValue")
        value.setMinimumHeight(30)
        value.setWordWrap(True)
        value.setTextInteractionFlags(Qt.TextSelectableByMouse)
        grid.addWidget(label, row, 0)
        grid.addWidget(value, row, 1)
        self._labels[key] = value

    def _build_preview_tab(self):
        scroll, page = self._make_tab_scroll("previewScroll")
        page.setObjectName("tabPage")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        self.image_label = QLabel("等待图像...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumHeight(220)
        self.image_label.setObjectName("imagePreview")
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.image_label, 1)

        metrics = QGroupBox("预览指标")
        metrics.setProperty("accent", "teal")
        self._preview_metrics_layout = QGridLayout(metrics)
        self._preview_metrics_layout.setHorizontalSpacing(12)
        self._preview_metrics_layout.setVerticalSpacing(8)
        for name, key in [
            ("图像保存", "preview_image_count"),
            ("图像日志", "preview_image_log_count"),
            ("点云保存", "preview_point_cloud_count"),
            ("当前目录", "preview_current_dir"),
        ]:
            label = QLabel(name)
            label.setObjectName("fieldLabel")
            value = QLabel("-")
            value.setObjectName("fieldValue")
            value.setMinimumHeight(30)
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            value.setWordWrap(True)
            self._preview_metrics.append((label, value))
            self._labels[key] = value
        layout.addWidget(metrics)
        return scroll

    def _build_history_tab(self):
        scroll, page = self._make_tab_scroll("historyScroll")
        page.setObjectName("tabPage")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        toolbar = QWidget()
        self._history_toolbar = toolbar
        toolbar.setObjectName("toolbarPanel")
        toolbar_layout = QVBoxLayout(toolbar)
        toolbar_layout.setContentsMargins(12, 12, 12, 12)
        toolbar_layout.setSpacing(10)
        path_row = QHBoxLayout()
        path_row.setSpacing(10)
        self._history_buttons_layout = QGridLayout()
        self._history_buttons_layout.setHorizontalSpacing(10)
        self._history_buttons_layout.setVerticalSpacing(8)

        self.data_root_edit = QLineEdit(os.environ.get("WELD_DATA_ROOT", DEFAULT_DATA_ROOT))
        self.data_root_edit.setMinimumWidth(0)
        self.data_root_edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.data_root_edit.setPlaceholderText("数据根目录")
        choose_btn = QPushButton("选择目录")
        self._decorate_button(choose_btn, "secondary", "SP_DirOpenIcon")
        choose_btn.clicked.connect(self._choose_data_root)
        refresh_btn = QPushButton("刷新")
        self._decorate_button(refresh_btn, "secondary", "SP_BrowserReload")
        refresh_btn.clicked.connect(self._refresh_history)
        open_btn = QPushButton("打开目录")
        self._decorate_button(open_btn, "secondary", "SP_DirOpenIcon")
        open_btn.clicked.connect(self._open_selected_history_dir)
        export_btn = QPushButton("导出ZIP")
        self._decorate_button(export_btn, "primary", "SP_DialogSaveButton")
        export_btn.clicked.connect(self._export_selected_history_zip)
        self._history_buttons = [choose_btn, refresh_btn, open_btn, export_btn]
        for col, button in enumerate(self._history_buttons):
            self._history_buttons_layout.addWidget(button, 0, col)

        root_label = QLabel("数据根目录")
        root_label.setObjectName("fieldLabel")
        path_row.addWidget(root_label)
        path_row.addWidget(self.data_root_edit, 1)
        toolbar_layout.addLayout(path_row)
        toolbar_layout.addLayout(self._history_buttons_layout)
        layout.addWidget(toolbar)

        self.history_table = QTableWidget(0, 10)
        self.history_table.setHorizontalHeaderLabels([
            "状态",
            "开始时间",
            "结束时间",
            "任务号",
            "工件号",
            "焊道号",
            "寄存器",
            "图像",
            "点云",
            "路径",
        ])
        self.history_table.setSelectionBehavior(qt_enum(QAbstractItemView, "SelectionBehavior", "SelectRows"))
        self.history_table.setSelectionMode(qt_enum(QAbstractItemView, "SelectionMode", "SingleSelection"))
        self.history_table.setEditTriggers(qt_enum(QAbstractItemView, "EditTrigger", "NoEditTriggers"))
        self.history_table.setAlternatingRowColors(True)
        self.history_table.setShowGrid(False)
        self.history_table.verticalHeader().setVisible(False)
        self.history_table.verticalHeader().setDefaultSectionSize(34)
        self.history_table.horizontalHeader().setSectionResizeMode(qt_enum(QHeaderView, "ResizeMode", "ResizeToContents"))
        self.history_table.horizontalHeader().setSectionResizeMode(9, qt_enum(QHeaderView, "ResizeMode", "Stretch"))
        layout.addWidget(self.history_table, 1)

        self.history_summary = QLabel("尚未扫描历史数据")
        self.history_summary.setObjectName("summaryLabel")
        layout.addWidget(self.history_summary)
        return scroll

    def _build_settings_tab(self):
        page_scroll, page = self._make_tab_scroll("settingsPageScroll")
        page.setObjectName("tabPage")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        toolbar = QWidget()
        self._settings_toolbar = toolbar
        toolbar.setObjectName("toolbarPanel")
        toolbar_layout = QVBoxLayout(toolbar)
        toolbar_layout.setContentsMargins(12, 12, 12, 12)
        toolbar_layout.setSpacing(10)
        path_row = QHBoxLayout()
        path_row.setSpacing(10)
        self._settings_buttons_layout = QGridLayout()
        self._settings_buttons_layout.setHorizontalSpacing(10)
        self._settings_buttons_layout.setVerticalSpacing(8)

        self.settings_path_edit = QLineEdit(default_nodemanage_yaml())
        self.settings_path_edit.setMinimumWidth(0)
        self.settings_path_edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.settings_path_edit.editingFinished.connect(self._load_settings)
        choose_btn = QPushButton("选择配置")
        self._decorate_button(choose_btn, "secondary", "SP_DirOpenIcon")
        choose_btn.clicked.connect(self._choose_settings_file)
        reload_btn = QPushButton("重新加载")
        self._decorate_button(reload_btn, "secondary", "SP_BrowserReload")
        reload_btn.clicked.connect(self._load_settings)
        save_btn = QPushButton("保存配置")
        self._decorate_button(save_btn, "primary", "SP_DialogSaveButton")
        save_btn.clicked.connect(self._save_settings)
        self._settings_buttons = [choose_btn, reload_btn, save_btn]
        for col, button in enumerate(self._settings_buttons):
            self._settings_buttons_layout.addWidget(button, 0, col)

        settings_label = QLabel("配置文件")
        settings_label.setObjectName("fieldLabel")
        path_row.addWidget(settings_label)
        path_row.addWidget(self.settings_path_edit, 1)
        toolbar_layout.addLayout(path_row)
        toolbar_layout.addLayout(self._settings_buttons_layout)
        layout.addWidget(toolbar)

        body = QWidget()
        body.setObjectName("settingsBody")
        self._settings_body_layout = QGridLayout(body)
        self._settings_body_layout.setHorizontalSpacing(14)
        self._settings_body_layout.setVerticalSpacing(14)

        for index, group in enumerate(SETTINGS_SCHEMA):
            panel = QGroupBox(group["title"])
            panel.setProperty("accent", ("blue", "teal", "orange", "green")[index % 4])
            form = QFormLayout(panel)
            form.setFieldGrowthPolicy(qt_enum(QFormLayout, "FieldGrowthPolicy", "AllNonFixedFieldsGrow"))
            for key, label, field_type, minimum, maximum, default in group["fields"]:
                widget = self._make_setting_widget(field_type, minimum, maximum)
                widget.setProperty("default_value", default)
                widget.setProperty("field_type", field_type)
                widget.setProperty("config_kind", "nodemanage")
                widget.setProperty("node_name", group["node"])
                widget.setProperty("param_key", key)
                self._setting_widgets[(group["node"], key)] = widget
                form.addRow(label, widget)
            self._settings_panels.append(panel)

        for index, group in enumerate(CAMERA_TCP_SCHEMA):
            panel = QGroupBox(group["title"])
            panel.setProperty("accent", ("teal", "purple", "green", "orange")[index % 4])
            form = QFormLayout(panel)
            form.setFieldGrowthPolicy(qt_enum(QFormLayout, "FieldGrowthPolicy", "AllNonFixedFieldsGrow"))
            for key, label, field_type, minimum, maximum, default in group["fields"]:
                widget = self._make_setting_widget(field_type, minimum, maximum)
                widget.setProperty("default_value", default)
                widget.setProperty("field_type", field_type)
                widget.setProperty("config_kind", "camera_tcp")
                widget.setProperty("section", group["section"] or "")
                widget.setProperty("param_key", key)
                self._camera_tcp_widgets[(group["section"], key)] = widget
                form.addRow(label, widget)
            self._camera_tcp_panels.append(panel)

        layout.addWidget(body, 1)

        self.settings_status_label = QLabel("参数配置未加载")
        self.settings_status_label.setObjectName("summaryLabel")
        layout.addWidget(self.settings_status_label)
        return page_scroll

    def _make_setting_widget(self, field_type, minimum, maximum):
        if field_type == "int":
            widget = QSpinBox()
            widget.setMinimumWidth(0)
            widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            widget.setRange(int(minimum), int(maximum))
            widget.valueChanged.connect(self._on_setting_widget_changed)
            return widget

        if field_type in ("double", "double3", "double6"):
            widget = QDoubleSpinBox()
            widget.setMinimumWidth(0)
            widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            widget.setRange(float(minimum), float(maximum))
            widget.setDecimals(6 if field_type == "double6" else 3)
            widget.setSingleStep(0.001 if field_type in ("double3", "double6") else 0.1)
            widget.valueChanged.connect(self._on_setting_widget_changed)
            return widget

        if field_type in ("bool", "int_bool"):
            widget = QCheckBox()
            widget.toggled.connect(self._on_setting_widget_changed)
            return widget

        widget = QLineEdit()
        widget.setMinimumWidth(0)
        widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        widget.editingFinished.connect(self._on_setting_widget_changed)
        return widget

    def _build_log_panel(self):
        panel = QGroupBox("运行日志")
        panel.setProperty("accent", "blue")
        layout = QVBoxLayout(panel)
        self.log_label = QLabel("等待 ROS 状态...")
        self.log_label.setObjectName("logText")
        self.log_label.setWordWrap(True)
        self.log_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.log_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.log_label.setMinimumHeight(64)
        layout.addWidget(self.log_label)
        return panel

    def _make_service_button(self, text, service_name, role="secondary", icon_name=None):
        button = QPushButton(text)
        self._decorate_button(button, role, icon_name)
        button.setEnabled(False)
        button.clicked.connect(lambda: self._ros.call_service(service_name))
        self._service_buttons[service_name] = button
        return button

    def _decorate_button(self, button, role="secondary", icon_name=None):
        button.setProperty("role", role)
        button.setMinimumHeight(34)
        button.setMinimumWidth(104)
        button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        if icon_name:
            self._set_standard_icon(button, icon_name)

    def _set_standard_icon(self, button, icon_name):
        try:
            icon = self.style().standardIcon(qt_enum(QStyle, "StandardPixmap", icon_name))
        except Exception:
            return
        button.setIcon(icon)
        button.setIconSize(QSize(15, 15))

    def _apply_responsive_layouts(self, force=False):
        width = self.centralWidget().width() if self.centralWidget() is not None else self.width()
        if width < 900:
            mode = "narrow"
        elif width < 1120:
            mode = "compact"
        else:
            mode = "wide"

        if not force and mode == self._responsive_mode:
            self._fit_scroll_bodies_to_viewport()
            return
        self._responsive_mode = mode

        self._arrange_grid_widgets(self._status_grid, self._status_items, 4 if mode == "wide" else 2)
        self._arrange_grid_widgets(
            self._operation_buttons_layout,
            self._operation_buttons,
            {"wide": 5, "compact": 3, "narrow": 2}[mode],
        )
        self._arrange_collection_layout(mode)
        self._arrange_preview_metrics(mode)
        self._arrange_grid_widgets(
            self._history_buttons_layout,
            self._history_buttons,
            4 if mode != "narrow" else 2,
        )
        self._arrange_grid_widgets(
            self._settings_buttons_layout,
            self._settings_buttons,
            {"wide": 3, "compact": 2, "narrow": 2}[mode],
        )
        if self._history_toolbar is not None:
            self._history_toolbar.setMinimumHeight(132 if mode == "narrow" else 92)
            self._history_toolbar.updateGeometry()
        if self._settings_toolbar is not None:
            self._settings_toolbar.setMinimumHeight(96 if mode == "wide" else 132)
            self._settings_toolbar.updateGeometry()
        self._arrange_grid_widgets(
            self._settings_body_layout,
            self._settings_panels + self._camera_tcp_panels,
            2 if mode == "wide" else 1,
        )
        self._fit_scroll_bodies_to_viewport()

    def _fit_scroll_bodies_to_viewport(self):
        for scroll in self._tab_scrolls:
            body = scroll.widget()
            if body is None:
                continue
            width = scroll.viewport().width()
            if width > 0:
                body.setFixedWidth(width)

    def _arrange_grid_widgets(self, layout, widgets, columns):
        if layout is None:
            return
        for index in reversed(range(layout.count())):
            item = layout.itemAt(index)
            widget = item.widget()
            if widget is not None:
                layout.removeWidget(widget)

        columns = max(1, columns)
        for index, widget in enumerate(widgets):
            layout.addWidget(widget, index // columns, index % columns)

        for column in range(8):
            layout.setColumnStretch(column, 1 if column < columns else 0)
        layout.invalidate()
        try:
            parent = layout.parentWidget()
        except Exception:
            parent = None
        if parent is not None:
            parent.updateGeometry()

    def _arrange_collection_layout(self, mode):
        if self._collection_layout is None or self._collection_left is None or self._fanuc_panel is None:
            return

        self._collection_layout.removeWidget(self._collection_left)
        self._collection_layout.removeWidget(self._fanuc_panel)
        if mode == "wide":
            self._collection_layout.addWidget(self._collection_left, 0, 0)
            self._collection_layout.addWidget(self._fanuc_panel, 0, 1)
            self._collection_layout.setColumnStretch(0, 2)
            self._collection_layout.setColumnStretch(1, 1)
            self._collection_layout.setRowStretch(0, 1)
            self._collection_layout.setRowStretch(1, 0)
            return

        self._collection_layout.addWidget(self._collection_left, 0, 0)
        self._collection_layout.addWidget(self._fanuc_panel, 1, 0)
        self._collection_layout.setColumnStretch(0, 1)
        self._collection_layout.setColumnStretch(1, 0)
        self._collection_layout.setRowStretch(0, 0)
        self._collection_layout.setRowStretch(1, 0)

    def _arrange_preview_metrics(self, mode):
        if self._preview_metrics_layout is None:
            return
        for label, value in self._preview_metrics:
            self._preview_metrics_layout.removeWidget(label)
            self._preview_metrics_layout.removeWidget(value)

        pairs_per_row = {"wide": 4, "compact": 2, "narrow": 1}[mode]
        for index, (label, value) in enumerate(self._preview_metrics):
            pair_col = index % pairs_per_row
            row = index // pairs_per_row
            col = pair_col * 2
            self._preview_metrics_layout.addWidget(label, row, col)
            self._preview_metrics_layout.addWidget(value, row, col + 1)

        for column in range(8):
            is_used_value_column = column < pairs_per_row * 2 and column % 2 == 1
            self._preview_metrics_layout.setColumnStretch(column, 1 if is_used_value_column else 0)
        self._preview_metrics_layout.invalidate()

    def _connect_ros(self):
        self._ros.ros_state_changed.connect(self._set_ros_state)
        self._ros.status_received.connect(self._update_status)
        self._ros.fanuc_received.connect(self._update_fanuc)
        self._ros.image_received.connect(self._update_image)
        self._ros.services_received.connect(self._update_services)
        self._ros.log_received.connect(self._append_log)

    def _set_ros_state(self, ok):
        self.ros_pill.set_state("在线" if ok else "离线", "ok" if ok else "bad")

    def _update_status(self, status):
        self._last_status_time = time.monotonic()
        running = status["running"]
        self.collect_pill.set_state("采集中" if running else "待机", "ok" if running else "idle")
        self._current_save_dir = status["current_save_dir"]
        self._labels["current_save_dir"].setText(status["current_save_dir"] or "-")
        self._labels["preview_current_dir"].setText(status["current_save_dir"] or "-")
        self._labels["auto_save"].setText("开启" if status["auto_save"] else "关闭")
        register_value = status["target_register_value"] if status["has_target_register_value"] else "未知"
        self._labels["target_register"].setText(
            f"R[{status['target_register_index']}] = {register_value}"
        )
        task_text = " / ".join([
            value for value in [
                status["task_id"],
                status["workpiece_id"],
                status["weld_seam_id"],
            ] if value
        ])
        self._labels["current_task"].setText(task_text or "-")

        for key in (
            "image_count",
            "point_cloud_count",
            "tool_pose_count",
            "fanuc_info_count",
            "image_log_count",
            "height_log_count",
            "estimated_line_count",
        ):
            self._labels[key].setText(str(status[key]))
        self._labels["preview_image_count"].setText(str(status["image_count"]))
        self._labels["preview_image_log_count"].setText(str(status["image_log_count"]))
        self._labels["preview_point_cloud_count"].setText(str(status["point_cloud_count"]))
        self._labels["last_error"].setText(status["last_error"] or "-")

    def _update_fanuc(self, fanuc):
        self._last_fanuc_time = time.monotonic()
        self.fanuc_pill.set_state("在线", "ok")
        units = {
            "voltage1": " V",
            "voltage2": " V",
            "current1": " A",
            "current2": " A",
            "wire_speed1": " mm/s",
            "wire_speed2": " mm/s",
            "override": "%",
        }
        for key, value in fanuc.items():
            label = self._labels.get(f"fanuc_{key}")
            if label is None:
                continue
            suffix = units.get(key, "")
            label.setText(f"{value}{suffix}")

    def _update_image(self, image):
        self._last_image = image
        self._last_image_time = time.monotonic()
        self.image_pill.set_state("在线", "ok")
        self._render_image()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._apply_responsive_layouts()
        QTimer.singleShot(0, self._fit_scroll_bodies_to_viewport)
        QTimer.singleShot(0, self._render_image)

    def _render_image(self):
        if self._last_image is None:
            return
        pixmap = QPixmap.fromImage(self._last_image)
        scaled = pixmap.scaled(
            self.image_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.image_label.setPixmap(scaled)

    def _update_services(self, services):
        for name, ready in services.items():
            button = self._service_buttons.get(name)
            if button is not None:
                button.setEnabled(ready)
        self.save_task_btn.setEnabled(services.get(DATA_COLLECT_SET_TASK, False))

    def _append_log(self, text):
        now = time.strftime("%H:%M:%S")
        current = self.log_label.text()
        lines = [line for line in current.splitlines() if line and line != "等待 ROS 状态..."]
        lines.append(f"[{now}] {text}")
        self.log_label.setText("\n".join(lines[-8:]))

    def _refresh_heartbeat(self):
        now = time.monotonic()
        if self._last_status_time and now - self._last_status_time > 3.0:
            self.collect_pill.set_state("无状态", "unknown")
        if self._last_fanuc_time and now - self._last_fanuc_time > 3.0:
            self.fanuc_pill.set_state("超时", "unknown")
        if self._last_image_time and now - self._last_image_time > 3.0:
            self.image_pill.set_state("等待", "unknown")

    def _save_task(self):
        task = {}
        for key, widget in self._task_inputs.items():
            if isinstance(widget, QTextEdit):
                task[key] = widget.toPlainText().strip()
            else:
                task[key] = widget.text().strip()
        self._ros.set_task(task)

    def _open_current_dir(self):
        self._open_dir(self._current_save_dir, "当前还没有采集目录。")

    def _choose_data_root(self):
        directory = QFileDialog.getExistingDirectory(self, "选择数据根目录", self.data_root_edit.text())
        if directory:
            self.data_root_edit.setText(directory)
            self._refresh_history()

    def _choose_settings_file(self):
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "选择参数配置",
            self.settings_path_edit.text(),
            "YAML Files (*.yaml *.yml);;All Files (*)",
        )
        if filename:
            self.settings_path_edit.setText(filename)
            self._load_settings()

    def _settings_yaml_path(self):
        return Path(self.settings_path_edit.text()).expanduser()

    def _load_settings(self):
        path = self._settings_yaml_path()
        self._loading_settings = True
        try:
            if path.exists():
                loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
                self._settings_data = loaded if isinstance(loaded, dict) else {}
                status = f"已加载：{path}"
            else:
                self._settings_data = {}
                status = f"配置文件不存在，保存时会创建：{path}"

            for group in SETTINGS_SCHEMA:
                node_params = self._settings_data.get(group["node"], {}).get("ros__parameters", {})
                if not isinstance(node_params, dict):
                    node_params = {}
                for key, _, field_type, _, _, default in group["fields"]:
                    value = node_params.get(key, default)
                    self._set_setting_widget_value(
                        self._setting_widgets[(group["node"], key)],
                        field_type,
                        value,
                    )
            camera_status = self._load_camera_tcp_settings()
            self.settings_status_label.setText(f"{status}；{camera_status}")
        except Exception as exc:
            self.settings_status_label.setText(f"加载失败：{exc}")
        finally:
            self._loading_settings = False

    def _set_setting_widget_value(self, widget, field_type, value):
        if field_type == "int":
            widget.setValue(int(value))
            return
        if field_type in ("double", "double3", "double6"):
            widget.setValue(float(value))
            return
        if field_type == "bool":
            widget.setChecked(self._as_bool(value))
            return
        if field_type == "int_bool":
            widget.setChecked(self._as_bool(value))
            return
        widget.setText(str(value))

    def _load_camera_tcp_settings(self):
        path = self._camera_tcp_yaml_path()
        try:
            if path.exists():
                loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
                self._camera_tcp_data = loaded if isinstance(loaded, dict) else {}
                status = f"已加载3D相机配置：{path}"
            else:
                self._camera_tcp_data = {}
                status = f"3D相机配置不存在，保存时会创建：{path}"

            for group in CAMERA_TCP_SCHEMA:
                section = group["section"]
                source = self._camera_tcp_data.get(section, {}) if section else self._camera_tcp_data
                if not isinstance(source, dict):
                    source = {}
                for key, _, field_type, _, _, default in group["fields"]:
                    value = source.get(key, default)
                    self._set_setting_widget_value(
                        self._camera_tcp_widgets[(section, key)],
                        field_type,
                        value,
                    )
            return status
        except Exception as exc:
            return f"3D相机配置加载失败：{exc}"

    def _camera_tcp_yaml_path(self):
        widget = self._setting_widgets.get(("camera_driver_3d", "cfg"))
        if widget is None:
            return Path(DEFAULT_CAMERA_TCP_YAML)
        return Path(widget.text().strip() or DEFAULT_CAMERA_TCP_YAML).expanduser()

    def _on_setting_widget_changed(self, *args):
        if self._loading_settings:
            return
        widget = self.sender()
        if widget is None:
            return
        self._apply_widget_live(widget)
        self.settings_status_label.setText("参数已实时发送，尚未保存到 YAML")

    def _apply_pending_settings(self):
        return

    def _apply_widget_live(self, widget):
        config_kind = widget.property("config_kind")
        if config_kind == "nodemanage":
            node_name = widget.property("node_name")
            key = widget.property("param_key")
            field_type = widget.property("field_type")
            value = self._setting_widget_value(widget, field_type)
            self._ros.set_parameters(node_name, {key: value})
            if node_name == "camera_driver_3d" and key == "cfg":
                self._loading_settings = True
                try:
                    camera_status = self._load_camera_tcp_settings()
                    self.settings_status_label.setText(camera_status)
                finally:
                    self._loading_settings = False
            return

        if config_kind == "camera_tcp":
            parameter_name = self._camera_tcp_parameter_name(widget)
            field_type = widget.property("field_type")
            value = self._setting_widget_value(widget, field_type)
            self._ros.set_parameters("camera_driver_3d", {parameter_name: value})

    def _camera_tcp_parameter_name(self, widget):
        section = widget.property("section")
        key = widget.property("param_key")
        if section:
            return f"{section}.{key}"
        return key

    def _save_settings(self):
        if self._loading_settings:
            return

        data = self._settings_data if isinstance(self._settings_data, dict) else {}
        for group in SETTINGS_SCHEMA:
            node_data = data.setdefault(group["node"], {})
            if not isinstance(node_data, dict):
                node_data = {}
                data[group["node"]] = node_data
            params = node_data.setdefault("ros__parameters", {})
            if not isinstance(params, dict):
                params = {}
                node_data["ros__parameters"] = params

            for key, _, field_type, _, _, _ in group["fields"]:
                widget = self._setting_widgets[(group["node"], key)]
                params[key] = self._setting_widget_value(widget, field_type)

        camera_data = self._camera_tcp_data if isinstance(self._camera_tcp_data, dict) else {}
        for group in CAMERA_TCP_SCHEMA:
            section = group["section"]
            target = camera_data.setdefault(section, {}) if section else camera_data
            if not isinstance(target, dict):
                target = {}
                if section:
                    camera_data[section] = target
            for key, _, field_type, _, _, _ in group["fields"]:
                widget = self._camera_tcp_widgets[(section, key)]
                target[key] = self._setting_widget_value(widget, field_type)

        path = self._settings_yaml_path()
        camera_path = self._camera_tcp_yaml_path()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
                encoding="utf-8",
            )
            camera_path.parent.mkdir(parents=True, exist_ok=True)
            camera_path.write_text(
                yaml.safe_dump(camera_data, allow_unicode=True, sort_keys=False),
                encoding="utf-8",
            )
            self._settings_data = data
            self._camera_tcp_data = camera_data
            self.settings_status_label.setText(f"已保存：{path}；{camera_path}")
        except Exception as exc:
            self.settings_status_label.setText(f"保存失败：{exc}")

    def _setting_widget_value(self, widget, field_type):
        if field_type == "int":
            return int(widget.value())
        if field_type in ("double", "double3", "double6"):
            return float(widget.value())
        if field_type == "bool":
            return bool(widget.isChecked())
        if field_type == "int_bool":
            return 1 if widget.isChecked() else 0
        return widget.text().strip()

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in ("1", "true", "yes", "on", "开启")

    def _refresh_history(self):
        data_root = Path(self.data_root_edit.text()).expanduser()
        self.history_table.setRowCount(0)
        self._history_records = []

        if not data_root.is_dir():
            self.history_summary.setText("数据根目录不存在")
            return

        for manifest_path in sorted(data_root.rglob("manifest.json"), reverse=True):
            record = self._load_manifest_record(manifest_path)
            if record is not None:
                self._history_records.append(record)

        self.history_table.setRowCount(len(self._history_records))
        for row, record in enumerate(self._history_records):
            values = [
                record["status"],
                record["started_at"],
                record["ended_at"],
                record["task_id"],
                record["workpiece_id"],
                record["weld_seam_id"],
                record["register"],
                str(record["image_count"]),
                str(record["point_cloud_count"]),
                record["path"],
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                self.history_table.setItem(row, col, item)

        self.history_summary.setText(f"共找到 {len(self._history_records)} 条采集记录")

    def _load_manifest_record(self, manifest_path):
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except Exception:
            return None

        task = manifest.get("task", {})
        target_register = manifest.get("target_register", {})
        counts = manifest.get("counts", {})
        register_value = target_register.get("value")
        if register_value is None:
            register_value = "unknown"
        return {
            "status": str(manifest.get("status", "")),
            "started_at": str(manifest.get("started_at", "")),
            "ended_at": str(manifest.get("ended_at") or ""),
            "task_id": str(task.get("task_id", "")),
            "workpiece_id": str(task.get("workpiece_id", "")),
            "weld_seam_id": str(task.get("weld_seam_id", "")),
            "register": f"R[{target_register.get('index', '')}]={register_value}",
            "image_count": int(counts.get("image", 0)),
            "point_cloud_count": int(counts.get("point_cloud", 0)),
            "path": str(manifest_path.parent),
        }

    def _selected_history_record(self):
        row = self.history_table.currentRow()
        if row < 0 or row >= len(self._history_records):
            QMessageBox.information(self, "历史数据", "请先选择一条采集记录。")
            return None
        return self._history_records[row]

    def _open_selected_history_dir(self):
        record = self._selected_history_record()
        if record:
            self._open_dir(record["path"], "历史数据目录不存在。")

    def _export_selected_history_zip(self):
        record = self._selected_history_record()
        if not record:
            return

        source_dir = Path(record["path"])
        if not source_dir.is_dir():
            QMessageBox.warning(self, "导出ZIP", "历史数据目录不存在。")
            return

        default_name = source_dir.name + ".zip"
        target, _ = QFileDialog.getSaveFileName(self, "导出采集数据", default_name, "Zip Files (*.zip)")
        if not target:
            return
        if not target.endswith(".zip"):
            target += ".zip"

        try:
            with zipfile.ZipFile(target, "w", compression=zipfile.ZIP_DEFLATED) as archive:
                for path in source_dir.rglob("*"):
                    if path.is_file():
                        archive.write(path, path.relative_to(source_dir.parent))
        except Exception as exc:
            QMessageBox.warning(self, "导出ZIP", f"导出失败：{exc}")
            return

        self._append_log(f"已导出采集数据：{target}")

    def _open_dir(self, directory, empty_message):
        if not directory:
            QMessageBox.information(self, "目录", empty_message)
            return
        if not os.path.isdir(directory):
            QMessageBox.warning(self, "目录", "目录不存在或尚未创建。")
            return
        QDesktopServices.openUrl(QUrl.fromLocalFile(directory))

    def _apply_style(self):
        self.setStyleSheet("""
            QWidget {
                color: #1d1d1f;
                font-family: "SF Pro Text", "Helvetica Neue", "PingFang SC", "Microsoft YaHei", sans-serif;
                font-size: 14px;
            }
            QMainWindow, QWidget#appRoot, QWidget#tabPage {
                background: #f6f7fb;
            }
            QWidget#topBar {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #ffffff, stop:0.58 #fbfdff, stop:1 #f8fbff);
                border: 1px solid #dfe4ee;
                border-radius: 8px;
            }
            QWidget#statusStrip {
                background: transparent;
            }
            QWidget#statusItem {
                background: #ffffff;
                border: 1px solid #e4e7ef;
                border-radius: 8px;
            }
            QWidget#statusItem[accent="blue"] {
                background: #f1f7ff;
                border-color: #cfe3ff;
            }
            QWidget#statusItem[accent="green"] {
                background: #effaf4;
                border-color: #ccefdc;
            }
            QWidget#statusItem[accent="orange"] {
                background: #fff8ed;
                border-color: #f8ddb0;
            }
            QWidget#statusItem[accent="purple"] {
                background: #f7f3ff;
                border-color: #dfd4ff;
            }
            QWidget#toolbarPanel {
                background: #ffffff;
                border: 1px solid #e1e5ee;
                border-radius: 8px;
            }
            QTabWidget::pane {
                border: 0;
                background: transparent;
                margin-top: 10px;
            }
            QTabBar::tab {
                background: #e9e9ed;
                color: #5f6067;
                border: 1px solid #dedee5;
                border-radius: 8px;
                padding: 9px 18px;
                margin-right: 6px;
            }
            QTabBar::tab:selected {
                background: #ffffff;
                color: #007aff;
                font-weight: 600;
            }
            QTabBar::tab:hover:!selected {
                background: #f8f8fb;
                color: #2a2a2f;
            }
            QGroupBox {
                background: #ffffff;
                border: 1px solid #dfe3ec;
                border-radius: 8px;
                margin-top: 16px;
                padding: 16px 14px 14px 14px;
                font-weight: 600;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 14px;
                padding: 0 7px;
                color: #6e6e73;
                background: #ffffff;
                font-size: 13px;
            }
            QGroupBox[accent="blue"] {
                border-color: #bfdbfe;
            }
            QGroupBox[accent="blue"]::title {
                color: #0a84ff;
            }
            QGroupBox[accent="green"] {
                border-color: #c7ead7;
            }
            QGroupBox[accent="green"]::title {
                color: #2f9e55;
            }
            QGroupBox[accent="teal"] {
                border-color: #bce9e4;
            }
            QGroupBox[accent="teal"]::title {
                color: #0f8b8d;
            }
            QGroupBox[accent="orange"] {
                border-color: #f4d1a4;
            }
            QGroupBox[accent="orange"]::title {
                color: #c26712;
            }
            QGroupBox[accent="purple"] {
                border-color: #d9ccff;
            }
            QGroupBox[accent="purple"]::title {
                color: #7a5af8;
            }
            QPushButton {
                background: #ffffff;
                color: #007aff;
                border: 1px solid #d6d6dd;
                border-radius: 8px;
                padding: 8px 13px;
                font-weight: 600;
            }
            QPushButton:hover {
                background: #f7f7fa;
                border-color: #c5c5cc;
            }
            QPushButton:pressed {
                background: #ececf1;
            }
            QPushButton:disabled {
                background: #ececf1;
                color: #a1a1a8;
                border-color: #e1e1e6;
            }
            QPushButton[role="primary"] {
                background: #007aff;
                color: #ffffff;
                border-color: #007aff;
            }
            QPushButton[role="primary"]:hover {
                background: #006ee6;
                border-color: #006ee6;
            }
            QPushButton[role="primary"]:pressed {
                background: #005ec2;
                border-color: #005ec2;
            }
            QPushButton[role="danger"] {
                background: #ff3b30;
                color: #ffffff;
                border-color: #ff3b30;
            }
            QPushButton[role="danger"]:hover {
                background: #e6342a;
                border-color: #e6342a;
            }
            QPushButton[role="danger"]:pressed {
                background: #c92d25;
                border-color: #c92d25;
            }
            QPushButton[role="success"] {
                background: #30b07a;
                color: #ffffff;
                border-color: #30b07a;
            }
            QPushButton[role="success"]:hover {
                background: #249b68;
                border-color: #249b68;
            }
            QPushButton[role="success"]:pressed {
                background: #1b8055;
                border-color: #1b8055;
            }
            QPushButton[role="warning"] {
                background: #ff9f0a;
                color: #ffffff;
                border-color: #ff9f0a;
            }
            QPushButton[role="warning"]:hover {
                background: #e58d08;
                border-color: #e58d08;
            }
            QPushButton[role="warning"]:pressed {
                background: #c77906;
                border-color: #c77906;
            }
            QPushButton[role="primary"]:disabled,
            QPushButton[role="danger"]:disabled,
            QPushButton[role="success"]:disabled,
            QPushButton[role="warning"]:disabled {
                background: #e5e7ee;
                color: #8e8e93;
                border-color: #e1e4eb;
            }
            QLineEdit, QTextEdit, QSpinBox, QDoubleSpinBox {
                background: #ffffff;
                border: 1px solid #d2d2d7;
                border-radius: 7px;
                padding: 8px;
                selection-background-color: #007aff;
                selection-color: #ffffff;
            }
            QLineEdit:focus, QTextEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {
                border: 1px solid #007aff;
                background: #ffffff;
            }
            QCheckBox {
                spacing: 8px;
                color: #1d1d1f;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
                border-radius: 5px;
                border: 1px solid #c8c8d0;
                background: #ffffff;
            }
            QCheckBox::indicator:checked {
                background: #007aff;
                border-color: #007aff;
            }
            QTableWidget {
                background: #ffffff;
                alternate-background-color: #f8f8fb;
                border: 1px solid #dedee5;
                border-radius: 8px;
                selection-background-color: #d9ebff;
                selection-color: #1d1d1f;
            }
            QTableWidget::item {
                padding: 7px 8px;
                border: 0;
            }
            QTableWidget::item:selected {
                background: #d9ebff;
            }
            QHeaderView::section {
                background: #f7f9fd;
                color: #6e6e73;
                border: 0;
                border-bottom: 1px solid #dedee5;
                padding: 8px 9px;
                font-weight: 600;
            }
            QScrollArea#settingsScroll,
            QScrollArea#collectionScroll,
            QScrollArea#previewScroll,
            QScrollArea#historyScroll,
            QScrollArea#settingsPageScroll {
                border: 0;
                background: transparent;
            }
            QScrollArea#settingsScroll > QWidget > QWidget,
            QScrollArea#collectionScroll > QWidget > QWidget,
            QScrollArea#previewScroll > QWidget > QWidget,
            QScrollArea#historyScroll > QWidget > QWidget,
            QScrollArea#settingsPageScroll > QWidget > QWidget {
                background: transparent;
            }
            QLabel#title {
                font-size: 24px;
                font-weight: 700;
                color: #1d1d1f;
            }
            QLabel#subtitle {
                color: #6e6e73;
                font-size: 13px;
            }
            QLabel#statusCaption {
                color: #6e6e73;
                font-size: 12px;
                font-weight: 600;
            }
            QLabel#fieldLabel {
                color: #6e6e73;
                font-size: 13px;
                font-weight: 600;
                padding-top: 6px;
            }
            QLabel#fieldValue {
                background: #f8fbff;
                border: 1px solid #ececf1;
                border-radius: 7px;
                color: #1d1d1f;
                font-weight: 500;
                padding: 6px 9px;
            }
            QLabel#summaryLabel {
                color: #6e6e73;
                background: transparent;
                padding: 4px 2px;
            }
            QLabel#logText {
                background: #fbfcff;
                border: 1px solid #e5e9f2;
                border-radius: 8px;
                color: #303035;
                padding: 10px 12px;
            }
            QLabel#imagePreview {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #101820, stop:0.52 #111114, stop:1 #182536);
                color: #dbe7f5;
                border: 1px solid #263548;
                border-radius: 8px;
                font-size: 15px;
            }
            StatusPill {
                border-radius: 13px;
                padding: 4px 10px;
                font-weight: 600;
                background: #e9e9ed;
                color: #4d4d53;
            }
            StatusPill[state="ok"] {
                background: #dff7e8;
                color: #1d7a3a;
            }
            StatusPill[state="idle"] {
                background: #eef2f7;
                color: #5c6570;
            }
            StatusPill[state="bad"] {
                background: #ffe2df;
                color: #b3261e;
            }
            StatusPill[state="unknown"] {
                background: #fff2d9;
                color: #8a5a00;
            }
            QScrollBar:vertical {
                background: transparent;
                width: 10px;
                margin: 2px;
            }
            QScrollBar::handle:vertical {
                background: #c7c7cc;
                border-radius: 5px;
                min-height: 36px;
            }
            QScrollBar::handle:vertical:hover {
                background: #aeaeb2;
            }
            QScrollBar::add-line:vertical,
            QScrollBar::sub-line:vertical {
                height: 0;
            }
            QScrollBar:horizontal {
                background: transparent;
                height: 10px;
                margin: 2px;
            }
            QScrollBar::handle:horizontal {
                background: #c7c7cc;
                border-radius: 5px;
                min-width: 36px;
            }
            QScrollBar::handle:horizontal:hover {
                background: #aeaeb2;
            }
            QScrollBar::add-line:horizontal,
            QScrollBar::sub-line:horizontal {
                width: 0;
            }
        """)


def enable_high_dpi_scaling():
    for attr_name in ("AA_EnableHighDpiScaling", "AA_UseHighDpiPixmaps"):
        try:
            attr = qt_enum(Qt, "ApplicationAttribute", attr_name)
            QApplication.setAttribute(attr, True)
        except Exception:
            continue


def main():
    enable_high_dpi_scaling()
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
