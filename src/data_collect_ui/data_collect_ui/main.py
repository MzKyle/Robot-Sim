import json
import os
import queue
import sys
import time
import zipfile
from pathlib import Path

import yaml

try:
    from PySide6.QtCore import QThread, Qt, QTimer, Signal, QUrl
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
        QTableWidget,
        QTableWidgetItem,
        QTabWidget,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
except ImportError:
    try:
        from PyQt5.QtCore import QThread, Qt, QTimer, QUrl, pyqtSignal as Signal
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
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from weld_interface.msg import DataCollectStatus, FanucRobotInfo
from weld_interface.srv import SetCollectionTask


DATA_COLLECT_ACTIVATE = "/data_collect_activate"
DATA_COLLECT_DEACTIVATE = "/data_collect_deactivate"
DATA_COLLECT_SET_TASK = "/data_collect_set_task"
START_FIX_SCAN = "/start_fix_scan"
STOP_FIX_SCAN = "/stop_fix_scan"
IMAGE_TOPIC = "/image_topic"
DEFAULT_DATA_ROOT = "/home/kyle/sany/weld_data_collect_ws/data"


def default_nodemanage_yaml():
    env_path = os.environ.get("AUTOCOVER_NODEMANAGE_YAML")
    if env_path:
        return env_path

    candidates = [
        "/etc/weld_data_collect/nodemanage.yaml",
        "/home/kyle/sany/weld_data_collect_ws/src/data_collect_bringup/config/nodemanage.yaml",
    ]
    for candidate in candidates:
        if Path(candidate).exists():
            return candidate
    return candidates[0]


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
            ("cfg", "配置文件", "text", None, None, "/home/kyle/sany/weld_data_collect_ws/src/camera_3d_driver/config/cameratcp.yaml"),
            ("publish_tf", "发布TF", "bool", None, None, True),
        ],
    },
    {
        "title": "Fanuc机器人",
        "node": "robot_driver_fanuc",
        "fields": [
            ("so_file_path", "动态库路径", "text", None, None, "/home/kyle/sany/weld_data_collect_ws/src/fanuc_robot/lib/libFanucRobot.so"),
            ("robot_ip", "机器人IP", "text", None, None, "10.16.140.114"),
            ("robot_port", "机器人端口", "int", 1, 65535, 60008),
            ("target_register_index", "目标寄存器", "int", 0, 9999, 100),
        ],
    },
    {
        "title": "数据采集",
        "node": "data_collect_node",
        "fields": [
            ("save_dir_root", "保存根目录", "text", None, None, "/home/kyle/sany/weld_data_collect_ws/data"),
            ("image_save_interval", "图像保存间隔", "int", 1, 100000, 12),
            ("image_log_save_interval", "图像日志间隔", "int", 1, 100000, 3),
            ("height_log_save_interval", "高度日志间隔", "int", 1, 100000, 4),
            ("fix_scan_interval", "点云保存间隔", "int", 1, 100000, 6),
            ("auto_save_flag", "自动采集", "int_bool", None, None, 0),
            ("target_register_index", "目标寄存器", "int", 0, 9999, 100),
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

    def run(self):
        rclpy.init(args=None)
        self._node = Node("data_collect_ui")
        self._clients = {
            DATA_COLLECT_ACTIVATE: self._node.create_client(Empty, DATA_COLLECT_ACTIVATE),
            DATA_COLLECT_DEACTIVATE: self._node.create_client(Empty, DATA_COLLECT_DEACTIVATE),
            START_FIX_SCAN: self._node.create_client(Empty, START_FIX_SCAN),
            STOP_FIX_SCAN: self._node.create_client(Empty, STOP_FIX_SCAN),
            DATA_COLLECT_SET_TASK: self._node.create_client(SetCollectionTask, DATA_COLLECT_SET_TASK),
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
        self.setMinimumWidth(72)
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
        self._settings_data = {}
        self._loading_settings = False
        self._settings_save_timer = QTimer(self)
        self._settings_save_timer.setSingleShot(True)
        self._settings_save_timer.timeout.connect(self._save_settings)
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
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(14)

        layout.addLayout(self._build_top_bar())

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_collection_tab(), "采集操作")
        self.tabs.addTab(self._build_preview_tab(), "实时预览")
        self.tabs.addTab(self._build_history_tab(), "历史数据")
        self.tabs.addTab(self._build_settings_tab(), "参数设置")
        layout.addWidget(self.tabs, 1)

        layout.addWidget(self._build_log_panel())
        self.setCentralWidget(root)
        self._apply_style()

    def _build_top_bar(self):
        layout = QHBoxLayout()
        title = QLabel("焊接数据采集操作台")
        title.setObjectName("title")
        subtitle = QLabel("采集状态、任务信息、实时预览和历史数据管理")
        subtitle.setObjectName("subtitle")

        title_box = QVBoxLayout()
        title_box.addWidget(title)
        title_box.addWidget(subtitle)

        self.ros_pill = StatusPill()
        self.collect_pill = StatusPill()
        self.fanuc_pill = StatusPill()
        self.image_pill = StatusPill()

        layout.addLayout(title_box, 1)
        layout.addWidget(QLabel("ROS"))
        layout.addWidget(self.ros_pill)
        layout.addWidget(QLabel("采集"))
        layout.addWidget(self.collect_pill)
        layout.addWidget(QLabel("Fanuc"))
        layout.addWidget(self.fanuc_pill)
        layout.addWidget(QLabel("图像"))
        layout.addWidget(self.image_pill)
        return layout

    def _build_collection_tab(self):
        page = QWidget()
        layout = QHBoxLayout(page)
        layout.setSpacing(14)
        layout.addWidget(self._build_collection_panel(), 2)
        layout.addWidget(self._build_fanuc_panel(), 1)
        return page

    def _build_collection_panel(self):
        panel = QGroupBox("采集控制")
        layout = QVBoxLayout(panel)

        buttons = QHBoxLayout()
        self.start_collect_btn = self._make_service_button("启动采集", DATA_COLLECT_ACTIVATE)
        self.stop_collect_btn = self._make_service_button("停止采集", DATA_COLLECT_DEACTIVATE)
        self.start_scan_btn = self._make_service_button("开始3D扫描", START_FIX_SCAN)
        self.stop_scan_btn = self._make_service_button("停止3D扫描", STOP_FIX_SCAN)
        self.open_dir_btn = QPushButton("打开保存目录")
        self.open_dir_btn.clicked.connect(self._open_current_dir)
        buttons.addWidget(self.start_collect_btn)
        buttons.addWidget(self.stop_collect_btn)
        buttons.addWidget(self.start_scan_btn)
        buttons.addWidget(self.stop_scan_btn)
        buttons.addWidget(self.open_dir_btn)
        layout.addLayout(buttons)

        layout.addWidget(self._build_task_panel())

        grid = QGridLayout()
        grid.setHorizontalSpacing(18)
        grid.setVerticalSpacing(10)
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
            label = QLabel(name)
            value = QLabel("-")
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            value.setWordWrap(True)
            grid.addWidget(label, row, 0)
            grid.addWidget(value, row, 1)
            self._labels[key] = value

        layout.addLayout(grid)
        layout.addStretch(1)
        return panel

    def _build_task_panel(self):
        panel = QGroupBox("采集任务")
        form = QFormLayout(panel)
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
        self.save_task_btn.setEnabled(False)
        self.save_task_btn.clicked.connect(self._save_task)
        form.addRow("", self.save_task_btn)
        return panel

    def _build_fanuc_panel(self):
        panel = QGroupBox("Fanuc 状态")
        layout = QGridLayout(panel)
        layout.setHorizontalSpacing(14)
        layout.setVerticalSpacing(10)
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
            label = QLabel(name)
            value = QLabel("-")
            value.setWordWrap(True)
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            layout.addWidget(label, row, 0)
            layout.addWidget(value, row, 1)
            self._labels[f"fanuc_{key}"] = value
        return panel

    def _build_preview_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setSpacing(12)

        self.image_label = QLabel("等待 /image_topic 图像...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumHeight(440)
        self.image_label.setObjectName("imagePreview")
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.image_label, 1)

        info = QGridLayout()
        info.setHorizontalSpacing(18)
        for col, (name, key) in enumerate([
            ("图像保存", "preview_image_count"),
            ("图像日志", "preview_image_log_count"),
            ("点云保存", "preview_point_cloud_count"),
            ("当前目录", "preview_current_dir"),
        ]):
            label = QLabel(name)
            value = QLabel("-")
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            value.setWordWrap(True)
            info.addWidget(label, 0, col * 2)
            info.addWidget(value, 0, col * 2 + 1)
            self._labels[key] = value
        layout.addLayout(info)
        return page

    def _build_history_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        controls = QHBoxLayout()

        self.data_root_edit = QLineEdit(os.environ.get("WELD_DATA_ROOT", DEFAULT_DATA_ROOT))
        self.data_root_edit.setPlaceholderText("数据根目录")
        choose_btn = QPushButton("选择目录")
        choose_btn.clicked.connect(self._choose_data_root)
        refresh_btn = QPushButton("刷新")
        refresh_btn.clicked.connect(self._refresh_history)
        open_btn = QPushButton("打开目录")
        open_btn.clicked.connect(self._open_selected_history_dir)
        export_btn = QPushButton("导出ZIP")
        export_btn.clicked.connect(self._export_selected_history_zip)

        controls.addWidget(QLabel("数据根目录"))
        controls.addWidget(self.data_root_edit, 1)
        controls.addWidget(choose_btn)
        controls.addWidget(refresh_btn)
        controls.addWidget(open_btn)
        controls.addWidget(export_btn)
        layout.addLayout(controls)

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
        self.history_table.horizontalHeader().setSectionResizeMode(qt_enum(QHeaderView, "ResizeMode", "ResizeToContents"))
        self.history_table.horizontalHeader().setSectionResizeMode(9, qt_enum(QHeaderView, "ResizeMode", "Stretch"))
        layout.addWidget(self.history_table, 1)

        self.history_summary = QLabel("尚未扫描历史数据")
        layout.addWidget(self.history_summary)
        return page

    def _build_settings_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setSpacing(12)

        controls = QHBoxLayout()
        self.settings_path_edit = QLineEdit(default_nodemanage_yaml())
        self.settings_path_edit.editingFinished.connect(self._load_settings)
        choose_btn = QPushButton("选择配置")
        choose_btn.clicked.connect(self._choose_settings_file)
        reload_btn = QPushButton("重新加载")
        reload_btn.clicked.connect(self._load_settings)

        controls.addWidget(QLabel("配置文件"))
        controls.addWidget(self.settings_path_edit, 1)
        controls.addWidget(choose_btn)
        controls.addWidget(reload_btn)
        layout.addLayout(controls)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        body = QWidget()
        body_layout = QGridLayout(body)
        body_layout.setHorizontalSpacing(14)
        body_layout.setVerticalSpacing(14)

        for index, group in enumerate(SETTINGS_SCHEMA):
            panel = QGroupBox(group["title"])
            form = QFormLayout(panel)
            form.setFieldGrowthPolicy(qt_enum(QFormLayout, "FieldGrowthPolicy", "AllNonFixedFieldsGrow"))
            for key, label, field_type, minimum, maximum, default in group["fields"]:
                widget = self._make_setting_widget(field_type, minimum, maximum)
                widget.setProperty("default_value", default)
                widget.setProperty("field_type", field_type)
                self._setting_widgets[(group["node"], key)] = widget
                form.addRow(label, widget)
            body_layout.addWidget(panel, index // 2, index % 2)

        scroll.setWidget(body)
        layout.addWidget(scroll, 1)

        self.settings_status_label = QLabel("参数配置未加载")
        layout.addWidget(self.settings_status_label)
        return page

    def _make_setting_widget(self, field_type, minimum, maximum):
        if field_type == "int":
            widget = QSpinBox()
            widget.setRange(int(minimum), int(maximum))
            widget.valueChanged.connect(self._schedule_settings_save)
            return widget

        if field_type == "double":
            widget = QDoubleSpinBox()
            widget.setRange(float(minimum), float(maximum))
            widget.setDecimals(3)
            widget.setSingleStep(0.1)
            widget.valueChanged.connect(self._schedule_settings_save)
            return widget

        if field_type in ("bool", "int_bool"):
            widget = QCheckBox()
            widget.toggled.connect(self._schedule_settings_save)
            return widget

        widget = QLineEdit()
        widget.editingFinished.connect(self._schedule_settings_save)
        return widget

    def _build_log_panel(self):
        panel = QGroupBox("运行日志")
        layout = QVBoxLayout(panel)
        self.log_label = QLabel("等待 ROS 状态...")
        self.log_label.setWordWrap(True)
        self.log_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.log_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.log_label.setMinimumHeight(84)
        layout.addWidget(self.log_label)
        return panel

    def _make_service_button(self, text, service_name):
        button = QPushButton(text)
        button.setEnabled(False)
        button.clicked.connect(lambda: self._ros.call_service(service_name))
        self._service_buttons[service_name] = button
        return button

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
        self._render_image()
        super().resizeEvent(event)

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
            self.settings_status_label.setText(status)
        except Exception as exc:
            self.settings_status_label.setText(f"加载失败：{exc}")
        finally:
            self._loading_settings = False

    def _set_setting_widget_value(self, widget, field_type, value):
        if field_type == "int":
            widget.setValue(int(value))
            return
        if field_type == "double":
            widget.setValue(float(value))
            return
        if field_type == "bool":
            widget.setChecked(self._as_bool(value))
            return
        if field_type == "int_bool":
            widget.setChecked(self._as_bool(value))
            return
        widget.setText(str(value))

    def _schedule_settings_save(self, *args):
        if self._loading_settings:
            return
        self.settings_status_label.setText("参数已修改，正在保存...")
        self._settings_save_timer.start(600)

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

        path = self._settings_yaml_path()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
                encoding="utf-8",
            )
            self._settings_data = data
            self.settings_status_label.setText(f"已自动保存：{path}")
        except Exception as exc:
            self.settings_status_label.setText(f"保存失败：{exc}")

    def _setting_widget_value(self, widget, field_type):
        if field_type == "int":
            return int(widget.value())
        if field_type == "double":
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
                font-size: 14px;
                color: #1d2329;
                background: #f5f7f9;
            }
            QTabWidget::pane {
                border: 1px solid #d9e0e7;
                background: #ffffff;
            }
            QTabBar::tab {
                background: #e8edf2;
                padding: 10px 18px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background: #ffffff;
                color: #234f8f;
                font-weight: 600;
            }
            QGroupBox {
                background: #ffffff;
                border: 1px solid #d9e0e7;
                border-radius: 6px;
                margin-top: 12px;
                padding: 14px;
                font-weight: 600;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
            }
            QPushButton {
                background: #234f8f;
                color: white;
                border: 0;
                border-radius: 4px;
                padding: 10px 14px;
                min-height: 18px;
            }
            QPushButton:hover {
                background: #1b4075;
            }
            QPushButton:disabled {
                background: #aeb9c5;
            }
            QLineEdit, QTextEdit {
                background: #ffffff;
                border: 1px solid #cbd5df;
                border-radius: 4px;
                padding: 7px;
            }
            QTableWidget {
                background: #ffffff;
                gridline-color: #e1e7ee;
                border: 1px solid #d9e0e7;
            }
            QLabel#title {
                font-size: 24px;
                font-weight: 700;
            }
            QLabel#subtitle {
                color: #66717d;
            }
            QLabel#imagePreview {
                background: #101820;
                color: #d7dee8;
                border-radius: 6px;
            }
            StatusPill {
                border-radius: 4px;
                padding: 6px 10px;
                font-weight: 600;
                background: #d9e0e7;
                color: #243140;
            }
            StatusPill[state="ok"] {
                background: #d9f0e3;
                color: #17613a;
            }
            StatusPill[state="idle"] {
                background: #e8edf2;
                color: #435160;
            }
            StatusPill[state="bad"] {
                background: #f6d8d8;
                color: #8a2424;
            }
            StatusPill[state="unknown"] {
                background: #efe7d1;
                color: #74581b;
            }
        """)


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
