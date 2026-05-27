"""ROS2 communication bridge for data collection UI."""
import queue
import time

try:
    from PySide6.QtCore import QThread, Signal
except ImportError:
    from PyQt5.QtCore import QThread, pyqtSignal as Signal

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_srvs.srv import Empty
from std_srvs.srv import Trigger
from weld_interface.msg import DataCollectStatus, FanucRobotInfo
from weld_interface.srv import SetCollectionTask

from .utils import (
    DATA_COLLECT_ACTIVATE,
    DATA_COLLECT_DEACTIVATE,
    DATA_COLLECT_SET_TASK,
    IMAGE_TOPIC,
    POINT_CLOUD_TOPIC,
    RELOAD_CAMERA_3D_CONFIG,
    START_FIX_SCAN,
    STOP_FIX_SCAN,
    image_msg_to_qimage,
    pointcloud2_msg_to_xyz,
)


class RosBridge(QThread):
    """ROS2 communication bridge running in a QThread."""

    status_received = Signal(dict)
    fanuc_received = Signal(dict)
    image_received = Signal(object)
    point_cloud_received = Signal(object)
    services_received = Signal(dict)
    log_received = Signal(str)
    ros_state_changed = Signal(bool)

    def __init__(self):
        super().__init__()
        self._commands = queue.Queue()
        self._running = True
        self._node = None
        self._clients = {}
        self._request_factories = {}
        self._pending_services = set()
        self._last_service_emit = 0.0
        self._last_image_emit = 0.0
        self._last_cloud_emit = 0.0

    def stop(self):
        """Stop the ROS bridge thread."""
        self._running = False
        self._commands.put(("stop", None))

    def call_service(self, service_name):
        """Call a service without UI-provided request data."""
        self._commands.put(("call_service", service_name))

    def set_task(self, task):
        """Set collection task."""
        self._commands.put(("set_task", task))

    def set_parameters(self, node_name, parameters):
        """Set node parameters."""
        self._commands.put(("set_parameters", {
            "node_name": node_name,
            "parameters": parameters,
        }))

    def run(self):
        """Main ROS bridge loop."""
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
        self._request_factories = {
            DATA_COLLECT_ACTIVATE: Empty.Request,
            DATA_COLLECT_DEACTIVATE: Empty.Request,
            START_FIX_SCAN: Empty.Request,
            STOP_FIX_SCAN: Empty.Request,
            RELOAD_CAMERA_3D_CONFIG: Trigger.Request,
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
        self._node.create_subscription(
            PointCloud2,
            POINT_CLOUD_TOPIC,
            self._on_point_cloud,
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
        """Process queued commands from UI thread."""
        while True:
            try:
                command, payload = self._commands.get_nowait()
            except queue.Empty:
                break

            if command == "stop":
                return
            if command == "call_service":
                self._call_service(payload)
            if command == "set_task":
                self._call_set_task(payload)
            if command == "set_parameters":
                self._call_set_parameters(payload)

    def _call_service(self, service_name):
        """Call a service that does not need UI-provided request data."""
        client = self._clients.get(service_name)
        if client is None:
            self.log_received.emit(f"未知服务：{service_name}")
            return
        request_factory = self._request_factories.get(service_name)
        if request_factory is None:
            self.log_received.emit(f"不支持的服务请求类型：{service_name}")
            return

        if not client.service_is_ready():
            self.log_received.emit(f"服务不可用：{service_name}")
            return
        if service_name in self._pending_services:
            self.log_received.emit(f"服务请求进行中：{service_name}")
            return

        self._pending_services.add(service_name)
        future = client.call_async(request_factory())
        future.add_done_callback(lambda done: self._on_service_done(service_name, done))
        self.log_received.emit(f"已发送服务请求：{service_name}")

    def _call_set_parameters(self, payload):
        """Call set_parameters service."""
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
        """Create a Parameter message from name and value."""
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
        """Handle set_parameters response."""
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
        """Call set_collection_task service."""
        client = self._clients.get(DATA_COLLECT_SET_TASK)
        if client is None or not client.service_is_ready():
            self.log_received.emit(f"服务不可用：{DATA_COLLECT_SET_TASK}")
            return
        if DATA_COLLECT_SET_TASK in self._pending_services:
            self.log_received.emit("采集任务请求进行中")
            return

        request = SetCollectionTask.Request()
        request.task_id = task["task_id"]
        request.workpiece_id = task["workpiece_id"]
        request.weld_seam_id = task["weld_seam_id"]
        request.operator_name = task["operator_name"]
        request.shift = task["shift"]
        request.notes = task["notes"]
        self._pending_services.add(DATA_COLLECT_SET_TASK)
        future = client.call_async(request)
        future.add_done_callback(lambda done: self._on_service_done(DATA_COLLECT_SET_TASK, done))
        self.log_received.emit("已发送采集任务信息")

    def _on_service_done(self, service_name, future):
        """Handle service call response."""
        try:
            response = future.result()
        except Exception as exc:
            self.log_received.emit(f"服务调用失败：{service_name}，{exc}")
            self._pending_services.discard(service_name)
            return
        self._pending_services.discard(service_name)
        message = getattr(response, "message", "")
        if message:
            self.log_received.emit(f"服务调用完成：{service_name}，{message}")
        else:
            self.log_received.emit(f"服务调用完成：{service_name}")

    def _emit_service_state_periodically(self):
        """Periodically emit service readiness state."""
        now = time.monotonic()
        if now - self._last_service_emit < 1.0:
            return
        self._last_service_emit = now
        self.services_received.emit({
            name: client.service_is_ready()
            for name, client in self._clients.items()
        })

    def _on_status(self, msg):
        """Handle DataCollectStatus message."""
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
        """Handle FanucRobotInfo message."""
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
        """Handle Image message."""
        now = time.monotonic()
        if now - self._last_image_emit < 0.15:
            return
        self._last_image_emit = now

        image = image_msg_to_qimage(msg)
        if image is not None:
            self.image_received.emit(image)

    def _on_point_cloud(self, msg):
        """Handle PointCloud2 message."""
        now = time.monotonic()
        if now - self._last_cloud_emit < 0.2:
            return
        self._last_cloud_emit = now
        points, error = pointcloud2_msg_to_xyz(msg)
        if error:
            self.log_received.emit(error)
            return
        self.point_cloud_received.emit(points)
