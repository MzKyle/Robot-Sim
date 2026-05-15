"""OpenGL point cloud rendering components."""
import ctypes
import math
import os
from collections import deque
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory

try:
    from PySide6.QtCore import Qt, QPoint, Signal
    from PySide6.QtOpenGLWidgets import QOpenGLWidget
    from PySide6.QtWidgets import QSizePolicy
except ImportError:
    from PyQt5.QtCore import Qt, QPoint, pyqtSignal as Signal
    from PyQt5.QtOpenGLWidget import QOpenGLWidget
    from PyQt5.QtWidgets import QSizePolicy

from .utils import MAX_PREVIEW_POINTS, qt_enum


ACCUMULATION_FRAME_LIMIT = 5


class CloudRendererLibrary:
    """Wrapper for C++ point cloud rendering library."""

    def __init__(self):
        self._lib = self._load_library()
        self._configure_signatures()

    def _load_library(self):
        candidates = []
        env_path = os.environ.get("DATA_COLLECT_CLOUD_RENDERER_LIB")
        if env_path:
            candidates.append(Path(env_path))
        try:
            share_dir = Path(get_package_share_directory("data_collect_cloud_renderer"))
            candidates.append(share_dir.parent.parent / "lib" / "libdata_collect_cloud_renderer.so")
        except Exception:
            pass
        workspace = Path(__file__).resolve().parents[3]
        candidates.extend([
            workspace / "install" / "data_collect_cloud_renderer" / "lib" / "libdata_collect_cloud_renderer.so",
            workspace / "build" / "data_collect_cloud_renderer" / "libdata_collect_cloud_renderer.so",
            Path("install/data_collect_cloud_renderer/lib/libdata_collect_cloud_renderer.so"),
            Path("build/data_collect_cloud_renderer/libdata_collect_cloud_renderer.so"),
        ])

        errors = []
        for candidate in candidates:
            try:
                if candidate.exists():
                    return ctypes.CDLL(str(candidate))
            except Exception as exc:
                errors.append(f"{candidate}: {exc}")
        details = "; ".join(errors) if errors else "未找到 libdata_collect_cloud_renderer.so"
        raise RuntimeError(details)

    def _configure_signatures(self):
        self._lib.dc_cloud_renderer_create.argtypes = []
        self._lib.dc_cloud_renderer_create.restype = ctypes.c_void_p
        self._lib.dc_cloud_renderer_destroy.argtypes = [ctypes.c_void_p]
        self._lib.dc_cloud_renderer_destroy.restype = None
        self._lib.dc_cloud_renderer_initialize.argtypes = [ctypes.c_void_p]
        self._lib.dc_cloud_renderer_initialize.restype = ctypes.c_int
        self._lib.dc_cloud_renderer_resize.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
        self._lib.dc_cloud_renderer_resize.restype = ctypes.c_int
        self._lib.dc_cloud_renderer_upload_points.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_size_t,
        ]
        self._lib.dc_cloud_renderer_upload_points.restype = ctypes.c_int
        self._upload_points_rgb = getattr(self._lib, "dc_cloud_renderer_upload_points_rgb", None)
        if self._upload_points_rgb is not None:
            self._upload_points_rgb.argtypes = [
                ctypes.c_void_p,
                ctypes.POINTER(ctypes.c_float),
                ctypes.POINTER(ctypes.c_float),
                ctypes.c_size_t,
            ]
            self._upload_points_rgb.restype = ctypes.c_int
        self._upload_points_interleaved = getattr(
            self._lib,
            "dc_cloud_renderer_upload_points_interleaved",
            None,
        )
        if self._upload_points_interleaved is not None:
            self._upload_points_interleaved.argtypes = [
                ctypes.c_void_p,
                ctypes.POINTER(ctypes.c_float),
                ctypes.c_size_t,
            ]
            self._upload_points_interleaved.restype = ctypes.c_int
        self._lib.dc_cloud_renderer_draw.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
        ]
        self._lib.dc_cloud_renderer_draw.restype = ctypes.c_int
        self._lib.dc_cloud_renderer_last_error.argtypes = [ctypes.c_void_p]
        self._lib.dc_cloud_renderer_last_error.restype = ctypes.c_char_p

    def create(self):
        renderer = self._lib.dc_cloud_renderer_create()
        if not renderer:
            raise RuntimeError("无法创建点云渲染器")
        return renderer

    def destroy(self, renderer):
        self._lib.dc_cloud_renderer_destroy(renderer)

    def initialize(self, renderer):
        return bool(self._lib.dc_cloud_renderer_initialize(renderer))

    def resize(self, renderer, width, height):
        return bool(self._lib.dc_cloud_renderer_resize(renderer, width, height))

    def upload_points(self, renderer, points):
        if points.ndim != 2 or points.shape[1] < 3:
            return False
        xyz = np.ascontiguousarray(points[:, :3], dtype=np.float32)
        if xyz.shape[0] == 0:
            return True
        if points.shape[1] >= 6 and self._upload_points_rgb is not None:
            rgb = np.ascontiguousarray(points[:, 3:6], dtype=np.float32)
            xyz_pointer = xyz.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
            rgb_pointer = rgb.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
            if self._upload_points_rgb(renderer, xyz_pointer, rgb_pointer, xyz.shape[0]):
                return True
        pointer = xyz.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        return bool(self._lib.dc_cloud_renderer_upload_points(renderer, pointer, xyz.shape[0]))

    def draw(self, renderer, yaw, pitch, distance, pan_x, pan_y, point_size):
        return bool(self._lib.dc_cloud_renderer_draw(
            renderer,
            float(yaw),
            float(pitch),
            float(distance),
            float(pan_x),
            float(pan_y),
            float(point_size),
        ))

    def last_error(self, renderer):
        raw = self._lib.dc_cloud_renderer_last_error(renderer)
        return raw.decode("utf-8", errors="replace") if raw else "未知点云渲染错误"


class PointCloudOpenGLWidget(QOpenGLWidget):
    """OpenGL widget for rendering point clouds."""

    status_changed = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("cloudPreview")
        self.setMinimumHeight(220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self._library = None
        self._renderer = None
        self._initialized = False
        self._unavailable = ""
        self._pending_points = None
        self._point_count = 0
        self._last_frame_points = None
        self._accumulated_frames = deque(maxlen=ACCUMULATION_FRAME_LIMIT)
        self._accumulation_enabled = False
        self._yaw = 0.55
        self._pitch = -0.55
        self._distance = 2.8
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._point_size = 4.0
        self._last_pos = QPoint()
        self._last_device_size = (0, 0)

    def set_accumulation_enabled(self, enabled):
        """Enable or disable short-window point cloud accumulation."""
        enabled = bool(enabled)
        if self._accumulation_enabled == enabled:
            return
        self._accumulation_enabled = enabled
        self._accumulated_frames.clear()
        if enabled:
            if self._last_frame_points is not None:
                self._accumulated_frames.append(self._last_frame_points)
                self._queue_points(self._last_frame_points)
            self.status_changed.emit(f"点云累积已开启：最近 {ACCUMULATION_FRAME_LIMIT} 帧")
        else:
            if self._last_frame_points is not None:
                self._queue_points(self._last_frame_points)
            self.status_changed.emit("点云累积已关闭：当前帧")
        self.update()

    def reset_view(self):
        """Reset view to default position."""
        self._yaw = 0.55
        self._pitch = -0.55
        self._distance = 2.8
        self._pan_x = 0.0
        self._pan_y = 0.0
        self.update()

    def set_points(self, points):
        """Queue points for rendering."""
        if points is None:
            return
        points = np.asarray(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] < 3:
            self.status_changed.emit("点云数据格式无效")
            return
        if points.shape[0] == 0:
            self.status_changed.emit("点云数据为空，保留上一帧")
            return
        frame_points = np.ascontiguousarray(points[:, :6] if points.shape[1] >= 6 else points[:, :3])
        self._last_frame_points = frame_points
        if self._accumulation_enabled:
            self._accumulated_frames.append(frame_points)
            frame_points = self._merged_accumulated_points()
            if frame_points is None:
                return
        self._queue_points(frame_points)
        self.update()

    def initializeGL(self):
        """Initialize OpenGL renderer."""
        try:
            self._library = CloudRendererLibrary()
            self._renderer = self._library.create()
            if not self._library.initialize(self._renderer):
                self._set_unavailable(self._library.last_error(self._renderer))
                return
            self._initialized = True
            self._unavailable = ""
            self._resize_renderer()
            self.status_changed.emit("点云渲染器已启动")
        except Exception as exc:
            self._set_unavailable(str(exc))

    def resizeGL(self, width, height):
        """Handle resize events."""
        self._resize_renderer(width, height)

    def paintGL(self):
        """Render the point cloud."""
        if not self._initialized or not self._renderer:
            return
        self._resize_renderer()
        if self._pending_points is not None:
            points = self._pending_points
            self._pending_points = None
            if not self._library.upload_points(self._renderer, points):
                self._set_unavailable(self._library.last_error(self._renderer))
                return
            self.status_changed.emit(f"点云在线：{self._point_count} 点 / {self._display_mode_text()}")
        if not self._library.draw(
            self._renderer,
            self._yaw,
            self._pitch,
            self._distance,
            self._pan_x,
            self._pan_y,
            self._auto_point_size(),
        ):
            self._set_unavailable(self._library.last_error(self._renderer))

    def _queue_points(self, points):
        """Queue an already-normalized point array for upload."""
        self._pending_points = np.ascontiguousarray(points, dtype=np.float32)
        self._point_count = self._pending_points.shape[0]

    def _merged_accumulated_points(self):
        """Merge recent frames and keep GPU uploads bounded."""
        if not self._accumulated_frames:
            return None
        if len(self._accumulated_frames) == 1:
            return self._accumulated_frames[0]
        points = np.concatenate(list(self._accumulated_frames), axis=0)
        if points.shape[0] > MAX_PREVIEW_POINTS:
            indices = np.linspace(0, points.shape[0] - 1, MAX_PREVIEW_POINTS, dtype=np.int64)
            points = points[indices]
        return np.ascontiguousarray(points, dtype=np.float32)

    def _display_mode_text(self):
        if self._accumulation_enabled:
            return f"累积 {len(self._accumulated_frames)} 帧"
        return "当前帧"

    def _auto_point_size(self):
        """Choose a RViz-like point size based on visible density."""
        count = max(1, self._point_count)
        area = max(1, self.width() * self.height())
        density_size = math.sqrt(area / count) * 3.2
        count_t = min(1.0, max(0.0, (math.log10(count) - 3.5) / 2.5))
        count_size = 7.0 - 4.5 * count_t
        logical_size = max(2.5, min(7.0, max(density_size, count_size)))
        ratio = self.devicePixelRatioF() if hasattr(self, "devicePixelRatioF") else self.devicePixelRatio()
        return logical_size * max(1.0, float(ratio))

    def _resize_renderer(self, width=None, height=None):
        """Resize the GL viewport using physical framebuffer pixels."""
        if not self._initialized or not self._renderer:
            return
        logical_width = self.width() if width is None else width
        logical_height = self.height() if height is None else height
        ratio = self.devicePixelRatioF() if hasattr(self, "devicePixelRatioF") else self.devicePixelRatio()
        if width is not None and height is not None and ratio > 1.0:
            if logical_width > self.width() or logical_height > self.height():
                ratio = 1.0
        device_width = max(1, int(round(logical_width * ratio)))
        device_height = max(1, int(round(logical_height * ratio)))
        size = (device_width, device_height)
        if size == self._last_device_size:
            return
        if not self._library.resize(self._renderer, device_width, device_height):
            self._set_unavailable(self._library.last_error(self._renderer))
            return
        self._last_device_size = size

    def closeEvent(self, event):
        """Handle close event."""
        self._destroy_renderer()
        super().closeEvent(event)

    def __del__(self):
        try:
            self._destroy_renderer()
        except Exception:
            pass

    def _destroy_renderer(self):
        """Clean up renderer resources."""
        if self._renderer and self._library:
            try:
                if self.isValid():
                    self.makeCurrent()
                    try:
                        self._library.destroy(self._renderer)
                    finally:
                        self.doneCurrent()
                else:
                    self._library.destroy(self._renderer)
            except Exception:
                pass
        self._renderer = None
        self._initialized = False

    def _set_unavailable(self, message):
        """Mark renderer as unavailable."""
        self._unavailable = message or "点云渲染不可用"
        self._initialized = False
        self.status_changed.emit(f"点云不可用：{self._unavailable}")

    def mousePressEvent(self, event):
        """Handle mouse press events."""
        self._last_pos = event.pos()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        """Handle mouse move events."""
        delta = event.pos() - self._last_pos
        self._last_pos = event.pos()
        buttons = event.buttons()
        left = qt_enum(Qt, "MouseButton", "LeftButton")
        right = qt_enum(Qt, "MouseButton", "RightButton")
        middle = qt_enum(Qt, "MouseButton", "MiddleButton")
        if buttons & left:
            self._yaw += delta.x() * 0.008
            self._pitch = max(-1.45, min(1.45, self._pitch + delta.y() * 0.008))
            self.update()
        elif buttons & right or buttons & middle:
            scale = 0.0025 * self._distance
            self._pan_x += delta.x() * scale
            self._pan_y -= delta.y() * scale
            self.update()
        super().mouseMoveEvent(event)

    def wheelEvent(self, event):
        """Handle mouse wheel events."""
        delta = event.angleDelta().y()
        if delta:
            factor = 0.82 if delta > 0 else 1.20
            self._distance = max(0.35, min(40.0, self._distance * factor))
            self.update()
        event.accept()
