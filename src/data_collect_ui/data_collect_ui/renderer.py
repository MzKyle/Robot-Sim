"""OpenGL point cloud rendering components."""
import ctypes
import os
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory

try:
    from PySide6.QtCore import Qt, QPoint, QSize, Signal
    from PySide6.QtOpenGLWidgets import QOpenGLWidget
    from PySide6.QtWidgets import QSizePolicy
except ImportError:
    from PyQt5.QtCore import Qt, QPoint, QSize, pyqtSignal as Signal
    from PyQt5.QtOpenGLWidget import QOpenGLWidget
    from PyQt5.QtWidgets import QSizePolicy

from .utils import qt_enum, qimage_format


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
        self._lib.dc_cloud_renderer_show_debug_points.argtypes = [ctypes.c_void_p]
        self._lib.dc_cloud_renderer_show_debug_points.restype = ctypes.c_int
        self._lib.dc_cloud_renderer_set_diagnostic_mode.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self._lib.dc_cloud_renderer_set_diagnostic_mode.restype = None
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
        pointer = points.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        return bool(self._lib.dc_cloud_renderer_upload_points(renderer, pointer, points.shape[0]))

    def show_debug_points(self, renderer):
        return bool(self._lib.dc_cloud_renderer_show_debug_points(renderer))

    def set_diagnostic_mode(self, renderer, enabled):
        self._lib.dc_cloud_renderer_set_diagnostic_mode(renderer, 1 if enabled else 0)

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
        self._yaw = 0.55
        self._pitch = -0.55
        self._distance = 0.95
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._point_size = 4.0
        self._last_pos = QPoint()
        self._diagnostic_mode = False

    def reset_view(self):
        """Reset view to default position."""
        self._yaw = 0.55
        self._pitch = -0.55
        self._distance = 0.95
        self._pan_x = 0.0
        self._pan_y = 0.0
        self.update()

    def set_diagnostic_mode(self, enabled):
        """Enable/disable diagnostic visualization mode."""
        if not self._initialized or not self._renderer:
            self.status_changed.emit("OpenGL 诊断等待渲染器初始化")
            return
        self.makeCurrent()
        try:
            self._diagnostic_mode = bool(enabled)
            self._library.set_diagnostic_mode(self._renderer, self._diagnostic_mode)
        finally:
            self.doneCurrent()
        self.status_changed.emit("OpenGL 诊断模式：应显示中央红色矩形" if enabled else "OpenGL 诊断模式已关闭")
        self.update()

    def show_debug_points(self):
        """Show debug points overlay."""
        if not self._initialized or not self._renderer:
            self.status_changed.emit("点云调试点阵等待渲染器初始化")
            return
        self.makeCurrent()
        try:
            self._diagnostic_mode = False
            self._library.set_diagnostic_mode(self._renderer, False)
            if not self._library.show_debug_points(self._renderer):
                self._set_unavailable(self._library.last_error(self._renderer))
                return
        finally:
            self.doneCurrent()
        self.status_changed.emit("点云调试点阵已载入")
        self.update()

    def set_points(self, points):
        """Queue points for rendering."""
        if points is None:
            return
        self._pending_points = np.ascontiguousarray(points, dtype=np.float32)
        self._point_count = self._pending_points.shape[0]
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
            self.status_changed.emit("点云渲染器已启动")
        except Exception as exc:
            self._set_unavailable(str(exc))

    def resizeGL(self, width, height):
        """Handle resize events."""
        if self._initialized and self._renderer:
            if not self._library.resize(self._renderer, width, height):
                self._set_unavailable(self._library.last_error(self._renderer))

    def paintGL(self):
        """Render the point cloud."""
        if not self._initialized or not self._renderer:
            return
        if self._pending_points is not None:
            points = self._pending_points
            self._pending_points = None
            if not self._library.upload_points(self._renderer, points):
                self._set_unavailable(self._library.last_error(self._renderer))
                return
            self.status_changed.emit(f"点云在线：{self._point_count} 点")
        if not self._library.draw(
            self._renderer,
            self._yaw,
            self._pitch,
            self._distance,
            self._pan_x,
            self._pan_y,
            self._point_size,
        ):
            self._set_unavailable(self._library.last_error(self._renderer))

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
            self._distance = max(0.08, min(12.0, self._distance * factor))
            self.update()
        event.accept()
