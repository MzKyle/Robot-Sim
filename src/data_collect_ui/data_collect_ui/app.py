"""Data Collection UI Application Launcher."""

import sys

try:
    from PySide6.QtWidgets import QApplication
except ImportError:
    try:
        from PyQt5.QtWidgets import QApplication
    except ImportError as exc:
        print("缺少 Qt Python 绑定，无法启动桌面界面。", file=sys.stderr)
        print("安装示例：sudo apt install python3-pyqt5", file=sys.stderr)
        print("或安装 PySide6：python3 -m pip install --user PySide6", file=sys.stderr)
        print(f"原始错误：{exc}", file=sys.stderr)
        raise SystemExit(2)

from .main import MainWindow, enable_high_dpi_scaling


class AppWindow(MainWindow):
    """Application window with extended functionality for future quality panel integration."""
    
    def __init__(self):
        """Initialize the application window."""
        super().__init__()


def main():
    """Application entry point."""
    enable_high_dpi_scaling()
    app = QApplication(sys.argv)
    window = AppWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
