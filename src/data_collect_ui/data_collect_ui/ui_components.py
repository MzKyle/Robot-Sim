"""UI components for data collection interface."""

try:
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QLabel, QSizePolicy
except ImportError:
    from PyQt5.QtCore import Qt
    from PyQt5.QtWidgets import QLabel, QSizePolicy


class StatusPill(QLabel):
    """Status indicator pill widget."""

    def __init__(self, text="未知"):
        super().__init__(text)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumWidth(76)
        self.setMinimumHeight(26)
        self.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
        self.setProperty("state", "unknown")

    def set_state(self, text, state):
        """Update the status pill with new text and state."""
        self.setText(text)
        self.setProperty("state", state)
        self.style().unpolish(self)
        self.style().polish(self)
