"""Modern simulated servo-drive commissioning workstation."""

from __future__ import annotations

import json
import math
import random
import sys
from collections import deque
from datetime import datetime
from pathlib import Path

from PyQt5.QtCore import QEvent, QPoint, QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QLinearGradient, QPainter, QPainterPath, QPen
from PyQt5.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
    QFrame, QGraphicsScene, QGraphicsView, QGridLayout, QHBoxLayout, QHeaderView,
    QLabel, QLineEdit, QListWidget, QListWidgetItem, QMainWindow, QMessageBox,
    QProgressBar, QPushButton, QSizeGrip, QScrollArea, QSpinBox, QTableWidget,
    QTableWidgetItem, QTextEdit, QToolButton, QVBoxLayout, QWidget,
)

from servo_animations import AnimatedStackedWidget
from servo_core import PARAMETERS, ServoModel, clamp
from servo_visuals import TechSchematic

SHELL, PANEL, PANEL_ALT = "#0d0c10", "#151419", "#1c1b21"
BORDER, TEXT, MUTED = "#45424a", "#f4f1f3", "#9f9aa2"
CYAN, BLUE, AMBER, RED, GREEN = "#c92f4d", "#8f99a4", "#d9ae68", "#e85a70", "#59c98b"
DESIGN_WIDTH, DESIGN_HEIGHT = 1440, 900


class ResponsiveView(QGraphicsView):
    """Uniformly scales the complete workstation while preserving its layout."""

    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("responsive-view")
        self.setScene(QGraphicsScene(self))
        self.setFrameShape(QFrame.NoFrame)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setAlignment(Qt.AlignCenter)
        self.setBackgroundBrush(QColor("#08080a"))
        self.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing | QPainter.SmoothPixmapTransform)
        self.surface = None
        self.proxy = None

    def set_surface(self, surface: QWidget) -> None:
        self.surface = surface
        self.proxy = self.scene().addWidget(surface)
        self.scene().setSceneRect(QRectF(0, 0, DESIGN_WIDTH, DESIGN_HEIGHT))
        self.fit_surface()

    def fit_surface(self) -> None:
        if self.proxy is None or self.viewport().width() <= 0 or self.viewport().height() <= 0:
            return
        scale = min(
            self.viewport().width() / DESIGN_WIDTH,
            self.viewport().height() / DESIGN_HEIGHT,
        )
        self.resetTransform()
        self.scale(scale, scale)
        self.centerOn(self.sceneRect().center())

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self.fit_surface()


class TitleBar(QFrame):
    def __init__(self, window: "ServoWindow") -> None:
        super().__init__()
        self.window, self.drag_origin = window, None
        self.setObjectName("titlebar")
        self.setFixedHeight(52)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(18, 0, 8, 0)
        mark = QLabel("A")
        mark.setObjectName("brand-mark")
        name = QLabel("AUUSN  /  MOTOR STUDIO")
        name.setObjectName("app-name")
        layout.addWidget(mark)
        layout.addSpacing(10)
        layout.addWidget(name)
        layout.addStretch()
        window.title_device = QLabel("SIMULATION / NO DEVICE")
        window.title_device.setObjectName("title-context")
        window.title_alarm = QLabel("0 ALARMS")
        window.title_alarm.setObjectName("title-alarm")
        layout.addWidget(window.title_device)
        layout.addSpacing(12)
        layout.addWidget(window.title_alarm)
        layout.addSpacing(10)
        window.fx_button = QToolButton()
        window.fx_button.setText("FX OFF")
        window.fx_button.setObjectName("fx-button")
        window.fx_button.setToolTip("开启或关闭全窗环境特效")
        window.fx_button.setCheckable(True)
        window.fx_button.setChecked(False)
        window.fx_button.setFixedSize(54, 32)
        window.fx_button.toggled.connect(window.toggle_effects)
        layout.addWidget(window.fx_button)
        layout.addSpacing(6)
        window.theme_button = QToolButton()
        window.theme_button.setText("LIGHT")
        window.theme_button.setObjectName("theme-button")
        window.theme_button.setToolTip("切换明亮 / 深色界面")
        window.theme_button.setFixedSize(58, 32)
        window.theme_button.clicked.connect(window.toggle_theme)
        layout.addWidget(window.theme_button)
        layout.addSpacing(18)
        for text, slot, object_name in (("-", window.showMinimized, "window-button"), ("□", window.toggle_maximized, "window-button"), ("×", window.close, "close-button")):
            button = QToolButton()
            button.setText(text)
            button.setObjectName(object_name)
            button.setFixedSize(38, 32)
            button.clicked.connect(slot)
            layout.addWidget(button)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self.drag_origin = event.globalPos() - self.window.frameGeometry().topLeft()

    def mouseMoveEvent(self, event) -> None:
        if self.drag_origin is not None and event.buttons() & Qt.LeftButton and not self.window.isMaximized():
            self.window.move(event.globalPos() - self.drag_origin)

    def mouseReleaseEvent(self, event) -> None:
        self.drag_origin = None
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self.window.toggle_maximized()


class ScopeChart(QWidget):
    COLORS = (CYAN, BLUE, AMBER, "#b58cff")

    def __init__(self) -> None:
        super().__init__()
        self.data = [deque(maxlen=180) for _ in range(4)]
        self.visible = [True, True, True, False]
        self.theme_name = "dark"
        self.dirty = True
        self.setMinimumHeight(270)
        self.setAttribute(Qt.WA_OpaquePaintEvent)

    def set_theme(self, theme_name: str) -> None:
        self.theme_name = theme_name
        self.dirty = True
        self.update()

    def add_sample(self, values: tuple[float, float, float, float]) -> None:
        for series, value in zip(self.data, values):
            series.append(value)
        self.dirty = True

    def clear(self) -> None:
        self.data = [deque(maxlen=180) for _ in range(4)]
        self.dirty = True
        self.update()

    def flush(self) -> None:
        if self.dirty and self.isVisible():
            self.update()

    def paintEvent(self, event) -> None:
        del event
        self.dirty = False
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        chart_background = "#f5f4f6" if self.theme_name == "light" else "#100f13"
        grid_color = "#d0cdd1" if self.theme_name == "light" else "#2d2d35"
        axis_color = "#647382" if self.theme_name == "light" else MUTED
        background = QLinearGradient(0, 0, 0, self.height())
        background.setColorAt(0, QColor("#ffffff" if self.theme_name == "light" else "#19181d"))
        background.setColorAt(0.08, QColor(chart_background))
        background.setColorAt(1, QColor("#ebe9ec" if self.theme_name == "light" else "#0d0c10"))
        painter.fillRect(self.rect(), background)
        area = QRectF(self.rect()).adjusted(48, 18, -18, -35)
        painter.setPen(QPen(QColor(grid_color), 1, Qt.DotLine))
        for i in range(9):
            x = area.left() + area.width() * i / 8
            painter.drawLine(QPointF(x, area.top()), QPointF(x, area.bottom()))
        for i in range(7):
            y = area.top() + area.height() * i / 6
            painter.drawLine(QPointF(area.left(), y), QPointF(area.right(), y))
        painter.setPen(QColor(axis_color))
        painter.setFont(QFont("Segoe UI", 8))
        painter.drawText(10, int(area.top() + 5), "+100")
        painter.drawText(28, int(area.center().y() + 4), "0")
        painter.drawText(10, int(area.bottom()), "-100")
        painter.drawText(int(area.right() - 66), self.height() - 12, "TIME / ms")
        for index, series in enumerate(self.data):
            if not self.visible[index] or len(series) < 2:
                continue
            path = QPainterPath()
            for i, value in enumerate(series):
                x = area.left() + area.width() * i / (len(series) - 1)
                y = area.center().y() - clamp(value, -100.0, 100.0) * area.height() / 200.0
                path.moveTo(x, y) if i == 0 else path.lineTo(x, y)
            if index == 0:
                fill = QPainterPath(path)
                fill.lineTo(area.right(), area.center().y())
                fill.lineTo(area.left(), area.center().y())
                gradient = QLinearGradient(area.topLeft(), area.bottomLeft())
                gradient.setColorAt(0, QColor(201, 47, 77, 44))
                gradient.setColorAt(1, QColor(201, 47, 77, 0))
                painter.fillPath(fill, gradient)
            painter.setPen(QPen(QColor(self.COLORS[index]), 1.8, Qt.SolidLine, Qt.RoundCap))
            painter.drawPath(path)


class MotionTracker(QWidget):
    """Low-cost command/feedback view for the overview page."""

    def __init__(self) -> None:
        super().__init__()
        self.theme_name = "dark"
        self.values = ("位置模式", 0.0, 0.0, 0.0, 0.0, 800.0, False)
        self.setMinimumHeight(300)
        self.setAttribute(Qt.WA_OpaquePaintEvent)

    def set_theme(self, theme_name: str) -> None:
        self.theme_name = theme_name
        self.update()

    def set_values(self, mode: str, target: float, position: float, speed: float, current: float, speed_limit: float, enabled: bool) -> None:
        values = (mode, round(target, 3), round(position, 3), round(speed, 3), round(current, 3), round(speed_limit, 3), enabled)
        if values != self.values:
            self.values = values
            self.update()

    def paintEvent(self, event) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        dark = self.theme_name == "dark"
        background = QColor("#100f13" if dark else "#f5f4f6")
        grid = QColor("#2d2d35" if dark else "#d0cdd1")
        muted = QColor(MUTED if dark else "#647382")
        surface = QLinearGradient(0, 0, 0, self.height())
        surface.setColorAt(0, QColor("#1c1b20" if dark else "#ffffff"))
        surface.setColorAt(0.09, background)
        surface.setColorAt(1, QColor("#0d0c10" if dark else "#eae8eb"))
        painter.fillRect(self.rect(), surface)
        mode, target, position, speed, current, speed_limit, enabled = self.values

        center = QPointF(self.width() * 0.30, self.height() * 0.48)
        radius = min(self.width() * 0.19, self.height() * 0.32)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(grid, 7, Qt.SolidLine, Qt.RoundCap))
        painter.drawArc(QRectF(center.x() - radius, center.y() - radius, radius * 2, radius * 2), 225 * 16, -270 * 16)

        if mode == "位置模式":
            command_ratio = ((target + 180.0) % 360.0) / 360.0
            feedback_ratio = ((position + 180.0) % 360.0) / 360.0
            unit, feedback, error = "°", position, target - position
        elif mode == "速度模式":
            command_ratio = (clamp(target / max(speed_limit, 1.0), -1.0, 1.0) + 1.0) / 2.0
            feedback_ratio = (clamp(speed / max(speed_limit, 1.0), -1.0, 1.0) + 1.0) / 2.0
            unit, feedback, error = "rpm", speed, target - speed
        else:
            command_ratio = (clamp(target / 12.0, -1.0, 1.0) + 1.0) / 2.0
            feedback_ratio = (clamp(current / 12.0, -1.0, 1.0) + 1.0) / 2.0
            unit, feedback, error = "A", current, target - current

        def point_for(ratio: float, scale: float) -> QPointF:
            angle = math.radians(225.0 - ratio * 270.0)
            return QPointF(center.x() + math.cos(angle) * radius * scale, center.y() - math.sin(angle) * radius * scale)

        painter.setPen(QPen(QColor(BLUE), 3, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(center, point_for(command_ratio, 0.82))
        painter.setPen(QPen(QColor(CYAN), 2, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(center, point_for(feedback_ratio, 0.68))
        painter.setBrush(QColor(GREEN if enabled else MUTED))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(center, 5, 5)

        painter.setFont(QFont("Segoe UI", 9, QFont.Bold))
        painter.setPen(muted)
        painter.drawText(QRectF(center.x() - radius, center.y() + radius * 0.42, radius * 2, 25), Qt.AlignCenter, mode)

        right = QRectF(self.width() * 0.54, self.height() * 0.18, self.width() * 0.40, self.height() * 0.66)
        rows = (("目标", target, BLUE), ("反馈", feedback, CYAN), ("误差", error, AMBER))
        painter.setFont(QFont("Segoe UI", 9))
        for index, (label, value, color) in enumerate(rows):
            y = right.top() + index * right.height() / 3
            painter.setPen(muted)
            painter.drawText(QRectF(right.left(), y, right.width() * 0.35, 28), Qt.AlignVCenter, label)
            painter.setPen(QColor(color))
            painter.setFont(QFont("Segoe UI", 13, QFont.Bold))
            painter.drawText(QRectF(right.left() + right.width() * 0.34, y, right.width() * 0.66, 28), Qt.AlignRight | Qt.AlignVCenter, f"{value:.1f} {unit}")
            painter.setFont(QFont("Segoe UI", 9))
            line_y = y + 34
            painter.setPen(QPen(grid, 3, Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(QPointF(right.left(), line_y), QPointF(right.right(), line_y))
            magnitude = min(abs(value) / max(abs(target), abs(feedback), 1.0), 1.0)
            painter.setPen(QPen(QColor(color), 3, Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(QPointF(right.left(), line_y), QPointF(right.left() + right.width() * magnitude, line_y))


class StatusPill(QLabel):
    def __init__(self) -> None:
        super().__init__()
        self.theme_name = "dark"
        self.state_name = "off"
        self.state_text = ""

    def set_state(self, text: str, state: str) -> None:
        self.state_text, self.state_name = text, state
        dark = {"ok": (GREEN, "#10251c"), "run": (CYAN, "#2a1017"), "warn": (AMBER, "#2c2516"), "off": (MUTED, "#1a191e")}
        light = {"ok": ("#18845b", "#e8f7ef"), "run": ("#b31536", "#fbe8ed"), "warn": ("#9a650d", "#fff4dc"), "off": ("#69656c", "#ecebed")}
        colors = light if self.theme_name == "light" else dark
        color, background = colors[state]
        self.setText(f"●  {text}")
        self.setStyleSheet(f"color:{color};background:{background};border:1px solid {color};border-radius:12px;padding:5px 11px;font-weight:700;")

    def set_theme(self, theme_name: str) -> None:
        self.theme_name = theme_name
        self.set_state(self.state_text, self.state_name)


class AmbientOverlay(QWidget):
    """Mouse-transparent, full-window ambient effects layer."""

    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent)
        self.theme_name = "dark"
        self.phase = 0.0
        self.effects_enabled = False
        self.connected = False
        self.running = False
        rng = random.Random(431)
        self.nodes = [(rng.random(), rng.random(), rng.uniform(0.0003, 0.0011)) for _ in range(16)]
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setAttribute(Qt.WA_NoSystemBackground)
        self.setVisible(False)

    def advance(self) -> None:
        if self.connected:
            self.phase = (self.phase + (0.006 if self.running else 0.0025)) % 1.0
        self.update()

    def set_operating_state(self, connected: bool, running: bool) -> None:
        self.connected = connected
        self.running = running

    def set_theme(self, theme_name: str) -> None:
        self.theme_name = theme_name
        self.update()

    def set_effects_enabled(self, enabled: bool) -> None:
        self.effects_enabled = enabled
        self.setVisible(enabled)

    def paintEvent(self, event) -> None:
        del event
        if not self.effects_enabled:
            return
        painter = QPainter(self)
        width, height = self.width(), self.height()
        if width <= 0 or height <= 0:
            return

        dark = self.theme_name == "dark"
        line_color = QColor(201, 47, 77, 9 if dark else 7)
        node_color = QColor(191, 202, 214, 66 if dark else 48)
        link_color = QColor(201, 47, 77, 22 if dark else 18)
        beam_color = QColor(218, 65, 94, 54 if dark else 42)
        corner_color = QColor(191, 202, 214, 72 if dark else 58)

        painter.setPen(QPen(line_color, 1))
        for y in range(6, height, 16):
            painter.drawLine(0, y, width, y)

        if self.connected:
            scan_y = int(self.phase * height)
            painter.setPen(QPen(beam_color, 1))
            painter.drawLine(0, scan_y, width, scan_y)
            painter.setPen(QPen(QColor(201, 47, 77, 16 if dark else 12), 3))
            painter.drawLine(0, min(height - 1, scan_y + 2), width, min(height - 1, scan_y + 2))

        points: list[QPointF] = []
        for x, y, speed in self.nodes:
            px = ((x + self.phase * speed * 120.0) % 1.0) * width
            py = ((y + math.sin((self.phase + x) * math.tau) * 0.012) % 1.0) * height
            points.append(QPointF(px, py))
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QPen(link_color, 1))
        for index, first in enumerate(points):
            for second in points[index + 1:]:
                dx, dy = first.x() - second.x(), first.y() - second.y()
                if dx * dx + dy * dy < 135 * 135:
                    painter.drawLine(first, second)
        painter.setPen(Qt.NoPen)
        painter.setBrush(node_color)
        for index, point in enumerate(points):
            radius = 2.2 if index % 5 == 0 else 1.2
            painter.drawEllipse(point, radius, radius)

        margin, arm = 9, 34
        painter.setPen(QPen(corner_color, 1.2))
        for x, y, sx, sy in ((margin, margin, 1, 1), (width - margin, margin, -1, 1), (margin, height - margin, 1, -1), (width - margin, height - margin, -1, -1)):
            painter.drawLine(x, y, x + sx * arm, y)
            painter.drawLine(x, y, x, y + sy * arm)


class ServoWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.model = ServoModel()
        self.theme_name = "dark"
        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_simulation)
        self.scope_timer = QTimer(self)
        self.scope_timer.setInterval(50)
        self.scope_timer.timeout.connect(self.sample_scope)
        self.scope_running = True
        self.scope_trigger_mode = "自动"
        self.scope_trigger_remaining = -1
        self.scope_trigger_ready = True
        self.scope_clock = 0.0
        self.scope_last_trigger: float | None = None
        self.fault_sequence = 0
        self.animation_frame = 0
        self.last_di_states: tuple[bool, ...] | None = None
        self.simulation_tick = 0
        self.control_mode = "位置模式"
        self.mode_targets = {"位置模式": 90.0, "速度模式": 800.0, "电流模式": 2.0}
        self.raw_di_states = [False] * 6
        self.filtered_di_states = [False] * 6
        self.di_filter_generation = [0] * 6
        self.torque_limit_active = False
        self.render_timer = QTimer(self)
        self.render_timer.setTimerType(Qt.PreciseTimer)
        self.render_timer.setInterval(33)
        self.render_timer.timeout.connect(self.update_animations)
        self.firmware_timer = QTimer(self)
        self.firmware_timer.setInterval(90)
        self.firmware_timer.timeout.connect(self.advance_firmware)
        self.setWindowTitle("AUUSN Motor Studio")
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Window)
        self.resize(DESIGN_WIDTH, DESIGN_HEIGHT)
        self.setMinimumSize(800, 500)
        self.pages = AnimatedStackedWidget(duration=220)
        self.scope, self.log_view = ScopeChart(), QTextEdit()
        self.motion_tracker = MotionTracker()
        self.nav_buttons: list[QPushButton] = []
        self.connection_pills: list[StatusPill] = []
        self.build_ui()
        self.timer.start()
        self.scope_timer.start()
        self.render_timer.start()
        self.log("系统已启动，当前为安全仿真模式。")

    def toggle_maximized(self) -> None:
        self.showNormal() if self.isMaximized() else self.showMaximized()

    def build_ui(self) -> None:
        host = QFrame()
        host.setObjectName("responsive-host")
        self.setCentralWidget(host)
        host_layout = QVBoxLayout(host)
        host_layout.setContentsMargins(0, 0, 0, 0)
        self.design_view = ResponsiveView()
        host_layout.addWidget(self.design_view)

        shell = QFrame()
        shell.setObjectName("window-shell")
        shell.setFixedSize(DESIGN_WIDTH, DESIGN_HEIGHT)
        outer = QVBoxLayout(shell)
        outer.setContentsMargins(1, 1, 1, 1)
        outer.setSpacing(0)
        outer.addWidget(TitleBar(self))
        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)
        body.addWidget(self.build_sidebar())
        body.addWidget(self.pages, 1)
        outer.addLayout(body, 1)
        grip = QHBoxLayout()
        grip.addStretch()
        grip.addWidget(QSizeGrip(shell))
        outer.addLayout(grip)
        for builder in (self.build_dashboard, self.build_scope_page, self.build_parameter_page, self.build_io_page, self.build_communication_page, self.build_diagnostics_page):
            page = builder()
            page.setMinimumSize(1040, 790)
            scroller = QScrollArea()
            scroller.setObjectName("page-scroll")
            scroller.setWidgetResizable(True)
            scroller.setFrameShape(QFrame.NoFrame)
            scroller.setWidget(page)
            self.pages.addWidget(scroller)
        self.ambient_overlay = AmbientOverlay(shell)
        self.ambient_overlay.setGeometry(shell.rect())
        self.ambient_overlay.raise_()
        self.design_surface = shell
        self.design_view.set_surface(shell)
        style = self.stylesheet(self.theme_name)
        self.setStyleSheet(style)
        self.design_surface.setStyleSheet(style)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        if hasattr(self, "ambient_overlay"):
            self.ambient_overlay.setGeometry(self.ambient_overlay.parentWidget().rect())
            self.ambient_overlay.raise_()

    def showEvent(self, event) -> None:
        super().showEvent(event)
        if hasattr(self, "render_timer"):
            self.render_timer.start()

    def hideEvent(self, event) -> None:
        if hasattr(self, "render_timer"):
            self.render_timer.stop()
        super().hideEvent(event)

    def changeEvent(self, event) -> None:
        super().changeEvent(event)
        if event.type() == QEvent.WindowStateChange and hasattr(self, "render_timer"):
            if self.isMinimized():
                self.render_timer.stop()
            elif self.isVisible():
                self.render_timer.start()

    def update_animations(self) -> None:
        self.animation_frame += 1
        speed_ratio = min(abs(self.model.speed) / max(self.model.speed_limit, 1.0), 1.0)
        self.sidebar_visual.set_operating_state(
            activity=speed_ratio if self.model.enabled else 0.0,
            direction=self.model.speed,
            connected=self.model.connected,
        )
        di_states = self.last_di_states or (False,) * 6
        self.di_visual.set_operating_state(
            activity=0.7 if self.model.connected else 0.0,
            connected=self.model.connected,
            channels=di_states,
        )
        do_states = tuple(button.isChecked() for button in self.do_test_buttons)
        self.do_visual.set_operating_state(
            activity=0.8 if any(do_states) else 0.0,
            connected=self.model.connected,
            channels=do_states,
        )
        self.communication_visual.set_operating_state(
            activity=0.65 if self.model.connected else 0.0,
            connected=self.model.connected,
        )
        self.service_visual.set_operating_state(
            activity=1.0 if self.firmware_timer.isActive() else 0.0,
            connected=self.firmware_timer.isActive(),
            progress=self.firmware_progress.value() / 100.0,
        )
        self.ambient_overlay.set_operating_state(self.model.connected, self.model.enabled)
        for schematic in self.findChildren(TechSchematic):
            if schematic.isVisible():
                schematic.advance()
        self.scope.flush()
        if self.animation_frame % 3 == 0 and self.ambient_overlay.isVisible():
            self.ambient_overlay.advance()

    def toggle_effects(self, enabled: bool) -> None:
        if hasattr(self, "ambient_overlay"):
            self.ambient_overlay.set_effects_enabled(enabled)
        self.fx_button.setText("FX ON" if enabled else "FX OFF")
        self.log("全窗环境特效已开启。" if enabled else "全窗环境特效已关闭。")

    def toggle_theme(self) -> None:
        self.theme_name = "light" if self.theme_name == "dark" else "dark"
        style = self.stylesheet(self.theme_name)
        self.setStyleSheet(style)
        self.design_surface.setStyleSheet(style)
        self.design_view.setBackgroundBrush(QColor("#e8e6e9" if self.theme_name == "light" else "#08080a"))
        self.theme_button.setText("DARK" if self.theme_name == "light" else "LIGHT")
        self.scope.set_theme(self.theme_name)
        self.motion_tracker.set_theme(self.theme_name)
        self.ambient_overlay.set_theme(self.theme_name)
        for pill in self.findChildren(StatusPill):
            pill.set_theme(self.theme_name)
        for schematic in self.findChildren(TechSchematic):
            schematic.set_theme(self.theme_name)
        theme_label = "明亮" if self.theme_name == "light" else "深色"
        self.log(f"界面已切换为{theme_label}风格。")

    def build_sidebar(self) -> QFrame:
        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(240)
        layout = QVBoxLayout(sidebar)
        layout.setContentsMargins(18, 22, 18, 18)
        layout.setSpacing(6)
        layout.addWidget(self.label("WORKSPACE", "eyebrow"))
        group = QButtonGroup(self)
        group.setExclusive(True)
        entries = (("总览监控", "OVERVIEW"), ("实时示波器", "OSCILLOSCOPE"), ("参数管理", "PARAMETERS"), ("数字 I/O", "DIGITAL IO"), ("通讯配置", "COMMUNICATION"), ("诊断与服务", "DIAGNOSTICS"))
        for index, (title, sub) in enumerate(entries):
            button = QPushButton(f"{index + 1:02d}  {title}\n{sub}")
            button.setCheckable(True)
            button.setObjectName("nav-button")
            button.setToolTip(f"切换到{title}")
            button.clicked.connect(lambda checked=False, page=index: self.pages.switch_to(page))
            group.addButton(button)
            self.nav_buttons.append(button)
            layout.addWidget(button)
        self.nav_buttons[0].setChecked(True)
        self.sidebar_visual = TechSchematic(variant="motor")
        layout.addWidget(self.sidebar_visual, 1)
        card = QFrame()
        card.setObjectName("device-card")
        box = QVBoxLayout(card)
        box.setContentsMargins(13, 12, 13, 12)
        box.addWidget(self.label("DEVICE", "eyebrow"))
        self.side_device = self.label("未连接", "device-name")
        self.side_link = self.label("USB  /  NODE 01", "muted")
        box.addWidget(self.side_device)
        box.addWidget(self.side_link)
        layout.addWidget(card)
        return sidebar

    def page_shell(self, title: str, subtitle: str) -> tuple[QWidget, QVBoxLayout]:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(28, 22, 28, 20)
        layout.setSpacing(16)
        header = QHBoxLayout()
        titles = QVBoxLayout()
        titles.addWidget(self.label(title, "page-title"))
        titles.addWidget(self.label(subtitle, "subtitle"))
        header.addLayout(titles)
        header.addStretch()
        connection_pill = StatusPill()
        connection_pill.set_state("离线仿真", "off")
        self.connection_pills.append(connection_pill)
        header.addWidget(connection_pill)
        layout.addLayout(header)
        return page, layout

    def build_dashboard(self) -> QWidget:
        page, layout = self.page_shell("驱动器总览", "实时状态、运动控制与关键健康指标")
        metrics = QHBoxLayout()
        metrics.setSpacing(10)
        self.position_value, self.speed_value = QLabel("0.00°"), QLabel("0 rpm")
        self.current_value, self.voltage_value, self.temperature_value = QLabel("0.00 A"), QLabel("48.2 V"), QLabel("33.0 °C")
        for title, value, accent in (("机械位置", self.position_value, CYAN), ("电机速度", self.speed_value, BLUE), ("Q 轴电流", self.current_value, AMBER), ("母线电压", self.voltage_value, GREEN), ("驱动温度", self.temperature_value, RED)):
            metrics.addWidget(self.metric_card(title, value, accent))
        layout.addLayout(metrics)
        center = QHBoxLayout()
        center.setSpacing(14)
        monitor = self.panel("轨迹跟随", "COMMAND / FEEDBACK / ERROR")
        monitor.layout().addWidget(self.motion_tracker)
        center.addWidget(monitor, 1)
        control = self.panel("运动控制", "SIMULATED COMMAND")
        control.setMinimumHeight(370)
        control.setMinimumWidth(350)
        control.setMaximumWidth(480)
        form = control.layout()
        form.setContentsMargins(14, 14, 14, 14)
        form.setSpacing(6)
        control_grid = QGridLayout()
        control_grid.setHorizontalSpacing(10)
        control_grid.setVerticalSpacing(6)
        for row in range(3):
            control_grid.setRowMinimumHeight(row, 36)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(("位置模式", "速度模式", "电流模式"))
        control_grid.addWidget(self.label("工作模式", "field-label"), 0, 0)
        control_grid.addWidget(self.mode_combo, 0, 1)
        self.target_spin = QDoubleSpinBox()
        self.target_spin.setRange(-999999, 999999)
        self.target_spin.setDecimals(2)
        self.target_spin.setValue(90)
        self.target_spin.setSuffix(" °")
        self.target_label = self.label("目标位置", "field-label")
        control_grid.addWidget(self.target_label, 1, 0)
        control_grid.addWidget(self.target_spin, 1, 1)
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(10, 6000)
        self.speed_spin.setValue(800)
        self.speed_spin.setSuffix(" rpm")
        self.limit_label = self.label("速度限制", "field-label")
        control_grid.addWidget(self.limit_label, 2, 0)
        control_grid.addWidget(self.speed_spin, 2, 1)
        form.addLayout(control_grid)
        presets = QHBoxLayout()
        self.command_presets = []
        for text, value in (("-90°", -90), ("0°", 0), ("+90°", 90), ("+360°", 360)):
            button = QPushButton(text)
            button.setObjectName("compact-button")
            button.setProperty("commandValue", value)
            button.clicked.connect(lambda checked=False, b=button: self.target_spin.setValue(float(b.property("commandValue"))))
            self.command_presets.append(button)
            presets.addWidget(button)
        form.addLayout(presets)
        self.mode_combo.currentTextChanged.connect(self.configure_control_mode)
        self.enable_button = QPushButton("伺服使能")
        self.enable_button.setObjectName("primary-button")
        self.enable_button.setCheckable(True)
        self.enable_button.clicked.connect(self.toggle_enable)
        form.addWidget(self.enable_button)
        actions = QHBoxLayout()
        for text, slot in (("位置清零", self.zero_position), ("故障复位", self.reset_faults)):
            button = QPushButton(text)
            button.clicked.connect(slot)
            actions.addWidget(button)
        form.addLayout(actions)
        self.drive_state = StatusPill()
        self.drive_state.set_state("等待连接", "off")
        form.addWidget(self.drive_state)
        center.addWidget(control)
        layout.addLayout(center, 1)
        bottom = QHBoxLayout()
        bottom.setSpacing(14)
        health = self.panel("系统健康", "LIVE DIAGNOSTICS")
        grid = QGridLayout()
        self.health_bars = []
        for row, (name, value) in enumerate((("通讯质量", 0), ("热负载", 18), ("振动估计", 7))):
            bar = QProgressBar()
            bar.setValue(value)
            self.health_bars.append(bar)
            grid.addWidget(self.label(name, "field-label"), row, 0)
            grid.addWidget(bar, row, 1)
        health.layout().addLayout(grid)
        bottom.addWidget(health, 3)
        events = self.panel("事件流", "SYSTEM EVENT STREAM")
        self.log_view.setReadOnly(True)
        self.log_view.setObjectName("log-view")
        self.log_view.setMaximumHeight(120)
        events.layout().addWidget(self.log_view)
        bottom.addWidget(events, 5)
        layout.addLayout(bottom)
        return page

    def build_scope_page(self) -> QWidget:
        page, layout = self.page_shell("实时示波器", "多通道信号采集、触发与波形观察")
        toolbar = QHBoxLayout()
        self.scope_action_buttons = []
        for text, slot, primary in (("开始采集", lambda: self.set_scope_running(True), True), ("停止采集", lambda: self.set_scope_running(False), False), ("清除波形", self.clear_scope, False)):
            button = QPushButton(text)
            if primary:
                button.setObjectName("primary-button")
            button.clicked.connect(slot)
            self.scope_action_buttons.append(button)
            toolbar.addWidget(button)
        self.scope_action_buttons[0].setEnabled(False)
        self.scope_period, self.scope_trigger, self.scope_source = QComboBox(), QComboBox(), QComboBox()
        self.scope_period.addItems(("10 ms", "20 ms", "50 ms", "100 ms"))
        self.scope_period.setCurrentText("50 ms")
        self.scope_period.currentTextChanged.connect(self.set_scope_period)
        self.scope_trigger.addItems(("自动", "上升沿", "下降沿", "单次"))
        self.scope_trigger.currentTextChanged.connect(self.set_scope_trigger)
        self.scope_source.addItems(("CH1", "CH2", "CH3", "CH4"))
        self.scope_source.currentIndexChanged.connect(self.reset_scope_trigger_reference)
        self.scope_trigger_level = QDoubleSpinBox()
        self.scope_trigger_level.setRange(-100.0, 100.0)
        self.scope_trigger_level.setDecimals(1)
        self.scope_trigger_level.setSuffix(" %")
        self.scope_trigger_level.setMaximumWidth(105)
        self.scope_trigger_level.valueChanged.connect(self.reset_scope_trigger_reference)
        toolbar.addStretch()
        self.scope_status = StatusPill()
        self.scope_status.set_state("自动采集中", "run")
        toolbar.addWidget(self.scope_status)
        layout.addLayout(toolbar)
        settings = QHBoxLayout()
        settings.addWidget(self.label("采样周期", "field-label"))
        settings.addWidget(self.scope_period)
        settings.addSpacing(12)
        settings.addWidget(self.label("触发方式", "field-label"))
        settings.addWidget(self.scope_trigger)
        settings.addSpacing(12)
        settings.addWidget(self.label("触发源", "field-label"))
        settings.addWidget(self.scope_source)
        settings.addWidget(self.label("触发电平", "field-label"))
        settings.addWidget(self.scope_trigger_level)
        settings.addStretch()
        layout.addLayout(settings)
        channels = QHBoxLayout()
        for i, text in enumerate(("CH1 单圈位置", "CH2 速度反馈", "CH3 Q轴电流", "CH4 命令误差")):
            check = QCheckBox(text)
            check.setChecked(self.scope.visible[i])
            check.setStyleSheet(f"color:{ScopeChart.COLORS[i]};font-weight:650;")
            check.toggled.connect(lambda value, idx=i: self.set_channel(idx, value))
            channels.addWidget(check)
        channels.addStretch()
        layout.addLayout(channels)
        panel = self.panel("实时波形", "NORMALIZED SIGNAL / ±100")
        panel.layout().addWidget(self.scope)
        layout.addWidget(panel, 1)
        stats = QHBoxLayout()
        self.scope_stats = []
        for name in ("峰值", "谷值", "均方根", "频率估计"):
            value = QLabel("--")
            self.scope_stats.append(value)
            stats.addWidget(self.metric_card(name, value, BLUE))
        layout.addLayout(stats)
        return page

    def build_parameter_page(self) -> QWidget:
        page, layout = self.page_shell("参数管理", "对象字典、参数读写、比较与断电保存")
        toolbar = QHBoxLayout()
        self.parameter_search = QLineEdit()
        self.parameter_search.setPlaceholderText("搜索索引、名称或分组")
        self.parameter_search.textChanged.connect(self.filter_parameters)
        self.parameter_group = QComboBox()
        self.parameter_group.addItems(("全部分组",) + tuple(sorted({row[6] for row in PARAMETERS})))
        self.parameter_group.currentTextChanged.connect(self.filter_parameters)
        toolbar.addWidget(self.parameter_search, 2)
        toolbar.addWidget(self.parameter_group)
        for text, slot, primary in (("读取全部", self.read_parameters, True), ("写入选中", self.write_parameter, False), ("保存 EEPROM", self.save_eeprom, False), ("导入", self.import_parameters, False), ("导出", self.export_parameters, False)):
            button = QPushButton(text)
            if primary:
                button.setObjectName("primary-button")
            button.clicked.connect(slot)
            toolbar.addWidget(button)
        layout.addLayout(toolbar)
        self.parameter_table = QTableWidget(len(PARAMETERS), 7)
        self.parameter_table.setHorizontalHeaderLabels(("索引", "子索引", "参数名称", "当前值", "默认值", "单位", "分组"))
        self.parameter_table.verticalHeader().setVisible(False)
        self.parameter_table.setAlternatingRowColors(True)
        self.parameter_table.setSelectionBehavior(QTableWidget.SelectRows)
        parameter_header = self.parameter_table.horizontalHeader()
        parameter_header.setSectionResizeMode(2, QHeaderView.Stretch)
        parameter_header.setSectionResizeMode(3, QHeaderView.Stretch)
        for column in (0, 1, 4, 5, 6):
            parameter_header.setSectionResizeMode(column, QHeaderView.ResizeToContents)
        for row, values in enumerate(PARAMETERS):
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                if column != 3 or values[6] == "监视参数":
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.parameter_table.setItem(row, column, item)
        self.parameter_table.itemChanged.connect(self.mark_parameter_dirty)
        self.parameter_table.currentCellChanged.connect(self.show_parameter_help)
        layout.addWidget(self.parameter_table, 1)
        self.parameter_help = self.label("选择参数后将在此显示用途、单位和写入状态。", "parameter-help")
        self.parameter_help.setWordWrap(True)
        layout.addWidget(self.parameter_help)
        footer = QHBoxLayout()
        self.parameter_status = self.label(f"已加载 {len(PARAMETERS)} 项参数", "muted")
        footer.addWidget(self.parameter_status)
        footer.addStretch()
        restore = QPushButton("恢复选中默认值")
        restore.clicked.connect(self.restore_default)
        footer.addWidget(restore)
        layout.addLayout(footer)
        return page

    def build_io_page(self) -> QWidget:
        page, layout = self.page_shell("数字 I/O", "输入状态监测、输出测试与功能映射")
        content = QHBoxLayout()
        content.setSpacing(14)
        inputs = self.panel("数字输入", "DI STATUS / FILTER")
        grid = QGridLayout()
        self.di_lights = []
        self.di_function_combos = []
        self.di_test_buttons = []
        functions = ("伺服使能", "故障复位", "正向限位", "反向限位", "位置触发", "增益切换")
        available_functions = ("未配置",) + functions + ("速度命令", "位置命令", "转矩限制")
        for row, function in enumerate(functions):
            light = self.label("●", "io-off")
            self.di_lights.append(light)
            combo = QComboBox()
            combo.addItems(available_functions)
            combo.setCurrentText(function)
            combo.currentTextChanged.connect(lambda text, index=row: self.apply_di_function(index, self.filtered_di_states[index]))
            self.di_function_combos.append(combo)
            test_button = QPushButton("LOW")
            test_button.setCheckable(True)
            test_button.setObjectName("di-toggle")
            test_button.setToolTip(f"模拟 DI{row + 1} 外部输入电平")
            test_button.toggled.connect(lambda checked, index=row: self.set_di_input(index, checked))
            self.di_test_buttons.append(test_button)
            grid.addWidget(self.label(f"DI{row + 1}", "io-name"), row, 0)
            grid.addWidget(light, row, 1)
            grid.addWidget(combo, row, 2)
            grid.addWidget(test_button, row, 3)
        inputs.layout().addLayout(grid)
        self.di_filter_spin = QSpinBox()
        self.di_filter_spin.setRange(0, 100)
        self.di_filter_spin.setValue(8)
        self.di_filter_spin.setSuffix(" ms")
        inputs.layout().addWidget(self.field("输入滤波", self.di_filter_spin))
        self.di_visual = TechSchematic(compact=True, variant="signal")
        inputs.layout().addWidget(self.di_visual, 1)
        content.addWidget(inputs, 1)
        outputs = self.panel("数字输出", "DO MAPPING / MANUAL TEST")
        grid = QGridLayout()
        self.do_function_combos = []
        self.do_test_buttons = []
        for row, function in enumerate(("驱动就绪", "故障输出", "目标位置到达", "零速到达", "转矩限制中", "制动器控制")):
            combo = QComboBox()
            combo.addItems((function, "未配置", "速度到达", "报警输出", "自定义状态"))
            self.do_function_combos.append(combo)
            button = QPushButton("OFF")
            button.setCheckable(True)
            button.setObjectName("io-toggle")
            button.toggled.connect(lambda checked, b=button: self.toggle_io_button(b, checked))
            self.do_test_buttons.append(button)
            grid.addWidget(self.label(f"DO{row + 1}", "io-name"), row, 0)
            grid.addWidget(combo, row, 1)
            grid.addWidget(button, row, 2)
        outputs.layout().addLayout(grid)
        apply_button = QPushButton("写入 I/O 配置")
        apply_button.setObjectName("primary-button")
        apply_button.clicked.connect(self.apply_io_configuration)
        outputs.layout().addWidget(apply_button)
        self.do_visual = TechSchematic(compact=True, variant="matrix")
        outputs.layout().addWidget(self.do_visual, 1)
        content.addWidget(outputs, 1)
        layout.addLayout(content, 1)
        return page

    def build_communication_page(self) -> QWidget:
        page, layout = self.page_shell("通讯配置", "USB、RS485、CANopen 与 EtherCAT 连接管理")
        protocols = QHBoxLayout()
        self.protocol_buttons = QButtonGroup(self)
        self.protocol_buttons.setExclusive(True)
        for index, name in enumerate(("USB", "RS485", "CANopen", "EtherCAT")):
            button = QPushButton(name)
            button.setCheckable(True)
            button.setObjectName("segment-button")
            button.setChecked(index == 0)
            self.protocol_buttons.addButton(button, index)
            protocols.addWidget(button)
        protocols.addStretch()
        layout.addLayout(protocols)
        body = QHBoxLayout()
        body.setSpacing(14)
        connection = self.panel("连接设置", "TRANSPORT LAYER")
        self.port_combo = QComboBox()
        self.port_combo.addItems(("自动检测", "USB Device 01", "COM3", "COM5", "CAN Adapter 0"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(("1 Mbps", "500 kbps", "250 kbps", "115200 bps", "57600 bps"))
        self.node_spin = QSpinBox()
        self.node_spin.setRange(1, 127)
        self.node_spin.setValue(1)
        self.timeout_spin = QSpinBox()
        self.timeout_spin.setRange(50, 10000)
        self.timeout_spin.setValue(500)
        self.timeout_spin.setSuffix(" ms")
        for title, widget in (("接口设备", self.port_combo), ("波特率", self.baud_combo), ("节点 ID", self.node_spin), ("通讯超时", self.timeout_spin)):
            connection.layout().addWidget(self.field(title, widget))
        self.connect_button = QPushButton("连接驱动器")
        self.connect_button.setObjectName("primary-button")
        self.connect_button.clicked.connect(self.toggle_connection)
        connection.layout().addWidget(self.connect_button)
        self.communication_visual = TechSchematic(compact=True, variant="network")
        connection.layout().addWidget(self.communication_visual, 1)
        body.addWidget(connection, 2)
        bus = self.panel("总线应用", "CANOPEN / ETHERCAT PROFILE")
        self.canopen_check = QCheckBox("激活 CANopen CiA 402")
        self.ethercat_check = QCheckBox("激活 EtherCAT CoE")
        self.heartbeat_check = QCheckBox("启用心跳与通讯超时保护")
        self.heartbeat_check.setChecked(True)
        self.canopen_check.toggled.connect(lambda checked: self.set_bus_profile("CANopen", checked))
        self.ethercat_check.toggled.connect(lambda checked: self.set_bus_profile("EtherCAT", checked))
        self.heartbeat_check.toggled.connect(lambda checked: self.log(f"通讯超时保护已{'开启' if checked else '关闭'}。"))
        for check in (self.canopen_check, self.ethercat_check, self.heartbeat_check):
            bus.layout().addWidget(check)
        bus.layout().addWidget(self.label("PDO 映射", "field-label"))
        pdo = QListWidget()
        pdo.addItems(("RxPDO1  6040h 控制字", "RxPDO1  607Ah 目标位置", "TxPDO1  6041h 状态字", "TxPDO1  6064h 位置反馈"))
        bus.layout().addWidget(pdo)
        map_button = QPushButton("验证并写入 PDO 映射")
        map_button.clicked.connect(self.validate_pdo_mapping)
        bus.layout().addWidget(map_button)
        body.addWidget(bus, 3)
        layout.addLayout(body, 1)
        return page

    def build_diagnostics_page(self) -> QWidget:
        page, layout = self.page_shell("诊断与服务", "故障追踪、配置向导与固件维护")
        content = QHBoxLayout()
        content.setSpacing(14)
        faults = self.panel("故障诊断", "ACTIVE AND HISTORICAL EVENTS")
        self.fault_list = QListWidget()
        self.fault_list.addItem("无活动故障")
        for text in ("历史  8131  通讯超时", "历史  FF00  加速度过大警告", "历史  7311  速度超差"):
            item = QListWidgetItem(text)
            item.setForeground(QColor(MUTED))
            self.fault_list.addItem(item)
        faults.layout().addWidget(self.fault_list)
        actions = QHBoxLayout()
        for text, slot, primary in (("故障复位", self.reset_faults, True), ("模拟故障", self.inject_fault, False), ("清除历史", self.clear_fault_history, False)):
            button = QPushButton(text)
            if primary:
                button.setObjectName("primary-button")
            button.clicked.connect(slot)
            actions.addWidget(button)
        faults.layout().addLayout(actions)
        content.addWidget(faults, 3)
        wizard = self.panel("快速配置向导", "MOTOR AND ENCODER MATCHING")
        wizard.layout().addWidget(self.label("01 电机  /  02 反馈  /  03 限值  /  04 验证", "step-line"))
        self.motor_type, self.feedback_type = QComboBox(), QComboBox()
        self.motor_type.addItems(("三相永磁同步电机", "直流无刷电机", "直流有刷电机"))
        self.feedback_type.addItems(("增量式 ABZ 编码器", "SSI 绝对值编码器", "RS485 多圈编码器", "Hall UVW"))
        self.rated_current, self.rated_speed = QDoubleSpinBox(), QSpinBox()
        self.rated_current.setRange(0.1, 100)
        self.rated_current.setValue(3.0)
        self.rated_current.setSuffix(" A")
        self.rated_speed.setRange(100, 20000)
        self.rated_speed.setValue(3000)
        self.rated_speed.setSuffix(" rpm")
        for title, widget in (("电机类型", self.motor_type), ("反馈类型", self.feedback_type), ("额定电流", self.rated_current), ("额定转速", self.rated_speed)):
            wizard.layout().addWidget(self.field(title, widget))
        apply_button = QPushButton("生成推荐配置")
        apply_button.setObjectName("primary-button")
        apply_button.clicked.connect(self.apply_wizard)
        wizard.layout().addWidget(apply_button)
        content.addWidget(wizard, 3)
        service = self.panel("固件维护", "LOCAL UPGRADE SIMULATION")
        self.firmware_path = QLineEdit()
        self.firmware_path.setReadOnly(True)
        self.firmware_path.setText("demo://AUUSN-ZY10D60-v2.3.2.bin")
        browse = QPushButton("选择文件")
        browse.clicked.connect(self.choose_firmware)
        self.firmware_progress = QProgressBar()
        self.upgrade_button = QPushButton("开始升级")
        self.upgrade_button.setEnabled(True)
        self.upgrade_button.clicked.connect(self.start_firmware)
        service.layout().addWidget(self.firmware_path)
        service.layout().addWidget(browse)
        service.layout().addWidget(self.firmware_progress)
        service.layout().addWidget(self.upgrade_button)
        warning = self.label("升级过程为界面模拟，不会向真实设备发送数据。", "warning")
        warning.setWordWrap(True)
        service.layout().addWidget(warning)
        self.service_visual = TechSchematic(compact=True, variant="firmware")
        service.layout().addWidget(self.service_visual, 1)
        content.addWidget(service, 3)
        layout.addLayout(content, 1)
        return page

    @staticmethod
    def label(text: str, object_name: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName(object_name)
        return label

    def panel(self, title: str, subtitle: str) -> QFrame:
        panel = QFrame()
        panel.setObjectName("panel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(11)
        heading = QHBoxLayout()
        heading.addWidget(self.label(title, "section-title"))
        heading.addStretch()
        heading.addWidget(self.label(subtitle, "eyebrow"))
        layout.addLayout(heading)
        return panel

    def metric_card(self, title: str, value: QLabel, accent: str) -> QFrame:
        card = QFrame()
        card.setObjectName("metric-card")
        card.setStyleSheet(f"QFrame#metric-card{{border-top:2px solid {accent};}}")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(14, 11, 14, 11)
        layout.addWidget(self.label(title, "metric-title"))
        value.setObjectName("metric-value")
        value.setStyleSheet(f"color:{accent};")
        layout.addWidget(value)
        return card

    def field(self, title: str, widget: QWidget) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        layout.addWidget(self.label(title, "field-label"))
        layout.addWidget(widget)
        return container

    def toggle_connection(self) -> None:
        self.model.connected = not self.model.connected
        if self.model.connected:
            protocol = self.protocol_buttons.checkedButton().text()
            self.connect_button.setText("断开连接")
            for pill in self.connection_pills:
                pill.set_state("已连接", "ok")
            self.drive_state.set_state("驱动器就绪", "ok")
            self.side_device.setText("ZY10D60")
            self.side_link.setText(f"{protocol}  /  NODE {self.node_spin.value():02d}")
            self.title_device.setText(f"{protocol.upper()} / ZY10D60 / NODE {self.node_spin.value():02d}")
            self.log(f"{protocol} 通道连接成功，节点 {self.node_spin.value():02d}。")
        else:
            self.model.enabled = False
            self.enable_button.setChecked(False)
            self.enable_button.setText("伺服使能")
            self.connect_button.setText("连接驱动器")
            for pill in self.connection_pills:
                pill.set_state("离线仿真", "off")
            self.drive_state.set_state("等待连接", "off")
            self.side_device.setText("未连接")
            self.title_device.setText("SIMULATION / NO DEVICE")
            self.log("驱动器连接已断开。")

    def set_bus_profile(self, profile: str, enabled: bool) -> None:
        if enabled:
            other = self.ethercat_check if profile == "CANopen" else self.canopen_check
            other.blockSignals(True)
            other.setChecked(False)
            other.blockSignals(False)
        self.log(f"{profile} 应用配置已{'激活' if enabled else '停用'}。")

    def validate_pdo_mapping(self) -> None:
        profile = "EtherCAT" if self.ethercat_check.isChecked() else "CANopen"
        self.log(f"{profile} PDO 映射包含 4 个对象，规则校验通过并已写入模拟缓存。")

    def toggle_enable(self, checked: bool) -> None:
        if checked and not self.model.connected:
            self.enable_button.setChecked(False)
            self.log("使能失败：请先在通讯配置页连接驱动器。")
            return
        self.model.enabled = checked
        self.enable_button.setText("关闭使能" if checked else "伺服使能")
        self.drive_state.set_state("运行中" if checked else "驱动器就绪", "run" if checked else "ok")
        self.log("伺服使能已开启。" if checked else "伺服使能已关闭。")

    def configure_control_mode(self, mode: str) -> None:
        self.mode_targets[self.control_mode] = self.target_spin.value()
        self.control_mode = mode
        configurations = {
            "位置模式": ("目标位置", -999999.0, 999999.0, 2, " °", (("-90°", -90), ("0°", 0), ("+90°", 90), ("+360°", 360))),
            "速度模式": ("目标速度", -6000.0, 6000.0, 0, " rpm", (("-1500", -1500), ("停止", 0), ("+1500", 1500), ("+3000", 3000))),
            "电流模式": ("目标电流", -12.0, 12.0, 2, " A", (("-3 A", -3), ("0 A", 0), ("+3 A", 3), ("+6 A", 6))),
        }
        title, minimum, maximum, decimals, suffix, presets = configurations[mode]
        self.target_spin.blockSignals(True)
        self.target_spin.setRange(minimum, maximum)
        self.target_spin.setDecimals(decimals)
        self.target_spin.setSuffix(suffix)
        self.target_spin.setValue(self.mode_targets[mode])
        self.target_spin.blockSignals(False)
        self.target_label.setText(title)
        self.limit_label.setText("速度上限" if mode != "位置模式" else "速度限制")
        for button, (text, value) in zip(self.command_presets, presets):
            button.setText(text)
            button.setProperty("commandValue", value)
        self.log(f"控制模式切换为{mode}，目标输入单位已更新。")

    def zero_position(self) -> None:
        self.model.position = 0.0
        if self.mode_combo.currentText() == "位置模式":
            self.target_spin.setValue(0.0)
        self.log("位置给定与反馈已清零。")

    def update_simulation(self) -> None:
        self.simulation_tick += 1
        mode = self.mode_combo.currentText()
        command = self.target_spin.value()
        active_functions = {
            combo.currentText()
            for combo, active in zip(self.di_function_combos, self.filtered_di_states)
            if active
        }
        self.torque_limit_active = "转矩限制" in active_functions
        if "正向限位" in active_functions and command > 0:
            command = self.model.position if mode == "位置模式" else 0.0
        if "反向限位" in active_functions and command < 0:
            command = self.model.position if mode == "位置模式" else 0.0
        if self.torque_limit_active and mode == "电流模式":
            command = clamp(command, -3.0, 3.0)
        self.model.target, self.model.speed_limit = command, float(self.speed_spin.value())
        self.model.step(self.timer.interval(), mode)
        if self.torque_limit_active:
            self.model.current = clamp(self.model.current, -3.0, 3.0)
        self.position_value.setText(f"{self.model.position:.2f}°")
        self.speed_value.setText(f"{self.model.speed:.0f} rpm")
        self.current_value.setText(f"{self.model.current:.2f} A")
        self.voltage_value.setText(f"{self.model.voltage:.1f} V")
        self.temperature_value.setText(f"{self.model.temperature:.1f} °C")
        self.motion_tracker.set_values(
            self.mode_combo.currentText(),
            self.model.target,
            self.model.position,
            self.model.speed,
            self.model.current,
            self.model.speed_limit,
            self.model.enabled,
        )
        if mode == "速度模式":
            error = self.model.target - self.model.speed
        elif mode == "电流模式":
            error = self.model.target - self.model.current
        else:
            error = self.model.target - self.model.position
        self.health_bars[0].setValue(98 if self.model.connected else 0)
        self.health_bars[1].setValue(int(clamp((self.model.temperature - 25) / 65 * 100, 0, 100)))
        self.health_bars[2].setValue(int(clamp(abs(self.model.current) * 4 + abs(math.sin(self.model.phase)) * 8, 0, 100)))
        states = tuple(self.filtered_di_states)
        if states != self.last_di_states:
            for light, active in zip(self.di_lights, states):
                light.setObjectName("io-on" if active else "io-off")
                light.style().unpolish(light)
                light.style().polish(light)
            self.last_di_states = states
        if self.simulation_tick % 4 == 0:
            values = list(self.scope.data[0])
            if values:
                rms = math.sqrt(sum(value * value for value in values) / len(values))
                statistics = (max(values), min(values), rms, abs(self.model.speed) / 60.0)
                for label, value in zip(self.scope_stats, statistics):
                    label.setText(f"{value:.1f}")
            else:
                for label in self.scope_stats:
                    label.setText("--")

    def set_di_input(self, index: int, checked: bool) -> None:
        self.raw_di_states[index] = checked
        self.di_test_buttons[index].setText("HIGH" if checked else "LOW")
        self.di_filter_generation[index] += 1
        generation = self.di_filter_generation[index]
        delay = self.di_filter_spin.value()
        QTimer.singleShot(delay, lambda: self.commit_di_input(index, checked, generation))

    def commit_di_input(self, index: int, checked: bool, generation: int) -> None:
        if generation != self.di_filter_generation[index] or checked != self.raw_di_states[index]:
            return
        self.filtered_di_states[index] = checked
        self.last_di_states = None
        self.apply_di_function(index, checked)
        level = "HIGH" if checked else "LOW"
        self.log(f"DI{index + 1} 滤波后电平变为 {level}，映射功能：{self.di_function_combos[index].currentText()}。")

    def apply_di_function(self, index: int, active: bool) -> None:
        function = self.di_function_combos[index].currentText()
        if function == "伺服使能":
            if active and not self.model.connected:
                self.log(f"DI{index + 1} 使能请求等待设备连接。")
                return
            self.enable_button.blockSignals(True)
            self.enable_button.setChecked(active)
            self.enable_button.blockSignals(False)
            self.toggle_enable(active)
            return
        if function == "转矩限制":
            self.torque_limit_active = active
        if not active:
            return
        if function == "故障复位":
            self.reset_faults()
        elif function == "速度命令":
            self.mode_combo.setCurrentText("速度模式")
        elif function == "位置命令":
            self.mode_combo.setCurrentText("位置模式")
        elif function in ("正向限位", "反向限位"):
            self.model.speed = 0.0
            self.log(f"{function}输入生效，已阻止对应方向运动。")
        elif function == "位置触发":
            self.log("位置触发输入已锁存到模拟事件流。")
        elif function == "增益切换":
            self.log("增益切换输入已激活模拟高响应参数组。")

    def set_scope_running(self, running: bool) -> None:
        self.scope_running = running
        if running and self.scope_trigger_mode != "自动":
            self.scope_trigger_remaining = 0
            self.scope_trigger_ready = True
            self.reset_scope_trigger_reference()
        self.scope_action_buttons[0].setEnabled(not running)
        self.scope_action_buttons[1].setEnabled(running)
        if running and self.scope_trigger_mode != "自动":
            self.scope_status.set_state(f"等待{self.scope_trigger_mode}触发", "warn")
        elif running:
            self.scope_status.set_state("自动采集中", "run")
        else:
            self.scope_status.set_state("采集已停止", "off")
        self.log("示波器开始采集。" if running else "示波器采集已暂停。")

    def clear_scope(self) -> None:
        self.scope.clear()
        self.scope_trigger_remaining = -1 if self.scope_trigger_mode == "自动" else 0
        self.reset_scope_trigger_reference()
        for label in self.scope_stats:
            label.setText("--")
        self.log("示波器显示缓存已清除。")

    def set_scope_period(self, text: str) -> None:
        interval = int(text.split()[0])
        self.scope_timer.setInterval(interval)
        self.log(f"示波器采样周期已设置为 {interval} ms。")

    def set_scope_trigger(self, mode: str) -> None:
        self.scope_trigger_mode = mode
        self.scope_trigger_remaining = -1 if mode == "自动" else 0
        self.scope_trigger_ready = True
        self.reset_scope_trigger_reference()
        if self.scope_running:
            self.scope_status.set_state("自动采集中" if mode == "自动" else f"等待{mode}触发", "run" if mode == "自动" else "warn")
        self.log(f"示波器触发方式切换为{mode}。")

    def reset_scope_trigger_reference(self, value=None) -> None:
        del value
        self.scope_last_trigger = None
        self.scope_trigger_ready = True

    def sample_scope(self) -> None:
        self.scope_clock += self.scope_timer.interval() / 1000.0
        mode = self.mode_combo.currentText()
        if mode == "速度模式":
            command_error = self.model.target - self.model.speed
            error_scale = max(self.model.speed_limit, 1.0)
        elif mode == "电流模式":
            command_error = self.model.target - self.model.current
            error_scale = 12.0
        else:
            command_error = self.model.target - self.model.position
            error_scale = 360.0
        single_turn_position = ((self.model.position + 180.0) % 360.0) - 180.0
        sample = (
            single_turn_position / 1.8,
            clamp(self.model.speed / max(self.model.speed_limit, 1.0) * 100.0, -100.0, 100.0),
            clamp(self.model.current / 12.0 * 100.0, -100.0, 100.0),
            clamp(command_error / error_scale * 100.0, -100.0, 100.0),
        )
        trigger_value = sample[self.scope_source.currentIndex()]
        if not self.scope_running:
            self.scope_last_trigger = trigger_value
            return

        if self.scope_last_trigger is None:
            self.scope_last_trigger = trigger_value
            if self.scope_trigger_mode != "自动":
                return
        trigger_level = self.scope_trigger_level.value()
        rising = self.scope_last_trigger < trigger_level <= trigger_value
        falling = self.scope_last_trigger > trigger_level >= trigger_value
        self.scope_last_trigger = trigger_value
        triggered = (
            self.scope_trigger_mode == "上升沿" and rising
        ) or (self.scope_trigger_mode == "下降沿" and falling) or (
            self.scope_trigger_mode == "单次" and (rising or falling)
        )
        if self.scope_trigger_mode != "自动" and self.scope_trigger_remaining == 0:
            if not self.scope_trigger_ready:
                opposite_edge = falling if self.scope_trigger_mode in ("上升沿", "单次") else rising
                self.scope_trigger_ready = opposite_edge
                return
            if not triggered:
                return
            self.scope_trigger_ready = False
            self.scope_trigger_remaining = 120
            self.scope.clear()
            self.scope_status.set_state("触发后采集中", "run")
            self.log(f"示波器捕获到{self.scope_trigger_mode}触发。")

        self.scope.add_sample(sample)
        if self.scope_trigger_remaining > 0:
            self.scope_trigger_remaining -= 1
            if self.scope_trigger_remaining == 0:
                if self.scope_trigger_mode == "单次":
                    self.scope_running = False
                    self.scope_action_buttons[0].setEnabled(True)
                    self.scope_action_buttons[1].setEnabled(False)
                    self.scope_status.set_state("单次采集完成", "ok")
                    self.log("示波器单次采集完成。")
                else:
                    self.scope_status.set_state(f"等待{self.scope_trigger_mode}触发", "warn")

    def set_channel(self, index: int, visible: bool) -> None:
        self.scope.visible[index] = visible
        self.scope.update()

    def mark_parameter_dirty(self, item: QTableWidgetItem) -> None:
        if item.column() == 3:
            item.setForeground(QColor(AMBER))
            self.parameter_status.setText("存在未写入的参数修改")

    def show_parameter_help(self, row: int, column: int, previous_row: int, previous_column: int) -> None:
        del column, previous_row, previous_column
        if row < 0:
            return
        name = self.parameter_table.item(row, 2).text()
        unit = self.parameter_table.item(row, 5).text() or "无单位"
        group = self.parameter_table.item(row, 6).text()
        access = "只读监视值" if group == "监视参数" else "可编辑，写入后保存 EEPROM 才能断电保持"
        self.parameter_help.setText(f"{name}  ·  {group}  ·  {unit}  ·  {access}")

    def filter_parameters(self) -> None:
        query, group, visible = self.parameter_search.text().strip().lower(), self.parameter_group.currentText(), 0
        for row in range(self.parameter_table.rowCount()):
            text = " ".join(self.parameter_table.item(row, col).text().lower() for col in (0, 2, 6))
            matches = (not query or query in text) and (group == "全部分组" or self.parameter_table.item(row, 6).text() == group)
            self.parameter_table.setRowHidden(row, not matches)
            visible += int(matches)
        self.parameter_status.setText(f"显示 {visible} / {len(PARAMETERS)} 项参数")

    def read_parameters(self) -> None:
        self.parameter_table.blockSignals(True)
        for row in range(self.parameter_table.rowCount()):
            item = self.parameter_table.item(row, 3)
            if self.parameter_table.item(row, 0).text() == "3000":
                item.setText(f"{self.model.temperature:.1f}")
            item.setForeground(QColor(TEXT))
        self.parameter_table.blockSignals(False)
        self.parameter_status.setText(f"读取完成，共 {len(PARAMETERS)} 项")
        self.log("参数表读取完成。")

    def write_parameter(self) -> None:
        row = self.parameter_table.currentRow()
        if row < 0:
            self.parameter_status.setText("请先选择一个参数")
            return
        if self.parameter_table.item(row, 6).text() == "监视参数":
            self.parameter_status.setText("监视参数为只读，不能写入")
            self.log("参数写入被拒绝：所选对象为只读监视参数。")
            return
        item = self.parameter_table.item(row, 3)
        item.setForeground(QColor(GREEN))
        name = self.parameter_table.item(row, 2).text()
        self.parameter_status.setText(f"{name} 已写入驱动器缓存")
        self.log(f"参数写入：{name} = {item.text()}。")

    def restore_default(self) -> None:
        row = self.parameter_table.currentRow()
        if row >= 0:
            self.parameter_table.item(row, 3).setText(self.parameter_table.item(row, 4).text())

    def save_eeprom(self) -> None:
        if self.model.enabled:
            self.parameter_status.setText("请先关闭使能再保存 EEPROM")
            self.log("EEPROM 保存被拒绝：驱动器仍处于使能状态。")
            return
        self.parameter_status.setText("EEPROM 保存成功，重新上电后仍有效")
        self.log("参数已保存到模拟 EEPROM。")

    def parameter_payload(self) -> list[dict[str, str]]:
        keys = ("index", "sub_index", "name", "value", "default", "unit", "group")
        return [dict(zip(keys, (self.parameter_table.item(row, col).text() for col in range(7)))) for row in range(self.parameter_table.rowCount())]

    def export_parameters(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "导出参数", "servo-parameters.json", "JSON (*.json)")
        if path:
            Path(path).write_text(json.dumps(self.parameter_payload(), ensure_ascii=False, indent=2), encoding="utf-8")
            self.log(f"参数已导出到 {path}。")

    def import_parameters(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "导入参数", "", "JSON (*.json)")
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            values = {(entry["index"], entry["sub_index"]): entry["value"] for entry in data}
            for row in range(self.parameter_table.rowCount()):
                key = self.parameter_table.item(row, 0).text(), self.parameter_table.item(row, 1).text()
                if key in values:
                    self.parameter_table.item(row, 3).setText(str(values[key]))
            self.log(f"已导入参数文件 {path}。")
        except (OSError, ValueError, KeyError, TypeError) as error:
            QMessageBox.warning(self, "导入失败", str(error))

    def toggle_io_button(self, button: QPushButton, checked: bool) -> None:
        button.setText("ON" if checked else "OFF")
        self.log(f"数字输出手动测试：{button.text()}。")

    def apply_io_configuration(self) -> None:
        di_active = sum(combo.currentText() != "未配置" for combo in self.di_function_combos)
        do_active = sum(combo.currentText() != "未配置" for combo in self.do_function_combos)
        self.log(f"数字 I/O 配置已写入模拟驱动器：DI {di_active} 路，DO {do_active} 路。")

    def reset_faults(self) -> None:
        for index in range(self.fault_list.count()):
            item = self.fault_list.item(index)
            if item.text().startswith("活动"):
                item.setText(item.text().replace("活动", "历史", 1))
                item.setForeground(QColor(MUTED))
        if not self.fault_list.count() or self.fault_list.item(0).text() != "无活动故障":
            self.fault_list.insertItem(0, "无活动故障")
        self.title_alarm.setText("0 ALARMS")
        state_text = "驱动器就绪" if self.model.connected else "等待连接"
        self.drive_state.set_state(state_text, "ok" if self.model.connected else "off")
        self.log("故障复位命令已执行。")

    def clear_fault_history(self) -> None:
        self.fault_list.clear()
        self.fault_list.addItem("无活动故障")
        self.log("本地故障历史已清除。")

    def inject_fault(self) -> None:
        faults = (("8131", "通讯超时"), ("7311", "速度超差"), ("FF03", "电机过热"), ("2211", "过流"))
        code, description = faults[self.fault_sequence % len(faults)]
        self.fault_sequence += 1
        item = QListWidgetItem(f"活动  {code}  {description}")
        item.setForeground(QColor(RED))
        if self.fault_list.count() and self.fault_list.item(0).text() == "无活动故障":
            self.fault_list.takeItem(0)
        self.fault_list.insertItem(0, item)
        active_count = sum(
            self.fault_list.item(index).text().startswith("活动")
            for index in range(self.fault_list.count())
        )
        self.title_alarm.setText(f"{active_count} ALARM" + ("S" if active_count != 1 else ""))
        self.model.enabled = False
        self.enable_button.setChecked(False)
        self.drive_state.set_state("故障停机", "warn")
        self.log(f"已注入模拟故障 {code}：{description}。")

    def apply_wizard(self) -> None:
        encoder_lines = {
            "增量式 ABZ 编码器": "4096",
            "SSI 绝对值编码器": "65536",
            "RS485 多圈编码器": "131072",
            "Hall UVW": "6",
        }
        recommended_current = min(self.rated_current.value() * 3.0, 100.0)
        updates = {
            "最大输出电流": f"{recommended_current:.1f}",
            "最大转速": str(self.rated_speed.value()),
            "编码器线数": encoder_lines[self.feedback_type.currentText()],
        }
        for row in range(self.parameter_table.rowCount()):
            name = self.parameter_table.item(row, 2).text()
            if name in updates:
                self.parameter_table.item(row, 3).setText(updates[name])
        self.log(f"推荐配置已生成：{self.motor_type.currentText()} / {self.feedback_type.currentText()} / 峰值电流 {recommended_current:.1f} A。")

    def choose_firmware(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "选择固件", "", "Firmware (*.bin *.hex);;All files (*)")
        if path:
            self.firmware_path.setText(path)
            self.upgrade_button.setEnabled(True)

    def start_firmware(self) -> None:
        self.firmware_progress.setValue(0)
        self.upgrade_button.setEnabled(False)
        self.firmware_timer.start()
        self.log("开始模拟固件升级。")

    def advance_firmware(self) -> None:
        value = self.firmware_progress.value() + random.randint(2, 7)
        self.firmware_progress.setValue(min(value, 100))
        if value >= 100:
            self.firmware_timer.stop()
            self.upgrade_button.setEnabled(True)
            self.log("模拟固件升级完成。")

    def log(self, message: str) -> None:
        if self.log_view is not None:
            self.log_view.append(f"{datetime.now():%H:%M:%S}   {message}")

    @staticmethod
    def stylesheet(theme_name: str = "dark") -> str:
        dark_style = f"""
            QMainWindow{{background:#08080a}} QWidget{{color:{TEXT};font-family:'Segoe UI','Microsoft YaHei UI';font-size:10pt;letter-spacing:0px}}
            QFrame#window-shell{{background:{SHELL};border:1px solid #514851;border-radius:10px}} QFrame#titlebar{{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 rgba(62,59,66,245),stop:0.08 rgba(34,32,38,245),stop:0.58 rgba(15,14,18,250),stop:1 rgba(28,25,30,250));border-bottom:1px solid #6b3240;border-top-left-radius:10px;border-top-right-radius:10px}}
            QLabel#brand-mark{{color:white;background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #e76a7e,stop:0.16 #cf3b58,stop:0.58 #ad203d,stop:1 #86162e);border:1px solid #ed8091;border-radius:4px;font-size:12pt;font-weight:900;padding:3px 8px}} QLabel#app-name{{font-size:10pt;font-weight:800}} QLabel#title-context,QLabel#eyebrow{{color:#a39da5;font-size:8pt;font-weight:650}} QLabel#title-alarm{{color:{GREEN};background:#10231d;border:1px solid #24523d;border-radius:10px;padding:3px 9px;font-size:8pt;font-weight:750}}
            QToolButton#window-button,QToolButton#close-button{{color:{MUTED};background:transparent;border:0;border-radius:5px;font-size:12pt}} QToolButton#window-button:hover{{color:{TEXT};background:#29282e}} QToolButton#close-button:hover{{color:white;background:#c51c3d}} QToolButton#theme-button{{color:{BLUE};background:#1b1b20;border:1px solid #55545d;border-radius:5px;font-size:8pt;font-weight:750}} QToolButton#theme-button:hover{{color:white;border-color:#aeb7c1}} QToolButton#fx-button{{color:#ff6681;background:#2b1018;border:1px solid #8f243b;border-radius:5px;font-size:8pt;font-weight:750}} QToolButton#fx-button:!checked{{color:{MUTED};background:#19191e;border-color:#3b3a42}}
            QFrame#sidebar{{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 rgba(12,11,15,250),stop:0.72 rgba(20,18,23,248),stop:1 rgba(31,24,29,245));border-right:1px solid #51404a}} QPushButton#nav-button{{color:{MUTED};background:transparent;border:0;border-left:2px solid transparent;border-radius:4px;padding:11px 13px;text-align:left;font-weight:650}} QPushButton#nav-button:hover{{color:{TEXT};background:rgba(92,38,52,70)}} QPushButton#nav-button:checked{{color:#e66a7f;background:rgba(112,25,48,82);border-left:2px solid {CYAN}}}
            QFrame#device-card{{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 rgba(44,42,48,225),stop:0.08 rgba(28,27,32,235),stop:1 rgba(16,15,19,245));border:1px solid #554e56;border-radius:7px}} QLabel#device-name{{font-size:12pt;font-weight:750}} QLabel#muted,QLabel#field-label,QLabel#metric-title{{color:{MUTED}}} QLabel#page-title{{font-size:20pt;font-weight:700}} QLabel#subtitle{{color:{MUTED};font-size:9pt}} QLabel#section-title{{font-size:11pt;font-weight:700}} QLabel#metric-title{{font-size:8pt;font-weight:650}} QLabel#metric-value{{font-size:20pt;font-weight:650}}
            QLabel#io-name{{font-weight:700}} QLabel#io-off{{color:#4a474f;font-size:15pt}} QLabel#io-on{{color:{GREEN};font-size:15pt}} QLabel#warning{{color:{AMBER};background:#271f13;border:1px solid #56401e;border-radius:5px;padding:9px}} QLabel#parameter-help{{color:#d0ccd1;background:#17161a;border-left:2px solid {CYAN};padding:8px 10px}} QLabel#step-line{{color:#ff6882;background:#281017;border:1px solid #662237;border-radius:5px;padding:8px;font-size:8pt;font-weight:700}}
            QFrame#panel,QFrame#metric-card{{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 rgba(52,50,57,205),stop:0.035 rgba(78,74,82,105),stop:0.09 rgba(28,27,32,225),stop:1 rgba(17,16,20,242));border:1px solid {BORDER};border-radius:7px}}
            QLineEdit,QSpinBox,QDoubleSpinBox,QComboBox{{min-height:22px;background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 rgba(64,62,69,230),stop:0.09 rgba(42,40,46,235),stop:0.58 rgba(24,23,28,245),stop:1 rgba(33,31,37,240));border:1px solid #5a565f;border-radius:5px;padding:6px 9px;selection-background-color:{CYAN};selection-color:white}} QLineEdit:focus,QSpinBox:focus,QDoubleSpinBox:focus,QComboBox:focus{{border-color:#d25970}} QComboBox::drop-down{{width:25px;border:0}} QComboBox QAbstractItemView{{background:#1d1c21;border:1px solid {BORDER};selection-background-color:#56202d}}
            QPushButton{{min-height:22px;background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 rgba(70,67,75,235),stop:0.10 rgba(43,41,47,240),stop:0.62 rgba(23,22,27,248),stop:1 rgba(35,33,39,245));border:1px solid #5a565f;border-radius:5px;padding:7px 13px;font-weight:600}} QPushButton:hover{{background:#33262c;border-color:#966071}} QPushButton:pressed{{background:#1a1014;border-color:#b94a61}} QPushButton:disabled{{color:#625e65;background:#161519;border-color:#302e34}} QPushButton#primary-button{{color:white;background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #e86a7f,stop:0.13 #d33d59,stop:0.58 #b52341,stop:1 #85172e);border-color:#e27587;font-weight:800}} QPushButton#primary-button:hover{{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #ef7c8e,stop:0.5 #cc3552,stop:1 #941c34)}} QPushButton#compact-button{{padding:5px 8px}} QPushButton#segment-button{{border-radius:4px;min-width:95px}} QPushButton#segment-button:checked{{color:#e66a7f;background:rgba(105,26,46,105);border-color:{CYAN}}} QPushButton#io-toggle:checked{{color:#07130d;background:{GREEN};border-color:{GREEN}}}
            QPushButton#di-toggle{{min-width:52px;padding:5px 8px}} QPushButton#di-toggle:checked{{color:white;background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #df6177,stop:1 #8d1931);border-color:#d86579}}
            QProgressBar{{min-height:14px;background:#100f13;border:1px solid #45424a;border-radius:5px;text-align:center}} QProgressBar::chunk{{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #df6177,stop:0.18 #c93653,stop:1 #8d1931);border-radius:4px}}
            QTableWidget,QListWidget,QTextEdit#log-view{{color:#d8d4d9;background:#0d0d11;alternate-background-color:#121216;border:1px solid #35343c;border-radius:5px;gridline-color:#2a292f;selection-background-color:#5a1828;selection-color:{TEXT}}} QTableWidget::item{{padding:7px}} QHeaderView::section{{color:{MUTED};background:#1b1b20;border:0;border-right:1px solid #37363e;border-bottom:1px solid #37363e;padding:8px;font-weight:700}}
            QCheckBox{{spacing:8px}} QCheckBox::indicator{{width:15px;height:15px;border:1px solid #5b5963;border-radius:3px;background:#101014}} QCheckBox::indicator:checked{{background:{CYAN};border-color:#ff607c}} QScrollBar:vertical{{width:8px;background:transparent}} QScrollBar::handle:vertical{{background:#514e56;border-radius:4px;min-height:24px}} QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical{{height:0}} QSizeGrip{{width:13px;height:13px;background:transparent}}
            QScrollArea#page-scroll{{background:{SHELL};border:0}} QScrollArea#page-scroll>QWidget>QWidget{{background:{SHELL}}}
        """
        if theme_name == "dark":
            return dark_style
        light_style = f"""
            QWidget{{color:#242126}}
            QFrame#window-shell{{background:#e8e6e9;border-color:#aaa5ab}}
            QFrame#titlebar{{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 rgba(255,255,255,250),stop:0.12 rgba(245,243,246,245),stop:0.62 rgba(217,214,219,245),stop:1 rgba(238,236,239,248));border-bottom-color:#a84a5e}}
            QLabel#app-name{{color:#211e22}} QLabel#title-context,QLabel#eyebrow{{color:#716b73}}
            QLabel#title-alarm{{color:#16764f;background:#e9f7ef;border-color:#86c8aa}}
            QToolButton#window-button{{color:#625d64}} QToolButton#window-button:hover{{color:#211e22;background:#dedcdf}}
            QToolButton#theme-button{{color:#5e5960;background:#efedef;border-color:#aaa5ac}} QToolButton#theme-button:hover{{color:#241f23;background:#ffffff;border-color:#777178}} QToolButton#fx-button{{color:#b31536;background:#f9e8ec;border-color:#d68597}} QToolButton#fx-button:!checked{{color:#777178;background:#e7e5e8;border-color:#bcb8be}}
            QFrame#sidebar{{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 rgba(255,255,255,235),stop:0.72 rgba(234,231,235,238),stop:1 rgba(215,211,216,242));border-right-color:#b8b3ba}}
            QPushButton#nav-button{{color:#6b656d}} QPushButton#nav-button:hover{{color:#211e22;background:#eee5e8}} QPushButton#nav-button:checked{{color:#b31536;background:#f7e3e8;border-left-color:#d32045}}
            QFrame#device-card{{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #ffffff,stop:1 #e8e6e9);border-color:#c1bcc2}} QLabel#device-name{{color:#211e22}}
            QLabel#muted,QLabel#field-label,QLabel#metric-title,QLabel#subtitle{{color:#716b73}} QLabel#page-title,QLabel#section-title,QLabel#io-name{{color:#211e22}}
            QLabel#io-off{{color:#a7b2ba}} QLabel#warning{{color:#8a5b0b;background:#fff5dd;border-color:#e6c77d}}
            QLabel#parameter-help{{color:#4d474e;background:#f4eef0;border-left-color:#d32045}} QLabel#step-line{{color:#b31536;background:#f9e8ec;border-color:#d99aaa}}
            QFrame#panel,QFrame#metric-card{{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 rgba(255,255,255,238),stop:0.045 rgba(255,255,255,205),stop:0.11 rgba(246,244,247,230),stop:1 rgba(230,227,231,238));border-color:#c2bdc3}}
            QLineEdit,QSpinBox,QDoubleSpinBox,QComboBox{{color:#242126;background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 rgba(255,255,255,245),stop:0.12 rgba(246,244,247,242),stop:0.62 rgba(222,219,223,245),stop:1 rgba(240,238,241,245));border-color:#b5b0b7;selection-background-color:#bd2946;selection-color:white}}
            QLineEdit:focus,QSpinBox:focus,QDoubleSpinBox:focus,QComboBox:focus{{border-color:#d32045}} QComboBox QAbstractItemView{{color:#242126;background:white;border-color:#bcb7bd;selection-background-color:#f2d8de}}
            QPushButton{{color:#2b272c;background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #ffffff,stop:0.48 #f0eef0,stop:0.55 #dbd9dc,stop:1 #efedef);border-color:#b7b2b9}} QPushButton:hover{{background:#f6ecef;border-color:#bd6679}} QPushButton:pressed{{background:#e6d4d9}} QPushButton:disabled{{color:#9b969c;background:#e7e5e8;border-color:#d2ced3}}
            QPushButton#primary-button{{color:white;background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #e86a7f,stop:0.15 #d33d59,stop:0.62 #ad203d,stop:1 #86162e);border-color:#a72a43}} QPushButton#primary-button:hover{{background:#cf3954}}
            QPushButton#segment-button:checked{{color:#b31536;background:#f8e5ea;border-color:#d32045}}
            QProgressBar{{color:#2b272c;background:#ebe8eb;border-color:#c3bec4}} QProgressBar::chunk{{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #f04b69,stop:1 #b31536)}}
            QTableWidget,QListWidget,QTextEdit#log-view{{color:#302b31;background:#ffffff;alternate-background-color:#f5f3f5;border-color:#c7c2c8;gridline-color:#dfdcdf;selection-background-color:#f0d4db;selection-color:#3a2027}}
            QHeaderView::section{{color:#625c64;background:#e9e6e9;border-right-color:#d2ced3;border-bottom-color:#c3bec4}}
            QCheckBox{{color:#2b272c}} QCheckBox::indicator{{background:white;border-color:#979199}} QCheckBox::indicator:checked{{background:#d32045;border-color:#b31536}}
            QScrollBar::handle:vertical{{background:#aaa4ab}} QScrollArea#page-scroll{{background:#e9e8ea}} QScrollArea#page-scroll>QWidget>QWidget{{background:#e9e8ea}}
        """
        return dark_style + light_style


def run() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = ServoWindow()
    window.show()
    sys.exit(app.exec_())
