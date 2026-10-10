"""Reusable animated technical illustrations with distinct motion languages."""

from __future__ import annotations

import math

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
from PyQt5.QtWidgets import QSizePolicy, QWidget


class TechSchematic(QWidget):
    """Responsive technical animation selected by a semantic visual variant."""

    VARIANTS = {"motor", "signal", "matrix", "network", "firmware"}

    def __init__(
        self,
        parent: QWidget | None = None,
        compact: bool = False,
        variant: str = "motor",
    ) -> None:
        super().__init__(parent)
        if variant not in self.VARIANTS:
            raise ValueError(f"Unknown technical visual variant: {variant}")
        self.compact = compact
        self.variant = variant
        self.theme_name = "dark"
        self.activity = 0.0
        self.direction = 1.0
        self.connected = False
        self.channels: tuple[bool, ...] = (False,) * 6
        self.channel_history = [[False] * 14 for _ in range(6)]
        self.phase = {
            "motor": 0.0,
            "signal": 0.17,
            "matrix": 0.34,
            "network": 0.51,
            "firmware": 0.68,
        }[variant]
        self.setMinimumHeight(108 if compact else 118)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def advance(self) -> None:
        if self.variant == "firmware" or self.activity <= 0.001:
            return
        speed = {
            "motor": 0.014,
            "signal": 0.019,
            "matrix": 0.012,
            "network": 0.010,
            "firmware": 0.008,
        }[self.variant]
        self.phase = (self.phase + speed * self.activity * self.direction) % 1.0
        self.update()

    def set_operating_state(
        self,
        *,
        activity: float = 0.0,
        direction: float = 1.0,
        connected: bool = False,
        channels: tuple[bool, ...] | None = None,
        progress: float | None = None,
    ) -> None:
        new_activity = max(0.0, min(1.0, activity))
        new_direction = -1.0 if direction < 0 else 1.0
        changed = (new_activity, new_direction, connected) != (self.activity, self.direction, self.connected)
        self.activity, self.direction, self.connected = new_activity, new_direction, connected
        if channels is not None:
            new_channels = tuple(channels[:6]) + (False,) * max(0, 6 - len(channels))
            changed = changed or new_channels != self.channels
            self.channels = new_channels
            if self.variant == "signal" and connected:
                previous_history = tuple(tuple(row) for row in self.channel_history)
                for index, value in enumerate(self.channels):
                    self.channel_history[index].append(value)
                    self.channel_history[index] = self.channel_history[index][-14:]
                changed = changed or previous_history != tuple(tuple(row) for row in self.channel_history)
        if progress is not None:
            new_progress = max(0.0, min(1.0, progress))
            changed = changed or abs(new_progress - self.phase) > 0.0001
            self.phase = new_progress
        if changed:
            self.update()

    def set_theme(self, theme_name: str) -> None:
        self.theme_name = theme_name
        self.update()

    def palette(self) -> tuple[QColor, QColor, QColor, QColor, QColor]:
        dark = self.theme_name == "dark"
        return (
            QColor(201, 47, 77, 205 if dark else 220),
            QColor(194, 200, 207, 180 if dark else 210),
            QColor(217, 174, 104, 205),
            QColor(111, 107, 116, 90 if dark else 112),
            QColor(17, 16, 20, 132 if dark else 22),
        )

    def paintEvent(self, event) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setClipRect(self.rect())
        colors = self.palette()
        self._draw_grid(painter, colors[3])
        {
            "motor": self._draw_motor,
            "signal": self._draw_signal,
            "matrix": self._draw_matrix,
            "network": self._draw_network,
            "firmware": self._draw_firmware,
        }[self.variant](painter, colors)

    def _draw_grid(self, painter: QPainter, faint: QColor) -> None:
        width, height = self.width(), self.height()
        grid = QColor(faint)
        grid.setAlpha(max(18, faint.alpha() // 3))
        painter.setPen(QPen(grid, 1, Qt.DotLine))
        spacing = 44 if self.compact else 36
        offset_x = int(self.phase * spacing)
        for x in range(-spacing + offset_x, width + spacing, spacing):
            painter.drawLine(QPointF(x, 0), QPointF(x, height))
        for y in range(spacing // 2, height, spacing):
            painter.drawLine(QPointF(0, y), QPointF(width, y))

    def _draw_motor(self, painter: QPainter, colors: tuple[QColor, ...]) -> None:
        cyan, blue, amber, faint, _ = colors
        width, height = self.width(), self.height()
        center = QPointF(width * 0.5, height * 0.48)
        radius = min(width * 0.32, height * 0.25)
        rotation = self.phase * math.tau

        painter.setBrush(Qt.NoBrush)
        for ring, color, direction in ((1.0, faint, 1), (1.28, blue, -1)):
            painter.setPen(QPen(color, 2 if ring == 1 else 1))
            for index in range(12):
                angle = direction * rotation + index * math.tau / 12
                span = math.tau / 22
                rect = QRectF(center.x() - radius * ring, center.y() - radius * ring, radius * ring * 2, radius * ring * 2)
                painter.drawArc(rect, int(math.degrees(angle) * 16), int(math.degrees(span) * 16))

        painter.setPen(QPen(cyan, 2, Qt.SolidLine, Qt.RoundCap))
        for index in range(3):
            angle = rotation * 1.7 + index * math.tau / 3
            painter.drawLine(center, QPointF(center.x() + math.cos(angle) * radius * 0.78, center.y() + math.sin(angle) * radius * 0.78))
        painter.setPen(QPen(blue, 2))
        painter.drawEllipse(center, radius * 0.37, radius * 0.37)
        painter.setBrush(blue)
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(center, 4.0, 4.0)

        for index, speed in enumerate((0.8, -1.2, 1.6)):
            angle = rotation * speed + index * 2.1
            orbit = radius * (1.15 + index * 0.14)
            point = QPointF(center.x() + math.cos(angle) * orbit, center.y() + math.sin(angle) * orbit)
            painter.setBrush((cyan, blue, amber)[index])
            painter.drawEllipse(point, 2.6, 2.6)

        baseline = height * 0.88
        wave = QPainterPath()
        painter.setPen(QPen(cyan, 1.2))
        for index in range(42):
            x = width * 0.08 + width * 0.84 * index / 41
            y = baseline + math.sin(index * 0.58 + rotation * 2.0) * 4.0
            wave.moveTo(x, y) if index == 0 else wave.lineTo(x, y)
        painter.drawPath(wave)

    def _draw_signal(self, painter: QPainter, colors: tuple[QColor, ...]) -> None:
        cyan, blue, _, faint, _ = colors
        width, height = self.width(), self.height()
        left, right = width * 0.15, width * 0.91
        top, bottom = height * 0.10, height * 0.90
        lanes = 6

        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        for lane in range(lanes):
            y = top + (bottom - top) * lane / (lanes - 1)
            active = self.channels[lane]
            painter.setPen(cyan if active else blue)
            painter.drawText(QRectF(width * 0.025, y - 10, width * 0.10, 20), Qt.AlignVCenter, f"DI{lane + 1}")
            painter.setPen(QPen(faint, 1))
            painter.drawLine(QPointF(left, y), QPointF(right, y))
            for tick in range(8):
                x = left + (right - left) * tick / 7
                painter.drawLine(QPointF(x, y - 3), QPointF(x, y + 3))

        for lane in range(lanes):
            baseline = top + (bottom - top) * lane / (lanes - 1)
            history = self.channel_history[lane]
            path = QPainterPath(QPointF(left, baseline + (7 if not history[0] else -7)))
            segments = 14
            for segment in range(segments):
                x1 = left + (right - left) * segment / segments
                x2 = left + (right - left) * (segment + 1) / segments
                bit = history[segment]
                y = baseline - 7 if bit else baseline + 7
                path.lineTo(x1, y)
                path.lineTo(x2, y)
            trace = QColor(cyan if self.channels[lane] else blue)
            trace.setAlpha(220 if self.channels[lane] else 125)
            painter.setPen(QPen(trace, 2.0 if self.channels[lane] else 1.1, Qt.SolidLine, Qt.RoundCap))
            painter.drawPath(path)
            painter.setPen(Qt.NoPen)
            painter.setBrush(cyan if self.channels[lane] else faint)
            painter.drawEllipse(QPointF(right + 8, baseline), 3.2, 3.2)

        if self.connected:
            scan_x = left + self.phase * (right - left)
            scan = QColor(cyan)
            scan.setAlpha(58)
            painter.setPen(QPen(scan, 1.4))
            painter.drawLine(QPointF(scan_x, top - 10), QPointF(scan_x, bottom + 10))

    def _draw_matrix(self, painter: QPainter, colors: tuple[QColor, ...]) -> None:
        cyan, blue, amber, faint, _ = colors
        width, height = self.width(), self.height()
        columns, rows = 10, 6
        gap = min(width / 80, 7.0)
        cell_w = (width * 0.82 - gap * (columns - 1)) / columns
        cell_h = (height * 0.67 - gap * (rows - 1)) / rows
        origin_x, origin_y = width * 0.09, height * 0.12
        head = int(self.phase * (columns + rows + 4))

        for row in range(rows):
            for column in range(columns):
                distance = abs(column - (head - row) % columns)
                energy = max(0.0, 1.0 - distance / 3.2) if self.channels[row] else 0.0
                color = QColor((cyan, blue, amber)[(row + column) % 3])
                color.setAlpha((45 + int(energy * 180)) if self.channels[row] else 22)
                painter.setPen(QPen(faint, 1))
                painter.setBrush(color)
                x = origin_x + column * (cell_w + gap)
                y = origin_y + row * (cell_h + gap)
                painter.drawRoundedRect(QRectF(x, y, cell_w, cell_h), 2, 2)

        painter.setPen(QPen(cyan, 1.4))
        flow = QPainterPath(QPointF(origin_x, height * 0.88))
        for index in range(32):
            x = origin_x + width * 0.82 * index / 31
            envelope = 5 + 10 * abs(math.sin(index * 0.33 + self.phase * math.tau))
            y = height * 0.88 - envelope if index % 2 else height * 0.88 + envelope * 0.25
            flow.lineTo(x, y)
        painter.drawPath(flow)

    def _draw_network(self, painter: QPainter, colors: tuple[QColor, ...]) -> None:
        cyan, blue, amber, faint, _ = colors
        width, height = self.width(), self.height()
        nodes = (
            QPointF(width * 0.13, height * 0.50), QPointF(width * 0.34, height * 0.22),
            QPointF(width * 0.38, height * 0.76), QPointF(width * 0.63, height * 0.35),
            QPointF(width * 0.70, height * 0.76), QPointF(width * 0.88, height * 0.50),
        )
        edges = ((0, 1), (0, 2), (1, 3), (2, 3), (2, 4), (3, 5), (4, 5), (1, 4))
        painter.setPen(QPen(faint, 1))
        for start, end in edges:
            painter.drawLine(nodes[start], nodes[end])

        for edge_index, (start, end) in enumerate(edges):
            if not self.connected:
                continue
            progress = (self.phase * (1.0 + edge_index % 3 * 0.22) + edge_index * 0.137) % 1.0
            source, target = nodes[start], nodes[end]
            point = QPointF(source.x() + (target.x() - source.x()) * progress, source.y() + (target.y() - source.y()) * progress)
            painter.setBrush((cyan, blue, amber)[edge_index % 3])
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(point, 2.8, 2.8)

        for index, node in enumerate(nodes):
            pulse = 5.0 + (3.0 * (0.5 + 0.5 * math.sin(self.phase * math.tau * 2 + index)) if self.connected else 0.0)
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen((cyan, blue, amber)[index % 3], 1.4))
            painter.drawEllipse(node, pulse, pulse)
            painter.setBrush((cyan, blue, amber)[index % 3])
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(node, 2.7, 2.7)

        if self.connected:
            sweep = self.phase * math.tau
            hub = nodes[3]
            painter.setPen(QPen(QColor(cyan.red(), cyan.green(), cyan.blue(), 70), 1))
            painter.drawLine(hub, QPointF(hub.x() + math.cos(sweep) * width * 0.22, hub.y() + math.sin(sweep) * height * 0.30))

    def _draw_firmware(self, painter: QPainter, colors: tuple[QColor, ...]) -> None:
        cyan, blue, amber, faint, background = colors
        width, height = self.width(), self.height()
        chip = QRectF(width * 0.30, height * 0.20, width * 0.40, height * 0.50)
        painter.setBrush(background)
        painter.setPen(QPen(blue, 1.6))
        painter.drawRoundedRect(chip, 5, 5)

        pin_count = 7
        for index in range(pin_count):
            x = chip.left() + chip.width() * (index + 1) / (pin_count + 1)
            painter.setPen(QPen(faint, 2))
            painter.drawLine(QPointF(x, chip.top() - 8), QPointF(x, chip.top()))
            painter.drawLine(QPointF(x, chip.bottom()), QPointF(x, chip.bottom() + 8))

        blocks_x, blocks_y = 8, 5
        active = int(self.phase * (blocks_x * blocks_y + 8))
        cell_w = chip.width() * 0.72 / blocks_x
        cell_h = chip.height() * 0.48 / blocks_y
        start_x, start_y = chip.left() + chip.width() * 0.14, chip.top() + chip.height() * 0.25
        painter.setPen(Qt.NoPen)
        for row in range(blocks_y):
            for column in range(blocks_x):
                index = row * blocks_x + column
                color = QColor(cyan if index <= active else faint)
                color.setAlpha(190 if index <= active else 45)
                painter.setBrush(color)
                painter.drawRect(QRectF(start_x + column * cell_w, start_y + row * cell_h, cell_w - 2, cell_h - 2))

        scan_y = chip.top() + (self.phase * 1.7 % 1.0) * chip.height()
        painter.setPen(QPen(amber, 1.5))
        painter.drawLine(QPointF(chip.left() + 5, scan_y), QPointF(chip.right() - 5, scan_y))

        for index in range(12):
            angle = self.phase * math.tau * -1.3 + index * math.tau / 12
            radius_x, radius_y = width * 0.31, height * 0.38
            point = QPointF(width * 0.5 + math.cos(angle) * radius_x, height * 0.45 + math.sin(angle) * radius_y)
            painter.setBrush(amber if index == active % 12 else blue)
            painter.setPen(Qt.NoPen)
            painter.drawRect(QRectF(point.x() - 1.5, point.y() - 1.5, 3, 3))

        progress_left, progress_right = width * 0.17, width * 0.83
        progress_y = height * 0.86
        painter.setPen(QPen(faint, 3, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(QPointF(progress_left, progress_y), QPointF(progress_right, progress_y))
        painter.setPen(QPen(cyan, 3, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(QPointF(progress_left, progress_y), QPointF(progress_left + (progress_right - progress_left) * self.phase, progress_y))
