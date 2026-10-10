"""Small reusable transitions for the desktop interface."""

from __future__ import annotations

from PyQt5.QtCore import QEasingCurve, QPoint, QPropertyAnimation
from PyQt5.QtWidgets import QStackedWidget, QWidget


class AnimatedStackedWidget(QStackedWidget):
    """A position-only transition that is safe with translucent windows."""

    def __init__(self, parent: QWidget | None = None, duration: int = 220) -> None:
        super().__init__(parent)
        self.duration = duration
        self._animation: QPropertyAnimation | None = None
        self._animated_widget: QWidget | None = None
        self._final_position = QPoint()

    def switch_to(self, index: int) -> None:
        if index < 0 or index >= self.count() or index == self.currentIndex():
            return
        previous = self.currentIndex()
        self._finish_transition()
        self.setCurrentIndex(index)
        page = self.currentWidget()
        if page is None:
            return

        self._animated_widget = page
        self._final_position = page.pos()
        direction = 1 if index > previous else -1
        start_position = self._final_position + QPoint(18 * direction, 0)
        page.move(start_position)

        slide = QPropertyAnimation(page, b"pos", self)
        slide.setDuration(self.duration)
        slide.setStartValue(start_position)
        slide.setEndValue(self._final_position)
        slide.setEasingCurve(QEasingCurve.OutCubic)
        self._animation = slide
        self._animation.finished.connect(self._finish_transition)
        self._animation.start()

    def _finish_transition(self) -> None:
        if self._animation is not None:
            self._animation.stop()
            self._animation.deleteLater()
            self._animation = None
        if self._animated_widget is not None:
            self._animated_widget.move(self._final_position)
            self._animated_widget = None
