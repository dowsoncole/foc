"""Offscreen interaction tests for the oscilloscope view."""

import os
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtCore import Qt, QEvent, QPoint, QPointF
from PyQt5.QtGui import QWheelEvent
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication, QLabel, QToolButton

from servo_ui import ScopeChart, ServoWindow


class ScopeChartViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_time_zoom_and_pan_are_bounded(self) -> None:
        chart = ScopeChart()
        chart.resize(1000, 500)
        chart.set_capacity(100)
        chart.set_sample_rate(1000)
        for index in range(100):
            chart.add_sample((index, index * 2, -index, 0.0))
        chart.zoom_time(2)
        self.assertLess(chart.time_window_samples, 100)
        start, end = chart.visible_bounds(100)
        self.assertEqual(end, 100)
        chart.pan_time(300)
        older_start, older_end = chart.visible_bounds(100)
        self.assertLess(older_end, end)
        self.assertGreaterEqual(older_start, 0)
        chart.pan_time(-10_000)
        self.assertEqual(chart.view_end_offset, 0)

    def test_hover_space_wheel_without_click(self) -> None:
        window = ServoWindow()
        window.pages.setCurrentIndex(1)
        window.show()
        window.open_scope_popup()
        for chart in (window.scope, window.scope_popup.chart):
            active_window = window if chart is window.scope else window.scope_popup
            active_window.activateWindow()
            self.app.processEvents()
            window.scope.clear()
            for i in range(100):
                window.scope.add_sample((i,) * 4)
            self.app.sendEvent(chart, QEvent(QEvent.Enter))
            self.assertTrue(chart.hasFocus())
            QTest.keyPress(chart, Qt.Key_Space)
            initial_range = chart.y_range
            pos = chart.plot_area().center()
            wheel = QWheelEvent(pos, QPointF(chart.mapToGlobal(pos.toPoint())),
                                QPoint(), QPoint(0, 120), Qt.NoButton,
                                Qt.NoModifier, Qt.NoScrollPhase, False)
            self.app.sendEvent(chart, wheel)
            self.assertLess(chart.time_window_samples, 100)
            self.assertEqual(chart.y_range, initial_range)
            self.app.sendEvent(chart, QEvent(QEvent.FocusOut))
            self.assertFalse(chart.space_pressed)
            QTest.keyRelease(chart, Qt.Key_Space)
        window.close()

    def test_time_zoom_keeps_cursor_sample_fixed(self) -> None:
        chart = ScopeChart()
        chart.resize(1000, 500)
        chart.set_capacity(1000)
        for index in range(1000):
            chart.add_sample((index,) * 4)
        for ratio in (0.0, 0.23, 0.5, 1.0):
            chart.time_window_samples = None
            chart.view_end_offset = 0
            x = chart.plot_area().left() + chart.plot_area().width() * ratio
            anchor = ratio * 999
            for steps in (2, -1):
                chart.zoom_time(steps, x)
                start, end = chart.visible_bounds(1000)
                self.assertAlmostEqual(start + ratio * (end - start - 1), anchor, delta=1)

    def test_receive_interval_is_fixed_at_100ms(self) -> None:
        window = ServoWindow()
        # The acquisition period is fixed at 100 ms and no longer exposed.
        window.set_scope_period("100 ms")
        window.waveform_source_combo.setCurrentText("内置正弦演示")
        self.assertEqual(window.scope_timer.interval(), 100)
        self.assertEqual(window.scope.sample_rate_hz, 1000)
        window.waveform_source.emit(10)
        window.sample_scope()
        self.assertGreaterEqual(len(window.scope.data[0]), 10)
        window.close()

    def test_popup_shares_data_but_has_independent_view(self) -> None:
        window = ServoWindow()
        window.scope.add_sample((1.0, 2.0, 3.0, 4.0))
        window.open_scope_popup()
        popup = window.scope_popup
        self.assertIsNotNone(popup)
        self.assertIs(popup.chart.data_source, window.scope)
        self.assertEqual(list(popup.chart.series_data()[1]), [2.0])
        self.assertFalse(popup.windowFlags() & Qt.FramelessWindowHint)
        self.assertTrue(popup.windowFlags() & Qt.WindowTitleHint)
        self.assertFalse(popup.testAttribute(Qt.WA_DeleteOnClose))
        self.assertEqual(popup.windowTitle(), "实时波形 - 独立视图")
        self.assertIs(popup.overlay_controls.parentWidget(), popup.chart)
        self.assertIs(popup.help_panel.parentWidget(), popup.chart)
        self.assertIsNone(popup.findChild(QToolButton, "scope-close-button"))
        popup.chart.y_range = 10.0
        self.assertNotEqual(popup.chart.y_range, window.scope.y_range)
        self.assertFalse(popup.help_panel.isHidden())
        self.assertTrue(popup.help_button.isChecked())
        popup.help_panel.lower()
        popup._position_overlays()
        self.assertTrue(popup.help_panel.isVisible())
        self.assertEqual(len(popup.help_panel.findChildren(QLabel)), 8)
        window.close()

    def test_display_reduction_preserves_spikes(self) -> None:
        values = [0.0] * 10000
        values[1234], values[1235] = 900.0, -800.0
        points = list(ScopeChart.display_points(values, 400))
        self.assertIn((1234, 900.0), points)
        self.assertIn((1235, -800.0), points)
        self.assertLessEqual(len(points), 1600)
        self.assertEqual(points, sorted(points))

    def test_scope_heading_is_outside_chart_panel(self) -> None:
        window = ServoWindow()
        window.pages.setCurrentIndex(1)
        window.show()
        self.app.processEvents()
        self.assertTrue(window.scope_panel.rect().contains(window.scope.geometry()))
        self.assertGreaterEqual(window.scope.height(), 270)
        self.assertIs(window.scope_heading_label.parentWidget(), window.scope_panel.parentWidget())
        self.assertIs(window.scope.parentWidget(), window.scope_panel)
        self.assertIsNot(window.scope_heading_label.parentWidget(), window.scope_panel)
        margins = window.scope_panel.layout().contentsMargins()
        self.assertEqual(
            (margins.left(), margins.top(), margins.right(), margins.bottom()),
            (1, 1, 1, 1),
        )
        self.assertEqual(window.scope_panel.layout().spacing(), 0)
        window.close()

    def test_scope_source_exception_is_isolated(self) -> None:
        class BrokenSource:
            rate_hz = 1000

            def drain(self):
                raise RuntimeError("test transport failure")

            def close(self):
                pass

        window = ServoWindow()
        window.waveform_source = BrokenSource()
        window.scope_timer.start()
        with patch("servo_ui.write_crash_report") as report:
            window.sample_scope()
        self.assertTrue(window.scope_faulted)
        self.assertFalse(window.scope_timer.isActive())
        self.assertIsNone(window.waveform_source)
        self.assertIn("安全停止", window.waveform_status.text())
        report.assert_called_once()
        window.close()

    def test_popup_can_be_closed_and_reopened(self) -> None:
        window = ServoWindow()
        for _ in range(3):
            window.open_scope_popup()
            popup = window.scope_popup
            self.assertTrue(popup.refresh_timer.isActive())
            popup.close()
            self.app.processEvents()
            self.assertFalse(popup.refresh_timer.isActive())
        window.open_scope_popup()
        self.assertTrue(window.scope_popup.isVisible())
        window.close()

    def test_statistics_use_selected_channel(self) -> None:
        window = ServoWindow()
        window.waveform_source_combo.setCurrentText("内置正弦演示")
        window.waveform_source.emit(1)
        window.sample_from_transport()
        window.scope.clear()
        window.scope.add_sample((1.0, 10.0, 100.0, 1000.0))
        window.scope.add_sample((3.0, 30.0, 300.0, 3000.0))
        window.scope_statistics_channel.setCurrentIndex(1)
        window.update_scope_statistics()
        self.assertEqual(window.scope.selected_channel, 1)
        self.assertEqual(window.scope_stats[0].text(), "30.0 A")
        self.assertEqual(window.scope_stats[1].text(), "10.0 A")
        window.close()

    def test_data_source_has_no_implicit_simulation_model(self) -> None:
        window = ServoWindow()
        items = [window.waveform_source_combo.itemText(index) for index in range(window.waveform_source_combo.count())]
        self.assertEqual(items, ["RS485 串口", "内置正弦演示"])
        self.assertFalse(window.scope_running)
        self.assertFalse(window.scope_action_buttons[0].isEnabled())
        window.sample_scope()
        self.assertEqual(len(window.scope.data[0]), 0)
        window.close()


if __name__ == "__main__":
    unittest.main()
