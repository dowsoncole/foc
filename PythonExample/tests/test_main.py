import unittest

from servo_core import PARAMETERS, ServoModel, clamp


class ServoModelTests(unittest.TestCase):
    def test_clamp_limits_values(self) -> None:
        self.assertEqual(clamp(-2, 0, 10), 0)
        self.assertEqual(clamp(12, 0, 10), 10)

    def test_parameter_indexes_are_unique(self) -> None:
        indexes = [(row[0], row[1]) for row in PARAMETERS]
        self.assertEqual(len(indexes), len(set(indexes)))

    def test_disconnected_drive_does_not_move(self) -> None:
        model = ServoModel(target=180.0, enabled=True, connected=False)
        model.step(50, "位置模式")
        self.assertEqual(model.position, 0.0)

    def test_enabled_drive_moves_toward_target(self) -> None:
        model = ServoModel(target=90.0, speed_limit=600.0, enabled=True, connected=True)
        model.step(50, "位置模式")
        self.assertGreater(model.position, 0.0)
        self.assertGreater(model.speed, 0.0)

    def test_speed_mode_tracks_signed_rpm_command(self) -> None:
        model = ServoModel(target=-1200.0, speed_limit=2000.0, enabled=True, connected=True)
        for _ in range(40):
            model.step(50, "速度模式")
        self.assertLess(model.speed, -1000.0)
        self.assertLess(model.position, 0.0)

    def test_speed_mode_obeys_speed_limit(self) -> None:
        model = ServoModel(target=5000.0, speed_limit=900.0, enabled=True, connected=True)
        for _ in range(80):
            model.step(50, "速度模式")
        self.assertAlmostEqual(model.speed, 900.0, delta=1.0)

    def test_temperature_stays_in_safe_simulation_bounds(self) -> None:
        model = ServoModel(target=100000.0, speed_limit=6000.0, enabled=True, connected=True)
        for _ in range(5000):
            model.step(50, "位置模式")
        self.assertGreaterEqual(model.temperature, 30.0)
        self.assertLessEqual(model.temperature, 88.0)


if __name__ == "__main__":
    unittest.main()
