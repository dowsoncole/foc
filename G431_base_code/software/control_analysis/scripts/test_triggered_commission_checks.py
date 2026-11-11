import unittest
from run_triggered_commission import require_trigger_healthy


class TriggerChecks(unittest.TestCase):
    def test_gate(self):
        good = dict(logic_version=1, logic_mode=2, logic_state=1, logic_reason=0,
                    zero_fields=[0, 0, 3, 0, 100, 0, 0, 1234])
        require_trigger_healthy(good)
        require_trigger_healthy(dict(good, pwm_commit_fault_count=0))
        with self.assertRaises(RuntimeError):
            require_trigger_healthy(dict(good, pwm_commit_fault_count=1))
        for key, value in (('logic_version', 0), ('logic_mode', 1),
                           ('logic_state', 0), ('logic_state', 2), ('logic_reason', 2)):
            with self.assertRaises(RuntimeError):
                require_trigger_healthy(dict(good, **{key: value}))
        for index in (0, 1, 2, 3, 5, 6):
            fields = good['zero_fields'].copy()
            fields[index] = 7
            with self.assertRaises(RuntimeError):
                require_trigger_healthy(dict(good, zero_fields=fields))


if __name__ == '__main__':
    unittest.main()
