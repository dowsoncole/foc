import copy
import struct
import unittest

from encoder_logic_capture import BOARD_ENCODER_OFFSET_RAD, require_waiting


class WaitingGateTests(unittest.TestCase):
    def setUp(self):
        self.blocks = dict(logic_trigger=struct.pack('<10I8s', 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, bytes(8)),
                           paused=b'\x01', busy_flag=b'\0', gpioa_idr=bytes(4),
                           gpioa_odr=bytes(4), gpioa_moder=struct.pack('<I', 1),
                           sensor_zero_guard=bytes(32), zero_fault_capture=bytes(328),
                           capture=bytes(312), diagnostics=bytes(128))
        self.state = dict(moe=False, enable=0, runtime_bytes=[0,0,0,0,1,0],
                          controller_running=0, encoder_sequence=0, calibration_ok=0,
                          i2t_initial_energy_a2s=83.0)

    def test_clean_wait(self):
        require_waiting(self.blocks, self.state)

    def test_motion_profile_explicit(self):
        blocks = copy.deepcopy(self.blocks)
        raw = bytearray(blocks['logic_trigger'])
        struct.pack_into('<I', raw, 4, 2)
        blocks['logic_trigger'] = bytes(raw)
        with self.assertRaises(RuntimeError):
            require_waiting(blocks, self.state)
        require_waiting(blocks, self.state, expected_mode=2)

    def test_reject_each_wrong_trigger_field(self):
        for offset in range(0, 48, 4):
            blocks = copy.deepcopy(self.blocks)
            raw = bytearray(blocks['logic_trigger'])
            struct.pack_into('<I', raw, offset, 4)
            blocks['logic_trigger'] = bytes(raw)
            with self.assertRaises(RuntimeError):
                require_waiting(blocks, self.state)

    def test_reject_activity(self):
        for key in ('sensor_zero_guard', 'zero_fault_capture', 'capture', 'diagnostics'):
            blocks = copy.deepcopy(self.blocks)
            blocks[key] = b'\x01' + blocks[key][1:]
            with self.assertRaises(RuntimeError):
                require_waiting(blocks, self.state)

    def test_accept_pa0_adc_pulse_while_waiting(self):
        # Non-atomic SWD reads may catch either edge, or disagree with each other.
        for idr, odr in ((0, 1), (1, 0), (1, 1)):
            blocks = dict(self.blocks, gpioa_idr=struct.pack('<I', idr),
                          gpioa_odr=struct.pack('<I', odr))
            require_waiting(blocks, self.state)

    def test_reject_pa0_not_output(self):
        for mode in (0, 2, 3):
            blocks = dict(self.blocks, gpioa_moder=struct.pack('<I', mode))
            with self.assertRaises(RuntimeError):
                require_waiting(blocks, self.state)

    def test_reject_paused_or_busy(self):
        for key, value in (('paused', b'\0'), ('busy_flag', b'\x01')):
            blocks = dict(self.blocks, **{key: value})
            with self.assertRaises(RuntimeError):
                require_waiting(blocks, self.state)

    def test_reject_motor_or_old_calibration(self):
        for key in ('moe', 'enable', 'controller_running', 'encoder_sequence', 'calibration_ok'):
            with self.assertRaises((RuntimeError, AssertionError)):
                require_waiting(self.blocks, dict(self.state, **{key: 1}))

    def test_accept_saved_boot_calibration(self):
        test = bytearray(192)
        struct.pack_into('<IIiIf', test, 112, 0, 1, -1, 2, BOARD_ENCODER_OFFSET_RAD)
        require_waiting(dict(self.blocks, test=bytes(test)), dict(self.state, calibration_ok=1))

    def test_reject_changed_boot_calibration(self):
        valid = [0, 1, -1, 2, BOARD_ENCODER_OFFSET_RAD]
        for index, value in ((0, 1), (1, 0), (1, 2), (2, 1), (3, 3),
                             (4, 5.3988), (4, float('nan')), (4, float('inf'))):
            with self.subTest(index=index, value=value):
                fields = valid.copy()
                fields[index] = value
                test = bytearray(192)
                struct.pack_into('<IIiIf', test, 112, *fields)
                with self.assertRaises(RuntimeError):
                    require_waiting(dict(self.blocks, test=bytes(test)),
                                    dict(self.state, calibration_ok=1))

    def test_saved_calibration_does_not_bypass_fault_record(self):
        test = bytearray(192)
        struct.pack_into('<IIiIf', test, 112, 0, 1, -1, 2, BOARD_ENCODER_OFFSET_RAD)
        blocks = dict(self.blocks, test=bytes(test), diagnostics=b'\x01' + bytes(127))
        with self.assertRaises(RuntimeError):
            require_waiting(blocks, dict(self.state, calibration_ok=1))

    def test_keep_thermal_debt(self):
        with self.assertRaises(RuntimeError):
            require_waiting(self.blocks, dict(self.state, i2t_initial_energy_a2s=0))


if __name__ == '__main__':
    unittest.main()
