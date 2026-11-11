import unittest
from audit_encoder_transactions import audit, classify


def frame(tx='30 09 00', rx='FF FF 6F'):
    return dict(partial_start='0', partial_end='0', included='0',
                rising_clocks='24', falling_clocks='24', tx_hex=tx, rx_hex=rx,
                excluded_reasons='clock_count_not_48;not_expected_a003_angle_burst',
                sample_edge_ambiguities='0', zero_width_events='0',
                cs_setup_ns='360', cs_hold_ns='1160')


class AuditTests(unittest.TestCase):
    def test_normal_zero_reads_are_not_truncated_angles(self):
        rows = [frame(), frame('30 0A 00', 'FF FF 10')]*3
        result = audit(rows)
        self.assertEqual(result['categories'], {'zero_read': 6})
        self.assertEqual(result['constant_zero_pos'], 0x6F1)

    def test_invalid_or_ambiguous_read_not_reclassified(self):
        for change in ({'rising_clocks': '23'}, {'rx_hex': 'FF FF'},
                       {'excluded_reasons': 'data_changes_at_sampling_edge'},
                       {'tx_hex': '50 00 00'}):
            self.assertEqual(classify(dict(frame(), **change)), 'unrecognized')
        self.assertEqual(classify(dict(frame(), partial_end='1')), 'partial')

    def test_variable_register_response_not_combined(self):
        self.assertIsNone(audit([frame(), frame(rx='FF FF 70'),
                                frame('30 0A 00', 'FF FF 10')])['constant_zero_pos'])
        self.assertIsNone(audit([frame(rx='00 00 6F'),
                                frame('30 0A 00', 'FF FF 10')])['constant_zero_pos'])


if __name__ == '__main__':
    unittest.main()
