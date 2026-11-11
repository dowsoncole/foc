"""Offline ABI/qualification checks for customer finite-window results."""
import math
import struct
import unittest
from analyze_customer_position_test import decode


class CustomerResultsTests(unittest.TestCase):
    def table(self,flags=0):
        return b''.join(struct.pack('<5f3I',i,.81,.405,-6.0206,-1.8*i,
                                    2000,flags,400 if flags==8 else 0) for i in range(1,31))

    def test_valid_full_table(self):
        rows=decode(self.table())
        self.assertEqual(len(rows),30)
        self.assertTrue(all(r['protocol_complete'] for r in rows))
        self.assertAlmostEqual(rows[0]['gain_db'],-6.0206,places=4)

    def test_slot_loss_separate_from_numerical_estimate(self):
        rows=decode(self.table(8))
        self.assertTrue(all(r['estimate_usable'] for r in rows))
        self.assertFalse(any(r['protocol_complete'] for r in rows))

    def test_clipped_aborted_short_or_degenerate_not_qualified(self):
        for flag in (1,2,4,16,32):
            self.assertFalse(any(r['estimate_usable'] for r in decode(self.table(flag))))

    def test_unrun_rows_and_nan_are_not_measurements(self):
        data=struct.pack('<5f3I',0,0,0,math.nan,math.nan,0,32,0)*30
        self.assertFalse(any(r['estimate_usable'] for r in decode(data)))
        self.assertIsNone(decode(data)[0]['gain_db'])

    def test_wrong_length_frequency_or_flags_rejected(self):
        for data in (b'',self.table()[:-4],self.table()[:32]*30,self.table(64)):
            with self.assertRaises(ValueError):decode(data)

    def test_inconsistent_missed_counter_rejected(self):
        data=bytearray(self.table());struct.pack_into('<I',data,28,1)
        with self.assertRaises(ValueError):decode(data)

    def test_zero_fundamental_not_qualified(self):
        data=bytearray(self.table());struct.pack_into('<f',data,4,0)
        self.assertFalse(decode(data)[0]['estimate_usable'])


if __name__=='__main__':unittest.main()
