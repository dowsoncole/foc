"""Synthetic voltage-record tests; never connects to a target."""
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest

from analyze_encoder_alignment import analyze_run, field_angle
from paired_static_alignment import point_rejections


class VoltageRecordsTests(unittest.TestCase):
    def fixture(self, root):
        (root/'before').mkdir()
        (root/'armed').mkdir()
        (root/'stopped').mkdir()
        (root/'before/voltage_alignment_profile_version.bin').write_bytes(struct.pack('<I', 1))
        for name in ('armed', 'stopped'):
            (root/name/'diagnostics.bin').write_bytes(bytes(128))
        alignment=dict(method='voltage', voltage_v=1, voltage_contract_version=1,
                       duration_s=5, angle_end_rad=math.pi/2, start_field_rad=0.4,
                       nominal_dc_bus_voltage_v=24)
        (root/'metadata.json').write_text(json.dumps(dict(alignment=alignment)))
        counters=dict(crc_errors=0,step_errors=0,alarms=0,start_errors=0,transfer_errors=0)
        after=dict(counters,speed_test_state=3,speed_test_fault=0,runtime_fault=0,controller_fault=0,moe=False)
        (root/'summary.json').write_text(json.dumps(dict(armed=counters,after=after)))
        data=bytearray(4416)
        struct.pack_into('<I',data,12,125)
        struct.pack_into('<I9f2I',data,64,10,5,1,.4,0,math.pi/2,0,0,15,20,40,16)
        struct.pack_into('<iIf',data,120,-1,2,5.3988)
        for i in range(125):
            t=i*.04
            angle=field_angle(t,5,math.pi/2,.4)
            # Nonzero Iq and an Id far from 1 are NOT voltage tracking errors.
            row=(t,angle,0,3,.3,min(1,t/2),((angle-5.6)/2)%(2*math.pi),0)
            struct.pack_into('<8f',data,320+32*i,*row)
        (root/'stopped/test.bin').write_bytes(data)
        return data

    def test_voltage_units_and_current_not_used_as_command(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); self.fixture(root)
            r=analyze_run(root)
            self.assertEqual(r['kind'],10)
            self.assertEqual(r['commanded_ud_v'],1)
            self.assertNotIn('commanded_id_amplitude_a',r)
            self.assertNotIn('rms_tail_id_error_a',r)
            self.assertAlmostEqual(r['estimated_offset_rad'],5.6,places=5)
            self.assertLess(r['rms_tail_ud_error_v'],1e-7)
            self.assertGreater(r['rms_tail_iq_command_frame_a'],.29)
            self.assertEqual(point_rejections(dict(bin=0,analysis=r),r['tail_mechanical_deg']),[])

    def test_wrong_profile_or_mixed_units_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); self.fixture(root)
            (root/'before/voltage_alignment_profile_version.bin').write_bytes(struct.pack('<I',2))
            with self.assertRaises(ValueError): analyze_run(root)
            (root/'before/voltage_alignment_profile_version.bin').write_bytes(struct.pack('<I',1))
            meta=json.loads((root/'metadata.json').read_text())
            meta['alignment']['current_a']=1
            (root/'metadata.json').write_text(json.dumps(meta))
            with self.assertRaises(ValueError): analyze_run(root)

    def test_bad_applied_voltage_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); data=self.fixture(root)
            for i in range(112,125): struct.pack_into('<f',data,320+i*32+5*4,.8)
            (root/'stopped/test.bin').write_bytes(data)
            r=analyze_run(root)
            self.assertTrue(point_rejections(dict(bin=0,analysis=r),r['tail_mechanical_deg']))


if __name__=='__main__': unittest.main()
