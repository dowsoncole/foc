"""Offline analysis checks with independently specified analytic responses."""
import unittest
import numpy as np
from measure_position_response import fit_trace, rates, analyze_step, snapshot_sane, phase_rms_from_trajectory
import struct
import tempfile
from pathlib import Path
from measure_position_response import retry_flash_words


class PositionAnalysisTests(unittest.TestCase):
    def test_sparse_flash_retry_requires_two_exact_reads(self):
        class Reader:
            swd_frequency=100000
            calls=0
            def dump(self,address,size,path):return address,size,path
            def run(self,commands,name):
                self.calls+=1
                for address,size,path in commands:path.write_bytes(bytes(range(8))[address-0x08000000:address-0x08000000+size])
        with tempfile.TemporaryDirectory() as folder:
            target=Reader()
            raw=b'\xff'+bytes(range(1,8))
            self.assertEqual(retry_flash_words(target,bytes(range(8)),0,raw,Path(folder)),bytes(range(8)))
            self.assertEqual(target.calls,2)
            self.assertEqual(target.swd_frequency,100000)

    def test_persistent_flash_mutation_is_never_corrected_to_expected(self):
        class Reader:
            swd_frequency=100000
            def dump(self,address,size,path):return path
            def run(self,commands,name):
                for path in commands:path.write_bytes(b'BAD!')
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(RuntimeError,'Persistent Flash mismatch'):
                retry_flash_words(Reader(),b'GOOD',0,b'BAD!',Path(folder))

    def test_large_or_truncated_flash_mismatch_is_refused(self):
        with tempfile.TemporaryDirectory() as folder:
            for raw in (b'X'*68,b'X'*67):
                with self.assertRaises(RuntimeError):
                    retry_flash_words(object(),bytes(68),0,raw,Path(folder))

    def test_finite_corrupted_reference_word_is_rejected(self):
        t=np.arange(64)*.004; w=2*np.pi*17
        rows=np.column_stack((t,228+2*np.cos(w*t),228+1.4*np.cos(w*t-1.4),t*0))
        rows[-2,1]=0  # Actual SWD corruption remained finite and ABI-shaped.
        with self.assertRaisesRegex(RuntimeError,'Reference trace'):
            fit_trace(rows.astype('<f4').tobytes(),17)

    def test_tracking_error_is_distinct_from_harmonic_residual(self):
        t=np.arange(64)/(20*64)
        rows=np.column_stack((t,2*np.cos(2*np.pi*20*t),
                              .8*np.cos(2*np.pi*20*t-np.pi/3),t*0))
        result,_=fit_trace(rows.astype('<f4').tobytes(),20)
        self.assertAlmostEqual(result['steady_tracking_rms_deg'],np.sqrt(1.52),places=5)
        self.assertLess(result['residual_rms_deg'],1e-5)

    def test_rms_is_per_phase_and_uses_sample_count(self):
        raw=bytearray(84)
        struct.pack_into('<3fI',raw,68,18,32,50,2)
        rms,n=phase_rms_from_trajectory(raw)
        self.assertEqual(n,2)
        self.assertEqual(rms,[3,4,5])
        struct.pack_into('<I',raw,80,0)
        self.assertEqual(phase_rms_from_trajectory(raw),(None,0))
        self.assertEqual(phase_rms_from_trajectory(bytes(68)),(None,0))

    def test_corrupt_rms_accumulator_cannot_appear_as_zero(self):
        for value in (-1,float('nan'),float('inf')):
            raw=bytearray(84)
            struct.pack_into('<3fI',raw,68,value,0,0,2)
            with self.assertRaises(RuntimeError):phase_rms_from_trajectory(raw)

    def test_real_fault_is_not_mistaken_for_a_bad_read(self):
        state=dict(position_state=2,position_fault=6,trajectory_state=3,trajectory_fault=6,
                   trace_count=64,samples=6000,mpt_valid=1,run_state=0,run_fault=0,user_fault=3,
                   mpt_deg=180,error_deg=.01,peak_error_deg=2,rms_error_deg=.2,
                   peak_command_rpm=4500,phase_peak_a=5)
        blocks=dict(trajectory=bytes(68),position=bytes(56))
        self.assertTrue(snapshot_sane(blocks,state,64))
        state['trace_count']=5844
        self.assertFalse(snapshot_sane(blocks,state,64))
        state['trace_count']=0
        raw=bytearray(84);struct.pack_into('<I',raw,80,2400000)
        blocks['trajectory']=raw
        self.assertTrue(snapshot_sane(blocks,state,64))
        struct.pack_into('<I',raw,80,2404001)
        self.assertFalse(snapshot_sane(blocks,state,64))

    def test_lag_and_gain_with_dc_offsets(self):
        t=2+np.arange(64)*.003
        w=2*np.pi*20
        rows=np.column_stack((t,180+2*np.cos(w*t),
                              180+.8*np.cos(w*t-np.pi/3),100*np.sin(w*t)))
        result,_=fit_trace(rows.astype('<f4').tobytes(),20)
        self.assertAlmostEqual(result['trace_gain_db'],20*np.log10(.4),places=3)
        self.assertAlmostEqual(result['trace_phase_deg'],-60,places=2)
        self.assertLess(result['residual_rms_deg'],1e-4)

    def test_phase_lead_is_a_measurable_response(self):
        t=np.arange(64)*.003;w=2*np.pi*20
        rows=np.column_stack((t,np.cos(w*t),np.cos(w*t+.2),t*0))
        result,_=fit_trace(rows.astype('<f4').tobytes(),20)
        self.assertGreater(result['trace_phase_deg'],0)

    def test_duplicate_tick_rejected(self):
        with self.assertRaises(RuntimeError):fit_trace(bytes(64*16),20)

    def test_constant_reference_rejected(self):
        t=np.arange(64)*.003
        rows=np.column_stack((t,t*0+180,np.sin(2*np.pi*20*t),t*0))
        with self.assertRaises(RuntimeError):fit_trace(rows.astype('<f4').tobytes(),20)

    def test_counter_wrap(self):
        b=dict(tick_ms=0xffffff00,requests=0xfffffff0,frames=0xfffffff0,updates=100)
        a=dict(tick_ms=744,requests=984,frames=984,updates=1100)
        self.assertEqual(rates(b,a),dict(requests_hz=1000,frames_hz=1000,updates_hz=1000))

    def test_first_order_step_metrics(self):
        t=np.arange(350)*.001
        x=np.where(t<.05,0.,2.)
        y=2*(1-np.exp(-np.maximum(0,t-.05)/.02))
        r=analyze_step(np.column_stack((t,x,y,t*0)))
        self.assertAlmostEqual(r['rise_10_90_s'],.02*np.log(9),delta=.001)
        self.assertAlmostEqual(r['settling_2pct_s'],-.02*np.log(.02),delta=.001)
        self.assertEqual(r['overshoot_pct'],0)

    def test_step_without_hold_or_edge_cannot_pass(self):
        t=np.arange(100)*.001;x=np.where(t<.05,0.,2.)
        y=2*(1-np.exp(-np.maximum(0,t-.05)/.02))
        self.assertIsNone(analyze_step(np.column_stack((t,x,y,t*0)))['settling_2pct_s'])
        with self.assertRaises(RuntimeError):analyze_step(np.column_stack((t,t*0,y,t*0)))


if __name__=='__main__':unittest.main()
