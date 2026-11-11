import unittest
import numpy as np
from measure_speed_response import analyze

DTYPE=np.dtype([('sequence','<u4'),('time','<f4'),('command','<f4'),
    ('applied','<f4'),('rpm','<f4'),('iq','<f4'),('raw_angle','<f4'),('iq_ref','<f4')])


def make_rows(t,ref,y):
    rows=np.zeros(len(t),dtype=DTYPE)
    rows['time']=t; rows['sequence']=np.arange(len(t))*20
    rows['command']=rows['applied']=ref; rows['rpm']=y
    position=np.cumsum(y*np.median(np.diff(t))/60)
    rows['raw_angle']=np.mod(-position*2097152.,2097152.)
    rows['iq_ref']=rows['iq']=.2*np.sin(2*np.pi*20*t)
    return rows


class MetricsTest(unittest.TestCase):
    def test_first_order_step(self):
        t=np.arange(512)*.002
        ref=np.where(t<.3,300.,400.)
        y=np.where(t<.3,300.,400.-100*np.exp(-np.maximum(0,t-.3)/.02))
        result,_=analyze(make_rows(t,ref,y),'step',0,.3)
        self.assertAlmostEqual(result['rise_10_90_s'],.044,delta=.003)
        self.assertAlmostEqual(result['settling_s'],.08,delta=.003)
        self.assertLess(abs(result['steady_error_rpm']),.01)

    def test_sine_complex_gain(self):
        t=np.arange(512)*.001; f=20.
        h=1/(1+1j*f/30)
        ref=300+50*np.sin(2*np.pi*f*t)
        y=300+50*abs(h)*np.sin(2*np.pi*f*t+np.angle(h))
        result,_=analyze(make_rows(t,ref,y),'sine',f,.1)
        self.assertAlmostEqual(result['gain_db'],20*np.log10(abs(h)),places=4)
        self.assertAlmostEqual(result['phase_deg'],np.angle(h,deg=True),places=3)
        self.assertGreater(result['measured_cycles'],4)

    def test_missing_tick_rejected(self):
        t=np.arange(512)*.002; ref=np.where(t<.3,300.,400.)
        rows=make_rows(t,ref,ref); rows['time'][200:]+=.003
        with self.assertRaisesRegex(RuntimeError,'Missing'): analyze(rows,'step',0,.3)

    def test_160hz_with_nonuniform_timestamps(self):
        t=np.cumsum(np.resize([.001,.00105],512));f=160.;h=.2*np.exp(-2j)
        ref=300+50*np.sin(2*np.pi*f*t)
        y=300+50*abs(h)*np.sin(2*np.pi*f*t+np.angle(h))
        rows=make_rows(t,ref,y)
        rows['iq_ref']=.2+.1*np.sin(2*np.pi*f*t)
        result,_=analyze(rows,'sine',f,.1)
        self.assertAlmostEqual(result['gain_db'],20*np.log10(abs(h)),delta=.001)
        self.assertAlmostEqual(result['phase_deg'],np.angle(h,deg=True),delta=.01)

    def test_low_frequency_after_one_second_warmup(self):
        t=.9+np.arange(512)*.00615;f=2.;h=.95*np.exp(-.12j)
        ref=300+np.where(t<1.,0.,50*np.sin(2*np.pi*f*(t-1.)))
        y=300+np.where(t<1.,0.,50*abs(h)*np.sin(2*np.pi*f*(t-1.)+np.angle(h)))
        rows=make_rows(t,ref,y);rows['iq_ref']=.2+.05*np.sin(2*np.pi*f*t)
        result,_=analyze(rows,'sine',f,1.)
        self.assertAlmostEqual(result['gain_db'],20*np.log10(abs(h)),places=4)
        self.assertAlmostEqual(result['phase_deg'],np.angle(h,deg=True),places=3)
        self.assertGreater(result['measured_cycles'],5)

    def test_unsettled_tail_not_reported_settled(self):
        t=np.arange(512)*.002; ref=np.where(t<.3,300.,400.)
        y=ref+np.where(t<.3,0.,5*np.sin(2*np.pi*10*t))
        result,_=analyze(make_rows(t,ref,y),'step',0,.3)
        self.assertIsNone(result['settling_s'])


if __name__=='__main__': unittest.main()
