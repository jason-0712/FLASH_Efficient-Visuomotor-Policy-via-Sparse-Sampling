import unittest
import numpy as np
from flash_fr3_stream_plan import make_stream_segments,prefix_end,validate_stream_bounds
from flash_fr3_model_plan import evaluate_segment

class StreamPlanTests(unittest.TestCase):
    def setUp(self):
        self.q=np.array([-.16,-.78,.11,-2.38,.07,1.56,.66])
        c=np.zeros((7,8));c[0,:7]=self.q;c[1,1]=.1
        self.curve={'basis':'legendre','coefficients':c.tolist(),'anchor_s':2/11,
                    'polynomial_duration_seconds':44/15,'valid_future_seconds':32/15}

    def test_nonzero_derivatives_survive_time_scaling_and_join(self):
        initial=np.stack([self.q,np.full(7,.001),np.full(7,.002)])
        p=make_stream_segments(self.curve,.5,initial,self.q)
        c=np.asarray(p['coefficients'])[0];d=p['segment_durations'][0]
        for k in range(3):
            expected=initial[k]-(self.q if k==0 else 0.)
            np.testing.assert_allclose(evaluate_segment(c,d,0.,k),expected,atol=1e-13)
        h=1e-5;t=.2
        for k in range(1,4):
            fd=(evaluate_segment(c,d,t+h,k-1)-evaluate_segment(c,d,t-h,k-1))/(2*h)
            np.testing.assert_allclose(fd,evaluate_segment(c,d,t,k),atol=1e-9)
        p2=make_stream_segments(self.curve,.6,prefix_end(p),self.q)
        for k in range(3):
            np.testing.assert_allclose(evaluate_segment(np.asarray(p2['coefficients'])[0],d,0,k),
                evaluate_segment(c,d,d,k),atol=1e-13)

    def test_extremum_bound_covers_dense_samples_and_rejects_large_motion(self):
        from flash_fr3_stream_plan import absolute_bound
        p=make_stream_segments(self.curve,.5,np.stack([self.q,np.zeros(7),np.zeros(7)]),self.q)
        self.assertTrue(validate_stream_bounds(p)['pass'])
        for c,d in zip(np.asarray(p['coefficients']),p['segment_durations']):
            for k in range(4):
                dense=np.abs([evaluate_segment(c,d,t,k) for t in np.linspace(0,d,1001)]).max(axis=0)
                self.assertTrue((absolute_bound(c,d,k)>=dense-1e-12).all())
        p['coefficients'][0][3][1]+=1
        with self.assertRaises(ValueError):validate_stream_bounds(p)

    def test_reject_stale_prediction_and_invalid_time_or_initial(self):
        initial=np.stack([self.q,np.zeros(7),np.zeros(7)])
        for age in (-.01,.71,np.nan):
            with self.assertRaises(ValueError):make_stream_segments(self.curve,age,initial,self.q)
        with self.assertRaises(ValueError):make_stream_segments(self.curve,.5,initial[:2],self.q)

    def test_three_prefixes_keep_one_origin_and_nonzero_derivatives(self):
        initial=np.stack([self.q,np.zeros(7),np.zeros(7)])
        for sequence in range(3):
            p=make_stream_segments(self.curve,.46,initial,self.q,source_prefix=.04,time_scale=12.5)
            self.assertEqual(p['segment_durations'],[.5,.5])
            np.testing.assert_array_equal(p['expected_start_q'],self.q)
            self.assertTrue(validate_stream_bounds(p)['pass'])
            c=np.asarray(p['coefficients'])[0]
            for k in range(3):
                expected=initial[k]-(self.q if k==0 else 0.)
                np.testing.assert_allclose(evaluate_segment(c,.5,0.,k),expected,atol=1e-13)
            initial=prefix_end(p)
            self.assertGreater(abs(initial[1,1]),0.)

if __name__=='__main__':unittest.main()
