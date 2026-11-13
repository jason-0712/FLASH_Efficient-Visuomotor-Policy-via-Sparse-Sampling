import unittest
import numpy as np
from flash_fr3_task_plan import make_task_segments,task_bounds,TaskSequence,TASK_LIMITS
from flash_fr3_model_plan import evaluate_segment

class TaskTests(unittest.TestCase):
    def setUp(self):
        self.q=np.array([-.16,-.77,.11,-2.38,.07,1.57,.66])
        c=np.zeros((7,8));c[0,:7]=self.q;c[1,1]=.1
        self.curve={'basis':'legendre','coefficients':c.tolist(),'anchor_s':2/11,
          'polynomial_duration_seconds':44/15,'valid_future_seconds':32/15}
    def test_plan_limits_c2_and_physical_derivatives(self):
        p=make_task_segments(self.curve,.4,self.q);self.assertTrue(task_bounds(p)['pass'])
        c=np.array(p['coefficients']);d=p['segment_durations']
        for k in range(3):
            np.testing.assert_allclose(evaluate_segment(c[0],d[0],0,k),0,atol=1e-12)
            np.testing.assert_allclose(evaluate_segment(c[0],d[0],d[0],k),evaluate_segment(c[1],d[1],0,k),atol=1e-12)
        for k in (1,2):np.testing.assert_allclose(evaluate_segment(c[1],d[1],d[1],k),0,atol=1e-12)
        h=1e-5
        for k in (1,2,3):
            fd=(evaluate_segment(c[0],d[0],1+h,k-1)-evaluate_segment(c[0],d[0],1-h,k-1))/(2*h)
            np.testing.assert_allclose(fd,evaluate_segment(c[0],d[0],1,k),atol=1e-8)
    def test_bad_bounds_stale_or_discontinuous_plan_rejected(self):
        p=make_task_segments(self.curve,.4,self.q);p['expected_start_q'][4]=.5
        self.assertFalse(task_bounds(p)['pass'])
        with self.assertRaises(ValueError):make_task_segments(self.curve,.71,self.q)
        p=make_task_segments(self.curve,.4,self.q);p['coefficients'][1][0][0]+=.01
        with self.assertRaises(ValueError):task_bounds(p)
    def test_grasp_then_transport_then_release_requires_new_confirmations(self):
        f=TaskSequence([self.q],[self.q])
        self.assertEqual(f.observe(.9,self.q,self.q,1),'confirm')
        self.assertEqual(f.observe(.9,self.q,self.q,2),'grasp')
        f.grasped(.029,True);self.assertEqual(f.mode,'carrying')
        self.assertEqual(f.observe(.1,self.q,self.q,3),'arm')
        f.carry_segments+=1
        self.assertEqual(f.observe(.1,self.q,self.q,4),'confirm')
        self.assertEqual(f.observe(.1,self.q,self.q,5),'release')
        f.released(True);self.assertEqual(f.mode,'released')
    def test_empty_grasp_never_enters_transport(self):
        f=TaskSequence([self.q],[self.q]);f.observe(1,self.q,self.q,1);f.observe(1,self.q,self.q,2)
        with self.assertRaises(ValueError):f.grasped(.001,True)
        self.assertEqual(f.mode,'approach')
    def test_old_observations_and_misaligned_pose_cannot_grasp(self):
        f=TaskSequence([self.q],[self.q]);self.assertEqual(f.observe(1,self.q,self.q+.1,1),'arm')
        with self.assertRaises(ValueError):f.observe(1,self.q,self.q,1)
        self.assertEqual(f.observe(1,self.q+.2,self.q+.2,2),'arm')
    def test_unconfirmed_or_failed_release_is_rejected(self):
        f=TaskSequence([self.q],[self.q])
        with self.assertRaises(ValueError):f.grasped(.029,True)
        with self.assertRaises(ValueError):f.released(True)

if __name__=='__main__':unittest.main()
