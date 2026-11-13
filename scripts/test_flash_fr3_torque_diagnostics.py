"""Prescribed sampled signals only; no dynamics, ROS or robot commands."""
import unittest
import numpy as np
from flash_fr3_torque_diagnostics import (baseline_window, incremental_response,
                                        sdk_prehold_window, sdk_torque_summary)


def samples(start=0., phase=0, reference_offset=0., measured_offset=-.0017):
    rows=[]
    for t in np.arange(15)*.04+start:
        a=np.zeros(54);a[0]=t;a[1]=phase;a[7]=measured_offset;a[21]=reference_offset
        rows.append({'data':a.tolist()})
    return rows


class TorqueDiagnosticsTests(unittest.TestCase):
    def test_response_reports_static_error_without_repairing_reference(self):
        before=samples();baseline=baseline_window(before)
        after=samples(4.,phase=1,reference_offset=.001,measured_offset=-.0007)
        original=[r['data'][:] for r in before+after]
        result=incremental_response(before+after,baseline)
        self.assertAlmostEqual(result['joint4_increment_ratio'],1.)
        self.assertTrue(result['joint4_increment_direction_matches'])
        self.assertAlmostEqual(result['final_mean_absolute_reference_error_rad'][3],.0017)
        self.assertEqual(result['tracking_validation'],'NOT_CERTIFIED')
        self.assertEqual(original,[r['data'] for r in before+after])

    def test_zero_response_is_not_reported_as_direction_match(self):
        before=samples();after=samples(4.,phase=1,reference_offset=.001)
        r=incremental_response(before+after,baseline_window(before))
        self.assertAlmostEqual(r['joint4_increment_ratio'],0.)
        self.assertFalse(r['joint4_increment_direction_matches'])

    def test_unsettled_reference_and_status_faults_rejected(self):
        for column in (7,21,25,1,2,53):
            with self.subTest(column=column):
                rows=samples()
                for i,row in enumerate(rows):row['data'][column]+=i*.0001
                with self.assertRaises(ValueError):baseline_window(rows)

    def test_missing_nonfinite_and_nonmonotonic_samples_rejected(self):
        cases=[samples()[:2],samples()]
        cases[1][8]['data'][0]=cases[1][7]['data'][0]
        nan=samples();nan[-1]['data'][7]=float('nan');cases.append(nan)
        gap=samples();gap[-1]['data'][0]+=.2;cases.append(gap)
        for rows in cases:
            with self.assertRaises(ValueError):baseline_window(rows)

    def test_end_window_requires_sufficient_samples(self):
        before=samples();baseline=baseline_window(before)
        with self.assertRaises(ValueError):incremental_response(before+samples(4.,phase=4)[:1],baseline)

    def sdk_samples(self):
        rows=[]
        for t in np.arange(133)*.04:
            phase=0 if t<1.5 else 1 if t<5.24 else 4
            a=[1.,t,100.+t,float(phase),1.,.001]
            a += [.1]*7+[.11]*7+[.12]*7+[2.]*7+[.3]*7+[.001]*7
            a += [0.]*7+[0.]*7+[.73,.2,.93,.999]
            if t>4.5:a[6+3]+=.15;a[20+3]+=.15
            rows.append({'data':a})
        return rows

    def test_sdk_separates_measured_torque_and_prior_command_without_certifying(self):
        rows=self.sdk_samples();original=[r['data'][:] for r in rows]
        result=sdk_torque_summary(rows)
        self.assertEqual(result['tracking_validation'],'NOT_CERTIFIED')
        self.assertAlmostEqual(result['sdk_desired_torque_increment_nm'][3],.15)
        self.assertAlmostEqual(result['written_torque_increment_nm'][3],.15)
        self.assertAlmostEqual(result['sdk_measured_torque_peak_abs_nm'][3],2.)
        self.assertEqual(original,[r['data'] for r in rows])

    def test_sdk_invalid_metadata_and_clock_rejected(self):
        for col,value in ((0,2.),(3,5.),(5,.003),(62,-1.),(64,1.2),(65,1.1),(6,float('nan'))):
            with self.subTest(column=col):
                rows=self.sdk_samples();rows[10]['data'][col]=value
                with self.assertRaises(ValueError):sdk_torque_summary(rows)
        rows=self.sdk_samples();rows[10]['data'][2]=rows[9]['data'][2]
        with self.assertRaises(ValueError):sdk_torque_summary(rows)

    def test_sdk_prehold_and_full_span_are_required(self):
        rows=self.sdk_samples()
        with self.assertRaises(ValueError):sdk_prehold_window(rows[:3])
        with self.assertRaises(ValueError):sdk_torque_summary(rows[:60])


if __name__=='__main__':unittest.main()
