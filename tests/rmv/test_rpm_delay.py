import importlib.util
from pathlib import Path
import unittest
spec=importlib.util.spec_from_file_location("rpm_delay",Path(__file__).resolve().parents[2]/"scripts/report_rmv_rpm_delay.py")
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def fixture(drop=120, unilateral=False, gap=False):
    rows=[dict(event="command_issued",request_id=1,mcu_us=1000000,clock_uncertainty_us=1000),
          dict(event="shot_event",request_id=1,shot_event=0,mcu_us=1005000),
          dict(event="shot_event",request_id=1,shot_event=1,mcu_us=1010000),
          dict(event="shot_event",request_id=1,shot_event=2,mcu_us=1900000)]
    for i,t in enumerate(range(700000,2000001,5000)):
        if gap and 1050000<=t<1090000:continue
        pulse=drop if 1090000<=t<=1120000 else 0
        noise=(i%3-1)*3
        rows.append(dict(event="wheel_sample",mcu_us=t,valid_flags=7,
                         rpm=[6470+noise-pulse,-(6470-noise-(0 if unilateral else pulse)),0]))
    return rows


class DropTiming(unittest.TestCase):
    def test_paired(self):
        r=module.analyze(fixture());s=r["shots"][0]
        self.assertEqual(r["detected_count"],1)
        self.assertEqual(s["command_to_drop_ms"],90)
        self.assertEqual(s["accepted_to_drop_ms"],85)
        self.assertEqual(s["start_to_drop_ms"],80)
        self.assertEqual(s["sample_bracket_width_ms"],5)
        self.assertFalse(r["automatic_parameter_update"])

    def test_noise_and_unilateral(self):
        self.assertEqual(module.analyze(fixture(drop=0))["detected_count"],0)
        self.assertEqual(module.analyze(fixture(unilateral=True))["detected_count"],0)

    def test_gap(self):
        r=module.analyze(fixture(gap=True))
        self.assertEqual(r["detected_count"],0)
        self.assertEqual(r["shots"][0]["status"],"sample_gap_at_drop")

    def test_missing_baseline_or_marker(self):
        rows=fixture()
        rows=[r for r in rows if r.get("event")!="wheel_sample" or r["mcu_us"]>=990000]
        self.assertEqual(module.analyze(rows)["shots"][0]["status"],"insufficient_baseline")
        rows=[r for r in fixture() if r.get("shot_event")!=0]
        self.assertEqual(module.analyze(rows)["shots"][0]["status"],"missing_shot_markers")

    def test_failed_shot(self):
        rows=fixture()+[dict(event="shot_event",request_id=1,shot_event=5,mcu_us=1910000)]
        self.assertEqual(module.analyze(rows)["detected_count"],0)

    def test_invalid_rpm_and_early_power_dip(self):
        rows=fixture(drop=0)
        for r in rows:
            if r["event"]=="wheel_sample":
                if r["mcu_us"]==1010000:r["rpm"]=[None,None,0]
                elif 1015000<=r["mcu_us"]<=1040000:r["rpm"]=[6300,-6300,0]
        report=module.analyze(rows)
        self.assertEqual(report["possible_power_coupling_count"],1)
        self.assertIsNone(report["command_to_drop_excluding_early_drops"])

    def test_twenty_distinct_shots(self):
        rows=[]
        for ident in range(1,21):
            for r in fixture():
                r=dict(r);r["mcu_us"]+=(ident-1)*1300000
                if "request_id" in r:r["request_id"]=ident
                rows.append(r)
        report=module.analyze(rows)
        self.assertEqual(report["command_count"],20);self.assertEqual(report["detected_count"],20)
        self.assertEqual(report["command_to_drop"]["mean_ms"],90)

    def test_previous_tail_and_multiple_pulses(self):
        rows=fixture(drop=0)
        for r in rows:
            if r["event"]=="wheel_sample" and 995000<=r["mcu_us"]<=1040000:r["rpm"]=[6300,-6300,0]
        self.assertEqual(module.analyze(rows)["detected_count"],0)
        rows=fixture()
        for r in rows:
            if r["event"]=="wheel_sample" and 1300000<=r["mcu_us"]<=1340000:r["rpm"]=[6300,-6300,0]
        self.assertEqual(module.analyze(rows)["shots"][0]["status"],"multiple_paired_drops")

    def test_after_completion_and_stale_feedback(self):
        rows=fixture()
        for r in rows:
            if r.get("shot_event")==2:r["mcu_us"]=1070000
        self.assertEqual(module.analyze(rows)["detected_count"],0)
        rows=fixture()
        for r in rows:
            if r["event"]=="wheel_sample":r["feedback_age_us"]=[50000,50000,50000]
        self.assertEqual(module.analyze(rows)["sample_count"],0)


if __name__=="__main__":
    unittest.main()
