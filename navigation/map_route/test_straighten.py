from pathlib import Path
import json
import math
import unittest
from straighten import fit
from bind_boot import binding_context


HERE=Path(__file__).resolve().parent


class StraightRouteTests(unittest.TestCase):
    def test_all_ten_actual_legs_are_straight_orthogonal_and_keep_goals(self):
        p=json.loads((HERE/'global_plan.json').read_text(encoding='utf-8'))
        points=p['outbound_waypoints_m'];self.assertEqual(len(points),11)
        for a,b,c in zip(points,points[1:],points[2:]):
            u=(b[0]-a[0],b[1]-a[1]);v=(c[0]-b[0],c[1]-b[1])
            self.assertAlmostEqual((u[0]*v[0]+u[1]*v[1])/(math.hypot(*u)*math.hypot(*v)),0.,places=10)
        self.assertEqual(points[0],[0.,0.]);self.assertEqual(points[-1],p['task_goal_m'])
        self.assertTrue(p['task_point_label_field_confirmed'])
        self.assertFalse(p['straight_line_planning']['manual_wobble_retained'])

    def test_unexpected_topology_is_rejected(self):
        with self.assertRaises(ValueError):fit([[0.,0.],[1.,0.],[1.,1.]])


class BindingTests(unittest.TestCase):
    def fixture(self):
        old=dict(boot_id=1,next_sequence=8941)
        observed=dict(boot=2,state=0,reject=0,frames=300,crc_errors=0,age_s=.01)
        profile=dict(source_confirmed_flashed_by_user=True,yaw_lever_m=.35,yaw_sign=-1.,gain_rpm_per_unit=3859.9)
        review=dict(user_confirmed_flashed=True,yaw_lever_m=.35,yaw_sign=-1.,gain_rpm_per_unit=3859.9)
        return old,observed,profile,review

    def test_sequence_is_preserved_and_boot_rebound(self):
        self.assertEqual(binding_context(*self.fixture()),dict(boot_id=2,next_sequence=8941))

    def test_auto_active_or_stale_link_cannot_bind(self):
        for field,value in [('state',128),('reject',6),('crc_errors',1),('age_s',.3)]:
            args=list(self.fixture());args[1][field]=value
            with self.assertRaises(RuntimeError):binding_context(*args)

    def test_current_zero_lever_or_unconfirmed_flash_cannot_bind(self):
        for field,value in [('yaw_lever_m',0.),('source_confirmed_flashed_by_user',False)]:
            args=list(self.fixture());args[2][field]=value
            with self.assertRaises(RuntimeError):binding_context(*args)


if __name__=='__main__':unittest.main()
