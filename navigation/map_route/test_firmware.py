import unittest
from firmware import NativeProfile


class UnitConversionTests(unittest.TestCase):
    def test_native_translation_preserves_motor_targets(self):
        p=NativeProfile();o=p.convert(-2.,-5.,0.)
        self.assertAlmostEqual(o['wire_vx']*p.gain,-318.)
        self.assertAlmostEqual(o['wire_vy']*p.gain,-795.)
        self.assertFalse(o['heading_control_available'])

    def test_current_flashed_zero_lever_refuses_yaw(self):
        p=NativeProfile()
        with self.assertRaises(RuntimeError):p.convert(0.,-2.,.01)
        o=p.convert(0.,-2.,.01,preview=True)
        self.assertEqual(o['wire_wz'],0.)
        self.assertFalse(o['heading_control_available'])

    def test_candidate_yaw_conversion_preserves_motor_target(self):
        p=NativeProfile(yaw_lever_m=.35,yaw_sign=-1.)
        o=p.convert(1.,-5.,.30)
        self.assertAlmostEqual(o['wire_wz']*p.gain*p.yaw_lever_m*p.yaw_sign,47.7)
        self.assertLess(o['wire_wz'],0.)

    def test_rpm_bound_applies_after_unit_conversion(self):
        with self.assertRaises(ValueError):NativeProfile().convert(8.,8.,0.)


if __name__=='__main__':unittest.main()
