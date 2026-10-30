import math
import unittest
from core import State, Tracker, SourceGuard

POINTS = [(0., 0.), (1.10, 0.), (1.10, -1.25)]


class FeedbackTests(unittest.TestCase):
    def test_cross_error_on_both_segments(self):
        t = Tracker(POINTS)
        o = t.step(State(.4, .04, 0., .2, 0., 0.), .02, .1)
        self.assertLess(o['wire_vx'], -.7)
        t.advance()
        o = t.step(State(1.14, -.4, 0., 0., -.2, 0.), .02, .1)
        self.assertGreater(o['wire_vy'], .7)

    def test_heading_and_body_rotation(self):
        t = Tracker(POINTS)
        o = t.step(State(0., 0., .08, 0., 0., 0.), .02, .1)
        self.assertGreater(o['wire_wz'], 0.)
        self.assertLess(o['wire_vx'], 0.)
        self.assertLess(o['wire_vy'], 0.)

    def test_absolute_target_does_not_reset_to_each_start(self):
        t = Tracker(POINTS); t.advance()
        o = t.step(State(1.06, -.1, 0., 0., 0., 0.), .02, 0.)
        self.assertAlmostEqual(o['cross_m'], -.04)
        self.assertAlmostEqual(o['remaining_m'], 1.15)
        self.assertLess(o['wire_vy'], 0.)

    def test_endpoint_needs_cross_tolerance(self):
        t = Tracker(POINTS)
        self.assertFalse(t.step(State(1.10, .04, 0., 0., 0., 0.), .02, 0.)['reached'])
        self.assertTrue(t.step(State(1.09, .01, 0., 0., 0., 0.), .02, 0.)['reached'])

    def test_signed_speed(self):
        t = Tracker(POINTS)
        o = t.step(State(.5, 0., 0., -.2, 0., 0.), .02, 0.)
        self.assertEqual(o['along_speed_m_s'], -.2)

    def test_bounds_and_nan(self):
        for s in [State(0., .081, 0., 0., 0., 0.), State(0., 0., .3, 0., 0., 0.)]:
            with self.assertRaises(RuntimeError): Tracker(POINTS).step(s, .02, 0.)
        with self.assertRaises(ValueError): State(float('nan'), 0, 0, 0, 0, 0)
        with self.assertRaises(ValueError): Tracker(POINTS).step(State(0,0,0,0,0,0), .02, .31)

    def test_delayed_actuator_simulation_with_bias(self):
        # Simplified first order plant with yaw error, 2 cm starting offset,
        # cross disturbance, and unequal directional speed scales.
        t = Tracker(POINTS)
        x, y, yaw, f, l, rate = 0., .02, .06, 0., 0., 0.
        max_cross = 0.; completed = []
        dt = .02
        for k in range(2500):
            s = State(x,y,yaw,f,l,rate)
            o = t.step(s,dt,.1); max_cross=max(max_cross,abs(o['cross_m']))
            if o['reached']:
                completed.append(math.hypot(x-POINTS[t.index+1][0], y-POINTS[t.index+1][1]))
                # Host performs a full stop before advance; simulate settling.
                for _ in range(60):
                    f *= .8; l *= .8; rate *= .8
                    x += (math.cos(yaw)*f-math.sin(yaw)*l)*dt
                    y += (math.sin(yaw)*f+math.cos(yaw)*l)*dt
                    yaw += rate*dt
                t.advance()
                if t.index == 2:break
                continue
            tf = -o['wire_vy']*(.0885 if o['wire_vy']<=0 else .095)*1.08
            tl = o['wire_vx']*.095*.92
            f += (tf-f)*.15; l += (tl+.012-l)*.15
            rate += (-o['wire_wz']*.8-rate)*.15
            x += (math.cos(yaw)*f-math.sin(yaw)*l)*dt
            y += (math.sin(yaw)*f+math.cos(yaw)*l)*dt
            yaw += rate*dt
        self.assertEqual(t.index,2)
        self.assertLess(max_cross,.06)
        self.assertLess(max(completed),.04)
        self.assertLess(k*dt,25.)


class SourceTests(unittest.TestCase):
    def test_repeat_does_not_refresh_source_age(self):
        s=State(0,0,0,0,0,0); g=SourceGuard()
        g.check(('boot','map','reg'),10**9,10**9,0,s,True)
        with self.assertRaises(RuntimeError):g.check(('boot','map','reg'),10**9,1400000000,.1,s,True)

    def test_identity_and_jumps(self):
        for identity,stamp,state in [(('new','map','reg'),1100000000,State(0,0,0,0,0,0)),
                                      (('boot','map','reg'),1100000000,State(.2,0,0,0,0,0)),
                                      (('boot','map','reg'),900000000,State(0,0,0,0,0,0))]:
            g=SourceGuard();g.check(('boot','map','reg'),10**9,10**9,0,State(0,0,0,0,0,0),True)
            with self.assertRaises(RuntimeError):g.check(identity,stamp,1100000000,.01,state,True)


if __name__ == '__main__': unittest.main()
