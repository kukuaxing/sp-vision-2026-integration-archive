from collections import deque
from types import SimpleNamespace
import math
import unittest

from runner import departure_readiness
from policy import StartGate


def observations(**changes):
    rows=deque()
    for i in range(25):
        # Recorded P00 noise exceeded 1.2cm while measured velocity stayed low.
        state=dict(x=.025+.01*math.sin(i*1.7),y=.025+.008*math.cos(i*1.7),
                   yaw=math.radians(-2.8),forward_speed=.006,left_speed=.004,yaw_rate=.002)
        state.update(changes)
        rows.append((i*.05,SimpleNamespace(**state)))
    return rows


class DepartureTests(unittest.TestCase):
    def test_stationary_p00_with_recorded_scale_pose_noise(self):
        ready,checks=departure_readiness(observations())
        self.assertTrue(ready);self.assertTrue(checks['stopped'])

    def test_workbench_or_wrong_heading_never_ready(self):
        for changes,key in [({'x':6.,'y':-2.},'at_p00'),({'yaw':math.radians(12)},'heading_ok')]:
            ready,checks=departure_readiness(observations(**changes))
            self.assertFalse(ready);self.assertFalse(checks[key])

    def test_motion_rotation_and_insufficient_observations_rejected(self):
        for changes in [dict(forward_speed=.10),dict(yaw_rate=.12)]:
            self.assertFalse(departure_readiness(observations(**changes))[0])
        self.assertFalse(departure_readiness(deque())[0])
        self.assertFalse(departure_readiness(deque(list(observations())[:3]))[0])

    def test_drifting_position_never_ready(self):
        rows=observations()
        for i,(_,s) in enumerate(rows):s.x=i*.015
        self.assertFalse(departure_readiness(rows)[0])

    def test_stale_feedback_or_initial_auto_still_requires_new_ready_edge(self):
        ready,_=departure_readiness(observations());gate=StartGate()
        for t in [0,1,2]:self.assertFalse(gate.update(t,ready,True,True))
        self.assertFalse(gate.update(3,ready,False,False))
        self.assertFalse(gate.update(4,ready,True,True))
        self.assertFalse(gate.update(5,ready,False,True))
        self.assertTrue(gate.update(5.1,ready,True,True))


if __name__=='__main__':unittest.main()
