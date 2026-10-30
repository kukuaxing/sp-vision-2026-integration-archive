from pathlib import Path
import json
import math
import unittest
from core import State
from mission import Mission, Follower, Path as Polyline, simplify, length, stopped

HERE=Path(__file__).resolve().parent


def simulate(plan,*,initial_yaw=.04,initial_left=.01):
    mission=Mission(plan);s=State(0.,initial_left,initial_yaw,0.,0.,0.)
    dt=.02;maximum_error=0.;dwell_time=0.;phase_counts={};previous='OUTBOUND'
    for k in range(15000):
        now=k*dt;o=mission.step(s,dt,.10,int(now*1e9),now)
        phase_counts[o['phase']]=phase_counts.get(o['phase'],0)+1
        maximum_error=max(maximum_error,o.get('lateral_error_m',0.))
        if o['phase']=='TASK_DWELL':dwell_time+=dt
        if o['phase']=='DONE':
            return dict(mission=mission,elapsed_s=now,state=s,max_error=maximum_error,
                        dwell_s=dwell_time,phase_counts=phase_counts)
        tf=-o['wire_vy']*(.0885 if o['wire_vy']<=0 else .095)*1.05
        tl=o['wire_vx']*.095*.95
        target_rate=-o['wire_wz']*.8
        f=s.forward_speed+(tf-s.forward_speed)*.2
        l=s.left_speed+(tl-s.left_speed)*.2
        w=s.yaw_rate+(target_rate-s.yaw_rate)*.2
        x=s.x+(math.cos(s.yaw)*f-math.sin(s.yaw)*l)*dt
        y=s.y+(math.sin(s.yaw)*f+math.cos(s.yaw)*l)*dt
        yaw=s.yaw+w*dt
        s=State(x,y,yaw,f,l,w)
    raise AssertionError('Mission did not finish within 300 seconds')


class GlobalMissionTests(unittest.TestCase):
    def test_fast_stop_accepts_stationary_pose_jitter_and_keeps_motion_guard(self):
        jitter=[(k*.1,State(.01 if k%2 else -.01,0,0,.008,-.006,.002)) for k in range(11)]
        self.assertFalse(stopped(jitter))
        self.assertTrue(stopped(jitter,fast=True))
        self.assertFalse(stopped(jitter[:3],fast=True))
        for states in [
            [(k*.1,State(k*.01,0,0,.10,0,0)) for k in range(6)],
            [(k*.1,State(0,0,0,0,0,.12)) for k in range(6)],
            [(k*.1,State(.04 if k%2 else 0,0,0,0,0,0)) for k in range(6)]]:
            self.assertFalse(stopped(states,fast=True))

    def test_competition_task_handover_is_short_and_never_before_stillness(self):
        plan=dict(outbound_waypoints_m=[[0.,0.],[1.,0.]],return_waypoints_m=[[1.,0.],[0.,0.]],
                  speed_ceiling_m_s=.45,task_dwell_s=5.)
        m=Mission(plan,external_task=True);m.enter('STOP_TASK',0.)
        for k in range(5):
            c=m.step(State(1.,0.,0.,.10,0.,0.),.02,0.,k,k*.1)
            self.assertEqual(c['phase'],'STOP_TASK')
        transition=None
        for k in range(5,15):
            c=m.step(State(1.+(.01 if k%2 else -.01),0.,0.,.008,0.,0.),.02,0.,k,k*.1)
            if c['phase']=='TASK_DWELL':transition=k*.1;break
        self.assertIsNotNone(transition);self.assertLessEqual(transition-.5,.6)
    def test_external_task_never_returns_by_old_dwell_time(self):
        plan=dict(outbound_waypoints_m=[[0.,0.],[1.,0.]],return_waypoints_m=[[1.,0.],[0.,0.]],
                  speed_ceiling_m_s=.45,task_dwell_s=5.)
        m=Mission(plan,external_task=True);m.enter('TASK_DWELL',0.)
        for k in range(500):
            c=m.step(State(1.,0.,0.,0.,0.,0.),.02,0.,k,k*.02)
            self.assertEqual(c['phase'],'TASK_DWELL')
        c=m.step(State(1.,0.,0.,0.,0.,0.),.02,0.,501,10.,task_complete=True)
        self.assertEqual(c['phase'],'RETURNING')
    def test_actual_p00_longitudinal_offset_is_not_lateral_error(self):
        plan=json.loads((HERE/'global_plan.json').read_text(encoding='utf-8'))
        f=Follower(plan['outbound_waypoints_m'])
        for k,(x,y) in enumerate([(-.04911911845222401,.00703765334751826),
                                  (-.04977711087340868,.010327165072386338)]):
            o=f.step(State(x,y,.0016,.01,-.006,.014),.02,.1,k)
            self.assertLess(o['lateral_error_m'],.01)
            self.assertEqual(o['progress_m'],0.)
            self.assertLess(o['wire_vy'],0.)
        self.assertEqual(f.path.points[0],(0.,0.))
        o=Follower([[0.,0.],[1.,0.]]).step(State(-.08,.09,0,0,0,0),.02,0.,1)
        self.assertLess(o['wire_vx'],0.)
        self.assertLess(o['wire_vy'],0.)

    def test_feedback_magnitudes_do_not_abort_and_commands_remain_bounded(self):
        f=Follower([[0.,0.],[1.,0.]])
        for k,rate in enumerate([.75,.8,1.21,-1.5]):
            o=f.step(State(.1,.09,.2,.81,0,rate),.02,0.,k)
            self.assertLessEqual(o['planned_speed_m_s'],.45)
            self.assertLessEqual(abs(o['wire_wz']),.3)
            self.assertLessEqual(sum(abs(o[key])*159 for key in ['wire_vx','wire_vy','wire_wz']),2000.)

    def test_recorded_entire_course_out_dwell_and_home(self):
        plan=json.loads((HERE/'global_plan.json').read_text(encoding='utf-8'))
        r=simulate(plan)
        self.assertTrue(r['mission'].task_reached and r['mission'].home_reached)
        self.assertLess(math.hypot(r['state'].x,r['state'].y),.075)
        self.assertLess(r['max_error'],.08)
        self.assertAlmostEqual(r['dwell_s'],plan['task_dwell_s'],delta=.03)
        self.assertLess(r['elapsed_s'],250.)
        phases=[e['phase'] for e in r['mission'].events]
        self.assertEqual(phases,['STOP_TASK','TASK_DWELL','RETURNING','STOP_HOME','RELEASING','DONE'])

    def test_speed_upgrade_reaches_point_nine_and_brakes_before_corner(self):
        from firmware import NativeProfile
        f=Follower([[0.,0.],[8.,0.]],.9)
        for k in range(150):
            c=f.step(State(0,0,0,0,0,0),.02,.1,k)
            NativeProfile(yaw_lever_m=.35,yaw_sign=-1.).convert(c['wire_vx'],c['wire_vy'],c['wire_wz'])
        self.assertAlmostEqual(c['planned_speed_m_s'],.9)
        f=Follower([[0.,0.],[3.,0.],[3.,1.]],.9)
        f.progress=2.3;f.vx=.9
        c=f.step(State(2.3,0,0,.9,0,0),.02,.1,1)
        self.assertLess(c['planned_speed_m_s'],.9)

    def test_speed_upgrade_reaches_one_point_eight_with_matching_conversion(self):
        from firmware import NativeProfile
        plan=json.loads((HERE/'global_plan.json').read_text(encoding='utf-8'))
        f=Follower([[0.,0.],[8.,0.]],plan['speed_ceiling_m_s'],motor_rpm_bound=plan['motor_rpm_ceiling'],
                   acceleration=plan['acceleration_m_s2'],deceleration=plan['deceleration_m_s2'])
        profile=NativeProfile.from_config(json.loads((HERE/'firmware_profile.json').read_text(encoding='utf-8')))
        for k in range(250):
            c=f.step(State(0,0,0,0,0,0),.02,.1,k)
            o=profile.convert(c['wire_vx'],c['wire_vy'],c['wire_wz'])
        self.assertAlmostEqual(c['planned_speed_m_s'],1.8)
        self.assertAlmostEqual(o['requested_motor_rpm'][1],-3233.898305084746)

    def test_return_uses_reversed_route_not_direct_chord(self):
        plan=json.loads((HERE/'global_plan.json').read_text(encoding='utf-8'))
        self.assertEqual(plan['return_waypoints_m'],list(reversed(plan['outbound_waypoints_m'])))
        self.assertGreater(length(plan['return_waypoints_m']),math.dist(plan['task_goal_m'],(0.,0.))+5.)
        plan['return_waypoints_m']=[plan['task_goal_m'],[0.,0.]]
        with self.assertRaises(ValueError):Mission(plan)

    def test_large_cross_error_and_slow_progress_keep_correcting(self):
        plan=dict(outbound_waypoints_m=[[0.,0.],[1.,0.]],return_waypoints_m=[[1.,0.],[0.,0.]],
                  speed_ceiling_m_s=.45,task_dwell_s=10.)
        o=Mission(plan).step(State(0,.09,0,0,0,0),.02,0.,1,0.)
        self.assertLess(o['wire_vx'],0.)
        m=Mission(plan)
        m.step(State(0,0,0,0,0,0),.02,0.,1,0.)
        self.assertEqual(m.step(State(0,0,0,0,0,0),.02,0.,2,5.1)['phase'],'OUTBOUND')

    def test_projection_cannot_jump_to_nearby_later_corridor(self):
        path=Polyline([[0.,0.],[3.,0.],[3.,.1],[0.,.1]])
        error,arc,*_=path.project((.1,.09),0.)
        self.assertLess(arc,.5)
        self.assertGreater(error,.08)

    def test_simplification_preserves_obstacle_detour(self):
        p=[[0,0],[0,1],[1,1],[1,0],[2,0]]
        out=simplify(p,.025)
        self.assertEqual(out,p)


if __name__=='__main__':unittest.main()
