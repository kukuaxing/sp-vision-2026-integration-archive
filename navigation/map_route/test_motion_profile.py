"""Offline plant response checks; never opens ROS, serial, camera or motors."""
import math
import json
from pathlib import Path
import unittest
import mission
from mission import Follower
from core import State


def simulate_profile(module,plan,*,dt=.02,lag=.08,forward_gain=1.05,left_gain=.95,source_age=.10,pose_noise=0.,external_task=False,observation_delay_s=0.,minimum_start_speed_m_s=0.):
    m=module.Mission(plan,external_task=external_task);s=State(0.,.01,.04,0.,0.,0.)
    previous=(0.,0.);previous_acceleration=(0.,0.)
    jerks=[];accelerations=[];maximum_error=0.;events=[];phase='OUTBOUND';arrival_speed=[]
    max_speed=0.;max_rpm=0.;history=[];endpoint_data={};endpoint_results=[]
    for k in range(int(250/dt)):
        history.append(s)
        measured=history[max(0,k-round(observation_delay_s/dt))]
        observed=State(measured.x+pose_noise*math.sin(k*dt*43.),measured.y+pose_noise*math.sin(k*dt*29.),
                       measured.yaw,measured.forward_speed,measured.left_speed,measured.yaw_rate)
        if m.phase in ('OUTBOUND','RETURNING'):
            label=m.phase;goal=m.follower.path.points[-1];previous_point=m.follower.path.points[-2]
            tangent=(goal[0]-previous_point[0],goal[1]-previous_point[1]);size=math.hypot(*tangent)
            if math.dist((s.x,s.y),goal)<=.30:
                data=endpoint_data.setdefault(label,dict(first_s=k*dt,max_overshoot_m=0.))
                overshoot=((s.x-goal[0])*tangent[0]+(s.y-goal[1])*tangent[1])/size
                data['max_overshoot_m']=max(data['max_overshoot_m'],overshoot)
        task_complete=(m.phase=='TASK_DWELL' and k*dt-m.phase_since>=5.)
        source_stamp_ns=int(max(0.,k*dt-observation_delay_s)*1e9)
        o=m.step(observed,dt,source_age,source_stamp_ns,k*dt,task_complete=task_complete)
        if o['phase']!=phase:
            events.append(dict(phase=o['phase'],time_s=k*dt))
            if o['phase'] in ('STOP_TASK','STOP_HOME'):
                arrival_speed.append(math.hypot(*previous))
                label='OUTBOUND' if o['phase']=='STOP_TASK' else 'RETURNING'
                data=endpoint_data.get(label,{})
                endpoint_results.append(dict(phase=label,last_30cm_s=k*dt-data.get('first_s',k*dt),
                                             max_overshoot_m=data.get('max_overshoot_m',0.),
                                             arrival_error_m=math.dist((s.x,s.y),m.follower.path.points[-1])))
            phase=o['phase']
        f=-o['wire_vy']*(.0885 if o['wire_vy']<=0 else .095)
        l=o['wire_vx']*.095
        velocity=(math.cos(s.yaw)*f-math.sin(s.yaw)*l,math.sin(s.yaw)*f+math.cos(s.yaw)*l)
        max_speed=max(max_speed,math.hypot(*velocity))
        max_rpm=max(max_rpm,159.*sum(abs(o[k]) for k in ('wire_vx','wire_vy','wire_wz')))
        acceleration=tuple((v-old)/dt for v,old in zip(velocity,previous))
        jerks.append(math.dist(acceleration,previous_acceleration)/dt)
        accelerations.append(math.hypot(*acceleration))
        previous=velocity;previous_acceleration=acceleration
        maximum_error=max(maximum_error,o.get('lateral_error_m',0.))
        if phase=='DONE':
            return dict(elapsed_s=k*dt,max_command_speed_m_s=max_speed,max_motor_rpm_bound=max_rpm,
                        peak_command_jerk_m_s3=max(jerks),
                        p95_command_jerk_m_s3=sorted(jerks)[int(.95*len(jerks))],
                        peak_command_acceleration_m_s2=max(accelerations),
                        max_lateral_error_m=maximum_error,home_error_m=math.hypot(s.x,s.y),
                        arrival_command_speed_m_s=arrival_speed,endpoint_metrics=endpoint_results,events=events)
        factor=min(1.,dt/lag)
        target_f=f*forward_gain;target_l=l*left_gain
        if math.hypot(s.forward_speed,s.left_speed)<.025 and math.hypot(f,l)<minimum_start_speed_m_s:
            target_f=target_l=0.
        vf=s.forward_speed+(target_f-s.forward_speed)*factor
        vl=s.left_speed+(target_l-s.left_speed)*factor
        w=s.yaw_rate+(-o['wire_wz']*.8-s.yaw_rate)*factor
        s=State(s.x+(math.cos(s.yaw)*vf-math.sin(s.yaw)*vl)*dt,
                s.y+(math.sin(s.yaw)*vf+math.cos(s.yaw)*vl)*dt,s.yaw+w*dt,vf,vl,w)
    raise AssertionError('Offline mission did not finish within 250 seconds')


class SmoothNavigationTests(unittest.TestCase):
    def test_round_trip_with_lag_gain_and_scheduling_variations(self):
        plan=json.loads((Path(__file__).parent/'global_plan.json').read_text(encoding='utf-8'))
        cases=[(.02,.08,1.05,.95,.10),(.02,.15,.9,1.1,.20),
               (.05,.10,1.,1.,.10),(.01,.06,1.1,.9,.05),
               (.02,.20,1.1,.9,.25),(.02,.12,.9,.9,.30),(.10,.12,1.,1.,.10)]
        for dt,lag,fg,lg,age in cases:
            with self.subTest(dt=dt,lag=lag,forward_gain=fg,left_gain=lg,source_age=age):
                r=simulate_profile(mission,plan,dt=dt,lag=lag,forward_gain=fg,left_gain=lg,source_age=age)
                self.assertLess(r['max_lateral_error_m'],.06)
                self.assertLess(r['home_error_m'],.035)
                self.assertLess(r['elapsed_s'],90.)
                self.assertLessEqual(r['max_command_speed_m_s'],plan['speed_ceiling_m_s']+1e-8)
                self.assertLessEqual(r['max_motor_rpm_bound'],plan['motor_rpm_ceiling']+1e-8)
                self.assertLessEqual(r['peak_command_acceleration_m_s2'],plan['deceleration_m_s2']+1e-8)
                self.assertLessEqual(r['peak_command_jerk_m_s3'],plan['jerk_limit_m_s3']+1e-6)
                self.assertLessEqual(max(r['arrival_command_speed_m_s']),.003+1e-9)
                self.assertEqual([v['phase'] for v in r['events']],
                                 ['STOP_TASK','TASK_DWELL','RETURNING','STOP_HOME','RELEASING','DONE'])

    def test_centimetre_localization_noise_does_not_prevent_arrival(self):
        plan=json.loads((Path(__file__).parent/'global_plan.json').read_text(encoding='utf-8'))
        r=simulate_profile(mission,plan,pose_noise=.01,external_task=True)
        self.assertLess(r['elapsed_s'],90.)
        self.assertLess(r['home_error_m'],.035)
        self.assertLess(r['max_lateral_error_m'],.06)
        self.assertLessEqual(r['peak_command_jerk_m_s3'],plan['jerk_limit_m_s3']+1e-6)

    def test_settling_hysteresis_and_displacement_recovery(self):
        f=Follower([[0.,0.],[1.,0.]])
        f.progress=.99
        f.step(State(.99,0.,0.,.01,0.,0.),.02,0.)
        self.assertTrue(f.terminal_settling)
        f.step(State(.97,0.,0.,.01,0.,0.),.02,0.)
        self.assertTrue(f.terminal_settling)
        f.step(State(.94,0.,0.,.01,0.,0.),.02,0.)
        self.assertFalse(f.terminal_settling)

    def test_arrival_requires_slow_measured_and_commanded_motion(self):
        f=Follower([[0.,0.],[1.,0.]])
        self.assertFalse(f.step(State(1.,0.,0.,.2,0.,0.),.02,0.)['reached'])
        f.vx=.2
        self.assertFalse(f.step(State(1.,0.,0.,0.,0.,0.),.02,0.)['reached'])
        for k in range(500):
            c=f.step(State(1.,0.,0.,0.,0.,0.),.02,0.)
            if c['reached']:break
        self.assertTrue(c['reached'])
        self.assertEqual(c['wire_vx'],0.)
        self.assertEqual(c['wire_vy'],0.)

    def test_endpoint_feedback_recovers_after_small_overshoot(self):
        f=Follower([[0.,0.],[1.,0.]])
        f.progress=1.
        c=f.step(State(1.04,0.,0.,0.,0.,0.),.02,0.)
        self.assertFalse(c['reached'])
        self.assertGreater(c['wire_vy'],0.)

    def test_mirrored_route_with_noisy_and_slow_drive(self):
        original=json.loads((Path(__file__).parent/'global_plan.json').read_text(encoding='utf-8'))
        plan=dict(original)
        for field in ('outbound_waypoints_m','return_waypoints_m'):
            plan[field]=[[x,-y] for x,y in original[field]]
        plan['task_goal_m']=[original['task_goal_m'][0],-original['task_goal_m'][1]]
        for lag,fg,lg,noise in ((.08,1.05,.95,.01),(.15,.9,1.1,0.)):
            with self.subTest(lag=lag):
                r=simulate_profile(mission,plan,lag=lag,forward_gain=fg,left_gain=lg,
                                   source_age=.15,pose_noise=noise,external_task=True)
                self.assertLess(r['elapsed_s'],90.)
                self.assertLess(r['max_lateral_error_m'],.06)
                self.assertLess(r['home_error_m'],.035)
                self.assertLessEqual(r['peak_command_jerk_m_s3'],plan['jerk_limit_m_s3']+1e-6)

    def test_invalid_corner_braking_margin_is_rejected(self):
        for value in (0.,.039,.201,float('nan'),float('inf')):
            with self.subTest(value=value),self.assertRaises(ValueError):
                Follower([[0.,0.],[1.,0.]],corner_brake_margin=value)

    def test_delayed_measurements_and_low_speed_start_threshold(self):
        plan=json.loads((Path(__file__).parent/'global_plan.json').read_text(encoding='utf-8'))
        for delay,lag,threshold in ((.10,.08,0.),(.10,.08,.10),(.15,.15,0.)):
            with self.subTest(delay=delay,lag=lag,start_threshold=threshold):
                r=simulate_profile(mission,plan,observation_delay_s=delay,source_age=delay,
                                   lag=lag,minimum_start_speed_m_s=threshold,external_task=True)
                self.assertLess(r['elapsed_s'],90.)
                self.assertLess(r['home_error_m'],.035)
                self.assertTrue(all(e['arrival_error_m']<.035 for e in r['endpoint_metrics']))
                # This checks endpoint robustness, not clearance on delayed bends.
                self.assertLess(max(e['max_overshoot_m'] for e in r['endpoint_metrics']),.065)
                self.assertLessEqual(r['peak_command_jerk_m_s3'],6.01)

    def test_terminal_prediction_rejects_coasting_away(self):
        f=Follower([[0.,0.],[1.,0.]])
        f.progress=.97
        c=f.step(State(.97,0.,0.,-.10,0.,0.),.02,.1)
        self.assertFalse(c['terminal_settling'])
        self.assertFalse(c['reached'])
        self.assertGreater(c['endpoint_predicted_error_m'],.035)
        f.terminal_settling=True
        self.assertFalse(f.step(State(.97,0.,0.,-.05,0.,0.),.02,.1)['reached'])

    def test_terminal_speed_envelope_accounts_for_source_age(self):
        caps=[]
        for age in (0.,.10,.25):
            f=Follower([[0.,0.],[1.,0.]],1.8,motor_rpm_bound=5500.,acceleration=1.1,deceleration=1.5)
            f.progress=.7;f.vx=.8
            caps.append(f.step(State(.7,0.,0.,.8,0.,0.),.02,age)['endpoint_speed_cap_m_s'])
        self.assertGreater(caps[0],caps[1]);self.assertGreater(caps[1],caps[2])

    def test_invalid_profiles_are_rejected(self):
        for key in ('acceleration','deceleration','jerk_limit','motor_rpm_bound','corner_speed'):
            for value in (0.,-1.,float('nan'),float('inf')):
                with self.subTest(key=key,value=value),self.assertRaises(ValueError):
                    Follower([[0.,0.],[1.,0.]],**{key:value})


if __name__=='__main__':unittest.main()
