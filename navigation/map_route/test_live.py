import collections
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import time
import types
import sys
import unittest
from unittest.mock import patch
import live
from core import State

ROOT=Path(__file__).resolve().parent.parent
codec=live.load('_tested_map_codec',ROOT/'full_course_20261022/aligned_course/guidance.py')
support=live.load('_tested_map_support',ROOT/'field_next_20261020/field_support.py')


def crc(data):
    value=65535
    for b in data:
        value^=b
        for _ in range(8):value=(value>>1)^(0x8408 if value&1 else 0)
    return value


class FakeIO:
    is_open=True
    def __init__(self,journal):self.journal=journal;self.frames=[]
    def submit(self,frame,**kwargs):
        seq=struct.unpack_from('<H',frame,6)[0]
        assert json.loads(self.journal.read_text())['next_sequence']==(seq+1)&65535
        self.frames.append((frame,kwargs));return True
    def pump(self,**kwargs):return True
    def close(self):self.is_open=False


class HostTests(unittest.TestCase):
    def test_cold_boot_null_course_pose_is_waiting_not_attribute_error(self):
        a=object.__new__(live.Adapter);a.error=None;a.node=None
        a.store=types.SimpleNamespace(update=lambda:None,reg={'registration_id':'r'})
        a.boot=dict(localization_ready=False,course_pose=None,epoch=None,boot_id='boot',
                    instance='monitor',updated_unix=time.time())
        now=time.monotonic();stamp=time.time_ns()
        a.pose=dict(xyz=[0.,0.,0.],yaw_deg=0.,stamp_ns=stamp,received=now)
        a.odom=dict(stamp=stamp,received=now,f=0.,l=0.,w=0.)
        a.state={};a.state_rx=now;a.state_stamp=stamp;a.boot_rx=now
        a.last_generation=None;a.checked_publishers=now
        with patch.object(live,'project',return_value=dict(xy_m=[0.,0.],yaw_deg=0.)), \
             patch.object(live.Path,'read_text',return_value='boot'):
            with self.assertRaisesRegex(RuntimeError,'Localization not healthy'):
                a.sample()
    def test_actual_disabled_entry_never_constructs_serial(self):
        with patch.object(live,'checked_sources',return_value={}),patch.object(sys,'argv',['live.py','--execute']),\
             patch.object(live,'Wire') as constructor:
            with self.assertRaisesRegex(RuntimeError,'newly flashed'):
                live.main()
            constructor.assert_not_called()

    def test_journal_before_write_and_zero_revoke(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'sequence.json';p.write_text(json.dumps(dict(next_sequence=42,boot_id=99)))
            w=object.__new__(live.Wire);w.codec=codec;w.crc16=crc;w.support=support
            w.sequence=42;w.boot=99;w.journal=p;w.started=False;w.io=FakeIO(p)
            w.profile=live.NativeProfile(yaw_lever_m=.35,yaw_sign=-1.)
            w.send(vx=-2.,vy=-3.,wz=.1)
            w.send(enabled=False)
            self.assertEqual(w.sequence,44)
            for frame,kwargs in w.io.frames:
                self.assertEqual(frame[:2],b'\xa6\x6a')
                self.assertEqual(len(frame),38)
                self.assertEqual(struct.unpack_from('<H',frame,36)[0],crc(frame[:36]))
            frame,kwargs=w.io.frames[-1]
            self.assertEqual(frame[22],0)  # payload enabled byte
            self.assertEqual(struct.unpack_from('<fff',frame,24),(0.,0.,0.))
            self.assertTrue(kwargs['stop'])

    def test_write_fails_closed_on_journal_mismatch(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'sequence.json';p.write_text(json.dumps(dict(next_sequence=42,boot_id=99)))
            w=object.__new__(live.Wire);w.codec=codec;w.crc16=crc;w.support=support
            w.sequence=42;w.boot=100;w.journal=p;w.started=False;w.io=FakeIO(p)
            w.profile=live.NativeProfile()
            with self.assertRaises(RuntimeError):w.send(vx=1.)
            self.assertEqual(w.io.frames,[])

    def test_combined_wheel_and_disabled_limits(self):
        with self.assertRaises(ValueError):codec.nav_frame(1,1,10.,3.,.1,True,crc)
        with self.assertRaises(ValueError):codec.nav_frame(1,1,.1,0.,0.,False,crc)
        for vx,vy,wz in [(0.,-12.2,.3),(-4.,-5.,.3),(0.,0.,0.)]:
            frame=codec.nav_frame(1,1,vx,vy,wz,True,crc)
            a,b,c=struct.unpack_from('<fff',frame,24)
            self.assertLessEqual(abs(a)+abs(b)+abs(c),2000/159)

    def test_stillness_is_velocity_and_position_not_only_timer(self):
        still=State(0,0,0,0,0,0)
        window=[(i*.1,still) for i in range(11)]
        self.assertTrue(live.stationary(window))
        self.assertFalse(live.stationary(window[:3]))
        self.assertFalse(live.stationary([(i*.1,State(.002*i,0,0,0,0,0)) for i in range(11)]))
        self.assertFalse(live.stationary([(i*.1,State(0,0,0,.04,0,0)) for i in range(11)]))

    def test_zero_state_stamp_does_not_refresh_measurement(self):
        a=object.__new__(live.Adapter);a.last_generation=None;a.state_stamp=0;a.error=None
        Msg=collections.namedtuple('Msg','data')
        a.on_state(Msg('epoch=0;generation=2;stamp_ns=123;status=NORMAL;pose_trusted=true'))
        a.on_state(Msg('epoch=0;generation=2;stamp_ns=0;status=DEGRADED;pose_trusted=false'))
        self.assertEqual(a.state_stamp,123)
        self.assertEqual(a.state['status'],'DEGRADED')
        a.on_state(Msg('epoch=0;generation=3;stamp_ns=0'))
        self.assertEqual(a.state_stamp,0)

    def run_simulated_host(self, interrupt_schedule=False, global_mission=False):
        # Entire user entry, two segment transitions and lease release, using
        # a fake ROS/MCU and advancing clock. No serial device is constructed.
        clock=[100.];plant=dict(x=0.,y=.01,yaw=.02,f=0.,l=0.,w=0.,cmd=(0.,0.,0.))
        instances=[]
        class FakeWire:
            def __init__(self):
                self.started=False;self.closed=False
                self.stream=types.SimpleNamespace(boot=99,state=0x80,reject=0,bad_crc=0,count=100)
                instances.append(self)
            def open_and_drop_privileges(self):pass
            def preflight(self):pass
            def receive(self):pass
            def health(self,nav_age,require_lease=True):
                if nav_age>.35 and require_lease:
                    assert self.stream.state&0x60==0x60
            def send(self,vx,vy,wz,enabled=True):
                self.started=True;plant['cmd']=(vx,vy,wz) if enabled else (0.,0.,0.)
                self.stream.state=0xe0 if enabled else 0x80
            def close(self):self.closed=True;plant['cmd']=(0.,0.,0.)
        class FakeAdapter:
            def __init__(self,node,*,check_pose_jumps=True):
                self.pose={};self.odom={};self.state={};self.boot={}
                actual=json.loads((ROOT/'map_route/global_plan.json').read_text(encoding='utf-8'))
                self.store=types.SimpleNamespace(reg={k:actual[k] for k in ['registration_id','map_source_signature']})
            def sample(self):
                return State(*(plant[k] for k in ['x','y','yaw','f','l','w'])),.08,int(clock[0]*1e9),('test',)
        def spin(node,timeout_sec):
            dt=.005
            if interrupt_schedule and instances[0].started and clock[0]>103:dt=.15
            clock[0]+=dt
            vx,vy,wz=plant['cmd'];yaw=plant['yaw']
            plant['f']+=(-vy*(.0885 if vy<=0 else .095)-plant['f'])*.1
            plant['l']+=(vx*.095-plant['l'])*.1
            plant['w']+=(-wz*.8-plant['w'])*.1
            plant['x']+=(math.cos(yaw)*plant['f']-math.sin(yaw)*plant['l'])*dt
            plant['y']+=(math.sin(yaw)*plant['f']+math.cos(yaw)*plant['l'])*dt
            plant['yaw']+=plant['w']*dt
        import math
        fake_ros=types.SimpleNamespace(init=lambda **kw:None,spin_once=spin,
            create_node=lambda n:types.SimpleNamespace(destroy_node=lambda:None),ok=lambda:True,shutdown=lambda:None)
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);(root/'logs').mkdir()
            fixture=root/'fixture';fixture.mkdir()
            config=json.loads((live.HERE/'route.json').read_text(encoding='utf-8'));config['physical_entry_enabled']=True
            (fixture/'route.json').write_text(json.dumps(config),encoding='utf-8')
            firmware=dict(gain_rpm_per_unit=3859.9,yaw_lever_m=.35,yaw_sign=-1.,source_confirmed_flashed_by_user=True,user_confirmed_flashed=True)
            if global_mission:
                firmware['motor_rpm_ceiling']=json.loads((live.HERE/'firmware_profile.json').read_text(encoding='utf-8'))['motor_rpm_ceiling']
            (fixture/'firmware_profile.json').write_text(json.dumps(firmware),encoding='utf-8')
            (fixture/'firmware_source_review.json').write_text(json.dumps(firmware),encoding='utf-8')
            global_plan=json.loads((live.HERE/'global_plan.json').read_text(encoding='utf-8'))
            global_plan.update(physical_entry_enabled=True,task_point_label_field_confirmed=True)
            (fixture/'global_plan.json').write_text(json.dumps(global_plan),encoding='utf-8')
            argv=['live.py','--execute']+(['--mission','global'] if global_mission else [])
            with patch.object(live,'ROOT',root),patch.object(live,'HERE',fixture),patch.object(live,'Wire',FakeWire),patch.object(live,'Adapter',FakeAdapter),\
                 patch.object(live,'checked_sources',return_value={}),patch.object(sys,'argv',argv),\
                 patch.object(sys.stdin,'isatty',return_value=True),patch.object(live.os,'tcgetpgrp',return_value=1,create=True),\
                 patch.object(live.os,'getpgrp',return_value=1,create=True),patch('builtins.input',return_value=''),\
                 patch.object(live.signal,'signal'),patch.object(live.time,'monotonic',side_effect=lambda:clock[0]),\
                 patch.dict(sys.modules,{'rclpy':fake_ros}):
                code=live.main()
            result=json.loads(next((root/'logs').glob('*/result.json')).read_text(encoding='utf-8'))
        self.assertTrue(instances[0].closed)
        return code,result

    def test_whole_host_two_legs_and_released_stop(self):
        code,result=self.run_simulated_host()
        self.assertEqual(code,0)
        self.assertEqual(result['outcome'],'completed')
        self.assertTrue(result['stop_observed'])
        self.assertEqual(len(result['segments']),2)
        self.assertLess(abs(result['final_state']['x']-1.1),.04)
        self.assertLess(abs(result['final_state']['y']+1.25),.04)

    def test_whole_host_scheduling_fault_aborts_and_closes(self):
        code,result=self.run_simulated_host(interrupt_schedule=True)
        self.assertEqual(code,2)
        self.assertIn('scheduling gap',result['error'])
        self.assertFalse(result['stop_observed'])

    def test_whole_global_host_outbound_dwell_return_and_release(self):
        code,result=self.run_simulated_host(global_mission=True)
        self.assertEqual(code,0)
        self.assertEqual(result['outcome'],'completed')
        self.assertTrue(result['task_reached'] and result['home_reached'] and result['stop_observed'])
        self.assertFalse(result['task_actions_sent'])
        self.assertLess(result['elapsed_s'],250.)

    def test_whole_global_host_scheduling_fault_closes(self):
        code,result=self.run_simulated_host(interrupt_schedule=True,global_mission=True)
        self.assertEqual(code,2)
        self.assertIn('scheduling gap',result['error'])
        self.assertFalse(result['home_reached'])


if __name__=='__main__':unittest.main()
