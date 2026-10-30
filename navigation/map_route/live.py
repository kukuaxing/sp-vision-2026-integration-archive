"""Registered map route adapter. Default is receive-only; --execute is local-human only."""
from pathlib import Path
import argparse
from collections import deque
import hashlib
import importlib.util
import json
import math
import os
import signal
import sys
import time
import uuid

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT/'boot'))
from course_registration import RegistrationStore, project
from core import State, SourceGuard, Tracker, wrap
from firmware import NativeProfile,validate_flashed_profile


def load(name, path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module
    spec.loader.exec_module(module)
    return module


def checked_sources():
    manifest=json.loads((HERE/'sources.json').read_text(encoding='utf-8'))
    for name, expected in manifest.items():
        p=(ROOT/name).resolve()
        if not p.is_relative_to(ROOT) or hashlib.sha256(p.read_bytes()).hexdigest()!=expected:
            raise RuntimeError('Reviewed source changed: '+name)
    return manifest


def age(stamp_ns):
    return (time.time_ns()-stamp_ns)/1e9


class Adapter:
    def __init__(self, node, *, check_pose_jumps=True):
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from std_msgs.msg import String
        from rclpy.qos import qos_profile_sensor_data
        self.node=node;self.store=RegistrationStore(ROOT)
        self.pose=None;self.odom=None;self.state={};self.state_stamp=0;self.state_rx=0.
        self.boot=None;self.boot_rx=0.;self.error=None;self.checked_publishers=0.
        self.guard=SourceGuard(check_pose_jumps=check_pose_jumps);self.last_generation=None
        self.subscriptions=[
            node.create_subscription(PoseWithCovarianceStamped,'/map_localization/localization_pose',self.on_pose,10),
            node.create_subscription(String,'/map_localization/localization_state',self.on_state,10),
            node.create_subscription(String,'/navigation/startup_status',self.on_boot,10),
            node.create_subscription(Odometry,'/navigation/odom',self.on_odom,qos_profile_sensor_data)]

    @staticmethod
    def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec

    def on_pose(self,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        vals=[p.x,p.y,p.z,q.x,q.y,q.z,q.w,*m.pose.covariance]
        if m.header.frame_id!='motion_plane_candidate' or not all(math.isfinite(v) for v in vals) or not .99<q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<1.01:
            self.error='Invalid map frame/pose';return
        self.pose=dict(xyz=[p.x,p.y,p.z],stamp_ns=self.stamp(m),received=time.monotonic(),
                       yaw_deg=math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))

    def on_odom(self,m):
        v=m.twist.twist.linear;w=m.twist.twist.angular
        if m.header.frame_id!='odom' or m.child_frame_id!='base_link' or not all(math.isfinite(x) for x in [v.x,v.y,v.z,w.x,w.y,w.z]):
            self.error='Invalid base velocity/frame';return
        self.odom=dict(stamp=self.stamp(m),received=time.monotonic(),f=v.x,l=v.y,w=w.z)

    def on_state(self,m):
        try:
            fields=dict(t.split('=',1) for t in m.data.split(';') if '=' in t)
            gen=(fields.get('epoch'),fields.get('generation'))
            if self.last_generation is not None and gen!=self.last_generation:
                self.state_stamp=0
            self.last_generation=gen
            # Maintenance reports have stamp_ns=0. They can revoke health,
            # but never make an old measurement fresh again.
            self.state_stamp=max(self.state_stamp,int(fields.get('stamp_ns','0')))
            self.state=fields;self.state_rx=time.monotonic()
        except (ValueError,TypeError):self.error='Malformed backend state'

    def on_boot(self,m):
        try:self.boot=json.loads(m.data);self.boot_rx=time.monotonic()
        except (ValueError,TypeError):self.error='Malformed startup status'

    def sample(self):
        now=time.monotonic();self.store.update()
        if self.error:raise RuntimeError(self.error)
        b=self.boot;p=self.pose;o=self.odom;r=self.store.reg;d=self.state
        if not b or not p or not o or not r:raise RuntimeError('Waiting for map/base/status/registration')
        local_boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        # During cold boot the monitor deliberately publishes course_pose=null
        # until a trusted registered map pose exists. This is a waiting state.
        course_pose=b.get('course_pose') or {}
        boot_epoch=tuple(b.get('epoch') or ())
        healthy=(b.get('localization_ready') is True and b.get('course_reference_valid') is True
                 and b.get('boot_id')==local_boot and now-self.boot_rx<.75
                 and 0<=time.time()-b['updated_unix']<.75
                 and boot_epoch==self.last_generation
                 and course_pose.get('registration_id')==r['registration_id']
                 and d.get('status')=='NORMAL' and d.get('pose_trusted')=='true'
                 and d.get('tf_frozen')=='false' and now-self.state_rx<.30
                 and -.05<=age(self.state_stamp)<=.30
                 and -.05<=age(o['stamp'])<=.30 and now-o['received']<.30
                 and abs(o['stamp']-p['stamp_ns'])<250000000)
        if now-self.checked_publishers>1.:
            for topic in ['/map_localization/localization_pose','/map_localization/localization_state',
                          '/navigation/odom','/navigation/startup_status']:
                if len(self.node.get_publishers_info_by_topic(topic))!=1:
                    raise RuntimeError('Expected exactly one producer: '+topic)
            self.checked_publishers=now
        if not healthy:
            detail=dict(phase=b.get('phase'),input_fresh=b.get('input_fresh'),
                        backend_status=d.get('status'),pose_trusted=d.get('pose_trusted'),
                        tf_frozen=d.get('tf_frozen'),state_age_s=age(self.state_stamp),
                        odom_age_s=age(o['stamp']),map_age_s=age(p['stamp_ns']),
                        state_receive_age_s=now-self.state_rx,odom_receive_age_s=now-o['received'],
                        monitor_receive_age_s=now-self.boot_rx,
                        monitor_age_s=time.time()-b['updated_unix'],
                        epoch_match=boot_epoch==self.last_generation,
                        boot_match=b.get('boot_id')==local_boot,
                        course_reference_valid=b.get('course_reference_valid'),
                        registration_match=course_pose.get('registration_id')==r['registration_id'],
                        pose_odom_stamp_difference_s=abs(o['stamp']-p['stamp_ns'])/1e9)
            raise RuntimeError('Localization not healthy: '+json.dumps(detail,separators=(',',':')))
        projected=project(r,p)
        s=State(*projected['xy_m'],math.radians(projected['yaw_deg']),o['f'],o['l'],o['w'])
        fingerprint=hashlib.sha256(json.dumps(r,sort_keys=True).encode()).hexdigest()
        identity=(b['boot_id'],b['instance'],*self.last_generation,r['registration_id'],fingerprint)
        source_age=self.guard.check(identity,p['stamp_ns'],time.time_ns(),now-p['received'],s,healthy)
        return s,source_age,p['stamp_ns'],identity


class Wire:
    """Existing exclusive transport, V2 codec, and shared durable sequence journal."""
    def __init__(self):
        if os.geteuid()!=0:raise RuntimeError('Use start.sh; exclusive owner audit requires the checked sudo worker')
        sys.path.insert(0,str(ROOT/'field_next_20261020'))
        shared=load('_map_route_shared',ROOT/'field_next_20261020/forward_trial.py')
        self.codec=load('_map_route_codec',ROOT/'full_course_20261022/aligned_course/guidance.py')
        self.support=load('_map_route_support',ROOT/'field_next_20261020/field_support.py')
        self.stream=shared.Stream();self.crc16=shared.crc16
        self.io=shared.SerialTransport('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',921600)
        self.started=False;self.sequence=None;self.boot=None;self.opened=False
        self.journal=ROOT/'logs/forward_trial_sequence.json'
        self.profile=NativeProfile.from_config(json.loads((HERE/'firmware_profile.json').read_text(encoding='utf-8')))

    def open_and_drop_privileges(self):
        import pwd
        import termios
        account=pwd.getpwuid(ROOT.stat().st_uid)
        if account.pw_uid==0:raise RuntimeError('Navigation must belong to ordinary user')
        self.io.open();self.opened=True
        termios.tcflush(self.io._fd,termios.TCIFLUSH)
        os.initgroups(account.pw_name,account.pw_gid);os.setgid(account.pw_gid);os.setuid(account.pw_uid)
        os.environ['HOME']=account.pw_dir

    def receive(self):
        self.stream.feed(self.io.read_available(),time.monotonic())

    def preflight(self):
        s=self.stream
        if s.boot is None or s.state is None or time.monotonic()-s.status_at>.20:
            raise RuntimeError('MCU heartbeat missing/stale')
        if not s.state&0x80:raise RuntimeError('Physical AUTO not enabled')
        if s.state&0x60:raise RuntimeError('An existing MCU navigation lease is active')
        if s.reject!=0:raise RuntimeError('Existing MCU rejection')
        j=json.loads(self.journal.read_text(encoding='utf-8'))
        if type(j.get('next_sequence')) is not int or not 0<=j['next_sequence']<=65535 or j.get('boot_id')!=s.boot:
            raise RuntimeError('MCU boot/sequence journal differs; deployment review needed')
        self.sequence=j['next_sequence'];self.boot=s.boot

    def health(self, nav_age, require_lease=True):
        s=self.stream
        if s.boot!=self.boot or time.monotonic()-s.status_at>.20 or s.state is None:
            raise RuntimeError('MCU heartbeat/boot invalid')
        if not s.state&0x80:raise RuntimeError('Operator left AUTO')
        if s.reject!=0:raise RuntimeError('MCU command rejection')
        if require_lease and nav_age>.35 and s.state&0x60!=0x60:raise RuntimeError('MCU navigation lease not accepted')

    def send(self, vx=0.,vy=0.,wz=0.,enabled=True):
        native=self.profile.convert(vx,vy,wz)
        frame=self.codec.nav_frame(self.boot,self.sequence,native['wire_vx'],native['wire_vy'],native['wire_wz'],enabled,self.crc16)
        self.sequence=(self.sequence+1)&65535
        self.support.persist_sequence(self.journal,self.sequence,self.boot)
        self.started=True
        self.io.submit(frame,ttl_s=.05,stop=(vx==0 and vy==0 and wz==0))
        if not self.io.pump(timeout_s=.01):raise RuntimeError('Serial write incomplete')

    def close(self):
        # Bounded independent revoke even when ROS/source guards have failed.
        # A closed/faulted link cannot guarantee stop delivery; MCU has its own TTL.
        error=None
        try:
            if self.started and self.io.is_open:
                until=time.monotonic()+.25
                while time.monotonic()<until:
                    self.send(enabled=False);time.sleep(.02)
        except Exception as e:error=str(e)
        finally:
            try:self.io.close()
            except Exception as e:error=error or str(e)
        return error


def stationary(window):
    if len(window)<8 or window[-1][0]-window[0][0]<.9:return False
    states=[v[1] for v in window]
    speeds=[math.hypot(s.forward_speed,s.left_speed) for s in states]
    diameter=max(math.hypot(a.x-b.x,a.y-b.y) for a in states for b in states)
    yaw_span=max(abs(wrap(a.yaw-b.yaw)) for a in states for b in states)
    return diameter<=.012 and max(speeds)<=.03 and sum(speeds)/len(speeds)<=.015 and yaw_span<=.02 and max(abs(s.yaw_rate) for s in states)<=.04


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--mission',choices=['two_leg','global'],default='two_leg')
    parser.add_argument('--seconds',type=float,default=5.,help='Receive-only duration')
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or not 1<=args.seconds<=30:raise ValueError('Preview must be 1..30 seconds')
    if args.mission=='global':
        from global_live import run
        return run(args)
    manifest=checked_sources();config=json.loads((HERE/'route.json').read_text(encoding='utf-8'))
    if (config['schema']!='registered_two_leg_route_v1' or config['frame']!='course_start'
        or config['heading_deg']!=0 or config['deadline_s']!=300 or config['automatic_obstacle_avoidance'] is not False):
        raise ValueError('Unsupported route contract')
    tracker=Tracker(config['waypoints_m'],config['speed_ceiling_m_s'])
    profile_config=json.loads((HERE/'firmware_profile.json').read_text(encoding='utf-8'))
    profile=NativeProfile.from_config(profile_config)
    if args.execute:
        if config.get('physical_entry_enabled') is not True:
            raise RuntimeError(config.get('physical_entry_block_reason','Physical entry not prepared'))
        if profile.yaw_lever_m<=0:
            raise RuntimeError('Current flashed firmware has no configured yaw lever')
        validate_flashed_profile(profile_config,json.loads((HERE/'firmware_source_review.json').read_text(encoding='utf-8')))
        if not sys.stdin.isatty() or os.tcgetpgrp(0)!=os.getpgrp():raise RuntimeError('Run directly in the foreground robot terminal')
        print('地图坐标两段：前进1.10米 → 右移1.25米，车头保持初始赛道方向。',flush=True)
        print('最高目标速度0.45米/秒；每段停稳后继续。只做这两段，不返回，不自动避障。',flush=True)
        print('请在P00沿P01摆车，核对两段及停车余量清空，自瞄和发射关闭、摇杆回中。',flush=True)
        input('现场就绪后由您切AUTO，再按Enter开始；异常立即STOP。正常等结束提示后STOP：')
        if checked_sources()!=manifest:raise RuntimeError('Source manifest changed while waiting')
    wire=None;node=None;ros=None;trace=None;out=None
    result=dict(mode='execute' if args.execute else 'receive_only',outcome='preflight_failed',
                motion_commands_sent=False,stop_observed=False,automatic_obstacle_avoidance=False,
                sources=manifest,route=config,firmware_profile=profile_config,segments=[],physical_accuracy_validated=False)
    def interrupted(signum,frame):raise InterruptedError('Operator/process interruption')
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    try:
        if args.execute:
            wire=Wire();wire.open_and_drop_privileges()
        out=ROOT/'logs'/('map_route_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
        out.mkdir();trace=(out/'trace.jsonl').open('w',buffering=1,encoding='utf-8')
        os.environ['ROS_LOG_DIR']=str(out/'ros')
        import rclpy
        ros=rclpy;ros.init(args=[]);node=ros.create_node('registered_map_route_once')
        adapter=Adapter(node)
        def spin():
            ros.spin_once(node,timeout_sec=.005)
            if wire:wire.receive()
        # Already-running localization normally takes ~1 second of subscriptions.
        limit=time.monotonic()+8.;stable_since=None;window=deque();last_stamp=None;last_reason='no input'
        while time.monotonic()<limit:
            spin()
            try:
                s,source_age,stamp,identity=adapter.sample()
                if wire:wire.preflight()
            except RuntimeError as e:
                if stable_since is not None:raise
                last_reason=str(e);continue
            if stable_since is None:stable_since=time.monotonic()
            if stamp!=last_stamp:window.append((time.monotonic(),s));last_stamp=stamp
            if stationary(window):break
        else:raise RuntimeError('Preflight: '+last_reason+' / stationary interval unavailable')
        result.update(initial_state=vars(s),localization_identity=identity)
        if math.hypot(s.x,s.y)>.075 or abs(s.yaw)>math.radians(8):
            raise RuntimeError('Place base centre on P00 and heading along P01 (7.5cm/8deg start envelope)')
        if checked_sources()!=manifest:raise RuntimeError('Source manifest changed during preflight')
        begin=time.monotonic();previous=begin;last_send=begin-.03
        deadline=begin+(300. if args.execute else args.seconds)
        phase='tracking';stop_began=None;segment_began=begin;window.clear();last_stamp=None
        while time.monotonic()<deadline:
            spin();now=time.monotonic()
            if now-last_send<.02:continue
            dt=now-previous;previous=now
            if dt>.10:raise RuntimeError('Control scheduling gap')
            s,source_age,stamp,identity=adapter.sample()
            if wire:wire.health(now-begin,require_lease=phase!='releasing')
            if phase=='tracking':
                command=tracker.step(s,dt,source_age)
                if args.execute and command['reached']:
                    phase='stopping';stop_began=now;window.clear();last_stamp=None
                if args.execute and now-segment_began>45:
                    raise RuntimeError('Segment made insufficient progress within 45 seconds')
            else:
                command=dict(segment=min(tracker.index+1,2),wire_vx=0.,wire_vy=0.,wire_wz=0.)
            command['native_firmware_command']=profile.convert(command['wire_vx'],command['wire_vy'],command['wire_wz'],preview=not args.execute)
            if wire:wire.send(command['wire_vx'],command['wire_vy'],command['wire_wz'],enabled=phase!='releasing')
            last_send=now
            record=dict(elapsed_s=now-begin,phase=phase,source_stamp_ns=stamp,state=vars(s),command=command,
                        map_pose=adapter.pose,body_velocity=adapter.odom,backend_state=adapter.state,
                        mcu_state=wire.stream.state if wire else None)
            trace.write(json.dumps(record,allow_nan=False)+'\n')
            if not args.execute:continue  # No serial object, no publisher, no route advance.
            if phase in ('stopping','releasing'):
                if stamp!=last_stamp:window.append((now,s));last_stamp=stamp
                while len(window)>1 and now-window[0][0]>1.15:window.popleft()
                target=tracker.points[min(tracker.index+1,len(tracker.points)-1)]
                error=math.hypot(s.x-target[0],s.y-target[1])
                if phase=='releasing':
                    if error>.075:raise RuntimeError('Drift after lease release')
                    if now-stop_began>=1.2 and stationary(window):
                        result.update(outcome='completed',stop_observed=True);break
                    if now-stop_began>3.:raise RuntimeError('Stop not stable after lease release')
                elif stationary(window):
                    if error>.075 or abs(s.yaw)>math.radians(5):
                        raise RuntimeError('Stopped outside endpoint/heading tolerance')
                    result['segments'].append(dict(segment=tracker.index+1,endpoint_error_m=error,final_state=vars(s)))
                    tracker.advance();phase='tracking';segment_began=now
                    if tracker.index==len(tracker.segments):
                        phase='releasing';stop_began=now;window.clear();last_stamp=None
                    else:print('第一段已停稳，开始右移1.25米。',flush=True)
                elif now-stop_began>5.:
                    raise RuntimeError('Stop was not observed within 5 seconds')
        else:
            if args.execute:raise RuntimeError('300 second mission deadline exceeded')
            result['outcome']='receive_only_completed'
        result.update(final_state=vars(s),elapsed_s=time.monotonic()-begin)
    except Exception as e:
        result.update(outcome='aborted',error=str(e))
        print('拒绝或中止：'+str(e),flush=True)
    finally:
        if wire:
            result['motion_commands_sent']=wire.started
            result['revoke_error']=wire.close()
            if result['revoke_error']:
                result['outcome']='stop_delivery_unconfirmed';result['stop_observed']=False
            result['mcu']=dict(boot=wire.stream.boot,state=wire.stream.state,reject=wire.stream.reject,
                               crc_errors=wire.stream.bad_crc,frames=wire.stream.count)
        if trace:trace.close()
        if node:node.destroy_node()
        if ros and ros.ok():ros.shutdown()
        if out:(out/'result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
        if args.execute:print('本次入口已关闭动作授权。请切STOP并确认停稳。',flush=True)
        else:print('只读预演结束，未打开串口，未发送动作。',flush=True)
        if out:print('记录：'+str(out),flush=True)
    return 0 if result['outcome'] in ('completed','receive_only_completed') else 2


if __name__=='__main__':
    try:sys.exit(main())
    except (RuntimeError,ValueError,OSError) as e:
        print('入口未启动：'+str(e)+'。请保持STOP。',file=sys.stderr)
        sys.exit(2)
