#!/usr/bin/env python3
"""Boot mission manager. Serial IO belongs exclusively to the vision process."""
from pathlib import Path
import argparse
from collections import deque
import json
import math
import os
import signal
import sys
import time

HERE=Path(__file__).resolve().parent
N=HERE.parent
sys.path.insert(0,str(N/'map_route'))
import live
from mission import Mission,stopped
from firmware import validate_flashed_profile
from policy import StartGate,ReturnPolicy,vision_health


def atomic(path,value):
    tmp=path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    tmp.replace(path)


def departure_readiness(window):
    """Use the validated competition stop guard; retain absolute start limits."""
    s=window[-1][1] if window else None
    checks=dict(stopped=stopped(window,fast=True),at_p00=False,heading_ok=False,
                distance_m=None,yaw_deg=None)
    if s is not None:
        distance=math.hypot(s.x,s.y);yaw=math.degrees(s.yaw)
        if math.isfinite(distance) and math.isfinite(yaw):
            checks.update(at_p00=distance<=.075,heading_ok=abs(yaw)<=8.,
                          distance_m=distance,yaw_deg=yaw)
    return all(checks[k] for k in ('stopped','at_p00','heading_ok')),checks


def run(ipc,*,check_only=False):
    sources=live.checked_sources()
    observe_only=os.environ.get('RM27_MISSION_OBSERVE_ONLY')=='1'
    plan=json.loads((N/'map_route/global_plan.json').read_text())
    task_policy=plan['competition_task']
    profile=validate_flashed_profile(json.loads((N/'map_route/firmware_profile.json').read_text()),
                                    json.loads((N/'map_route/firmware_source_review.json').read_text()))
    if plan.get('physical_entry_enabled') is not True:raise RuntimeError('Route not enabled')
    if plan.get('task_point_label_field_confirmed') is not True:raise RuntimeError('Task endpoint not confirmed')
    Mission(plan,external_task=True) # Also validates reviewed path, speed and return endpoints.
    if check_only:
        print('Mission configuration valid; no ROS node, serial, camera or actuation started.')
        return
    import rclpy
    rclpy.init(args=[]);node=rclpy.create_node('rm27_competition_mission')
    log=N/'logs/mission';log.mkdir(parents=True,exist_ok=True)
    trace=(log/'events.jsonl').open('a',buffering=1,encoding='utf-8')
    motion_trace=(log/'motion.jsonl').open('a',buffering=1,encoding='utf-8')
    stopping=False
    def stop(sig,frame):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    adapter=live.Adapter(node,check_pose_jumps=False)
    gate=StartGate();mission=None;policy=None;window=deque();last_stamp=None
    last_tick=time.monotonic();begin=None;vision_id=None;pose_generation=None;phase='WAIT_READY';reason=None
    status_at=0;task_return_since=None;last_vision_id=None;motion_trace_at=0
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    last_wait_signature=None;last_wait_at=0
    def command(v,now,nav=False,task=False,values=(0.,0.,0.)):
        atomic(ipc/'command.json',dict(schema=2,count_source='host_tx',monotonic_ns=int(now*1e9),
            vision_instance=v.get('vision_instance',''),mcu_boot=v.get('mcu_boot',0),
            nav_enabled=nav,task_enabled=task,vx=values[0],vy=values[1],wz=values[2],
            count_limit=policy.baseline+policy.maximum if policy else 0,
            count_generation=policy.generation if policy else 0))
    def event(value):
        trace.write(json.dumps(dict(boot_id=boot_id,monotonic_s=time.monotonic(),**value),ensure_ascii=False)+'\n')
        print(json.dumps(value,ensure_ascii=False),flush=True)
    try:
        while not stopping:
            rclpy.spin_once(node,timeout_sec=.005)
            now=time.monotonic()
            if now-last_tick<.02:continue
            dt=now-last_tick;last_tick=now
            v={};ready=False;auto=False;fresh=False;error=None;used=0;start_checks=None;wait_reason=None
            motion_command=None
            try:
                v=json.loads((ipc/'vision.json').read_text())
                auto=vision_health(v,int(now*1e9));fresh=True
                if mission and v['vision_instance']!=vision_id:raise RuntimeError('Vision restarted during mission')
                if mission and v['pose_generation']!=pose_generation:raise RuntimeError('MCU pose generation changed')
                if last_vision_id!=v['vision_instance']:
                    if mission:raise RuntimeError('Vision instance changed')
                    gate=StartGate();last_vision_id=v['vision_instance']
                s,source_age,stamp,identity=adapter.sample()
                reg=adapter.store.reg
                if reg['registration_id']!=plan['registration_id'] or reg['map_source_signature']!=plan['map_source_signature']:
                    raise RuntimeError('Plan/map/P00 identity mismatch')
                if stamp!=last_stamp:window.append((now,s));last_stamp=stamp
                while len(window)>1 and now-window[0][0]>1.15:window.popleft()
                ready,start_checks=departure_readiness(window)
                if mission:
                    if dt>.1:raise RuntimeError('Mission control scheduling gap')
                    if not auto:raise RuntimeError('Remote left both-UP AUTO')
                    if now-begin>=plan['deadline_s']:raise RuntimeError('300s mission deadline exceeded')
                    if now-begin>.5 and mission.phase not in ('RELEASING','DONE') and v['mcu_state']&0x60!=0x60:
                        raise RuntimeError('MCU NAV lease not accepted')
                    used,reason=policy.update(now,v,task=mission.phase=='TASK_DWELL')
                    if reason and task_return_since is None:
                        task_return_since=now;event(dict(return_reason=reason,sent_requests=used))
                    # Revoke task first, wait beyond the maximum MCU lease, then move home.
                    complete=task_return_since is not None and now-task_return_since>=.25
                    c=mission.step(s,dt,source_age,stamp,now,task_complete=complete)
                    motion_command={k:c[k] for k in (
                        'planned_world_vx_m_s','planned_world_vy_m_s','planned_speed_m_s',
                        'planned_acceleration_m_s2','terminal_blend','path_segment',
                        'progress_m','lateral_error_m','endpoint_error_m','endpoint_speed_cap_m_s',
                        'endpoint_predicted_error_m','terminal_settling') if k in c}
                    native=profile.convert(c['wire_vx'],c['wire_vy'],c['wire_wz'])
                    task=c['phase']=='TASK_DWELL' and reason is None
                    command(v,now,nav=c['enabled'],task=task,
                            values=(native['wire_vx'],native['wire_vy'],native['wire_wz']))
                    if phase!=c['phase']:
                        phase=c['phase'];event(dict(phase=phase,sent_requests=used))
                    if phase=='DONE':
                        (log/'active_route.json').unlink(missing_ok=True)
                        mission=None;gate=StartGate();window.clear();last_stamp=None
                else:
                    command(v,now)
                    start_requested=gate.update(now,ready,auto,fresh)
                    if start_requested and not observe_only:
                        if not v.get('control_ready'):raise RuntimeError('AUTO aim reference not ready')
                        if live.checked_sources()!=sources:raise RuntimeError('Reviewed sources changed')
                        mission=Mission(plan,external_task=True)
                        policy=ReturnPolicy(v['fire_requests_sent'],v['count_generation'],v['mcu_boot'],
                            minimum=task_policy['lost_target_check_after_requests'],
                            maximum=task_policy['return_at_sent_requests'],lost_s=task_policy['armor_lost_s'])
                        vision_id=v['vision_instance'];begin=now;task_return_since=None;reason=None
                        pose_generation=v['pose_generation']
                        # Persist before the first enabled NAV command, so a
                        # killed supervisor cannot silently restart an active route.
                        atomic(log/'active_route.json',dict(active=True,boot_id=boot_id,
                            vision_instance=vision_id,monotonic_s=now))
                        phase='OUTBOUND';event(dict(phase=phase,baseline=policy.baseline))
                    else:
                        phase='READY_WAIT_BOTH_UP' if ready and (gate.armed or observe_only) else 'WAIT_READY'
                        if not start_checks['at_p00']:wait_reason='Not at P00 (limit 0.075m)'
                        elif not start_checks['heading_ok']:wait_reason='Start heading outside 8deg'
                        elif not start_checks['stopped']:wait_reason='Waiting for stationary observations'
                        elif not gate.armed and not observe_only:wait_reason='Hold non-AUTO while ready, then switch both-UP'
                        else:wait_reason='Ready; waiting for new both-UP edge'
                if now-motion_trace_at>=.1:
                    motion_trace.write(json.dumps(dict(boot_id=boot_id,monotonic_s=now,phase=phase,state=vars(s),
                        source_stamp_ns=stamp,source_age_s=source_age,sent_requests=used,
                        navigation_command=motion_command),allow_nan=False)+'\n')
                    motion_trace_at=now
            except (RuntimeError,ValueError,KeyError,OSError,TypeError) as e:
                error=str(e);wait_reason=error;command(v,now)
                if mission:
                    event(dict(phase='ABORTED',error=error));mission=None;policy=None
                    gate=StartGate()
                    # An abort is terminal until service restart.
                    phase='FAULT'
                    break
                gate.update(now,False,auto,fresh);phase='WAIT_READY'
                # Localization may replace its provisional generation before
                # departure. Rebuild only the idle guard, never an active route.
                adapter.guard=live.SourceGuard(check_pose_jumps=False)
                window.clear();last_stamp=None
            wait_signature=(phase,error,wait_reason)
            if mission is None and wait_signature!=last_wait_signature and now-last_wait_at>=1.:
                event(dict(event='startup_wait',phase=phase,ready=ready,auto=auto,
                           error=error,wait_reason=wait_reason,start_checks=start_checks))
                last_wait_signature=wait_signature;last_wait_at=now
            if now-status_at>=.2:
                atomic(log/'status.json',dict(phase=phase,ready=ready,error=error,monotonic_s=now,
                    boot_id=boot_id,start_checks=start_checks,wait_reason=wait_reason,
                    sent_requests=used,return_reason=reason,vision_instance=v.get('vision_instance'),
                    auto=auto,observe_only=observe_only,nav_route_speed_m_s=plan['speed_ceiling_m_s'],
                    nav_motor_rpm_ceiling=plan.get('motor_rpm_ceiling',2000.),
                    nav_jerk_limit_m_s3=plan.get('jerk_limit_m_s3',6.),
                    nav_corner_brake_margin_m=plan.get('corner_brake_margin_m',.10)))
                status_at=now
    except Exception as exc:
        phase='FAULT';error=str(exc)
        event(dict(phase=phase,error=error,unexpected_error=True))
        raise
    finally:
        try:command(v if 'v' in locals() else {},time.monotonic())
        finally:
            atomic(log/'status.json',dict(phase=phase if phase=='FAULT' else 'STOPPED',ready=False,
                error=error if 'error' in locals() else None,monotonic_s=time.monotonic(),
                boot_id=boot_id,vision_instance=v.get('vision_instance') if 'v' in locals() else None))
            trace.close();motion_trace.close();node.destroy_node();rclpy.shutdown()
    if phase=='FAULT':raise RuntimeError('Mission fault latched; inspect status and restart service')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--check',action='store_true')
    p.add_argument('--ipc',type=Path,default=Path(os.environ.get('RM27_MISSION_DIR','/run/rm27-mission')))
    a=p.parse_args();run(a.ipc,check_only=a.check)
