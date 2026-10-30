"""Run the complete registered navigation-only round trip with existing guarded IO."""
from pathlib import Path
import json
import math
import os
import signal
import sys
import time
import uuid
import live as common
from mission import Mission
from firmware import NativeProfile,validate_flashed_profile


def run(args):
    root=common.ROOT;here=common.HERE
    sources=common.checked_sources()
    plan=json.loads((here/'global_plan.json').read_text(encoding='utf-8'))
    firmware=json.loads((here/'firmware_profile.json').read_text(encoding='utf-8'))
    profile=NativeProfile.from_config(firmware)
    if (plan['schema']!='registered_global_round_trip_v1' or plan['frame']!='course_start'
        or plan['heading_deg']!=0 or plan['deadline_s']!=300 or plan['automatic_obstacle_avoidance'] is not False):
        raise ValueError('Unsupported global mission contract')
    mission=Mission(plan,observe_only=not args.execute)
    if args.execute:
        if plan.get('physical_entry_enabled') is not True:
            raise RuntimeError(plan['physical_entry_block_reason'])
        if profile.yaw_lever_m<=0:raise RuntimeError('Flashed yaw profile is not configured')
        validate_flashed_profile(firmware,json.loads((here/'firmware_source_review.json').read_text(encoding='utf-8')))
        if plan.get('task_point_label_field_confirmed') is not True:
            raise RuntimeError('Task point in the route preview has not been confirmed')
        if not sys.stdin.isatty() or os.tcgetpgrp(0)!=os.getpgrp():
            raise RuntimeError('Run directly in the foreground robot terminal')
        print(f"全局地图往返：P00→任务点→P00，约{plan['round_trip_length_m']:.2f}米。",flush=True)
        print(f"目标速度上限{plan['speed_ceiling_m_s']:.2f}米/秒；任务点停稳后停留{plan['task_dwell_s']:.0f}秒再返回。",flush=True)
        print('不控制自瞄、摩擦轮或发射；全程保持车头方向，以平移通过转角。',flush=True)
        print('请将底盘中心放在P00、车头朝首段；自瞄和发射关闭，核对预览路线和全线通道清空，摇杆回中。',flush=True)
        print('本轮无自动避障。异常立即STOP；正常等待全程结束提示后STOP。',flush=True)
        print('运动偏差继续纠偏；通信、定位或控制失效、遥控STOP及300秒任务上限仍会终止。',flush=True)
        input('现场就绪后由您切AUTO，再按Enter开始：')
        if common.checked_sources()!=sources:raise RuntimeError('Sources changed while waiting for human start')
    wire=None;node=None;ros=None;out=None;trace=None
    report=dict(mode='execute' if args.execute else 'receive_only_global',outcome='preflight_failed',
                motion_commands_sent=False,task_reached=False,home_reached=False,stop_observed=False,
                automatic_obstacle_avoidance=False,task_actions_sent=False,route=plan,firmware_profile=firmware,sources=sources)
    def interrupted(signum,frame):raise InterruptedError('Operator/process interruption')
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    try:
        if args.execute:
            wire=common.Wire();wire.open_and_drop_privileges()
        out=root/'logs'/('global_round_trip_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
        out.mkdir();trace=(out/'trace.jsonl').open('w',buffering=1,encoding='utf-8')
        os.environ['ROS_LOG_DIR']=str(out/'ros')
        import rclpy
        ros=rclpy;ros.init(args=[]);node=ros.create_node('registered_global_round_trip_once')
        adapter=common.Adapter(node,check_pose_jumps=False)
        def spin():
            ros.spin_once(node,timeout_sec=.005)
            if wire:wire.receive()
        window=[];stamp_previous=None;stable_since=None;limit=time.monotonic()+8.;last_reason='no input'
        while time.monotonic()<limit:
            spin()
            try:
                s,source_age,stamp,identity=adapter.sample()
                r=adapter.store.reg
                if r['registration_id']!=plan['registration_id'] or r['map_source_signature']!=plan['map_source_signature']:
                    raise RuntimeError('Global path does not match the active map/P00 reference')
                if wire:wire.preflight()
            except RuntimeError as e:
                if stable_since is not None:raise
                last_reason=str(e);continue
            if stable_since is None:stable_since=time.monotonic()
            if stamp!=stamp_previous:window.append((time.monotonic(),s));stamp_previous=stamp
            if common.stationary(window):break
        else:raise RuntimeError('Global preflight: '+last_reason+' / stationary interval unavailable')
        if math.hypot(s.x,s.y)>.075 or abs(s.yaw)>math.radians(8):
            raise RuntimeError('Place base centre at P00, heading along first segment')
        if common.checked_sources()!=sources:raise RuntimeError('Global source manifest changed during preflight')
        report.update(initial_state=vars(s),localization_identity=identity)
        begin=previous=time.monotonic();last_send=begin-.03;last_phase=None;last_progress=-2.
        deadline=begin+(300. if args.execute else args.seconds)
        while time.monotonic()<deadline:
            spin();now=time.monotonic()
            if now-last_send<.02:continue
            dt=now-previous;previous=now
            if dt>.10:raise RuntimeError('Global control scheduling gap')
            s,source_age,stamp,identity=adapter.sample()
            if wire:wire.health(now-begin,require_lease=mission.phase not in ('RELEASING','DONE'))
            command=mission.step(s,dt,source_age,stamp,now)
            command['native_firmware_command']=profile.convert(command['wire_vx'],command['wire_vy'],command['wire_wz'],preview=not args.execute)
            if wire:wire.send(command['wire_vx'],command['wire_vy'],command['wire_wz'],enabled=command['enabled'])
            last_send=now
            trace.write(json.dumps(dict(elapsed_s=now-begin,source_stamp_ns=stamp,state=vars(s),command=command,
                         map_pose=adapter.pose,body_velocity=adapter.odom,backend_state=adapter.state,
                         mcu_state=wire.stream.state if wire else None),allow_nan=False)+'\n')
            if command['phase']!=last_phase:
                names=dict(OUTBOUND='去程跟踪',STOP_TASK='任务点停车观察',TASK_DWELL='任务点停留',
                           RETURNING='返程跟踪',STOP_HOME='起点停车观察',RELEASING='撤销授权并观察',DONE='完整往返结束')
                print(names[command['phase']],flush=True);last_phase=command['phase'];last_progress=-2.
            if args.execute and command['phase'] in ('OUTBOUND','RETURNING') and command.get('progress_m',0)-last_progress>=2.:
                print(f"{command['phase']}: {command.get('progress_m',0):.1f} / {mission.follower.path.total:.1f}米",flush=True)
                last_progress=command.get('progress_m',0.)
            if command['phase']=='DONE':
                report.update(outcome='completed',stop_observed=True);break
        else:
            if args.execute:raise RuntimeError('Global mission exceeded 300 seconds')
            report['outcome']='receive_only_completed'
        report.update(final_state=vars(s),elapsed_s=time.monotonic()-begin)
    except Exception as e:
        report.update(outcome='aborted',error=str(e));print('全局导航拒绝或中止：'+str(e),flush=True)
        if 'adapter' in locals():
            report['failure_inputs']=dict(map_pose=adapter.pose,body_velocity=adapter.odom,
                                          backend_state=adapter.state,startup_status=adapter.boot)
        if 's' in locals():report['failure_state']=vars(s)
    finally:
        report.update(task_reached=mission.task_reached,home_reached=mission.home_reached,events=mission.events,final_phase=mission.phase)
        if wire:
            report['motion_commands_sent']=wire.started;report['revoke_error']=wire.close()
            if report['revoke_error']:report.update(outcome='stop_delivery_unconfirmed',stop_observed=False)
            report['mcu']=dict(boot=wire.stream.boot,state=wire.stream.state,reject=wire.stream.reject,
                               crc_errors=wire.stream.bad_crc,frames=wire.stream.count)
        if trace:trace.close()
        if node:node.destroy_node()
        if ros and ros.ok():ros.shutdown()
        if out:(out/'result.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
        if args.execute:print('本次入口已关闭动作授权。请切STOP并确认停稳。',flush=True)
        else:print('全局路线只读预演结束，未打开串口，未发送动作。',flush=True)
        if out:print('记录：'+str(out),flush=True)
    return 0 if report['outcome'] in ('completed','receive_only_completed') else 2
