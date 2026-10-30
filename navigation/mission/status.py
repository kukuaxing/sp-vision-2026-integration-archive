#!/usr/bin/env python3
"""Read readiness receipts only; no services, devices or control are started."""
from pathlib import Path
import argparse
import json
import time

N=Path(__file__).resolve().parents[1]


def read(path):
    try:return json.loads(path.read_text())
    except (OSError,ValueError):return {}


def readiness(now,boot_id,supervisor,mission,navigation):
    supervisor_fresh=supervisor.get('boot_id')==boot_id and 0<=now-supervisor.get('monotonic_s',-1)<2
    mission_fresh=mission.get('boot_id')==boot_id and 0<=now-mission.get('monotonic_s',-1)<1
    nav_fresh=navigation.get('boot_id')==boot_id and 0<=time.time()-navigation.get('updated_unix',0)<2
    ready=bool(supervisor_fresh and supervisor.get('phase')=='RUNNING' and mission_fresh and
        mission.get('phase')=='READY_WAIT_BOTH_UP' and mission.get('ready') and
        not mission.get('auto') and not mission.get('observe_only') and nav_fresh and
        navigation.get('localization_ready'))
    fault=supervisor_fresh and supervisor.get('phase')=='FAULT_LATCHED'
    if ready:message='已就绪：保持在P00，允许重新拨到双上开始任务。'
    elif fault:message='任务故障已锁定：'+str(supervisor.get('reason'))
    elif not supervisor_fresh:message='启动监管未运行或状态过期。'
    elif supervisor.get('phase')=='WAIT_DEVICES':message='等待硬件：'+str(supervisor.get('reason'))
    elif not nav_fresh or not navigation.get('localization_ready'):message='等待导航：'+str(navigation.get('phase','尚未收到状态'))
    elif mission_fresh:message='任务状态：'+str(mission.get('phase'))+'；'+str(mission.get('wait_reason') or mission.get('error') or '')
    else:message='等待视觉/任务管理器首帧状态。'
    return dict(ready=ready,fault_latched=bool(fault),message=message,
        supervisor_fresh=supervisor_fresh,mission_fresh=mission_fresh,navigation_fresh=nav_fresh,
        supervisor=supervisor.get('phase'),navigation=navigation.get('phase'),mission=mission.get('phase'))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--json',action='store_true');a=parser.parse_args()
    result=readiness(time.monotonic(),Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        read(N/'logs/mission/supervisor.json'),read(N/'logs/mission/status.json'),read(N/'logs/boot/status.json'))
    print(json.dumps(result,ensure_ascii=False,indent=2) if a.json else result['message'])
    raise SystemExit(0 if result['ready'] else 2 if result['fault_latched'] else 1)
