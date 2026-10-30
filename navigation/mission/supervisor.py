#!/usr/bin/env python3
"""Wait for devices, own one vision/mission session, recover idle failures only."""
from pathlib import Path
import argparse
import fcntl
import json
import os
import re
import signal
import socket
import subprocess
import time
import uuid
import yaml
from startup_watchdog import SessionWatchdog,interrupted_route

N=Path(__file__).resolve().parents[1];P=N.parent


def read_json(path):
    try:
        value=json.loads(path.read_text())
        return value if isinstance(value,dict) else {}
    except (OSError,ValueError,TypeError):return {}


def device_snapshot(project=P,sysfs=Path('/sys/bus/usb/devices')):
    """Presence/permissions only: no serial open or camera SDK."""
    config=yaml.safe_load((project/'configs/standard.yaml').read_text())
    if config.get('board_protocol')!='SENTINEL' or config.get('xuc_serial_baud')!=921600:
        raise ValueError('Boot mission requires the configured Sentinel V2 921600 link')
    port=Path(config['xuc_serial_port']);usb=str(config['vid_pid']).lower()
    if not re.fullmatch('[0-9a-f]{4}:[0-9a-f]{4}',usb):raise ValueError('Invalid camera USB VID:PID')
    cameras=[]
    for path in sysfs.glob('*'):
        try:
            if (path/'idVendor').read_text().strip().lower()+':'+(path/'idProduct').read_text().strip().lower()==usb:
                cameras.append(str(path.resolve()))
        except OSError:pass
    token=None
    try:
        st=port.stat();token=[str(port.resolve()),st.st_rdev,st.st_ino]
    except OSError:pass
    accessible=token is not None and os.access(port,os.R_OK|os.W_OK)
    reasons=[]
    if token is None:reasons.append('Waiting for MCU USB serial')
    elif not accessible:reasons.append('Waiting for serial read/write permission')
    if not cameras:reasons.append('Waiting for HikRobot USB camera')
    return dict(ready=accessible and bool(cameras),serial=str(port),serial_token=token,
                cameras=sorted(cameras),reason='; '.join(reasons) or 'Devices enumerated')


def notify(message):
    address=os.environ.get('NOTIFY_SOCKET')
    if not address:return
    if address.startswith('@'):address='\0'+address[1:]
    try:
        with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as sock:sock.sendto(message.encode(),address)
    except OSError:pass


def stop_children(children,ipc):
    (ipc/'command.json').unlink(missing_ok=True) # revoke before ending serial owner
    def terminate(child,sig):
        if child.poll() is None:
            try:os.killpg(child.pid,sig)
            except ProcessLookupError:pass
    if len(children)>1:terminate(children[-1],signal.SIGTERM)
    if children:time.sleep(.3) # exceed last <=150ms MCU lease
    for child in children:terminate(child,signal.SIGTERM)
    until=time.monotonic()+6
    while any(p.poll() is None for p in children) and time.monotonic()<until:time.sleep(.05)
    for child in children:terminate(child,signal.SIGKILL)
    for child in children:
        try:child.wait(timeout=2)
        except subprocess.TimeoutExpired:pass


def run():
    log=N/'logs/mission';log.mkdir(parents=True,exist_ok=True)
    ipc=Path(os.environ.get('RM27_MISSION_DIR','/run/rm27-mission'))
    if not ipc.is_dir():raise RuntimeError('Missing managed mission IPC directory')
    boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip();instance=uuid.uuid4().hex
    lock=(log/'supervisor.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    stopping=False;children=[];watchdog=None;launch_devices=None
    devices={};next_probe=0.;next_launch=0.;last_write=0.;last_signature=None
    stable_since=None;phase='WAIT_DEVICES';reason='Waiting for devices';attempt=0
    if interrupted_route(read_json(log/'active_route.json'),boot):
        phase='FAULT_LATCHED';reason='Previous active route interrupted; inspect and explicitly restart service'
    def stop(sig,frame):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    def report(now):
        nonlocal last_write,last_signature
        status=dict(schema=1,boot_id=boot,instance=instance,pid=os.getpid(),monotonic_s=now,
            phase=phase,reason=reason,devices=devices,launch_attempts=attempt,
            child_pids=[p.pid for p in children if p.poll() is None],requires_new_both_up=True,
            supervisor_sends_motion_commands=False,mission_may_have_started=bool(watchdog and watchdog.activity_seen))
        temp=log/'supervisor.json.tmp';temp.write_text(json.dumps(status,ensure_ascii=False,indent=2));temp.replace(log/'supervisor.json')
        signature=(phase,reason)
        if signature!=last_signature:
            print(json.dumps(status,ensure_ascii=False),flush=True)
            notify('STATUS='+phase+': '+reason);last_signature=signature
        last_write=now
    try:
        while not stopping:
            now=time.monotonic()
            if now>=next_probe:
                try:devices=device_snapshot()
                except (OSError,ValueError,KeyError,TypeError) as exc:
                    devices=dict(ready=False,reason='Configuration/presence check: '+str(exc))
                next_probe=now+1.
            if children:
                try:
                    changed=devices.get('serial_token')!=launch_devices.get('serial_token') or devices.get('cameras')!=launch_devices.get('cameras')
                    decision,reason=watchdog.inspect(now,read_json(ipc/'vision.json'),read_json(log/'status.json'),
                        read_json(ipc/'command.json'),children_ok=all(p.poll() is None for p in children),
                        devices_ok=devices.get('ready',False) and not changed)
                except (ValueError,TypeError,KeyError) as exc:
                    decision,reason=watchdog.failure('Malformed session status: '+str(exc))
                if decision!='running':
                    stop_children(children,ipc);children=[];stable_since=None
                    phase='FAULT_LATCHED' if decision=='latch' else 'RETRY_PREPARING';next_launch=time.monotonic()+5
                    report(time.monotonic())
                else:phase='RUNNING'
            elif phase!='FAULT_LATCHED':
                if not devices.get('ready'):
                    phase='WAIT_DEVICES';reason=devices.get('reason');stable_since=None
                else:
                    if stable_since is None:stable_since=now
                    if now>=next_launch and now-stable_since>=1.:
                        for name in ('command.json','vision.json'):(ipc/name).unlink(missing_ok=True)
                        watchdog=SessionWatchdog(now,boot);launch_devices=devices.copy();attempt+=1
                        try:
                            env=dict(os.environ,RM27_MISSION_DIR=str(ipc))
                            for args in [['/bin/bash',str(P/'tools/field_start_autoaim.sh')],['/bin/bash',str(N/'mission/start.sh')]]:
                                children.append(subprocess.Popen(args,start_new_session=True,env=env))
                            phase='RUNNING';reason='Starting vision and idle mission manager'
                        except OSError as exc:
                            stop_children(children,ipc);children=[];stable_since=None
                            phase='RETRY_PREPARING';reason=str(exc);next_launch=now+5
            if now-last_write>=.5:report(now)
            time.sleep(.1)
    finally:
        stop_children(children,ipc);children=[]
        if stopping:
            # A deliberate service stop/restart clears the latch only after
            # revocation and child cleanup. New session still needs a fresh RC edge.
            (log/'active_route.json').unlink(missing_ok=True)
            phase='STOPPED';reason='Supervisor stopped'
        else:
            phase='FAULT_LATCHED';reason='Supervisor failed; activity marker retained'
        report(time.monotonic());lock.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    if args.check:print(json.dumps(device_snapshot(),ensure_ascii=False,indent=2))
    else:run()
