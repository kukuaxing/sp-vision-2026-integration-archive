#!/usr/bin/env python3
"""Manage only this navigation directory's MID-360 driver process group."""
from pathlib import Path
import argparse
import json
import os
import signal
import subprocess
import time
ROOT=Path(__file__).resolve().parent
PID_FILE=ROOT/'logs/lidar_driver.pid'
def active_pid():
    try:
        pid=int(PID_FILE.read_text().strip())
        if pid<=1 or (Path('/proc')/str(pid)).stat().st_uid!=os.getuid(): return None
        proc=Path('/proc')/str(pid)
        state=(proc/'stat').read_text().rsplit(')',1)[1].split()[0]
        args=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        if state in ('Z','X') or os.getpgid(pid)!=pid: return None
        if 'mid360_observe.launch.py' not in args or str(ROOT/'real_bringup/MID360.vehicle.json') not in args:return None
        return pid
    except (ValueError,OSError,IndexError):return None

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['start','stop','status'])
    a=p.parse_args();pid=active_pid()
    if a.action=='status':
        print(json.dumps({'driver_process_running':pid is not None,'pid':pid,'log':str(ROOT/'logs/lidar_driver.log'),'note':'Process status only; use run.sh probe to check live messages.'},indent=2));return 0
    if a.action=='start':
        if pid:print('MID-360 driver already running:',pid);return 0
        with (ROOT/'logs/lidar_driver.log').open('a') as log:
            child=subprocess.Popen(['bash',str(ROOT/'start_lidar.sh')],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            PID_FILE.write_text(str(child.pid)+'\n')
            time.sleep(1)
            if child.poll() is not None:print('Driver launch failed; inspect logs/lidar_driver.log');return 2
        print('MID-360 driver launch process:',child.pid);return 0
    if pid is None:print('No matching MID-360 launch process; no signal sent.');return 0
    os.killpg(pid,signal.SIGINT)
    until=time.monotonic()+8
    while time.monotonic()<until:
        if active_pid() is None:print('MID-360 driver stopped.');return 0
        time.sleep(.1)
    print('MID-360 driver has not exited; inspect log. No forced kill sent.');return 3
if __name__=='__main__':raise SystemExit(main())
