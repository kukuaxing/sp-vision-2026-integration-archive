#!/usr/bin/env python3
"""Manage only this navigation directory's FAST-LIO localization process group."""
from pathlib import Path
import argparse
import json
import os
import signal
import subprocess
import time
ROOT=Path(__file__).resolve().parent
PID_FILE=ROOT/'logs/localization.pid'
def active_pid():
    try:
        pid=int(PID_FILE.read_text().strip())
        if pid<=1 or (Path('/proc')/str(pid)).stat().st_uid!=os.getuid(): return None
        proc=Path('/proc')/str(pid)
        state=(proc/'stat').read_text().rsplit(')',1)[1].split()[0]
        args=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        if state in ('Z','X') or os.getpgid(pid)!=pid: return None
        if 'localization.launch.py' not in args or str(ROOT/'localization/fastlio.vehicle.yaml') not in args:return None
        return pid
    except (ValueError,OSError,IndexError):return None


def critical_nodes(pid):
    result = {'mapping_process_running': False, 'base_odometry_process_running': False}
    if pid is None:
        return result
    expected = {'mapping_process_running': str(ROOT/'install/localization/fast_lio/lib/fast_lio/fastlio_mapping'),
                'base_odometry_process_running': str(ROOT/'localization/base_odometry.py')}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if entry.stat().st_uid != os.getuid() or os.getpgid(int(entry.name)) != pid:
                continue
            if (entry/'stat').read_text().rsplit(')', 1)[1].split()[0] in ('Z', 'X'):
                continue
            args = (entry/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            for key, text in expected.items():
                if text in args:
                    result[key] = True
        except (OSError, ValueError, IndexError):
            continue
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['start','stop','status'])
    a=p.parse_args();pid=active_pid()
    if a.action=='status':
        nodes=critical_nodes(pid);print(json.dumps({'launch_process_running':pid is not None,**nodes,'localization_processes_running':all(nodes.values()),'pid':pid,'log':str(ROOT/'logs/localization.log'),'note':'Process checks only; verify_live.py checks actual messages and TF.'},indent=2));return 0
    if a.action=='start':
        if pid:
            ready=all(critical_nodes(pid).values())
            print('FAST-LIO localization already running:' if ready else 'Localization launch exists but a critical node is missing; stop/start required:',pid)
            return 0 if ready else 2
        with (ROOT/'logs/localization.log').open('a') as log:
            child=subprocess.Popen(['bash',str(ROOT/'start_localization.sh')],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            PID_FILE.write_text(str(child.pid)+'\n')
            time.sleep(1)
            if child.poll() is not None or not all(critical_nodes(child.pid).values()):print('Localization launch failed; inspect logs/localization.log');return 2
        print('FAST-LIO localization launch process:',child.pid);return 0
    if pid is None:print('No matching FAST-LIO localization launch process; no signal sent.');return 0
    os.killpg(pid,signal.SIGINT)
    until=time.monotonic()+8
    while time.monotonic()<until:
        if active_pid() is None:print('FAST-LIO localization stopped.');return 0
        time.sleep(.1)
    print('FAST-LIO localization has not exited; inspect log. No forced kill sent.');return 3
if __name__=='__main__':raise SystemExit(main())
