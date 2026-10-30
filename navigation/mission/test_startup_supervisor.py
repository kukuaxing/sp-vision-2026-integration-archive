"""Run the actual supervisor event loop with fake time/devices/children only."""
import contextlib
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parent))
import supervisor


class SupervisorLoopTests(unittest.TestCase):
    def test_shutdown_revokes_ipc_before_manager_and_delays_serial_owner_stop(self):
        # Real local processes, but no ROS/camera/UART. Check termination order
        # against the same production cleanup function used by the service.
        source="""from pathlib import Path
import sys,time,signal,json
root=Path(sys.argv[1]);name=sys.argv[2]
def end(sig,frame):
 with (root/'events').open('a') as out:out.write(json.dumps(dict(name=name,time=time.monotonic(),command_exists=(root/'command.json').exists()))+'\\n')
 raise SystemExit(0)
signal.signal(signal.SIGTERM,end)
(root/name).touch()
while True:time.sleep(.02)
"""
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'command.json').write_text('{}');children=[]
            try:
                for name in ('vision','manager'):
                    children.append(subprocess.Popen([sys.executable,'-c',source,str(root),name],start_new_session=True))
                until=time.monotonic()+3
                while not all((root/name).exists() for name in ('vision','manager')) and time.monotonic()<until:time.sleep(.01)
                self.assertTrue((root/'manager').exists() and (root/'vision').exists())
                supervisor.stop_children(children,root)
                events=[json.loads(line) for line in (root/'events').read_text().splitlines()]
                self.assertEqual([e['name'] for e in events],['manager','vision'])
                self.assertFalse(any(e['command_exists'] for e in events))
                self.assertGreaterEqual(events[1]['time']-events[0]['time'],.25)
                self.assertTrue(all(p.poll()==0 for p in children))
            finally:
                for p in children:
                    if p.poll() is None:os.killpg(p.pid,signal.SIGKILL)
                    p.wait(timeout=2)

    def exercise(self,*,until=12,present=lambda t:True,active=False,child_fails=False,step=.1):
        now=[100.];handlers={};launches=[];stops=[];boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        class Child:
            def __init__(self,args,**kwargs):self.args=args;self.pid=10000+len(launches);self.ended=False;launches.append(self)
            def poll(self):return 1 if self.ended or (child_fails and now[0]>=104) else None
        def sleep(value):
            now[0]+=step
            if now[0]>=100+until:handlers[signal.SIGTERM](signal.SIGTERM,None)
        def terminate(children,ipc):
            stops.append(len(children))
            for p in children:p.ended=True
        def report(path):
            if path.name=='vision.json':return dict(monotonic_ns=int(now[0]*1e9),frame_ns=int(now[0]*1e9),link_fresh=True,pose_fresh=True,vision_instance='fake')
            if path.name=='status.json':return dict(boot_id=boot,monotonic_s=now[0],phase='OUTBOUND' if active else 'WAIT_READY')
            return {}
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);nav=root/'navigation';ipc=root/'ipc';ipc.mkdir()
            output=io.StringIO()
            with patch.object(supervisor,'N',nav),patch.object(supervisor,'P',root), \
                 patch.dict(os.environ,RM27_MISSION_DIR=str(ipc)), \
                 patch.object(supervisor.time,'monotonic',side_effect=lambda:now[0]), \
                 patch.object(supervisor.time,'sleep',side_effect=sleep), \
                 patch.object(supervisor.signal,'signal',side_effect=lambda signum,handler:handlers.update({signum:handler})), \
                 patch.object(supervisor,'device_snapshot',side_effect=lambda:dict(ready=present(now[0]-100),serial_token=['fake'],cameras=['fake'],reason='fake missing device')), \
                 patch.object(supervisor,'read_json',side_effect=report), \
                 patch.object(supervisor.subprocess,'Popen',side_effect=Child), \
                 patch.object(supervisor,'stop_children',side_effect=terminate), \
                 patch.object(supervisor,'notify'),contextlib.redirect_stdout(output):
                supervisor.run()
            records=[json.loads(line) for line in output.getvalue().splitlines()]
        return launches,stops,records

    def test_missing_devices_wait_over_an_hour_without_launch_or_exit(self):
        launches,_,events=self.exercise(until=3700,present=lambda _:False,step=60)
        self.assertEqual(len(launches),0)
        self.assertEqual(events[0]['phase'],'WAIT_DEVICES')
        self.assertEqual(events[-1]['phase'],'STOPPED')

    def test_late_camera_and_usb_launch_once_after_stability(self):
        launches,_,events=self.exercise(present=lambda t:t>=2)
        self.assertEqual(len(launches),2)
        self.assertTrue(any(e['phase']=='RUNNING' for e in events))

    def test_idle_disconnect_rebuilds_session(self):
        launches,_,events=self.exercise(present=lambda t:not 4<=t<7)
        self.assertEqual(len(launches),4)
        self.assertTrue(any(e['phase']=='RETRY_PREPARING' for e in events))

    def test_active_child_failure_never_relaunches(self):
        launches,_,events=self.exercise(active=True,child_fails=True)
        self.assertEqual(len(launches),2)
        self.assertTrue(any(e['phase']=='FAULT_LATCHED' for e in events))


if __name__=='__main__':unittest.main()
