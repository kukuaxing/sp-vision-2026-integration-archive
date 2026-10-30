import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from startup_watchdog import SessionWatchdog,interrupted_route
from policy import StartGate
import supervisor
spec=importlib.util.spec_from_file_location('mission_readiness_status',HERE/'status.py')
status=importlib.util.module_from_spec(spec);spec.loader.exec_module(status)


def vision(now):
    return dict(monotonic_ns=int(now*1e9),frame_ns=int(now*1e9),link_fresh=True,pose_fresh=True,vision_instance='v1')


class StartupRecoveryTests(unittest.TestCase):
    def test_supervisor_crash_retains_same_boot_route_latch(self):
        self.assertTrue(interrupted_route(dict(active=True,boot_id='b'),'b'))
        self.assertFalse(interrupted_route(dict(active=True,boot_id='old'),'b'))
        self.assertFalse(interrupted_route({},'b'))
    def test_device_late_does_not_consume_start_edge(self):
        gate=StartGate()
        for now in (0,60,120,3600):self.assertFalse(gate.update(now,False,True,False))
        self.assertFalse(gate.update(3601,True,True,True))
        self.assertFalse(gate.update(3602,True,False,True))
        self.assertTrue(gate.update(3603,True,True,True))

    def test_preparation_child_failure_retries(self):
        w=SessionWatchdog(10,'boot')
        self.assertEqual(w.inspect(11,{}, {},{},children_ok=False)[0],'retry')

    def test_different_boot_or_old_manager_fault_is_not_reused(self):
        w=SessionWatchdog(10,'boot')
        for report in [dict(boot_id='old',monotonic_s=11,phase='FAULT'),dict(boot_id='boot',monotonic_s=9,phase='FAULT')]:
            self.assertEqual(w.inspect(12,vision(12),report,{})[0],'running')

    def test_current_mission_fault_latches_even_between_polls(self):
        w=SessionWatchdog(10,'boot')
        self.assertEqual(w.inspect(11,{},dict(boot_id='boot',monotonic_s=10.9,phase='FAULT'),{},children_ok=False)[0],'latch')

    def test_active_or_old_sent_command_survives_child_failure(self):
        w=SessionWatchdog(10,'boot')
        cmd=dict(monotonic_ns=10500000000,vision_instance='v1',nav_enabled=True)
        self.assertEqual(w.inspect(12,vision(12),{},cmd,children_ok=False)[0],'latch')

    def test_outbound_loss_latches_but_idle_usb_loss_retries(self):
        w=SessionWatchdog(10,'boot')
        self.assertEqual(w.inspect(11,vision(11),dict(boot_id='boot',monotonic_s=11,phase='OUTBOUND'),{})[0],'running')
        self.assertEqual(w.inspect(12,vision(12),{},{},devices_ok=False)[0],'latch')
        self.assertEqual(SessionWatchdog(10,'boot').inspect(12,vision(12),{},{},devices_ok=False)[0],'retry')

    def test_dead_serial_reader_and_frozen_image_recover_idle(self):
        for key in ('frame_ns','monotonic_ns','link_fresh','pose_fresh'):
            w=SessionWatchdog(10,'boot',grace_s=2,loss_s=1)
            for now,expected in [(10.5,'running'),(11,'running'),(13,'retry')]:
                v=vision(now);v[key]=False if key.endswith('fresh') else 1
                self.assertEqual(w.inspect(now,v,{}, {})[0],expected)

    def test_healthy_streams_wait_for_map_without_restart(self):
        w=SessionWatchdog(10,'boot')
        for now in (11,60,3600):self.assertEqual(w.inspect(now,vision(now),{},{})[0],'running')

    def test_presence_late_symlink_and_camera_enumeration(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'configs').mkdir();usb=root/'sysfs';usb.mkdir()
            port=root/'serial-link';target=root/'ttyUSB1'
            config=dict(board_protocol='SENTINEL',xuc_serial_baud=921600,xuc_serial_port=str(port),vid_pid='2bdf:0001')
            import yaml
            (root/'configs/standard.yaml').write_text(yaml.safe_dump(config))
            self.assertFalse(supervisor.device_snapshot(root,usb)['ready'])
            target.touch();port.symlink_to(target)
            self.assertFalse(supervisor.device_snapshot(root,usb)['ready'])
            cam=usb/'1-3';cam.mkdir();(cam/'idVendor').write_text('2bdf');(cam/'idProduct').write_text('0001')
            first=supervisor.device_snapshot(root,usb);self.assertTrue(first['ready'])
            other=root/'ttyUSB2';other.touch();port.unlink();port.symlink_to(other)
            self.assertNotEqual(first['serial_token'],supervisor.device_snapshot(root,usb)['serial_token'])

    def test_status_does_not_treat_running_as_ready_or_reuse_old_boot(self):
        sup=dict(boot_id='b',monotonic_s=100,phase='RUNNING');nav=dict(boot_id='b',updated_unix=200,localization_ready=True)
        mission=dict(boot_id='b',monotonic_s=100,phase='READY_WAIT_BOTH_UP',ready=True,auto=False)
        with patch.object(status.time,'time',return_value=200):
            self.assertTrue(status.readiness(100,'b',sup,mission,nav)['ready'])
            self.assertFalse(status.readiness(100,'other',sup,mission,nav)['ready'])
            mission['auto']=True;self.assertFalse(status.readiness(100,'b',sup,mission,nav)['ready'])
            self.assertFalse(status.readiness(105,'b',sup,mission,nav)['ready'])


if __name__=='__main__':unittest.main()
