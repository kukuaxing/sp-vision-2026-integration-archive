#!/usr/bin/env python3
"""Exercise launcher lifecycle in a temporary fake project; never use real hardware."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def wait_for(predicate, process, timeout=12):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        if process.poll() is not None:
            raise AssertionError('launcher exited early')
        time.sleep(0.05)
    raise AssertionError('timed out')


with tempfile.TemporaryDirectory(prefix='rm27-desktop-test-') as td:
    root = Path(td)
    for d in ['tools', 'scripts', 'configs', 'build', 'fakebin']:
        (root / d).mkdir()
    for name in ['field_start_autoaim.sh', 'field_stop_autoaim.sh']:
        shutil.copy2(ROOT / 'tools' / name, root / 'tools' / name)
    shutil.copy2(ROOT / 'scripts/check_deployment.py', root / 'scripts/deployment_protocol_check.py')
    (root / 'configs/standard.yaml').write_text('enemy_color: "blue"\nxuc_allow_control: false\nxuc_allow_shoot: false\nauto_fire: false\nsentinel_use_detect_color: true\n')
    (root / 'scripts/runtime_env.sh').write_text(f'export PATH="{root}/fakebin:/usr/bin:/bin"\n')
    (root / 'scripts/check_deployment.py').write_text('''import json, os, sys
from deployment_protocol_check import protocol_report
assert '--require-runtime' in sys.argv
protocols = protocol_report({'xuc_enable': True, 'board_protocol': os.getenv('BOARD_PROTOCOL', 'SP'), 'xuc_serial_baud': 921600}, {'calibration_pose_source': 'hipnuc'})
errors = ['simulated missing hardware'] if os.getenv('FAIL_CHECK') else protocols['runtime_blockers']
print(json.dumps({'errors': errors, 'hardware': {'serial_path': '/nonexistent-rm27-test-device'}, **protocols}))
sys.exit(1 if errors else 0)
''')
    zenity = root / 'fakebin/zenity'
    zenity.write_text('''#!/usr/bin/python3
import os, sys
from pathlib import Path
with open(os.environ['DIALOG_LOG'], 'a') as f: f.write(repr(sys.argv[1:])+'\\n')
if '--radiolist' in sys.argv:
    if os.getenv('CANCEL_COLOR'): sys.exit(1)
    print(os.getenv('CHOOSE_COLOR', 'red'))
''')
    zenity.chmod(0o755)
    # A distinct compiled process allows the real stop script to verify /proc/PID/exe.
    c = root / 'stub.c'
    c.write_text('''#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
volatile sig_atomic_t running=1;
void stop(int s) { running=0; }
int main(void) {
  signal(SIGINT,stop); signal(SIGTERM,stop);
  if(!getenv("NO_HEARTBEAT")) { puts("[SYNC][XUC] simulated"); fflush(stdout); }
  while(running) usleep(10000);
  return 0;
}
''')
    subprocess.run(['cc', str(c), '-o', str(root / 'build/standard')], check=True)
    env = dict(os.environ, DISPLAY=':fake', DIALOG_LOG=str(root / 'dialogs.log'))
    start = ['bash', str(root / 'tools/field_start_autoaim.sh')]
    stop = ['bash', str(root / 'tools/field_stop_autoaim.sh')]
    state = root / 'logs/field-state/field_autoaim_enemy_color'
    pidfile = root / 'logs/xuc-bringup/field_autoaim.pid'
    runtime = root / 'logs/xuc-bringup/field_autoaim.runtime.yaml'
    dialogs = root / 'dialogs.log'

    def run(args, extra=None):
        return subprocess.run(args, env=dict(env, **(extra or {})), capture_output=True, text=True, timeout=25)

    r = run(start + ['--select-color'], {'FAIL_CHECK': '1'})
    assert r.returncode != 0 and not state.exists() and not runtime.exists()
    assert '--radiolist' not in dialogs.read_text()
    print('PASS: failed preflight does not select/persist a color or launch')
    dialogs.write_text('')
    r = run(start + ['--select-color'], {'BOARD_PROTOCOL': 'RMV'})
    assert r.returncode != 0 and 'RMV' in r.stdout and 'SP' in r.stdout
    assert not state.exists() and not runtime.exists() and not pidfile.exists()
    assert '--radiolist' not in dialogs.read_text()
    assert not (root / 'logs/xuc-bringup/field_autoaim.last_log').exists()
    r = run(start + ['--select-color'], {'BOARD_PROTOCOL': ''})
    assert r.returncode != 0 and not state.exists() and not runtime.exists()
    assert '--radiolist' not in dialogs.read_text()
    print('PASS: RMV/SP mismatch and undeclared protocol stop before color selection and process launch')
    dialogs.write_text('')
    assert run(start + ['--check', '--color', 'red']).returncode == 0
    assert not state.exists() and not runtime.exists() and not dialogs.read_text()
    assert run(start + ['--select-color'], {'CANCEL_COLOR': '1'}).returncode == 0
    assert not state.exists() and not pidfile.exists()
    print('PASS: check-only and cancellation leave saved color unchanged')

    def cycle(args, color, extra=None, duplicate=False):
        log = root / 'launcher.log'
        with log.open('w') as output:
            p = subprocess.Popen(start + args, env=dict(env, **(extra or {})), stdout=output, stderr=subprocess.STDOUT)
            try:
                wait_for(lambda: '[启动成功]' in log.read_text(), p)
                assert state.read_text().strip() == color
                assert f'enemy_color: "{color}"' in runtime.read_text()
                assert 'xuc_allow_control: false' in runtime.read_text()
                assert ('sentinel_use_detect_color: false' if args else 'sentinel_use_detect_color: true') in runtime.read_text()
                if duplicate:
                    r = run(start + ['--color', 'blue'])
                    assert r.returncode != 0 and state.read_text().strip() == color
                assert run(stop).returncode == 0
                assert p.wait(timeout=10) == 0
                assert not pidfile.exists()
            finally:
                if p.poll() is None:
                    run(stop)
                    p.terminate()
                    p.wait(timeout=15)

    dialogs.write_text('')
    cycle(['--select-color'], 'red', {'CHOOSE_COLOR': 'red'}, duplicate=True)
    assert "'TRUE', 'blue'" in dialogs.read_text()
    print('PASS: first default, selected red, exclusive lock, graceful stop')
    dialogs.write_text('')
    assert run(start + ['--select-color'], {'CANCEL_COLOR': '1'}).returncode == 0
    assert state.read_text().strip() == 'red' and "'TRUE', 'red'" in dialogs.read_text()
    cycle(['--select-color'], 'blue', {'CHOOSE_COLOR': 'blue'})
    dialogs.write_text('')
    cycle([], 'blue')
    assert dialogs.read_text() == ''
    print('PASS: remembered defaults, blue selection, unattended restart without dialog')
    cycle(['--color', 'red'], 'red', {'BOARD_PROTOCOL': 'SENTINEL'})
    print('PASS: Sentinel preflight and explicit manual color override')
    r = run(start, {'NO_HEARTBEAT': '1'})
    assert r.returncode != 0 and not pidfile.exists()
    print('PASS: no heartbeat stops child and reports failure')
    unrelated = subprocess.Popen(['sleep', '30'])
    try:
        pidfile.write_text(str(unrelated.pid))
        assert run(stop).returncode != 0 and unrelated.poll() is None
    finally:
        unrelated.terminate()
        unrelated.wait()
    print('PASS: shutdown refuses unrelated process')
print('All desktop lifecycle checks passed; no real camera or serial opened.')
