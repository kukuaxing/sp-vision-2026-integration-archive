#!/usr/bin/env python3
"""Install this checkout's desktop launchers without starting autoaim."""
from pathlib import Path
import os
import subprocess

root = Path(__file__).resolve().parents[1]
desktop = Path(subprocess.check_output(['xdg-user-dir', 'DESKTOP'], text=True).strip())
desktop.mkdir(parents=True, exist_ok=True)
for action, title, script, icon, extra in [
    ('start', '启动自瞄', 'field_start_autoaim.sh', 'camera-photo', ' --select-color'),
    ('stop', '关闭自瞄', 'field_stop_autoaim.sh', 'process-stop', ''),
]:
    target = root / 'tools' / script
    target.chmod(0o755)
    # Desktop Exec escaping is distinct from shell quoting.
    quoted = str(target).replace('\\', '\\\\').replace('"', '\\"').replace('`', '\\`').replace('$', '\\$').replace('%', '%%')
    entry = desktop / f'rm27-autoaim-{action}.desktop'
    entry.write_text(
        '[Desktop Entry]\nType=Application\nVersion=1.0\n'
        f'Name={title}\nComment=rm27 自瞄\n'
        f'Exec=/usr/bin/bash "{quoted}"{extra}\n'
        f'Icon={icon}\nTerminal=true\nStartupNotify=false\nCategories=Utility;\n',
        encoding='utf-8',
    )
    entry.chmod(0o755)
    subprocess.run(['desktop-file-validate', str(entry)], check=True)
    env = os.environ.copy()
    env.setdefault('DBUS_SESSION_BUS_ADDRESS', f'unix:path=/run/user/{os.getuid()}/bus')
    result = subprocess.run(['gio', 'set', str(entry), 'metadata::trusted', 'true'], env=env, capture_output=True, text=True)
    print(str(entry))
    if result.returncode:
        print('Desktop trust metadata unavailable; right-click and choose Allow Launching.')
