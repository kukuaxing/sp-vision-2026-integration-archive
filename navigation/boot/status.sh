#!/usr/bin/env bash
set -euo pipefail
/usr/bin/python3 - <<'PY'
from pathlib import Path
import json,time
p=Path('/home/rcsdart/rm27/navigation/logs/boot/status.json')
if not p.exists():raise SystemExit('导航自动启动尚未生成状态。查看：systemctl status rm27-navigation')
r=json.loads(p.read_text());current=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
fresh=r['boot_id']==current and 0<=time.time()-r['updated_unix']<2
ready=fresh and r['localization_ready']
print('地图定位已就绪' if ready else '地图定位未就绪：'+r['phase'])
print('本轮首次就绪用时：'+str(r['first_ready_s'])+' 秒；底盘动作未启用。')
print('输入新鲜度：'+str(r['input_fresh']))
if r.get('pose'):print('地图位置及方向：'+str(r['pose']))
if r.get('course_pose_usable'):print('P00赛道坐标：'+str(r['course_pose']))
elif r.get('course_registration_error'):print('P00配准：'+r['course_registration_error'])
if r.get('last_error'):print('原因：'+r['last_error'])
if not ready:raise SystemExit(2)
PY
