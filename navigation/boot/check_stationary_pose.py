"""Read the running localization monitor for a short stationary pose check. No ROS/device commands."""
from pathlib import Path
import json,math,time,statistics
from course_registration import RegistrationStore

N=Path('/home/rcsdart/rm27/navigation');status=N/'logs/boot/status.json'
boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
samples=[];instance=None;began=None;limit=time.monotonic()+20;last_reason='no status'
print('保持 STOP 和静止，正在读取实时地图定位；不发送动作。',flush=True)
while time.monotonic()<limit:
    try:r=json.loads(status.read_text())
    except (FileNotFoundError,json.JSONDecodeError):time.sleep(.1);continue
    now=time.time();p=r.get('pose');last_reason=r.get('phase','unknown')
    good=(r.get('boot_id')==boot and 0<=now-r['updated_unix']<1 and r.get('localization_ready') and
        r.get('map_state')=='NORMAL' and r.get('pose_trusted')=='true' and
        all(r.get('input_fresh',{}).get(k) is True for k in ['odom','cloud','base']) and
        p is not None and -.1<now-p['stamp_ns']/1e9<.75)
    if not good:
        samples=[];instance=None;began=None;time.sleep(.05);continue
    if instance is not None and instance!=r['instance']:raise SystemExit('定位进程在核对期间重启，请等待定位就绪后重新核对。')
    instance=r['instance']
    if not samples or p['stamp_ns']>samples[-1]['stamp_ns']:
        samples.append(p)
        if began is None:began=time.monotonic()
    if began is not None and time.monotonic()-began>=1.0 and len(samples)>=4:break
    time.sleep(.05)
else:raise SystemExit('实时定位未就绪：'+last_reason+'；查看 bash ~/rm27/navigation/boot/status.sh')
columns=list(zip(*(p['xyz'] for p in samples)));ranges=[max(c)-min(c) for c in columns]
yaw=[p['yaw_deg'] for p in samples]
for i in range(1,len(yaw)):
    while yaw[i]-yaw[i-1]>180:yaw[i]-=360
    while yaw[i]-yaw[i-1]<-180:yaw[i]+=360
if math.sqrt(sum(v*v for v in ranges))>=.03 or max(yaw)-min(yaw)>=math.degrees(.03):
    raise SystemExit('核对期间估计位姿变化过大，请保持静止后重试。')
out=N/'logs'/time.strftime('live_map_check_%Y%m%d_%H%M%S');out.mkdir(exist_ok=False)
result=dict(mode='live_quick',map_frame='motion_plane_candidate',body_frame='base_link',
    median_body_xyz_m=[statistics.median(c) for c in columns],median_body_yaw_deg=statistics.median(yaw),
    position_range_m=ranges,yaw_range_deg=max(yaw)-min(yaw),samples=len(samples),
    observe_s=time.monotonic()-began,boot_id=boot,localization_instance=instance,
    physical_reference_label_unconfirmed=True,navigation_ready=False,motion_commands_sent=False)
result.update(RegistrationStore(N).snapshot(dict(xyz=result['median_body_xyz_m'],
    yaw_deg=result['median_body_yaw_deg'],stamp_ns=samples[-1]['stamp_ns'])))
(out/'stationary_pose.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2));print('实时地图位姿已记录：'+str(out/'stationary_pose.json'))
