"""Register a human-confirmed P00/first-leg reference from a completed static check. No motion IO."""
from pathlib import Path
import argparse,json,math,time,uuid
from course_registration import signature,transform_matrix,validate,project,digest

p=argparse.ArgumentParser();p.add_argument('--sample',type=Path,required=True)
p.add_argument('--operator-confirmation',required=True);a=p.parse_args()
n=Path('/home/rcsdart/rm27/navigation');sample=a.sample.resolve()
if not sample.is_relative_to((n/'logs').resolve()):raise SystemExit('Reference sample must be in navigation/logs')
source=json.loads(sample.read_text());dest=n/'boot/course_registration.json'
if dest.exists():raise SystemExit('A P00 registration already exists; inspect it before replacing the reference')
if source['mode']!='live_quick' or source['map_frame']!='motion_plane_candidate' or source['body_frame']!='base_link':
    raise SystemExit('Unsupported stationary sample')
if source['samples']<4 or source['observe_s']<1:raise SystemExit('Insufficient stationary samples')
if math.sqrt(sum(v*v for v in source['position_range_m']))>=.03 or source['yaw_range_deg']>=math.degrees(.03):
    raise SystemExit('Stationary check did not pass')
origin=source['median_body_xyz_m'][:2];yaw=source['median_body_yaw_deg']
reg=dict(schema='p00_map_registration_v1',registration_id=uuid.uuid4().hex,created_unix=time.time(),
    map_frame='motion_plane_candidate',course_frame='course_start',body_frame='base_link',
    course_definition='P00=(0,0); +X along the first-leg centreline toward P01; +Y left',
    reference_map_xy_m=origin,reference_map_yaw_deg=yaw,transform_course_from_map=transform_matrix(origin,yaw),
    map_source_signature=signature(n),source_sample=str(sample),source_sample_sha256=digest(sample),
    physical_p00_and_heading_confirmed=True,confirmation_method='direct_user_chat',operator_confirmation=a.operator_confirmation,
    source_boot_id=source['boot_id'],source_localization_instance=source['localization_instance'],
    geometric_accuracy_surveyed=False,full_route_geometry_verified=False,motion_authorized=False,
    vertical_reference_registered=False,requires_same_map_and_sensor_geometry=True)
validate(reg,signature(n));check=project(reg,dict(xyz=source['median_body_xyz_m'],yaw_deg=yaw,stamp_ns=0))
assert max(abs(v) for v in check['xy_m'])<1e-12 and abs(check['yaw_deg'])<1e-12
tmp=dest.with_suffix('.tmp');tmp.write_text(json.dumps(reg,indent=2));tmp.replace(dest)
print(json.dumps(reg,indent=2));print('P00参照已保存。此操作不授权运动。')
