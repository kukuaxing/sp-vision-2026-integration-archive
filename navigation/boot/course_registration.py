"""Persistent planar P00 reference tied to the actual prior map and sensor geometry. No device IO."""
from pathlib import Path
import hashlib,json,math,time

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def geometry(root):
    import yaml
    backend=yaml.safe_load((root/'boot/map_backend.yaml').read_text())['/**']['ros__parameters']
    lio=yaml.safe_load((root/'localization/fastlio.vehicle.yaml').read_text())['/**']['ros__parameters']['mapping']
    mount=json.loads((root/'sentinel_translation/mid360_mount.json').read_text())
    prior=Path(backend['prior_map']['directory']).resolve()
    if not prior.is_relative_to((root/'data').resolve()):raise ValueError('Prior map is outside navigation/data')
    relevant=dict(backend_body=backend['common']['body'],map_frame=backend['common']['map_frame'],
        sensor_frame=backend['common']['sensor_frame'],lidar_imu_rotation=lio['extrinsic_R'],
        lidar_imu_translation=lio['extrinsic_T'],lidar_base_rotation=mount['rotation_lidar_to_base'],
        lidar_base_translation=mount['translation_m'])
    return prior,relevant
def signature(root):
    prior,relevant=geometry(root);h=hashlib.sha256();files=sorted(p for p in prior.rglob('*') if p.is_file())
    if not files:raise ValueError('Empty prior map')
    for p in files:
        h.update(p.relative_to(prior).as_posix().encode()+b'\0');h.update(bytes.fromhex(digest(p)))
    return dict(prior_directory=str(prior),prior_sha256=h.hexdigest(),
        geometry_sha256=hashlib.sha256(json.dumps(relevant,sort_keys=True,separators=(',',':')).encode()).hexdigest())
def wrap_degrees(angle):return (angle+180.)%360.-180.
def transform_matrix(origin,yaw_deg):
    a=math.radians(yaw_deg);c=math.cos(a);s=math.sin(a);x,y=origin
    return [[c,s,-c*x-s*y],[-s,c,s*x-c*y],[0.,0.,1.]]
def project(reg,pose):
    x,y=pose['xyz'][:2];m=reg['transform_course_from_map']
    return dict(frame='course_start',base_frame='base_link',stamp_ns=pose['stamp_ns'],
        xy_m=[m[0][0]*x+m[0][1]*y+m[0][2],m[1][0]*x+m[1][1]*y+m[1][2]],
        yaw_deg=wrap_degrees(pose['yaw_deg']-reg['reference_map_yaw_deg']),registration_id=reg['registration_id'],
        vertical_reference_registered=False)
def validate(reg,current_signature):
    if reg.get('schema')!='p00_map_registration_v1' or reg.get('map_frame')!='motion_plane_candidate':
        raise ValueError('Unsupported course registration schema or map frame')
    if reg.get('physical_p00_and_heading_confirmed') is not True:raise ValueError('Physical reference not confirmed')
    if reg.get('map_source_signature')!=current_signature:raise ValueError('Map or sensor geometry changed')
    o=reg['reference_map_xy_m'];y=reg['reference_map_yaw_deg'];m=reg['transform_course_from_map']
    if len(o)!=2 or not all(math.isfinite(v) for v in [*o,y]):raise ValueError('Invalid registration values')
    expected=transform_matrix(o,y)
    if len(m)!=3 or any(len(row)!=3 for row in m):raise ValueError('Invalid matrix shape')
    if any(not math.isfinite(m[i][j]) or abs(m[i][j]-expected[i][j])>1e-10 for i in range(3) for j in range(3)):
        raise ValueError('Registration matrix inconsistent with reference')
    return reg

class RegistrationStore:
    """Refresh hashes only when watched files change; no dependence on LIO startup origin."""
    def __init__(self,root):self.root=root;self.reg=None;self.error='not_registered';self.checked=-1.;self.watch=None
    def update(self):
        now=time.monotonic()
        if now-self.checked<1:return
        self.checked=now;path=self.root/'boot/course_registration.json'
        if not path.exists():self.reg=None;self.error='not_registered';return
        try:
            prior,_=geometry(self.root)
            paths=[path,self.root/'boot/map_backend.yaml',self.root/'localization/fastlio.vehicle.yaml',
                self.root/'sentinel_translation/mid360_mount.json',*sorted(p for p in prior.rglob('*') if p.is_file())]
            watch=tuple((str(p),p.stat().st_mtime_ns,p.stat().st_size) for p in paths)
            if watch!=self.watch:
                self.reg=validate(json.loads(path.read_text()),signature(self.root));self.watch=watch
            self.error=None
        except (OSError,ValueError,KeyError,TypeError) as e:self.reg=None;self.error=str(e);self.watch=None
    def snapshot(self,pose):
        self.update()
        return dict(course_reference_valid=self.reg is not None,course_registration_error=self.error,
            course_pose=project(self.reg,pose) if self.reg and pose else None)
