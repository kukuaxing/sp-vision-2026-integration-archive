"""Separate navigation cloud with per-scan self filtering. Reads a closed bag only."""
from pathlib import Path
import argparse
import hashlib
import json
import sqlite3
import time
import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def rotation(q):
    x,y,z,w=np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec


def build(root):
    start=time.monotonic();run=root/'data/mapping_20261025_022124';db=run/'bag/bag_0.db3'
    if not (run/'bag/metadata.yaml').exists():raise RuntimeError('Closed bag metadata required')
    level=json.loads((run/'processing/motion_plane_candidate_manifest.json').read_text())
    reg=json.loads((root/'boot/course_registration.json').read_text())
    r=np.array(level['rotation_old_odom_to_candidate']);t=np.array(level['translation_old_odom_to_candidate'])
    m=np.array(reg['transform_course_from_map'])
    conn=sqlite3.connect(db.as_uri()+'?mode=ro',uri=True)
    topics=dict(conn.execute('select name,id from topics'));poses={}
    for data, in conn.execute('select data from messages where topic_id=? order by id',(topics['/navigation/odom'],)):
        msg=deserialize_message(data,Odometry);p=msg.pose.pose.position;q=msg.pose.pose.orientation
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':raise RuntimeError('Base odometry frame mismatch')
        poses[stamp(msg)]=(np.array([p.x,p.y,p.z]),rotation([q.x,q.y,q.z,q.w]))
    arrays=[];raw_arrays=[];total=removed=frames=0
    for data, in conn.execute('select data from messages where topic_id=? order by id',(topics['/lio/cloud_registered'],)):
        msg=deserialize_message(data,PointCloud2)
        if msg.header.frame_id!='odom' or msg.is_bigendian:raise RuntimeError('Unexpected registered cloud layout')
        if stamp(msg) not in poses:raise RuntimeError('A matching actual base pose is required for every cloud')
        pos,rot=poses[stamp(msg)];fields={f.name:f for f in msg.fields}
        if any(fields[k].datatype!=7 for k in ['x','y','z']):raise RuntimeError('Non-float32 cloud')
        dtype=np.dtype(dict(names=['x','y','z'],formats=['<f4']*3,
            offsets=[fields[k].offset for k in ['x','y','z']],itemsize=msg.point_step))
        a=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
        xyz=np.column_stack([a[k].ravel() for k in ['x','y','z']]);xyz=xyz[np.isfinite(xyz).all(axis=1)]
        body=(xyz-pos)@rot
        own=(abs(body[:,0])<=.297)&(abs(body[:,1])<=.305)&(body[:,2]>=-.12)&(body[:,2]<=.65)
        total+=len(xyz);removed+=int(own.sum());frames+=1
        target=xyz@r.T+t
        target[:,:2]=target[:,:2]@m[:2,:2].T+m[:2,2]
        roi=(target[:,0]>-.7)&(target[:,0]<8.1)&(target[:,1]>-2.1)&(target[:,1]<2.6)&(target[:,2]>-.15)&(target[:,2]<1.2)
        arrays.append(target[roi&~own].astype(np.float32));raw_arrays.append(target[roi].astype(np.float32))
    conn.close()
    out=root/'data/global_mission';out.mkdir(exist_ok=True)
    def voxel(arrays):
        xyz=np.concatenate(arrays);cells=np.floor(xyz/.04).astype(np.int32)
        _,i=np.unique(cells,axis=0,return_index=True)
        return xyz[np.sort(i)]
    clean=voxel(arrays);raw=voxel(raw_arrays)
    np.savez_compressed(out/'navigation_cloud.npz',xyz=clean,raw_xyz=raw)
    result=dict(schema='navigation_self_filtered_cloud_v1',frame='course_start',
                source=str(db),source_size_bytes=db.stat().st_size,source_mtime_ns=db.stat().st_mtime_ns,
                registration_id=reg['registration_id'],map_source_signature=reg['map_source_signature'],
                matched_frames=frames,raw_finite_points=total,removed_body_points=removed,
                removed_body_fraction=removed/total,roi_raw_voxels=len(raw),roi_clean_voxels=len(clean),
                self_box_body_m=[[-.297,.297],[-.305,.305],[-.12,.65]],voxel_m=.04,
                ground_reference_measured=False,physical_collision_map_accepted=False,
                localization_prior_modified=False,motion_commands_sent=False,elapsed_s=time.monotonic()-start)
    result['cloud_sha256']=hashlib.sha256((out/'navigation_cloud.npz').read_bytes()).hexdigest()
    (out/'navigation_cloud_manifest.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path('/home/rcsdart/rm27/navigation'))
    build(p.parse_args().root)
