"""Extract one outbound taught path in registered map coordinates; never access devices."""
from pathlib import Path
import argparse
import hashlib
import json
import math
import time
import numpy as np
from mission import simplify, length, Path as Polyline
from straighten import fit as fit_straight_course


ROOT=Path(__file__).resolve().parent.parent


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def prepare(poses,registration):
    # A backend may revise the same timestamp. Retain the last revision once.
    records={}
    for line in poses.read_text(encoding='utf-8').splitlines():
        r=json.loads(line)
        if r['frame']!='motion_plane_candidate':raise ValueError('Wrong recorded map frame')
        records[r['stamp_ns']]=r
    rows=[records[s] for s in sorted(records)]
    reg=json.loads(registration.read_text(encoding='utf-8'))
    if not reg['physical_p00_and_heading_confirmed']:raise ValueError('P00 not registered')
    ts=np.array([r['stamp_ns'] for r in rows],dtype=np.int64)
    t=(ts-ts[0])/1e9
    xy=np.array([r['xyz'][:2] for r in rows],dtype=float)
    m=np.array(reg['transform_course_from_map']);xy=xy@m[:2,:2].T+m[:2,2]
    if not np.isfinite(xy).all() or len(rows)<500:raise ValueError('Recorded route invalid or too short')
    still=np.zeros(len(rows),dtype=bool)
    for i in range(len(rows)):
        j=np.searchsorted(t,t[i]+1.)
        if j<len(rows):still[i]=np.linalg.norm(xy[j]-xy[i])/(t[j]-t[i])<.025
    episodes=[];start=None
    for i,flag in enumerate(np.r_[still,False]):
        if flag and start is None:start=i
        if not flag and start is not None:
            if t[i-1]-t[start]>=5.:
                center=np.median(xy[start:i],axis=0)
                if np.linalg.norm(center)>2.:
                    episodes.append(dict(start=start,end=i,xy=center.tolist(),start_s=float(t[start]),end_s=float(t[i-1])))
            start=None
    if not episodes:raise ValueError('No distant task dwell candidate found')
    task=max(episodes,key=lambda e:np.linalg.norm(e['xy']))
    # Require an actual return near P00 after the selected distant stop.
    final=np.median(xy[-100:],axis=0)
    if np.linalg.norm(final)>.4 or t[-1]-task['end_s']<20:
        raise ValueError('No supported return to start after candidate task point')
    source=[[0.,0.]]
    for p in xy[:task['start']]:
        if math.dist(p,source[-1])>=.01:source.append(p.tolist())
    if math.dist(task['xy'],source[-1])>=.001:source.append(task['xy'])
    else:source[-1]=task['xy']
    compressed=simplify(source,.025)
    poly=Polyline(compressed)
    # Verify simplified chords preserve the ordered source spans, not merely
    # distance to an unrelated neighbouring leg of the course.
    boundaries=[];previous=0
    for p in compressed[1:]:
        candidates=[(math.dist(source[i],p),i) for i in range(previous+1,len(source))]
        _,i=min(candidates);boundaries.append((previous,i));previous=i
    max_error=0.
    for k,(lo,hi) in enumerate(boundaries):
        a,b=compressed[k:k+2];dx,dy=b[0]-a[0],b[1]-a[1];den=dx*dx+dy*dy
        for p in source[lo:hi+1]:
            f=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den))
            max_error=max(max_error,math.hypot(p[0]-a[0]-f*dx,p[1]-a[1]-f*dy))
    if max_error>.025000001:raise ValueError('Simplification exceeded taught path bound')
    out,straightening=fit_straight_course(source)
    poly=Polyline(out)
    site=json.loads((ROOT/'map_route/mission_site.json').read_text(encoding='utf-8'))
    confirmed=(site['registration_id']==reg['registration_id'] and site['source_capture']=='mapping_20261025_022124'
               and site['task_point_label_field_confirmed'] is True and math.dist(site['task_goal_m'],out[-1])<.03)
    q=np.array([r['xyzw'] for r in rows]);yaw=np.arctan2(2*(q[:,3]*q[:,2]+q[:,0]*q[:,1]),1-2*(q[:,1]**2+q[:,2]**2))
    yaw=np.unwrap(yaw-math.radians(reg['reference_map_yaw_deg']))
    result=dict(schema='registered_global_round_trip_v1',frame='course_start',created_unix=time.time(),
        outbound_waypoints_m=out,return_waypoints_m=list(reversed(out)),task_goal_m=out[-1],home_goal_m=[0.,0.],
        outbound_length_m=poly.total,round_trip_length_m=2*poly.total,
        speed_ceiling_m_s=1.80,motor_rpm_ceiling=5500.,acceleration_m_s2=1.10,deceleration_m_s2=1.50,jerk_limit_m_s3=6.,corner_speed_m_s=.20,corner_brake_margin_m=.08,
        task_dwell_s=5.,deadline_s=300.,heading_deg=0.,
        registration_id=reg['registration_id'],map_source_signature=reg['map_source_signature'],
        source_pose_path=str(poses.resolve()),source_pose_sha256=sha(poses),registration_sha256=sha(registration),
        source='Actual recorded global-map base trajectory, not old training-course segment lengths',
        source_capture='mapping_20261025_022124',full_traversal_user_confirmed=True,
        user_confirmation='是，到达任务区后返回',
        task_point_inference='Farthest sustained distant stationary interval before recorded return to start',
        task_dwell_candidate=task,all_distant_stationary_candidates=episodes,
        task_point_label_field_confirmed=confirmed,task_point_confirmation=site['user_confirmation'] if confirmed else None,
        route_clearance_surveyed=False,straight_line_planning=straightening,reference_recorded_polyline_m=compressed,
        recorded_start_offset_from_registered_p00_m=float(np.linalg.norm(xy[0])),
        recorded_final_xy_m=final.tolist(),recorded_outbound_yaw_range_deg=[float(np.degrees(yaw[:task['end']].min())),float(np.degrees(yaw[:task['end']].max()))],
        raw_taught_points=len(source),simplified_points=len(out),reference_compression_max_error_m=max_error,
        task_actions='Navigation stop only; no self-aim, friction wheel or firing commands',
        automatic_obstacle_avoidance=False,static_map_collision_certified=False,
        physical_tracking_validated=False,physical_entry_enabled=False,
        physical_entry_block_reason='Newly flashed firmware yaw profile and MCU boot binding need updating; route preview/clearance must be checked before a human start')
    return result,xy,t


def plot(plan,xy,t,destination,cloud_path=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    reg=json.loads((ROOT/'boot/course_registration.json').read_text(encoding='utf-8'));m=np.array(reg['transform_course_from_map'])
    fig,axes=plt.subplots(2,1,figsize=(13,8),sharex=True,sharey=True)
    points=np.array(plan['outbound_waypoints_m']);goal=points[-1]
    background=None
    if cloud_path and cloud_path.exists():
        cloud=np.load(cloud_path)['xyz'];cloud=cloud[(cloud[:,2]>-.025)&(cloud[:,2]<1.2)]
        cloud=cloud[::max(1,len(cloud)//40000)]
        background=cloud[:,:2]@m[:2,:2].T+m[:2,2]
    for ax,title,p,color in [(axes[0],'OUTBOUND: registered P00 to recorded task dwell',points,'#1268ba'),
                              (axes[1],'RETURN: reverse the same path to registered P00',points[::-1],'#d55e00')]:
        if background is not None:ax.scatter(*background.T,s=.6,c='#b4bac2',alpha=.45,rasterized=True)
        ax.plot(*xy.T,color='#a8b1bb',lw=.7,alpha=.4,label='Original recorded motion')
        ax.plot(*p.T,color=color,lw=2.,label='Global path candidate')
        for a,b in zip(p,p[1:]):
            if np.linalg.norm(b-a)>.45:
                mid=(a+b)/2;v=(b-a)/np.linalg.norm(b-a)*.18
                ax.annotate('',xy=mid+v/2,xytext=mid-v/2,arrowprops=dict(arrowstyle='->',color=color,lw=2))
        ax.scatter(0,0,c='#169c53',s=75,zorder=5);ax.text(.08,.13,'P00 (0,0)',weight='bold')
        ax.scatter(*goal,c='#a8238b',marker='*',s=130,zorder=5)
        ax.text(goal[0]-.9,goal[1]+.13,f'Task ({goal[0]:.2f}, {goal[1]:.2f})',weight='bold')
        ax.set_title(title,loc='left');ax.set_ylabel('Left / m');ax.grid(alpha=.25);ax.set_aspect('equal')
        ax.set_xlim(-.5,7.9);ax.set_ylim(-1.9,2.3)
    axes[-1].set_xlabel('Forward from P00 / m')
    axes[0].legend(loc='lower left',fontsize=8)
    fig.suptitle(f"Straight global route candidate: {plan['round_trip_length_m']:.2f} m; task dwell 10 s",weight='bold')
    fig.text(.02,.02,'Task location confirmed by operator. Grey points are context; footprint clearance and new firmware tracking remain unverified.',fontsize=8)
    fig.tight_layout(rect=(0,.04,1,.95));fig.savefig(destination,dpi=170);plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--poses',type=Path,default=ROOT/'work/localization_backend_setup/results/full_capture/poses.jsonl')
    p.add_argument('--registration',type=Path,default=ROOT/'boot/course_registration.json')
    p.add_argument('--out',type=Path,default=ROOT/'map_route/global_plan.json')
    p.add_argument('--plot',type=Path);p.add_argument('--plot-dependencies',type=Path)
    args=p.parse_args();result,xy,t=prepare(args.poses,args.registration)
    args.out.write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    if args.plot:
        if args.plot_dependencies:
            import sys
            sys.path.insert(0,str(args.plot_dependencies))
        plot(result,xy,t,args.plot,ROOT/'work/mapping_20261025_022124/processing/registered_map_motion_plane_candidate.npz')
    print(json.dumps({k:result[k] for k in ['task_goal_m','outbound_length_m','round_trip_length_m','raw_taught_points','simplified_points','straight_line_planning','task_dwell_candidate']},indent=2))
