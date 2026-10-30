#!/usr/bin/env python3
"""Independent rigid-transform scoring; does not import the candidate fitter."""
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import cv2
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]


def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def statistics(values):
    a=np.asarray(values,float);a=a[np.isfinite(a)]
    if not len(a): return None
    return dict(n=len(a),mean=float(a.mean()),rmse=float(np.sqrt(np.mean(a*a))),
        median_abs=float(np.median(abs(a))),p95_abs=float(np.quantile(abs(a),.95)),max_abs=float(max(abs(a))))


def audit():
    report=json.loads((ROOT/'logs/joint-pose-geometry.json').read_text())
    source=ROOT/'logs/fixed-target-geometry.json'
    assert digest(source)==report['historical_sha256']
    assert all(digest(ROOT/p)==v for p,v in report['sha256'].items())
    assert report['sha256']['configs/standard.yaml']=='e087e02c2c6ad257acef5a0808950a42f15445c4ed00714807023dad7427fd90'
    assert report['sha256']['build/standard']=='dbc3d740d18255383af4f211b9bb8b740bbee0c65d31b0d18c8a757f378d5cfe'
    cfg=yaml.safe_load((ROOT/'configs/standard.yaml').read_text())
    assert cfg['corner_refine_brightness_invariant_subpixel'] is False
    k=np.array(cfg['camera_matrix']).reshape(3,3);dist=np.array(cfg['distort_coeffs'])
    rc=np.array(cfg['R_camera2gimbal']).reshape(3,3);tc=np.array(cfg['t_camera2gimbal'])
    a=rc[:,2];b=np.array([3.,0.,.03]);b/=np.linalg.norm(b)
    v=np.cross(a,b);c=float(a@b)
    skew=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    rw=np.eye(3)+skew+skew@skew/(1+c)
    obj=np.array([[0,.0675,.028],[0,-.0675,.028],[0,-.0675,-.028],[0,.0675,-.028]])
    def project(xyz,yaw,pitch=math.radians(15)):
        # Independent full extrinsic route, rather than transforming each point.
        rz=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
        ry=np.array([[math.cos(pitch),0,math.sin(pitch)],[0,1,0],[-math.sin(pitch),0,math.cos(pitch)]])
        rv=cv2.Rodrigues(rc.T@rw.T@rz@ry)[0]
        tv=rc.T@(rw.T@np.asarray(xyz)-tc)
        return cv2.projectPoints(obj,rv,tv,k,dist)[0].reshape(4,2)
    checked=0;max_difference=0.
    def check(row,pixels,truth,yaw):
        nonlocal checked,max_difference
        xyz=np.array(row['xyz']);angle=row['yaw']
        residual=project(xyz,angle)-pixels
        values=dict(x_m=xyz[0]-truth[0],position_m=np.linalg.norm(xyz-truth),
            bearing_deg=np.degrees((np.arctan2(xyz[1],xyz[0])-np.arctan2(truth[1],truth[0])+np.pi)%(2*np.pi)-np.pi),
            yaw_deg=np.degrees((angle-yaw+np.pi)%(2*np.pi)-np.pi),
            model_rms_px=np.sqrt(np.mean(np.sum(residual**2,axis=1))))
        for key,value in values.items():
            delta=abs(value-row['joint'][key]);max_difference=max(max_difference,float(delta))
            assert delta<1e-8,(key,delta)
        assert values['model_rms_px']<=row['baseline']['model_rms_px']+1e-8
        checked+=1
    rng=np.random.default_rng(120031)
    modes=('ideal','iid_025px','bars_92pct','pitch_10deg','pitch_20deg')
    assert set(report['controlled'])==set(modes)
    for cell,(distance,angle) in enumerate((d,a) for d in (2.,3.,4.) for a in (-50.,-25.,0.,25.,50.)):
        truth=np.array([distance,.07,.03]);yaw=np.radians(angle)
        for draw in range(8):
            noise=rng.normal(0,.25,(4,2))
            for mode in modes:
                pitch=np.radians(10 if mode=='pitch_10deg' else 20 if mode=='pitch_20deg' else 15)
                pixels=project(truth,yaw,pitch)
                if mode=='bars_92pct':
                    for first,last in ((0,3),(1,2)):
                        center=(pixels[first]+pixels[last])/2
                        pixels[[first,last]]=center+.92*(pixels[[first,last]]-center)
                if mode!='ideal':pixels+=noise
                row=report['controlled'][mode]['rows'][cell*8+draw]
                assert (row['distance'],row['angle_deg'],row['draw'])==(distance,angle,draw)
                check(row,pixels,truth,yaw)
    previous=json.loads(source.read_text())
    assert len(previous['experiments'])==len(report['historical'])==8
    for experiment in previous['experiments']:
        valid=[r for r in experiment['rows'] if r['pose_valid'] and r['within_55deg']]
        results=report['historical'][experiment['mode']]['rows']
        assert len(valid)==len(results)
        for original,row in zip(valid,results):
            assert (original['frame'],original['truth_face'],original['t'])==(row['frame'],row['truth_face'],row['t'])
            check(row,np.array(original['measured_corners']),np.array(original['truth_xyz']),original['phase_rad'])
    for group in list(report['controlled'].values())+list(report['historical'].values()):
        rows=group['rows'];summary=group['summary'];assert summary['n']==len(rows)
        assert summary['changed']==sum(r['changed'] for r in rows)
        for method in ('raw','baseline','joint'):
            for key in rows[0][method]:
                values=statistics([r[method][key] for r in rows])
                for name,value in values.items(): assert abs(value-summary['errors'][method][key][name])<1e-10
    tests=subprocess.run([sys.executable,str(ROOT/'simulation/test_joint_pose_geometry.py')],capture_output=True,text=True)
    assert tests.returncode==0,tests.stderr
    log=(ROOT/'logs/joint-pose-ctest.log').read_text()
    match=re.search(r'100% tests passed, 0 tests failed out of (\d+)',log)
    assert match,'full CTest did not pass'
    hygiene=subprocess.run([sys.executable,str(ROOT/'scripts/check_source_tree.py')],cwd=ROOT,capture_output=True,text=True)
    assert hygiene.returncode==0,hygiene.stdout+hygiene.stderr
    result=dict(scope='independent file-only geometry scoring, no physical devices started',
        checked_joint_poses=checked,controlled_poses=600,historical_poses=checked-600,
        max_independent_metric_difference=max_difference,aggregate_recomputation=True,
        candidate_unit_tests=7,ctest_passed=int(match[1]),
        formal_config_and_binary_unchanged=True,endpoint_candidate_enabled=False,
        source_tree_check=hygiene.stdout.strip(),decision='do_not_deploy_joint_pose',
        report_sha256=digest(ROOT/'logs/joint-pose-geometry.json'),
        sha256={p:digest(ROOT/p) for p in ('simulation/audit_joint_pose.py','simulation/test_joint_pose_geometry.py')})
    (ROOT/'logs/joint-pose-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':audit()
