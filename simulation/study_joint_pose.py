#!/usr/bin/env python3
"""Diagnose joint xyz/yaw fitting. File-only geometry, not a hit-rate trial."""
import argparse
import hashlib
import json
import time
from pathlib import Path
import numpy as np
import yaml
from joint_pose_geometry import Geometry, wrap

ROOT = Path(__file__).resolve().parents[1]


def stats(values):
    v = np.asarray(values, float)
    v = v[np.isfinite(v)]
    if not len(v): return None
    return dict(n=len(v), mean=float(v.mean()), rmse=float(np.sqrt(np.mean(v*v))),
                median_abs=float(np.median(np.abs(v))), p95_abs=float(np.quantile(np.abs(v), .95)),
                max_abs=float(np.max(np.abs(v))))


def measure(g, corners, truth_xyz, truth_yaw, baseline=None):
    raw = g.ippe(corners)
    baseline = g.yaw_only(corners, raw) if baseline is None else np.asarray(baseline, float)
    begin = time.perf_counter()
    fit = g.fit(corners, baseline)
    elapsed = (time.perf_counter()-begin)*1000
    def error(p):
        return dict(x_m=float(p[0]-truth_xyz[0]), position_m=float(np.linalg.norm(p[:3]-truth_xyz)),
                    bearing_deg=float(np.degrees(wrap(np.arctan2(p[1],p[0])-np.arctan2(truth_xyz[1],truth_xyz[0])))),
                    yaw_deg=float(np.degrees(wrap(p[3]-truth_yaw))),
                    model_rms_px=float(np.sqrt(np.mean(np.sum((g.project(p)-corners)**2, axis=1)))))
    return dict(raw=error(raw), baseline=error(baseline), joint=error(fit.pose),
                changed=fit.accepted, yaw_noise_sigma_deg=fit.yaw_sigma_deg if np.isfinite(fit.yaw_sigma_deg) else None,
                iterations=fit.iterations, elapsed_ms=elapsed, xyz=fit.pose[:3].tolist(), yaw=float(fit.pose[3]))


def summarize(rows):
    return dict(n=len(rows), changed=sum(r['changed'] for r in rows),
        errors={method:{key:stats([r[method][key] for r in rows]) for key in rows[0][method]}
                for method in ('raw','baseline','joint')},
        elapsed_ms=stats([r['elapsed_ms'] for r in rows]),
        yaw_noise_sigma_deg=stats([r['yaw_noise_sigma_deg'] for r in rows if r['yaw_noise_sigma_deg'] is not None]))


def run(config, source):
    g = Geometry.probe_fixture(config)
    # Fixed before evaluation: 15 geometric cells x 8 paired draws; no threshold search.
    rng = np.random.default_rng(120031)
    groups={k:[] for k in ('ideal','iid_025px','bars_92pct','pitch_10deg','pitch_20deg')}
    for distance in (2.,3.,4.):
        for degrees in (-50.,-25.,0.,25.,50.):
            truth=np.array([distance,.07,.03,np.radians(degrees)])
            for draw in range(8):
                noise = rng.normal(0,.25,(4,2))
                for mode in groups:
                    pitch = np.radians(10 if mode=='pitch_10deg' else 20 if mode=='pitch_20deg' else 15)
                    corners=g.project(truth,pitch)
                    if mode=='bars_92pct':
                        for a,b in ((0,3),(1,2)):
                            center=(corners[a]+corners[b])/2
                            corners[[a,b]]=center+(corners[[a,b]]-center)*.92
                    if mode!='ideal': corners += noise
                    result=measure(g,corners,truth[:3],truth[3])
                    result.update(distance=distance,angle_deg=degrees,draw=draw)
                    groups[mode].append(result)
    old=json.loads(source.read_text())
    historical={}
    max_replay=0.
    for experiment in old['experiments']:
        rows=[]
        for r in experiment['rows']:
            if not (r['pose_valid'] and r['within_55deg']): continue
            p=np.asarray(r['measured_corners'],float)
            raw=g.ippe(p)
            max_replay=max(max_replay,float(np.linalg.norm(raw[:3]-r['estimated_xyz'])))
            base=np.r_[r['estimated_xyz'],r['yaw_optimized']]
            result=measure(g,p,np.array(r['truth_xyz']),r['phase_rad'],base)
            result.update(frame=r['frame'],t=r['t'],truth_face=r['truth_face'])
            rows.append(result)
        historical[experiment['mode']]=dict(summary=summarize(rows),rows=rows)
        print(experiment['mode'],json.dumps(historical[experiment['mode']]['summary']),flush=True)
    if max_replay>=1e-4: raise AssertionError('Python IPPE did not reproduce production translation')
    controlled={k:dict(summary=summarize(v), rows=v) for k,v in groups.items()}
    for mode,group in controlled.items(): print(mode,json.dumps(group['summary']),flush=True)
    if controlled['ideal']['summary']['errors']['joint']['position_m']['max_abs']>=1e-4:
        raise AssertionError('ideal constrained projection did not round trip')
    # All rows must decrease their own measured-corner objective, not truth error.
    for group in list(controlled.values())+list(historical.values()):
        assert all(r['joint']['model_rms_px']<=r['baseline']['model_rms_px']+1e-8 for r in group['rows'])
    paths=['simulation/joint_pose_geometry.py','simulation/study_joint_pose.py','configs/standard.yaml',
           'tasks/auto_aim/solver.cpp','tasks/auto_aim/solver.hpp','build/standard']
    hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths}
    return dict(scope='offline geometry diagnosis only; historical rows reused, no new hit-rate validation',
        seed=120031,assumed_pixel_sigma=.25,assumed_pitch_deg=15,
        historical_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),sha256=hashes,
        production_ippe_replay_max_m=max_replay,controlled=controlled,historical=historical,
        decision='experimental_only_no_production_change')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=ROOT/'logs/fixed-target-geometry.json')
    parser.add_argument('--output',type=Path,default=ROOT/'logs/joint-pose-geometry.json')
    args=parser.parse_args()
    config=yaml.safe_load((ROOT/'configs/standard.yaml').read_text())
    result=run(config,args.input)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')


if __name__=='__main__': main()
