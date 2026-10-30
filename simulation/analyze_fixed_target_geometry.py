#!/usr/bin/env python3
"""Summarize evaluator-only geometry ablations; never imported by robot control."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import cv2
import yaml

ROOT=Path(__file__).resolve().parents[1]


def summary(values):
    x=np.asarray(values,dtype=float)
    if not x.size:return None
    return {'n':int(len(x)),'mean':float(np.mean(x)),'median':float(np.median(x)),
            'p10':float(np.quantile(x,.1)),'p90':float(np.quantile(x,.9)),
            'max_abs':float(np.max(np.abs(x)))}


def analyze(source):
    data=json.loads(source.read_text())
    result={'scope':data['scope'],'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'experiments':[]}
    cfg=yaml.safe_load((ROOT/'configs/standard.yaml').read_text())
    k=np.array(cfg['camera_matrix']).reshape(3,3);dist=np.array(cfg['distort_coeffs'])
    rcam=np.array(cfg['R_camera2gimbal']).reshape(3,3);trans=np.array(cfg['t_camera2gimbal'])
    a=rcam[:,2];b=np.array([3,0,.03]);b=b/np.linalg.norm(b)
    axis=np.cross(a,b);axis=axis/np.linalg.norm(axis)
    rotation=cv2.Rodrigues(axis*np.arccos(np.dot(a,b)))[0]
    obj=np.array([[0,.135/2,.056/2],[0,-.135/2,.056/2],
                  [0,-.135/2,-.056/2],[0,.135/2,-.056/2]],np.float32)
    def recover(p):
        okay,rv,tv=cv2.solvePnP(obj,np.asarray(p,np.float32),k,dist,flags=cv2.SOLVEPNP_IPPE)
        assert okay
        return rotation@(rcam@tv.ravel()+trans)
    for experiment in data['experiments']:
        rows=[r for r in experiment['rows'] if r['within_55deg']]
        valid=[r for r in rows if r['pose_valid']]
        height_ratio=[];width_ratio=[];endpoint_axial=[];corner_rms=[]
        ablations={name:[] for name in ('unchanged','restore_length_only','restore_centers_only','truth')}
        replay_errors=[]
        for r in rows:
            p=np.array(r['measured_corners']);truth=np.array(r['truth_corners'])
            corner_rms.append(np.sqrt(np.mean(np.sum((p-truth)**2,axis=1))))
            for a,b in ((0,3),(1,2)):
                axis=truth[b]-truth[a];length=np.linalg.norm(axis);axis/=length
                height_ratio.append(np.linalg.norm(p[b]-p[a])/length)
                endpoint_axial.extend([np.dot(p[a]-truth[a],axis),-np.dot(p[b]-truth[b],axis)])
            width_ratio.append(np.linalg.norm((p[1]+p[2]-p[0]-p[3])*.5)/
                np.linalg.norm((truth[1]+truth[2]-truth[0]-truth[3])*.5))
            variants={'unchanged':p.copy(),'restore_length_only':p.copy(),
                      'restore_centers_only':p.copy(),'truth':truth.copy()}
            for a,b in ((0,3),(1,2)):
                middle=(p[a]+p[b])*.5;true_middle=(truth[a]+truth[b])*.5
                d=p[b]-p[a];true_length=np.linalg.norm(truth[b]-truth[a]);d/=np.linalg.norm(d)
                variants['restore_length_only'][a]=middle-d*true_length*.5
                variants['restore_length_only'][b]=middle+d*true_length*.5
                variants['restore_centers_only'][a]+=true_middle-middle
                variants['restore_centers_only'][b]+=true_middle-middle
            for name,pp in variants.items():
                ablations[name].append(float(recover(pp)[0]-r['truth_xyz'][0]))
            if r['pose_valid']:
                replay_errors.append(float(np.linalg.norm(recover(p)-r['estimated_xyz'])))
        metrics={'mode':experiment['mode'],'oracle_roi_only':experiment['oracle_roi_only'],
            'matched_measurements':len(rows),'valid_poses':len(valid),
            'position_x_error_m':summary([r['xyz_error'][0] for r in valid]),
            'position_y_error_m':summary([r['xyz_error'][1] for r in valid]),
            'position_z_error_m':summary([r['xyz_error'][2] for r in valid]),
            'lightbar_length_ratio':summary(height_ratio),'lightbar_spacing_ratio':summary(width_ratio),
            'endpoint_inward_shift_px':summary(endpoint_axial),'corner_rms_px':summary(corner_rms),
            'raw_pnp_rms_px':summary([r['raw_rms_px'] for r in valid]),
            'center_x_error_m':summary([r['center'][0]-3 for r in experiment['fits']]),
            'center_y_error_m':summary([r['center'][1] for r in experiment['fits']]),
            'oracle_shadow_center_x_error_m':summary([r['center'][0]-3 for r in experiment['oracle_center_fits']]),
            'independent_python_pnp_replay_error_m':summary(replay_errors),
            'tracker_selected_x_error_m':summary([r['x_error_m'] for r in experiment['tracker_selected']]),
            'tracker_mapping_offsets':{str(offset):sum((r['truth_face']-r['internal_face'])%4==offset
                for r in experiment['tracker_selected']) for offset in range(4)},
            'all_angles_x_error_m':summary([r['xyz_error'][0] for r in experiment['rows'] if r['pose_valid']]),
            'oracle_endpoint_ablations_x_error_m':{k:summary(v) for k,v in ablations.items()},
            'phase_bins':[]}
        for low,high in [(-55,-30),(-30,-10),(-10,10),(10,30),(30,55)]:
            group=[r for r in valid if low<=np.degrees(np.remainder(r['phase_rad']+np.pi,2*np.pi)-np.pi)<high]
            metrics['phase_bins'].append({'degrees':[low,high],
                'x_error_m':summary([r['xyz_error'][0] for r in group])})
        result['experiments'].append(metrics)
    assert {e['mode'] for e in result['experiments']}=={'ideal','refiner_clean','refiner_dark',
        'refiner_exposure','refiner_blur','refiner_degraded','yolo_clean','yolo_degraded'},'partial probe result'
    assert all(e['independent_python_pnp_replay_error_m']['max_abs']<1e-4 for e in result['experiments'])
    ideal=result['experiments'][0]
    assert ideal['position_x_error_m']['max_abs']<2e-5 and ideal['center_x_error_m']['max_abs']<2e-5
    assert all(e['oracle_shadow_center_x_error_m'] and
        e['oracle_shadow_center_x_error_m']['max_abs']<1e-4 for e in result['experiments'])
    files=['configs/standard.yaml','tests/offline/CMakeLists.txt','simulation/src/fixed_target_geometry_probe.cpp',
        'simulation/src/fixed_target_camera.hpp','simulation/src/fixed_target_world.hpp',
        'simulation/src/fixed_target_renderer.hpp','simulation/src/fixed_target_renderer_test.cpp',
        'simulation/analyze_fixed_target_geometry.py','tasks/auto_aim/adaptive_corner_refiner.cpp',
        'tasks/auto_aim/adaptive_corner_refiner.hpp','tasks/auto_aim/solver.cpp','tasks/auto_aim/tracker.cpp',
        'tasks/auto_aim/fixed_center_model.hpp','tasks/auto_aim/yolos/yolo26.cpp',
        'build/native-offline/bin/fixed_target_geometry_probe',cfg['yolo26_model_path']]
    if Path(cfg['yolo26_model_path']).suffix=='.xml':files.append(str(Path(cfg['yolo26_model_path']).with_suffix('.bin')))
    result['sha256']={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in files}
    result['checks']={'complete_eight_ablations':True,'independent_python_pnp_matches_cpp':True,
        'ideal_metric_roundtrip':True,'same_association_truth_shadow_fit_unbiased':True,
        'formal_config_is_original_diagnostic_baseline':result['sha256']['configs/standard.yaml']==
            '0d8b655759f528553a4c2e00baed3dc76e0c8435d11941def133f0411d6ef0d7'}
    # A deliberate production deployment may change YAML later. Record its
    # current hash without insisting the old diagnostic baseline stays forever.
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=ROOT/'logs/fixed-target-geometry.json')
    parser.add_argument('--output',type=Path,default=ROOT/'logs/fixed-target-geometry-analysis.json')
    args=parser.parse_args();result=analyze(args.input)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    for e in result['experiments']:
        print(e['mode'],json.dumps({k:e[k] for k in ('position_x_error_m','center_x_error_m',
            'lightbar_length_ratio','lightbar_spacing_ratio','endpoint_inward_shift_px')}))


if __name__=='__main__':main()
