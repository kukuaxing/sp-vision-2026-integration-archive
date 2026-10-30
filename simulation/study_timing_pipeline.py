#!/usr/bin/env python3
"""Predeclared clock/pose fault matrix; file-only helpers, not full standard/MCU."""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor,as_completed
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile
from run_fixed_target import ROOT


def interval(**kw):return dict(start_s=12.,end_s=14.,**kw)


CASES={
    'clean':[],
    'image_40ms':[interval(image_delay_s=.040)],
    'image_120ms':[interval(image_delay_s=.120)],
    'future_stamp_40ms':[interval(image_stamp_bias_s=.040)],
    'back_stamp_80ms':[interval(image_stamp_bias_s=-.080)],
    'duplicate_stamp':[dict(start_s=t,end_s=t+.02,repeat_image_stamp=True) for t in (12.,20.,28.)],
    'out_of_order':[dict(start_s=t,end_s=t+.02,image_stamp_bias_s=-.060) for t in (12.,20.,28.)],
    'image_loss_80ms':[dict(start_s=12.,end_s=12.08,drop_images=True)],
    'imu_loss_220ms':[dict(start_s=12.,end_s=12.22,drop_imu=True)],
    'imu_frozen_220ms':[dict(start_s=12.,end_s=12.22,freeze_imu_clock=True)],
    'imu_transport_burst_70ms':[dict(start_s=12.,end_s=12.3,imu_transport_delay_s=.070)],
    'reference_during_wait':[dict(start_s=11.9,end_s=12.1,post_wait_s=.040),
        dict(start_s=12.015,end_s=12.025,new_generation=True)],
    'post_wait_120ms':[interval(post_wait_s=.120)],
    'hidden_sensor_30ms':[dict(start_s=0.,end_s=60.,sensor_latency_s=.030)],
    'constant_transport_25ms':[dict(start_s=0.,end_s=60.,imu_transport_delay_s=.025)],
    'combined':[interval(image_delay_s=.040),dict(start_s=12.1,end_s=12.32,freeze_imu_clock=True),
        dict(start_s=12.2,end_s=12.21,new_generation=True)],
    'qualified_reference_change':[interval(qualified_reference_change=True)],
    'qualified_image_expiry':[interval(qualified_image_expiry=True)],
}


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images',action='store_true');args=parser.parse_args()
    tag='images' if args.images else 'keypoints'
    base=json.loads((ROOT/'simulation/fixed_target.json').read_text())
    base['sampling'].update(seeds=[141001],seed=141001,capture_intervals_s=[.020],
        processing_age_s=.00836,keypoint_noise_px=.25,frame_dropout_probability=0.,
        frame_age_jitter_s=0.,timestamp_bias_s=0.,pose_yaw_bias_deg=0.,pose_pitch_bias_deg=0.)
    base['target']['initial_phases_rad']=[math.radians(13.)]
    base['plant'].update(muzzle_delay_s=.340,muzzle_delay_jitter_s=.025,speed_stddev_mps=.3,
        yaw_spread_stddev_deg=.12,pitch_spread_stddev_deg=.2,barrel_yaw_bias_deg=.1,
        barrel_pitch_bias_deg=.08,servo_time_constant_s=.06)
    base['controller']=dict(apply_visual_residual_correction=True,center_crossing_schedule=False,
        prediction_time_offset_s=0.,directional_prediction_offsets_s={'-1':0.,'1':0.})
    if args.images:
        base['observation']=dict(mode='rendered_yolo26',brightness=.65,exposure_s=.006,exposure_samples=5,
            lens_blur_sigma_px=.6,sensor_noise_stddev=2.)
    else:base['observation']=dict(mode='keypoints')
    selected=('clean','image_120ms','qualified_reference_change','qualified_image_expiry','back_stamp_80ms') if args.images else tuple(CASES)
    executable=ROOT/'build/native-offline/bin'/('fixed_target_image_sim' if args.images else 'fixed_target_sim')
    result=dict(scope='new diagnostic fault matrix; actual PoseHistory + geometry/tracker/prediction/TX guard helper; not complete standard/UART/MCU',
        physical_device_opened=False,controller_tuned=False,kind=tag,complete=False,
        declared_cases=CASES,seed=141001,phase_deg=13.,experiments=[])
    names=['configs/standard.yaml','build/standard','simulation/study_timing_pipeline.py',
        'simulation/src/fixed_target_pipeline.hpp','simulation/src/fixed_target_pipeline_test.cpp',
        'simulation/src/fixed_target_sim.cpp','io/sentinel/pose_history.hpp','tasks/auto_aim/fire_pose_guard.hpp',
        str(executable.relative_to(ROOT))]
    result['sha256']={n:digest(ROOT/n) for n in names}
    def one(name):
        scenario=copy.deepcopy(base);scenario['timing_pipeline']=dict(faults=CASES[name])
        if args.images:scenario['observation']['preview_directory']='logs/fixed-target-images/timing-pipeline/'+name
        raw=ROOT/'logs'/('timing-pipeline-'+tag+'-'+name+'.json')
        with tempfile.TemporaryDirectory(prefix='rm-timing-pipeline-') as temp:
            source=Path(temp)/'scenario.json';source.write_text(json.dumps(scenario))
            with (ROOT/'logs'/('timing-pipeline-'+tag+'-'+name+'.log')).open('w') as log:
                subprocess.run([str(executable),str(ROOT/'configs/standard.yaml'),str(source),str(raw)],
                    cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=1200)
        data=json.loads(raw.read_text());assert len(data['runs'])==2
        runs=[]
        for r in data['runs']:
            events=r['timing_pipeline']['events'];assert events
            runs.append(dict(direction=r['direction'],issued=r['issued'],hits=r['valid_hits'],
                hp=r['hp'],first_issue_s=r['first_issue_s'],last_issue_s=r['shots'][-1]['issue_s'] if r['shots'] else None,
                rejected_capture_frames=sum(not e['capture_valid'] for e in events),
                final_rejected_frames=sum(e['prior_gate'] and not e['final_allowed'] for e in events),
                resets=sum(e['reset'] for e in events),events=len(events)))
        return dict(name=name,raw=str(raw.relative_to(ROOT)),sha256=digest(raw),runs=runs)
    finished={}
    with ThreadPoolExecutor(max_workers=2 if args.images else 1) as pool:
        futures={pool.submit(one,name):name for name in selected}
        for future in as_completed(futures):
            entry=future.result();finished[entry['name']]=entry
            result['experiments']=[finished[n] for n in selected if n in finished]
            (ROOT/'logs'/('timing-pipeline-'+tag+'.json')).write_text(json.dumps(result,indent=2)+'\n')
            print(entry['name'],json.dumps(entry['runs']),flush=True)
    result['complete']=True
    (ROOT/'logs'/('timing-pipeline-'+tag+'.json')).write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
