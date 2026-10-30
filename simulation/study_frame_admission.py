#!/usr/bin/env python3
"""Distinguish exact old-frame replays from new-frame timestamp faults."""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor,as_completed
import json
import math
from pathlib import Path
import subprocess
import tempfile
from run_fixed_target import ROOT,digest


def pulses(**values):return [dict(start_s=t,end_s=t+.02,**values) for t in (12.,20.,28.)]


CASES={
 'clean':[],
 'exact_duplicate':pulses(replay_packet_lag_frames=1),
 'exact_reorder':pulses(replay_packet_lag_frames=3),
 'short_replay_burst':[dict(start_s=12.,end_s=12.06,replay_packet_burst=True)],
 'long_replay_burst':[dict(start_s=12.,end_s=12.24,replay_packet_burst=True)],
 'new_frame_duplicate_stamp':pulses(repeat_image_stamp=True),
 'new_frame_back_stamp':pulses(image_stamp_bias_s=-.060),
 'replay_reference_change':[dict(start_s=12.,end_s=12.06,replay_packet_burst=True),
    dict(start_s=12.015,end_s=12.025,new_generation=True)],
 'qualified_reference_change':[dict(start_s=12.,end_s=14.,qualified_reference_change=True)],
 'qualified_image_expiry':[dict(start_s=12.,end_s=14.,qualified_image_expiry=True)],
}
SPLITS={'development':(151001,21.),'holdout':(151007,53.)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage',choices=SPLITS,required=True);args=parser.parse_args()
    images=args.stage=='holdout';seed,phase=SPLITS[args.stage]
    # All condition names and the exact-pair criterion are fixed before results.
    selected=('clean','exact_duplicate','exact_reorder','new_frame_duplicate_stamp') if images else tuple(CASES)
    prior=json.loads((ROOT/'logs/timing-pipeline-keypoints-clean.json').read_text())
    base=copy.deepcopy(prior['scenario']);base['sampling'].update(seeds=[seed],seed=seed)
    base['target']['initial_phases_rad']=[math.radians(phase)]
    if images:base['observation']=dict(mode='rendered_yolo26',brightness=.65,exposure_s=.006,exposure_samples=5,lens_blur_sigma_px=.6,sensor_noise_stddev=2.)
    exe=ROOT/'build/native-offline/bin'/('fixed_target_image_sim' if images else 'fixed_target_sim')
    files=['configs/standard.yaml','build/standard','tasks/auto_aim/frame_admission.hpp',
        'tests/offline/frame_admission_test.cpp','simulation/src/fixed_target_sim.cpp',
        'simulation/src/fixed_target_pipeline.hpp','simulation/study_frame_admission.py',str(exe.relative_to(ROOT))]
    result=dict(stage=args.stage,scope='new exact metadata replay pairs; timestamp-only faults remain distinct; no devices',
        complete=False,physical_device_opened=False,cases=CASES,splits=SPLITS,
        selected_before_results=True,sha256={f:digest(ROOT/f) for f in files},experiments=[])
    def one(name,enabled):
        scenario=copy.deepcopy(base);scenario['timing_pipeline']=dict(faults=CASES[name])
        scenario['controller']['reject_verified_replay']=enabled
        label=name+('-candidate' if enabled else '-baseline')
        raw=ROOT/'logs'/('frame-admission-'+args.stage+'-'+label+'.json')
        if images:scenario['observation']['preview_directory']='logs/fixed-target-images/frame-admission/'+args.stage+'/'+label
        with tempfile.TemporaryDirectory(prefix='rm-frame-admission-') as tmp:
            path=Path(tmp)/'scenario.json';path.write_text(json.dumps(scenario))
            with (ROOT/'logs'/('frame-admission-'+args.stage+'-'+label+'.log')).open('w') as log:
                subprocess.run([str(exe),str(ROOT/'configs/standard.yaml'),str(path),str(raw)],cwd=ROOT,
                    stdout=log,stderr=subprocess.STDOUT,check=True,timeout=1200)
        data=json.loads(raw.read_text());summary=[]
        for r in data['runs']:
            skipped=[e for e in r['timing_pipeline']['admission_events'] if e['rejected']]
            summary.append(dict(direction=r['direction'],issued=r['issued'],hits=r['valid_hits'],hp=r['hp'],
                last_issue_s=r['shots'][-1]['issue_s'] if r['shots'] else None,skipped=len(skipped),
                processed_control_frames=len(r['timing_pipeline']['events'])))
        return dict(name=name,candidate=enabled,raw=str(raw.relative_to(ROOT)),sha256=digest(raw),runs=summary)
    keys=[(name,enabled) for name in selected for enabled in (False,True)];finished={}
    with ThreadPoolExecutor(max_workers=2) as pool:
        futures={pool.submit(one,*key):key for key in keys}
        for future in as_completed(futures):
            key=futures[future];entry=future.result();finished[key]=entry
            result['experiments']=[finished[k] for k in keys if k in finished]
            (ROOT/'logs'/('frame-admission-'+args.stage+'.json')).write_text(json.dumps(result,indent=2)+'\n')
            print(entry['name'],entry['candidate'],json.dumps(entry['runs']),flush=True)
    result['complete']=True
    (ROOT/'logs'/('frame-admission-'+args.stage+'.json')).write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
