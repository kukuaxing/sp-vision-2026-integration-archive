#!/usr/bin/env python3
"""Paired controller experiments with fixed world errors and held-out seeds."""
import argparse
import copy
import datetime as dt
import json
from pathlib import Path
import subprocess
import tempfile
from run_fixed_target import ROOT,digest,summarize,render

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images',action='store_true')
    parser.add_argument('--holdout',action='store_true')
    parser.add_argument('--delay-sensitivity',action='store_true')
    args=parser.parse_args()
    source=ROOT/'logs/fixed-target-image-stress.json'
    prior=json.loads(source.read_text())
    base=copy.deepcopy(prior['experiments'][0]['scenario'])
    if not args.images:
        base['observation']={'mode':'keypoints'};base['sampling']['keypoint_noise_px']=1.
    if args.holdout:
        base['sampling']['seeds']=[211,397,619]
        base['target']['initial_phases_rad']=[1.5707963267948966,2.356194490192345]
    else:
        base['sampling']['seeds']=[27,83,149]
    executable=ROOT/'build/native-offline/bin'/('fixed_target_image_sim' if args.images else 'fixed_target_sim')
    files=['configs/standard.yaml','simulation/optimize_fixed_target.py','simulation/src/fixed_target_sim.cpp',
        'simulation/src/fixed_target_timing.hpp','simulation/src/fixed_target_world.hpp','simulation/src/fixed_target_plant.hpp',
        'simulation/src/fixed_target_camera.hpp','tasks/auto_aim/rotation_fire_window.hpp',
        'tasks/auto_aim/high_speed_vision.hpp','tasks/auto_aim/fixed_center_model.hpp',
        'tasks/auto_aim/aimer.cpp','tasks/auto_aim/target.cpp','tasks/auto_aim/tracker.cpp','tasks/auto_aim/solver.cpp',
        str(executable.relative_to(ROOT)),str(source.relative_to(ROOT))]
    if args.images:files+=['assets/yolo26.xml','assets/yolo26.bin','simulation/src/fixed_target_renderer.hpp',
        'tasks/auto_aim/yolos/yolo26.cpp','tasks/auto_aim/adaptive_corner_refiner.cpp']
    data={'schema':1,'kind':'fixed_target_images' if args.images else 'fixed_target_controller_comparison',
        'objective':prior['objective'],'algorithm_parameters':prior['algorithm_parameters'],
        'generated_utc':dt.datetime.now(dt.timezone.utc).isoformat(),
        'physical_device_opened':False,'truth_supplied_to_predictor':False,'ground_truth_fallback':False,
        'scope':'paired fixed physical distributions; experimental host timing only; no real hardware',
        'split':'heldout seeds and phases' if args.holdout else 'development seeds',
        'sha256':{name:digest(ROOT/name) for name in files},'experiments':[]}
    stem='fixed-target-timing-'+('image' if args.images else 'geometry')+('-holdout' if args.holdout else '-development')
    output=ROOT/'logs'/(stem+'.json')
    development=ROOT/'logs/fixed-target-timing-geometry-development.json'
    if args.holdout or args.images:
        frozen=json.loads(development.read_text())['selection']
        candidates=[('baseline','原帧级下发',False,0.),('calibrated','独立开发集标定',frozen['subframe'],frozen['offset_s'])]
        data['frozen_selection']=frozen
        data['sha256'][str(development.relative_to(ROOT))]=digest(development)
    else:
        candidates=[('baseline','原帧级下发',False,0.),('center_deadline','帧内过中线下发',True,0.)]
        candidates += [(f'offset_{ms}ms',f'预测标定+{ms}ms',False,ms/1000) for ms in (5,10,15,20,25,30)]
    if args.delay_sensitivity:
        assert not args.images and args.holdout
        candidates=[(f'{name}_delay{ms}',f'{label}·物理{ms}ms',scheduled,offset,ms)
            for ms in (320,340,360) for name,label,scheduled,offset in candidates]
        stem+='-delay-sensitivity';output=ROOT/'logs'/(stem+'.json')
    for candidate in candidates:
        name,label,scheduled,offset=candidate[:4]
        case=copy.deepcopy(base);case.setdefault('controller',{}).update(
            center_crossing_schedule=scheduled,prediction_time_offset_s=offset)
        if len(candidate)>4:case['plant']['muzzle_delay_s']=candidate[4]/1000
        if args.images:case['observation']['preview_directory']='logs/fixed-target-images/'+stem+'/'+name
        with tempfile.TemporaryDirectory(prefix='rm-timing-') as folder:
            src,out=Path(folder)/'scenario.json',Path(folder)/'result.json';src.write_text(json.dumps(case))
            with (ROOT/'logs'/(stem+'-'+name+'.log')).open('w') as log:
                subprocess.run([str(executable),str(ROOT/'configs/standard.yaml'),str(src),str(out)],
                    cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=1800)
            e=json.loads(out.read_text());e.update(name=name,label=label,summary=summarize(e))
        data['experiments'].append(e)
        output.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(name,json.dumps(e['summary']),flush=True)
    if not args.holdout and not args.images:
        # Fit on development runs only; this file must be frozen before testing
        # on the independent seeds/phases. No online truth or shot feedback.
        best=max(data['experiments'],key=lambda e:(e['summary']['zero_hp_runs'],
            e['summary']['valid_hits'],e['summary']['worst_run_budget_hit_fraction']))
        ctl=best['scenario']['controller']
        data['selection']={'name':best['name'],'offset_s':ctl['prediction_time_offset_s'],
            'subframe':ctl['center_crossing_schedule'],
            'basis':'development synthetic hit outcomes only; requires separate hardware calibration before deployment'}
        output.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print('selection',json.dumps(data['selection']),flush=True)
    render(data,ROOT/'logs'/(stem+'.html'))
    print(output,flush=True)

if __name__=='__main__':main()
