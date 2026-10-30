#!/usr/bin/env python3
"""Second independent validation after the common-offset hypothesis failed."""
import copy
import datetime as dt
import json
from pathlib import Path
import subprocess
import tempfile
from run_fixed_target import ROOT,digest,summarize,render

def main():
    source=ROOT/'logs/fixed-target-timing-image-holdout.json'
    development=json.loads(source.read_text())
    selected={}
    for sign in (-1,1):
        best=max(development['experiments'],key=lambda e:sum(r['valid_hits'] for r in e['runs'] if r['direction']==sign))
        selected[str(sign)]=best['scenario']['controller']['prediction_time_offset_s']
    case=copy.deepcopy(development['experiments'][0]['scenario'])
    case['sampling']['seeds']=[997,1543,2039]
    case['target']['initial_phases_rad']=[0.39269908169872414,1.1780972450961724]
    files=[str(source.relative_to(ROOT)),'configs/standard.yaml','simulation/validate_directional_timing.py',
        'simulation/src/fixed_target_sim.cpp','simulation/src/fixed_target_timing.hpp',
        'simulation/src/fixed_target_camera.hpp','simulation/src/fixed_target_renderer.hpp',
        'simulation/src/fixed_target_world.hpp','simulation/src/fixed_target_plant.hpp',
        'tasks/auto_aim/high_speed_vision.hpp','tasks/auto_aim/fixed_center_model.hpp',
        'tasks/auto_aim/face_prediction_residual.hpp','tasks/auto_aim/rotation_fire_window.hpp',
        'tasks/auto_aim/aimer.cpp','tasks/auto_aim/target.cpp','tasks/auto_aim/tracker.cpp','tasks/auto_aim/solver.cpp',
        'tasks/auto_aim/yolos/yolo26.cpp','tasks/auto_aim/adaptive_corner_refiner.cpp',
        'simulation/run_fixed_target.py','simulation/fixed_target_report.html',
        'assets/yolo26.xml','assets/yolo26.bin','build/native-offline/bin/fixed_target_image_sim']
    result={'schema':1,'kind':'fixed_target_images','objective':development['objective'],
        'algorithm_parameters':development['algorithm_parameters'],
        'physical_device_opened':False,'truth_supplied_to_predictor':False,'ground_truth_fallback':False,
        'generated_utc':dt.datetime.now(dt.timezone.utc).isoformat(),
        'scope':'stage1 common offset rejected on images; per-direction calibration frozen before new independent test; not hardware',
        'split':'directional heldout seeds and phases','frozen_directional_selection':selected,
        'selection_source':str(source.relative_to(ROOT)),
        'selection_protocol':'first image holdout reused explicitly as stage2 development; final seeds/phases are new; no further tuning on final test',
        'sha256':{name:digest(ROOT/name) for name in files},'experiments':[]}
    print('frozen_directional_selection',json.dumps(selected),flush=True)
    stem='fixed-target-directional-timing'
    for name,label,offsets in [('baseline','原帧级下发',{}),('directional','分旋向离线标定',selected)]:
        scenario=copy.deepcopy(case)
        scenario['controller']={'center_crossing_schedule':False,'prediction_time_offset_s':0.,
                                'directional_prediction_offsets_s':offsets}
        scenario['observation']['preview_directory']='logs/fixed-target-images/'+stem+'/'+name
        with tempfile.TemporaryDirectory(prefix='rm-directional-') as folder:
            src,out=Path(folder)/'scenario.json',Path(folder)/'result.json';src.write_text(json.dumps(scenario))
            with (ROOT/'logs'/(stem+'-'+name+'.log')).open('w') as log:
                subprocess.run([str(ROOT/'build/native-offline/bin/fixed_target_image_sim'),
                    str(ROOT/'configs/standard.yaml'),str(src),str(out)],cwd=ROOT,stdout=log,
                    stderr=subprocess.STDOUT,check=True,timeout=1800)
            experiment=json.loads(out.read_text());experiment.update(name=name,label=label,summary=summarize(experiment))
        result['experiments'].append(experiment)
        (ROOT/'logs'/(stem+'.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(name,json.dumps(experiment['summary']),flush=True)
    render(result,ROOT/'logs'/(stem+'.html'))

if __name__=='__main__':main()
