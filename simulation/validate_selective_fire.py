#!/usr/bin/env python3
"""Prospective face-selection / elevation-calibration experiment; no devices."""
import argparse
import copy
import datetime as dt
import json
import math
from pathlib import Path
import subprocess
import tempfile
import numpy as np
from analyze_fixed_target_misses import analyze
from run_fixed_target import ROOT,digest,summarize,render


def calibrate(development):
    # Historical labelled training data only. This is an offline impact
    # calibration, NOT an online visual estimate of unknown barrel errors.
    baseline=development['experiments'][0]
    rows=analyze({'experiments':[baseline]})['shots']
    shots=[s for r in baseline['runs'] for s in r['shots']]
    tilt=math.radians(baseline['scenario']['target']['armor_pitch_deg'])
    errors=[]
    for row,shot in zip(rows,shots):
        cross=row['nearest_crossing']
        if cross is None:continue
        distance=shot['physical_speed_mps']*math.cos(shot['world_elevation'])*(cross['time_s']-shot['muzzle_s'])
        if distance>1:errors.append(math.degrees(math.atan2(cross['v_m'],distance*math.cos(tilt))))
    median=float(np.median(errors))
    trim=float(np.clip(round(-median/.02)*.02,-.3,.3))
    return {'samples':len(errors),'median_vertical_angle_deg':median,'elevation_trim_deg':trim,
        'quantization_deg':.02,'bound_deg':.3,
        'scope':'offline labelled simulated impacts; real hardware needs separate calibration'}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--screen',action='store_true')
    parser.add_argument('--image-screen',action='store_true')
    args=parser.parse_args()
    source=ROOT/'logs/fixed-target-timing-image-holdout.json'
    timing=ROOT/'logs/fixed-target-directional-timing.json'
    development=json.loads(source.read_text());prior=json.loads(timing.read_text())
    calibration=calibrate(development);offsets=prior['frozen_directional_selection']
    base=copy.deepcopy(development['experiments'][0]['scenario'])
    assert not(args.screen and args.image_screen)
    if args.screen:
        base['observation']={'mode':'keypoints'};base['sampling']['keypoint_noise_px']=1.
    elif args.image_screen:
        base['sampling']['seeds']=[211]
    else:
        base['sampling']['seeds']=[3011,4001,5003]
        base['target']['initial_phases_rad']=[math.pi/6,5*math.pi/12]
    executable=ROOT/'build/native-offline/bin'/('fixed_target_sim' if args.screen else 'fixed_target_image_sim')
    files={p for p in prior['sha256'] if not p.startswith('logs/') and not p.startswith('build/')}
    files.update(['simulation/validate_selective_fire.py','simulation/analyze_fixed_target_misses.py',
        str(source.relative_to(ROOT)),str(timing.relative_to(ROOT)),str(executable.relative_to(ROOT))])
    selection=None
    screen_path=ROOT/'logs/fixed-target-selective-image-screen.json'
    if not args.screen and not args.image_screen:
        screen=json.loads(screen_path.read_text())
        eligible=screen['experiments']
        best=max(eligible,key=lambda e:(e['summary']['zero_hp_runs'],e['summary']['valid_hits'],
            e['summary']['worst_run_budget_hit_fraction'],-(e['summary']['max_kill_s'] or 999.)))
        selection={'name':best['name'],'controller':best['scenario']['controller'],
                   'source':str(screen_path.relative_to(ROOT))}
        files.add(str(screen_path.relative_to(ROOT)))
    result={'schema':1,'kind':'fixed_target_controller_comparison' if args.screen else 'fixed_target_images',
        'comparison_type':'selective_fire','objective':prior['objective'],
        'algorithm_parameters':prior['algorithm_parameters'],'calibration':calibration,
        'fixed_directional_offsets_s':offsets,
        'calibration_source':str(source.relative_to(ROOT)),'timing_source':str(timing.relative_to(ROOT)),
        'split':'development geometry screening' if args.screen else 'development image screening' if args.image_screen else 'selective heldout seeds and phases',
        'physical_device_opened':False,'truth_supplied_to_predictor':False,'ground_truth_fallback':False,
        'generated_utc':dt.datetime.now(dt.timezone.utc).isoformat(),
        'scope':'same fixed physical errors; smaller fitted radius and offline vertical calibration; shot-count penalty unchanged',
        'sha256':{name:digest(ROOT/name) for name in sorted(files)},'experiments':[]}
    if selection:result['frozen_policy']=selection
    stem='fixed-target-selective-'+('screen' if args.screen else 'image-screen' if args.image_screen else 'images')
    candidates=[('baseline','既有分旋向标定',False,0.,False)]
    if args.screen:
        candidates += [('elevation_only','仅俯仰标定',False,calibration['elevation_trim_deg'],False),
                       ('selective','小半径选面+俯仰标定',True,calibration['elevation_trim_deg'],False)]
    elif args.image_screen:
        candidates += [('elevation_only','仅俯仰标定',False,calibration['elevation_trim_deg'],False),
                       ('visual_elevation','历史视觉残差+俯仰标定',False,calibration['elevation_trim_deg'],True)]
    else:
        ctl=selection['controller']
        candidates += [('refined','独立开发组选定策略',ctl['prefer_smaller_radius'],ctl['elevation_trim_deg'],ctl['apply_visual_residual_correction'])]
    print('frozen_calibration',json.dumps(calibration),flush=True)
    for name,label,selective,trim,visual in candidates:
        case=copy.deepcopy(base)
        case['controller']={'center_crossing_schedule':False,'prediction_time_offset_s':0.,
            'directional_prediction_offsets_s':offsets,'prefer_smaller_radius':selective,'elevation_trim_deg':trim,
            'apply_visual_residual_correction':visual}
        if not args.screen:case['observation']['preview_directory']='logs/fixed-target-images/'+stem+'/'+name
        with tempfile.TemporaryDirectory(prefix='rm-selective-') as folder:
            src,out=Path(folder)/'scenario.json',Path(folder)/'result.json';src.write_text(json.dumps(case))
            with (ROOT/'logs'/(stem+'-'+name+'.log')).open('w') as log:
                subprocess.run([str(executable),str(ROOT/'configs/standard.yaml'),str(src),str(out)],cwd=ROOT,
                    stdout=log,stderr=subprocess.STDOUT,check=True,timeout=2400)
            experiment=json.loads(out.read_text());experiment.update(name=name,label=label,summary=summarize(experiment))
        result['experiments'].append(experiment)
        (ROOT/'logs'/(stem+'.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(name,json.dumps(experiment['summary']),flush=True)
    render(result,ROOT/'logs'/(stem+'.html'))

if __name__=='__main__':main()
