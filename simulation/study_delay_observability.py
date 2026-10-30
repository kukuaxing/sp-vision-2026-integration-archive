#!/usr/bin/env python3
"""Diagnostic twins: identical armor observations, different independent muzzle noise."""
import copy
import json
import math
from pathlib import Path
from run_fixed_target import ROOT,digest
from study_recorded_cadence import run_case


def main():
    source=ROOT/'logs/recorded-cadence-stress.json';prior=json.loads(source.read_text());assert prior['complete']
    base=copy.deepcopy(next(e for e in prior['experiments'] if e['name']=='baseline_jitter_10')['scenario'])
    base['sampling']['seeds']=[91001]
    # Explicit diagnostic intervention: exact servo removes numerical event
    # partition coupling. No recoil, projectile imagery, feeder or muzzle
    # sensor is supplied to the armor-only frontend in either world.
    base['plant']['servo_time_constant_s']=0.
    deps={'simulation/study_delay_observability.py','simulation/study_recorded_cadence.py',
        'simulation/run_fixed_target.py',str(source.relative_to(ROOT)),
        'configs/standard.yaml','build/standard','build/native-offline/bin/fixed_target_image_sim',
        'assets/yolo26.xml','assets/yolo26.bin','data/field/tracker_observations.json'}
    for folder in ('simulation/src','tasks/auto_aim','tools'):
        deps.update(str(p.relative_to(ROOT)) for p in (ROOT/folder).rglob('*') if p.suffix in ('.cpp','.hpp','.h'))
    result={'schema':1,'study':'armor_only_delay_observability_twins','complete':False,
        'scope':'diagnostic reused development subset; instant servo; not a new controller, accuracy validation or real-world identifiability proof',
        'physical_device_opened':False,'truth_supplied_to_predictor':False,
        'physical_jitter_sigma_not_measured':True,'sha256':{k:digest(ROOT/k) for k in sorted(deps)},'experiments':[]}
    for ms in (10,40):
        case=copy.deepcopy(base);case['plant']['muzzle_delay_jitter_s']=ms*.001
        name='sigma_'+str(ms)+'ms'
        case['observation']['preview_directory']='logs/fixed-target-images/delay-observability/'+name
        e=run_case(case,name,'observability');result['experiments'].append(e)
        (ROOT/'logs/delay-observability-twins.json').write_text(json.dumps(result,indent=2)+'\n')
        print(name,json.dumps(e['summary']),flush=True)
    result['complete']=True
    (ROOT/'logs/delay-observability-twins.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
