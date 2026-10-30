#!/usr/bin/env python3
"""Paired endpoint deployment checks on files only. Never runs standard/devices."""
import argparse
import copy
import json
import math
from pathlib import Path
import subprocess
import tempfile
import yaml
from run_fixed_target import ROOT,digest,summarize
from analyze_fixed_target_geometry import analyze


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry',action='store_true')
    parser.add_argument('--images',action='store_true')
    args=parser.parse_args()
    assert args.geometry != args.images,'select exactly one check'
    config=yaml.safe_load((ROOT/'configs/standard.yaml').read_text())
    stem='endpoint-deployment-'+('geometry' if args.geometry else 'images')
    executable=ROOT/'build/native-offline/bin'/('fixed_target_geometry_probe' if args.geometry else 'fixed_target_image_sim')
    source=ROOT/'logs/fixed-target-selective-images.json'
    result={'schema':1,'physical_device_opened':False,'formal_program_executed':False,
        'candidate_frozen':{'corner_refine_brightness_invariant_subpixel':True},
        'split':'diagnostic development' if args.geometry else 'new endpoint image holdout',
        'experiments':[],'sha256':{p:digest(ROOT/p) for p in [
            'configs/standard.yaml','simulation/validate_endpoint_deployment.py',
            'tasks/auto_aim/adaptive_corner_refiner.cpp','tasks/auto_aim/adaptive_corner_refiner.hpp',
            'tasks/auto_aim/yolos/yolo26.cpp',str(executable.relative_to(ROOT))]}}
    if args.images:
        prior=json.loads(source.read_text());result['sha256'][str(source.relative_to(ROOT))]=digest(source)
        case=copy.deepcopy(prior['experiments'][0]['scenario'])
        case['sampling']['seeds']=[6011,7001,8011]
        case['target']['initial_phases_rad']=[math.pi/12,math.pi/3]
    with tempfile.TemporaryDirectory(prefix='rm-endpoint-') as tmp:
        for enabled in (False,True):
            name='candidate' if enabled else 'baseline';cfg=copy.deepcopy(config)
            cfg['corner_refine_brightness_invariant_subpixel']=enabled
            cp=Path(tmp)/'config.yaml';cp.write_text(yaml.safe_dump(cfg,sort_keys=False))
            raw=ROOT/'logs'/(stem+'-'+name+'.json')
            args_cpp=[str(executable),str(cp)]
            if args.images:
                scenario=Path(tmp)/'scenario.json'
                case['observation']['preview_directory']='logs/fixed-target-images/'+stem+'/'+name
                scenario.write_text(json.dumps(case));args_cpp.append(str(scenario))
            args_cpp.append(str(raw))
            with (ROOT/'logs'/(stem+'-'+name+'.log')).open('w') as log:
                subprocess.run(args_cpp,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=2400)
            if args.geometry:
                experiment={'name':name,'geometry':analyze(raw)}
                last=experiment['geometry']['experiments'][-1]
                print(name,'center_x_error_m',last['center_x_error_m']['mean'],flush=True)
            else:
                experiment=json.loads(raw.read_text());experiment.update(name=name)
                experiment['summary']=summarize(experiment)
                print(name,json.dumps(experiment['summary']),flush=True)
            experiment['tested_config_sha256']=digest(cp)
            experiment['endpoint_enabled']=enabled;experiment['raw_sha256']=digest(raw)
            result['experiments'].append(experiment)
            (ROOT/'logs'/(stem+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    assert len(result['experiments'])==2
    if args.images:
        baseline,candidate=result['experiments']
        a,b=baseline['runs'],candidate['runs'];assert len(a)==len(b)==12
        for old,new in zip(a,b):
            assert (old['seed'],old['direction'],old['initial_phase_rad'])==(new['seed'],new['direction'],new['initial_phase_rad'])
            assert old['issued']==new['issued']==50
            for x,y in zip(old['shots'],new['shots']):
                assert abs(x['physical_speed_mps']-y['physical_speed_mps'])<1e-10
                assert abs((x['muzzle_s']-x['issue_s'])-(y['muzzle_s']-y['issue_s']))<1e-10
        result['paired_random_physics_check_passed']=True
        old,new=baseline['summary'],candidate['summary']
        result['candidate_nonregression_passed']=all(new[k]>=old[k] for k in
            ('valid_hits','worst_run_budget_hit_fraction','zero_hp_runs'))
    result['complete']=True
    (ROOT/'logs'/(stem+'.json')).write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
