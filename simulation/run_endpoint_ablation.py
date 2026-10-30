#!/usr/bin/env python3
"""Diagnostic endpoint/servo/camera ablations on a declared old development subset."""
import copy
import json
import math
from pathlib import Path
import subprocess
import tempfile
import yaml
from run_fixed_target import ROOT,digest,summarize


def main():
    source=ROOT/'logs/endpoint-deployment-images.json'
    prior=json.loads(source.read_text());assert prior['complete']
    cfg=yaml.safe_load((ROOT/'configs/standard.yaml').read_text())
    base=copy.deepcopy(prior['experiments'][0]['scenario'])
    base['sampling']['seeds']=[6011];base['target']['initial_phases_rad']=[math.pi/12]
    executable=ROOT/'build/native-offline/bin/fixed_target_image_sim'
    files=['simulation/src/fixed_target_sim.cpp','simulation/src/fixed_target_plant.hpp',
        'simulation/src/fixed_target_world.hpp','simulation/src/fixed_target_counterfactual.cpp',
        'simulation/run_endpoint_ablation.py','configs/standard.yaml',
        'tasks/auto_aim/adaptive_corner_refiner.cpp','tasks/auto_aim/yolos/yolo26.cpp',
        str(executable.relative_to(ROOT)),str(source.relative_to(ROOT))]
    result={'schema':1,'scope':'diagnostic ablations only, not a new hit-rate validation or deployment decision',
        'split':'old endpoint holdout subset explicitly reused as development',
        'physical_device_opened':False,'truth_supplied_to_predictor':False,
        'sha256':{p:digest(ROOT/p) for p in files},'experiments':[]}
    for condition in ('measured_follower','fixed_camera','instant_follower'):
        for enabled in (False,True):
            endpoint='candidate' if enabled else 'baseline';name=condition+'_'+endpoint
            config=copy.deepcopy(cfg);config['corner_refine_brightness_invariant_subpixel']=enabled
            case=copy.deepcopy(base);case['diagnostics']={'fixed_camera_pose':condition=='fixed_camera'}
            if condition=='instant_follower':case['plant']['servo_time_constant_s']=0.
            case['observation']['preview_directory']='logs/fixed-target-images/endpoint-ablation/'+name
            with tempfile.TemporaryDirectory(prefix='rm-endpoint-ablation-') as tmp:
                cp,sc,out=[Path(tmp)/p for p in ('config.yaml','scenario.json','output.json')]
                cp.write_text(yaml.safe_dump(config,sort_keys=False));sc.write_text(json.dumps(case))
                with (ROOT/'logs'/('endpoint-ablation-'+name+'.log')).open('w') as log:
                    subprocess.run([str(executable),str(cp),str(sc),str(out)],cwd=ROOT,
                        stdout=log,stderr=subprocess.STDOUT,check=True,timeout=900)
                e=json.loads(out.read_text());e.update(name=name,condition=condition,endpoint_enabled=enabled)
            assert len(e['runs'])==2 and all(r['issued']==50 for r in e['runs'])
            if condition=='measured_follower':
                reference=next(x for x in prior['experiments'] if x['name']==endpoint)
                max_difference=0.
                for run in e['runs']:
                    old=next(r for r in reference['runs'] if r['seed']==run['seed'] and
                        r['direction']==run['direction'] and r['initial_phase_rad']==run['initial_phase_rad'])
                    for a,b in zip(old['shots'],run['shots']):
                        for key,value in a.items():
                            if isinstance(value,(float,int)) and not isinstance(value,bool):
                                difference=abs(value-b[key]);max_difference=max(max_difference,difference)
                                assert difference<1e-9,(name,key,difference)
                            else:assert value==b[key],(name,key)
                e['historical_shot_trace_reproduced']=True;e['max_historical_scalar_difference']=max_difference
            e['summary']=summarize(e);result['experiments'].append(e)
            (ROOT/'logs/endpoint-ablation.json').write_text(json.dumps(result,indent=2)+'\n')
            print(name,json.dumps(e['summary']),flush=True)
    result['complete']=True
    output=ROOT/'logs/endpoint-ablation.json';output.write_text(json.dumps(result,indent=2)+'\n')
    subprocess.run([str(ROOT/'build/native-offline/bin/fixed_target_counterfactual'),str(output),
        str(ROOT/'logs/endpoint-ablation-counterfactual.json')],cwd=ROOT,check=True,timeout=180)


if __name__=='__main__':main()
