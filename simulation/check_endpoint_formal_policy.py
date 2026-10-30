#!/usr/bin/env python3
"""File-only smoke with formal delay values and visual residual application."""
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
    case=copy.deepcopy(prior['experiments'][0]['scenario'])
    case['sampling']['seeds']=[12007];case['target']['initial_phases_rad']=[math.radians(40)]
    case['controller']['directional_prediction_offsets_s']={'-1':0.,'1':0.}
    case['controller']['prediction_time_offset_s']=0.
    case['controller']['apply_visual_residual_correction']=True
    config=yaml.safe_load((ROOT/'configs/standard.yaml').read_text())
    result={'scope':'formal prediction delay + residual smoke; not full standard/MCU, no devices',
        'physical_device_opened':False,'truth_supplied_to_predictor':False,
        'extra_prediction_offset_s':0.,'apply_visual_residual_correction':True,'experiments':[],
        'source_sha256':digest(source),'producer_sha256':digest(Path(__file__))}
    with tempfile.TemporaryDirectory(prefix='rm-formal-endpoint-') as tmp:
        for enabled in (False,True):
            name='candidate' if enabled else 'baseline';cfg=copy.deepcopy(config)
            cfg['corner_refine_brightness_invariant_subpixel']=enabled
            cp=Path(tmp)/'config.yaml';cp.write_text(yaml.safe_dump(cfg,sort_keys=False))
            case['observation']['preview_directory']='logs/fixed-target-images/endpoint-formal/'+name
            sc=Path(tmp)/'scenario.json';sc.write_text(json.dumps(case))
            out=ROOT/'logs'/('endpoint-formal-'+name+'.json')
            with (ROOT/'logs'/('endpoint-formal-'+name+'.log')).open('w') as log:
                subprocess.run([str(ROOT/'build/native-offline/bin/fixed_target_image_sim'),str(cp),str(sc),str(out)],
                    cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=900)
            experiment=json.loads(out.read_text());experiment.update(name=name,summary=summarize(experiment))
            assert len(experiment['runs'])==2 and all(r['issued']==50 for r in experiment['runs'])
            for r in experiment['runs']:
                assert all(s['prediction_time_offset_s']==0 and s['visual_correction_applied'] for s in r['shots'])
            result['experiments'].append(experiment)
            print(name,json.dumps(experiment['summary']),flush=True)
    old,new=[e['summary'] for e in result['experiments']]
    result['nonregression_passed']=all(new[k]>=old[k] for k in ('valid_hits','worst_run_budget_hit_fraction','zero_hp_runs'))
    result['complete']=True
    (ROOT/'logs/endpoint-formal-policy.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
