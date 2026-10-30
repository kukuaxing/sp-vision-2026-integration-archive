#!/usr/bin/env python3
"""Audit the diagnostic interventions and summarize measured servo/phase traces."""
import json
import math
from pathlib import Path
import subprocess
import tempfile
import numpy as np
import yaml
from run_fixed_target import ROOT,digest,summarize
from analyze_fixed_target_misses import analyze as collision_audit


def stats(values):
    x=np.asarray(values,dtype=float)
    return {'n':len(x),'mean':float(x.mean()),'median':float(np.median(x)),
            'p05':float(np.quantile(x,.05)),'p95':float(np.quantile(x,.95)),
            'max_abs':float(np.max(np.abs(x)))}


def main():
    path=ROOT/'logs/endpoint-ablation.json';data=json.loads(path.read_text())
    assert data['complete'] and len(data['experiments'])==6
    expected={mode+'_'+endpoint for mode in ('measured_follower','fixed_camera','instant_follower')
        for endpoint in ('baseline','candidate')}
    assert {e['name'] for e in data['experiments']}==expected
    for name,value in data['sha256'].items():assert digest(ROOT/name)==value,name
    prior=json.loads((ROOT/'logs/endpoint-deployment-images.json').read_text())
    result={'schema':1,'scope':'diagnostic development, not independent accuracy validation',
        'physical_device_opened':False,'experiments':[]}
    for e in data['experiments']:
        assert len(e['runs'])==2 and summarize(e)==e['summary']
        reference=next(x for x in data['experiments'] if x['name']=='measured_follower_'+('candidate' if e['endpoint_enabled'] else 'baseline'))
        assert e['scenario']['target']==reference['scenario']['target']
        assert e['scenario']['sampling']==reference['scenario']['sampling']
        plant=dict(e['scenario']['plant']);original=reference['scenario']['plant']
        if e['condition']=='instant_follower':assert plant.pop('servo_time_constant_s')==0.
        else:plant.pop('servo_time_constant_s')
        assert plant=={k:v for k,v in original.items() if k!='servo_time_constant_s'}
        assert e['scenario']['controller']==reference['scenario']['controller']
        for r in e['runs']:
            assert r['issued']==r['ammo_budget']==50
            assert r['diagnostic_fixed_camera_pose']==(e['condition']=='fixed_camera')
            for s in r['shots']:
                assert s['capture_s']<=s['decision_s']+1e-9<=s['issue_s']+2e-9<=s['muzzle_s']+3e-9
            rr=next(x for x in reference['runs'] if x['direction']==r['direction'])
            for a,b in zip(rr['shots'],r['shots']):
                assert abs(a['physical_speed_mps']-b['physical_speed_mps'])<1e-10
                assert abs((a['muzzle_s']-a['issue_s'])-(b['muzzle_s']-b['issue_s']))<1e-10
        if e['condition']=='measured_follower':
            pp=next(x for x in prior['experiments'] if x['endpoint_enabled']==e['endpoint_enabled'])
            for r in e['runs']:
                old=next(x for x in pp['runs'] if (x['seed'],x['direction'],x['initial_phase_rad'])==
                    (r['seed'],r['direction'],r['initial_phase_rad']))
                for a,b in zip(old['shots'],r['shots']):
                    for key,value in a.items():
                        if isinstance(value,(float,int)) and not isinstance(value,bool):assert abs(value-b[key])<1e-9
                        else:assert value==b[key]
        shots=[s for r in e['runs'] for s in r['shots']]
        servo={axis:stats([abs(math.remainder(s['muzzle_target_'+axis+'_rad']-s['muzzle_servo_'+axis+'_rad'],2*math.pi))*180/math.pi
            for s in shots]) for axis in ('yaw','elevation')}
        command_change={axis:stats([abs(math.remainder(s['muzzle_target_'+axis+'_rad']-s['issue_desired_'+axis+'_rad'],2*math.pi))*180/math.pi
            for s in shots]) for axis in ('yaw','elevation')}
        phase_error=[]
        for r in e['runs']:
            for s in r['shots']:
                center,point=s['estimated_center_world_m'],s['model_point_world_m']
                normal=math.atan2(center[1]-point[1],center[0]-point[0])
                # Quarter-turn gauge removes arbitrary internal face numbering.
                # This diagnostic cannot establish independent physical face ID.
                phase_error.append(math.remainder(normal-r['initial_phase_rad']-r['omega_rad_s']*s['model_impact_s'],math.pi/2)*180/math.pi)
        result['experiments'].append({'name':e['name'],'summary':e['summary'],
            'launch_servo_error_deg':servo,'command_change_during_delay_deg':command_change,
            'model_phase_error_modulo_quarter_turn_deg':stats(phase_error)})
    # Independent NumPy geometry checks all 600 original diagnostic records.
    collisions=collision_audit(data)
    assert len(collisions['shots'])==600
    result['independent_collision_recompute_passed']=True
    counter=ROOT/'build/native-offline/bin/fixed_target_counterfactual'
    for src,record in ((path,ROOT/'logs/endpoint-ablation-counterfactual.json'),
                       (ROOT/'logs/endpoint-deployment-images.json',ROOT/'logs/endpoint-frozen-timing.json')):
        with tempfile.TemporaryDirectory(prefix='rm-frozen-audit-') as tmp:
            out=Path(tmp)/'replay.json'
            subprocess.run([str(counter),str(src),str(out)],cwd=ROOT,check=True,timeout=180,stdout=subprocess.DEVNULL)
            assert json.loads(out.read_text())==json.loads(record.read_text())
    result['frozen_interventions_reproduced']=True
    for name,input_name,verified in (('endpoint-full-uncertainty.json','endpoint-deployment-images.json',1200),
                                     ('endpoint-ablation-uncertainty.json','endpoint-ablation.json',200)):
        u=json.loads((ROOT/'logs'/name).read_text())
        assert u['input_sha256']==digest(ROOT/'logs'/input_name)
        assert u['evaluator_sha256']==digest(ROOT/'simulation/score_frozen_uncertainty.py')
        assert u['zero_intervention_shots_verified']==verified
    tests=ROOT/'build/native-offline/Testing/Temporary/LastTest.log';log=tests.read_text()
    # CTest keeps the previous LastTest.log until its current run finishes.
    # Require this study's stdout summary too; an older 54-pass log is not enough.
    assert '100% tests passed, 0 tests failed out of 54' in (ROOT/'logs/endpoint-ablation-tests.log').read_text()
    assert log.count('Test Passed.')==54 and 'Test Failed.' not in log
    release=json.loads((ROOT/'logs/simulation-deployment-validation.json').read_text())
    assert digest(ROOT/'build/standard')==release['formal_binary_sha256']
    assert digest(ROOT/'configs/standard.yaml')==release['formal_yaml_sha256']
    assert yaml.safe_load((ROOT/'configs/standard.yaml').read_text())['corner_refine_brightness_invariant_subpixel'] is False
    result.update(ctest_passed=54,formal_program_and_config_unchanged=True,
        historical_measured_shot_trace_reproduced=True,paired_random_physics_and_causality_passed=True)
    files=list(data['sha256'])+['simulation/analyze_endpoint_ablation.py','simulation/score_frozen_uncertainty.py',
        'logs/endpoint-ablation.json','logs/endpoint-ablation-counterfactual.json','logs/endpoint-frozen-timing.json',
        'logs/endpoint-full-uncertainty.json','logs/endpoint-ablation-uncertainty.json',
        'build/native-offline/bin/fixed_target_counterfactual','build/standard']
    result['sha256']={name:digest(ROOT/name) for name in files}
    result['ctest_log_sha256']=digest(tests)
    output=ROOT/'logs/endpoint-ablation-analysis.json';output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('experiments','sha256')}))


if __name__=='__main__':main()
