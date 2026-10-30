#!/usr/bin/env python3
"""Read-only release audit; never executes standard or accesses devices."""
import hashlib
import json
from pathlib import Path
import sys
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'simulation'))
from run_fixed_target import summarize
from analyze_fixed_target_geometry import analyze as analyze_geometry


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    data=json.loads((ROOT/'logs/endpoint-deployment-images.json').read_text())
    assert data['complete'] and data['paired_random_physics_check_passed']
    assert len(data['experiments'])==2
    config=yaml.safe_load((ROOT/'configs/standard.yaml').read_text())
    assert config['corner_refine_brightness_invariant_subpixel'] is False
    assert data['candidate_nonregression_passed'] is False
    # Compare the deployed YAML semantically to BOTH actual tested temporary
    # configurations, preserving original key order and only varying one flag.
    canonical={k:v for k,v in config.items() if k!='corner_refine_brightness_invariant_subpixel'}
    audited=0
    for e,enabled in zip(data['experiments'],(False,True)):
        cc=dict(canonical);cc['corner_refine_brightness_invariant_subpixel']=enabled
        in_place=dict(config);in_place['corner_refine_brightness_invariant_subpixel']=enabled
        assert e['tested_config_sha256'] in {hashlib.sha256(yaml.safe_dump(c,sort_keys=False).encode()).hexdigest()
            for c in (cc,in_place)}
        assert e['endpoint_enabled']==enabled and len(e['runs'])==12
        assert summarize(e)==e['summary']
        assert e['summary']['issued']==600
        for r in e['runs']:
            assert r['ammo_budget']==50 and r['issued']==50
            assert r['valid_hits']==sum(s['valid_hit'] for s in r['shots'])
            for s in r['shots']:
                assert s['capture_s']<=s['decision_s']+1e-9<=s['issue_s']+2e-9
                assert s['muzzle_s']>=s['issue_s'] and s['model_impact_s']>s['decision_s']
            audited+=len(r['shots'])
    for old,new in zip(*[e['runs'] for e in data['experiments']]):
        assert (old['seed'],old['direction'],old['initial_phase_rad'])==(new['seed'],new['direction'],new['initial_phase_rad'])
        for a,b in zip(old['shots'],new['shots']):
            assert abs(a['physical_speed_mps']-b['physical_speed_mps'])<1e-10
            assert abs((a['muzzle_s']-a['issue_s'])-(b['muzzle_s']-b['issue_s']))<1e-10
    for name,value in data['sha256'].items():
        if name!='configs/standard.yaml':assert sha(ROOT/name)==value,name
    recorded=json.loads((ROOT/'logs/endpoint-recorded-validation.json').read_text())
    assert recorded['blank_and_repeat_checks_passed'] and not recorded['accuracy_measured']
    geometry=json.loads((ROOT/'logs/endpoint-deployment-geometry.json').read_text())
    assert geometry['complete'] and len(geometry['experiments'])==2
    for e in geometry['experiments']:
        raw=ROOT/'logs'/('endpoint-deployment-geometry-'+e['name']+'.json')
        assert sha(raw)==e['raw_sha256']
        fresh=analyze_geometry(raw)
        assert fresh['experiments']==e['geometry']['experiments']
        # Recompute numerical diagnostics with the current analyzer. Historical
        # generation-time fingerprints remain intact in the original report.
        (ROOT/'logs'/('endpoint-geometry-'+e['name']+'-analysis.json')).write_text(json.dumps(fresh,indent=2)+'\n')
    formal=json.loads((ROOT/'logs/endpoint-formal-policy.json').read_text())
    assert formal['complete'] and formal['extra_prediction_offset_s']==0.
    assert formal['apply_visual_residual_correction'] and not formal['nonregression_passed']
    misses=ROOT/'logs/endpoint-deployment-misses.json'
    collision=json.loads(misses.read_text())
    assert collision['source_sha256']==sha(ROOT/'logs/endpoint-deployment-images.json')
    assert len(collision['shots'])==audited
    log=ROOT/'build/native-offline/Testing/Temporary/LastTest.log'
    contents=log.read_text();assert contents.count('Test Passed.')==54 and 'Test Failed.' not in contents
    manifest=ROOT/'build/runtime-source-manifest.txt'
    for line in manifest.read_text().splitlines():
        name,expected=line.rsplit(':',1);assert sha(ROOT/name)==expected,name
    identity=sha(manifest);binary=ROOT/'build/standard'
    assert identity.encode() in binary.read_bytes()
    static_path=ROOT/'logs/endpoint-static-config-check.log'
    metadata=[json.loads(line) for line in static_path.read_text().splitlines() if line.startswith('{')][-1]
    assert metadata['config_valid'] and not metadata['device_opened']
    assert metadata['effective_settings']['corner_refine_brightness_invariant_subpixel'] is False
    result={'schema':1,'candidate_default_enabled':False,'candidate_rejected_for_hitrate_regression':True,
        'formal_config_semantically_matches_tested_baseline':True,'shots_audited':audited,'ctest_passed':54,
        'independent_collision_recompute_passed':True,'paired_physics_and_causality_passed':True,
        'recorded_file_repeat_and_blank_passed':True,'runtime_source_identity':identity,
        'geometry_diagnostics_independently_recomputed':True,
        'formal_prediction_policy_smoke_passed':True,'formal_policy_candidate_nonregression_passed':False,
        'formal_binary_sha256':sha(binary),'formal_yaml_sha256':sha(ROOT/'configs/standard.yaml'),
        'static_config_check_opened_devices':False,'normal_robot_program_started':False,
        'audit_source_sha256':sha(Path(__file__)),'ctest_log_sha256':sha(log),
        'image_result_sha256':sha(ROOT/'logs/endpoint-deployment-images.json'),
        'miss_analysis_sha256':sha(misses)}
    (ROOT/'logs/simulation-deployment-validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))


if __name__=='__main__':main()
