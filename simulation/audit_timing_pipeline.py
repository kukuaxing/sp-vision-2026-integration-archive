#!/usr/bin/env python3
"""Audit causal event contracts, helper compatibility and independent collisions."""
import copy
from concurrent.futures import ProcessPoolExecutor
import json
import math
from pathlib import Path
import re
import subprocess
import tempfile
import yaml
from run_fixed_target import ROOT,digest
from analyze_fixed_target_misses import analyze
from audit_delay_observability import equal
from study_timing_pipeline import CASES


def independent_score(path):
    signature=dict(input_sha256=digest(ROOT/path),audit_sha256=digest(Path(__file__)),
        scorer_sha256=digest(ROOT/'simulation/analyze_fixed_target_misses.py'))
    receipt=ROOT/'logs'/(Path(path).stem+'-collision.json')
    if receipt.exists():
        cached=json.loads(receipt.read_text())
        if cached.get('signature')==signature:return cached['result']
    raw=json.loads((ROOT/path).read_text());raw['name']=Path(path).stem
    result=analyze({'experiments':[raw]})
    duplicates=0
    for run in raw['runs']:
        ledger=set()
        for s in result['shots']:
            if s['direction']!=run['direction'] or not s['nearest_crossing']:continue
            hit=s['nearest_crossing'];face=hit['face']
            phase=run['initial_phase_rad']+run['omega_rad_s']*hit['time_s']+face*math.pi/2
            key=(face,math.floor((phase+math.pi)/(2*math.pi)))
            duplicates+=key in ledger;ledger.add(key)
    assert duplicates==0,('duplicate evaluated physical passage',path,duplicates)
    output=dict(path=path,shots=len(result['shots']),misses=result['misses'],duplicate_physical_passages=duplicates)
    receipt.write_text(json.dumps(dict(signature=signature,result=output),indent=2)+'\n')
    return output


def compatibility():
    historical=ROOT/'logs/delay-observability-default-replay-shots.json'
    old=json.loads(historical.read_text());case=copy.deepcopy(old['scenario'])
    assert 'timing_pipeline' not in case
    # The historical wrapper selected this cyclic offset per run; it was not
    # copied into its aggregate scenario. Reconstruct the actual tested clock.
    offsets={r['cadence_interval_offset'] for r in old['runs']}
    assert len(offsets)==1
    case['sampling']['capture_interval_offset']=offsets.pop()
    case['observation']['preview_directory']='logs/fixed-target-images/timing-pipeline/default-compat'
    output=ROOT/'logs/timing-pipeline-default-replay.json'
    receipt=ROOT/'logs/timing-pipeline-default-compat.json'
    signature={p:digest(ROOT/p) for p in ('simulation/src/fixed_target_sim.cpp',
        'simulation/src/fixed_target_pipeline.hpp','configs/standard.yaml',
        'build/native-offline/bin/fixed_target_image_sim',str(historical.relative_to(ROOT)))}
    if receipt.exists():
        r=json.loads(receipt.read_text())
        if r.get('signature')==signature and digest(output)==r.get('trace_sha256'):return r
    with tempfile.TemporaryDirectory(prefix='rm-timing-compat-') as temp:
        source=Path(temp)/'scenario.json';source.write_text(json.dumps(case))
        with (ROOT/'logs/timing-pipeline-default-replay.log').open('w') as log:
            subprocess.run([str(ROOT/'build/native-offline/bin/fixed_target_image_sim'),str(ROOT/'configs/standard.yaml'),
                str(source),str(output)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=1200)
    current=json.loads(output.read_text());checked=0
    for a,b in zip(old['runs'],current['runs']):
        assert 'timing_pipeline' not in b and len(a['shots'])==len(b['shots'])==50
        for x,y in zip(a['shots'],b['shots']):equal(x,y);checked+=1
    assert checked==100
    r=dict(signature=signature,shots_verified=checked,tolerance=1e-9,
        trace_sha256=digest(output),scope='old two-run default compatibility, not new heldout validation')
    receipt.write_text(json.dumps(r,indent=2)+'\n');return r


def main():
    cfg=yaml.safe_load((ROOT/'configs/standard.yaml').read_text())
    summaries=[];jobs=[];events_checked=total_shots=resets=nonmonotonic=qualified=0
    for kind in ('keypoints','images'):
        manifest_path=ROOT/'logs'/('timing-pipeline-'+kind+'.json')
        manifest=json.loads(manifest_path.read_text())
        assert manifest['complete'] and manifest['declared_cases']==CASES and not manifest['physical_device_opened']
        assert all(digest(ROOT/path)==value for path,value in manifest['sha256'].items())
        expected=set(CASES) if kind=='keypoints' else {'clean','image_120ms','qualified_reference_change','qualified_image_expiry','back_stamp_80ms'}
        assert {e['name'] for e in manifest['experiments']}==expected
        for entry in manifest['experiments']:
            assert digest(ROOT/entry['raw'])==entry['sha256'];jobs.append(entry['raw'])
            raw=json.loads((ROOT/entry['raw']).read_text());case=raw['scenario']
            assert case['timing_pipeline']['faults']==CASES[entry['name']]
            assert case['target']['ammo']==50 and case['target']['hp']==300 and case['sampling']['duration_s']==60
            assert case['sampling']['seeds']==[141001] and case['sampling']['capture_intervals_s']==[.02]
            assert case['controller']['apply_visual_residual_correction']
            assert case['controller']['directional_prediction_offsets_s']=={'-1':0.,'1':0.}
            for r,summary in zip(raw['runs'],entry['runs']):
                events=r['timing_pipeline']['events'];fire=[e for e in events if e['fire']]
                assert len(r['shots'])==r['issued']==50 and len(fire)==50
                assert r['valid_hits']==sum(s['valid_hit'] for s in r['shots'])==summary['hits']
                assert r['hp']==max(0,300-10*r['valid_hits'])
                ledger=set();previous=None
                for e in events:
                    assert e['capture_s']<=e['decision_s']+1e-9<=e['tx_check_s']+2e-9
                    expected_fresh=-1e-9<=e['tx_check_s']-e['observation_s']<=.100000001
                    assert e['image_fresh']==expected_fresh
                    if not e['capture_valid'] or not e['capture_context'] or not e['decision_pose_valid']:
                        assert not e['measurement_updated'] and not e['fire']
                    if e['decision_generation']!=e['transmit_generation'] or not e['transmit_pose_valid'] or not e['image_fresh']:
                        assert not e['fire']
                    if e['reset']:
                        resets+=1;assert e['residual_cycles']==0 and abs(e['correction_rad'])<1e-12
                    if previous and e['observation_s']<=previous['observation_s']+1e-10 and previous['fixed_ready'] and e['track_id']==previous['track_id']:
                        assert not e['measurement_updated'] and not e['fire'];nonmonotonic+=1
                    if e['qualified_fault']:
                        assert e['prior_gate'] and not e['final_allowed'] and not e['fire'];qualified+=1
                        if entry['name']=='qualified_reference_change':
                            assert e['decision_generation']!=e['transmit_generation'] and e['pose_reject']==2
                        elif entry['name']=='qualified_image_expiry':assert not e['image_fresh']
                        else:raise AssertionError('undeclared qualified fault')
                    if e['fire']:
                        assert e['prior_gate'] and e['final_allowed'] and e['pose_reject']==0 and e['image_fresh']
                        # Passage counters may legitimately rebase on a model
                        # reset. Their integers are meaningful only in-context.
                        key=(e['transmit_generation'],e['track_id'],e['residual_generation'],e['selected_face'],e['passage'])
                        assert key not in ledger,('repeated model passage',kind,entry['name'],key)
                        ledger.add(key)
                    previous=e;events_checked+=1
                if entry['name'].startswith('qualified_'):assert sum(e['qualified_fault'] for e in events)==1
                for e,s in zip(fire,r['shots']):
                    assert abs(e['tx_check_s']-s['issue_s'])<1e-9
                    assert e['transmit_generation']==s['pipeline_generation']
                    assert 0<=s['capture_s']<=s['decision_s']<=s['issue_s']<=s['muzzle_s'] and s['issue_s']<60
                    assert abs(s['capture_s']-s['scheduled_capture_s'])<1e-9
                for a,b in zip(r['shots'],r['shots'][1:]):
                    assert b['issue_s']-a['issue_s']>=cfg['sentinel_fire_min_interval_ms']/1000-1e-9
                total_shots+=len(r['shots'])
            summaries.append(dict(kind=kind,name=entry['name'],runs=entry['runs']))
    with ProcessPoolExecutor(max_workers=2) as pool:score=list(pool.map(independent_score,jobs))
    assert sum(r['shots'] for r in score)==total_shots
    compat=compatibility()
    log=(ROOT/'logs/timing-pipeline-ctest.log').read_text()
    match=re.search(r'100% tests passed, 0 tests failed out of (\d+)',log);assert match
    assert int(match[1])==55
    assert digest(ROOT/'configs/standard.yaml')=='e087e02c2c6ad257acef5a0808950a42f15445c4ed00714807023dad7427fd90'
    assert digest(ROOT/'build/standard')=='dbc3d740d18255383af4f211b9bb8b740bbee0c65d31b0d18c8a757f378d5cfe'
    hygiene=subprocess.run(['python3','scripts/check_source_tree.py'],cwd=ROOT,capture_output=True,text=True)
    assert hygiene.returncode==0,hygiene.stdout+hygiene.stderr
    result=dict(scope='diagnostic helper-chain injection, not new controller gain or complete standard/MCU test',
        physical_device_opened=False,formal_program_and_config_unchanged=True,
        events_checked=events_checked,shots_independently_scored=total_shots,nonmonotonic_rejections_checked=nonmonotonic,
        resets_checked=resets,qualified_wait_faults_checked=qualified,duplicate_model_passages=0,
        summaries=summaries,collision_checks=score,default_compatibility=compat,ctest_passed=55,
        source_tree=hygiene.stdout.strip(),audit_sha256=digest(Path(__file__)))
    (ROOT/'logs/timing-pipeline-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('summaries','collision_checks','default_compatibility')},indent=2))


if __name__=='__main__':main()
