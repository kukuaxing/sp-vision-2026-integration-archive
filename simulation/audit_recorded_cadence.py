#!/usr/bin/env python3
"""Audit cadence clocks, time margins, physical pairing and independent hits."""
import argparse
import copy
import json
import math
from pathlib import Path
import subprocess
import tempfile
import yaml
from run_fixed_target import ROOT,digest
from study_recorded_cadence import SPLITS,POLICY,metrics
from analyze_fixed_target_misses import analyze


def replay_default():
    """Compatibility replay only, never used to choose a policy."""
    signature={k:digest(ROOT/k) for k in ('logs/endpoint-ablation.json','configs/standard.yaml',
        'build/native-offline/bin/fixed_target_image_sim','simulation/audit_recorded_cadence.py')}
    receipt=ROOT/'logs/recorded-cadence-default-replay.json'
    raw=ROOT/'logs/recorded-cadence-default-replay-shots.json'
    if receipt.exists():
        result=json.loads(receipt.read_text())
        assert result['inputs']==signature and result['trace_sha256']==digest(raw)
        assert result['shots_verified']==100
        return result
    prior=json.loads((ROOT/'logs/endpoint-ablation.json').read_text())
    old=next(e for e in prior['experiments'] if e['name']=='measured_follower_baseline')
    case=copy.deepcopy(old['scenario'])
    case['observation']['preview_directory']='logs/fixed-target-images/recorded-cadence/default-replay'
    with tempfile.TemporaryDirectory(prefix='rm-cadence-default-') as tmp:
        src,out=Path(tmp)/'scenario.json',Path(tmp)/'output.json';src.write_text(json.dumps(case))
        with (ROOT/'logs/recorded-cadence-default-replay.log').open('w') as log:
            subprocess.run([str(ROOT/'build/native-offline/bin/fixed_target_image_sim'),
                str(ROOT/'configs/standard.yaml'),str(src),str(out)],cwd=ROOT,
                stdout=log,stderr=subprocess.STDOUT,check=True,timeout=900)
        current=json.loads(out.read_text())
    assert len(current['runs'])==len(old['runs'])==2
    verified=0
    for a,b in zip(old['runs'],current['runs']):
        assert len(a['shots'])==len(b['shots'])==50
        for x,y in zip(a['shots'],b['shots']):
            for key,value in x.items():
                if isinstance(value,(float,int)) and not isinstance(value,bool):assert abs(value-y[key])<1e-9,key
                else:assert value==y[key],key
            verified+=1
    raw.write_text(json.dumps(current,indent=2)+'\n')
    result={'inputs':signature,'shots_verified':verified,'tolerance':1e-9,
        'trace_sha256':digest(raw),'scope':'historical compatibility only, not new heldout data'}
    receipt.write_text(json.dumps(result,indent=2)+'\n')
    return result


def check_clock(case,r,intervals):
    expected=[0.]
    offset=r['seed']%len(intervals)
    assert r['cadence_interval_offset']==offset
    while expected[-1]<case['sampling']['duration_s']:
        expected.append(expected[-1]+intervals[(offset+len(expected)-1)%len(intervals)])
    events=r['sampling_events']
    assert events and [e['index'] for e in events]==list(range(len(events)))
    previous=0.;busy=0
    for e in events:
        t=expected[e['index']]
        assert abs(t-e['scheduled_s'])<1e-8
        dropped=t<previous-1e-9
        assert e['busy_drop']==dropped
        if dropped:busy+=1
        else:
            assert e['decision_s']>=t and e['decision_s']-t<.2
            previous=e['decision_s']
    assert busy==r['busy_capture_drops']
    accepted={};sequence=0
    for e in events:
        if not e['busy_drop']:
            sequence+=1
            accepted[round(e['scheduled_s'],8)]=(e,sequence)
    # The final sampled event may terminate after pending shots settle, before
    # invoking Tracker/detection. All preceding accepted events are processed.
    assert r['processed_frames'] in (sequence,sequence-1)
    for s in r['shots']:
        assert abs(s['capture_s']-s['scheduled_capture_s'])<1e-9
        event,seq=accepted[round(s['capture_s'],8)]
        assert s['processed_frame_sequence']==seq
        assert abs(event['decision_s']-s['decision_s'])<1e-9
        assert abs(s['next_capture_s']-expected[event['index']+1])<1e-8
    return busy,len(events)


def nonregression(candidate,baseline):
    a,b=candidate['summary'],baseline['summary']
    return (a['valid_hits']>=b['valid_hits'] and a['worst_run_budget_hit_fraction']>=b['worst_run_budget_hit_fraction']
        and a['full_budget_runs']==a['zero_hp_runs']==a['runs'] and a['max_kill_s']<=60
        and all(a['direction_summary'][str(d)]['valid_hits']>=b['direction_summary'][str(d)]['valid_hits'] for d in (-1,1)))


def main():
    inputs=json.loads((ROOT/'data/field/tracker_observations.json').read_text())
    intervals=[b['t']-a['t'] for a,b in zip(inputs,inputs[1:])]
    release=json.loads((ROOT/'logs/simulation-deployment-validation.json').read_text())
    assert digest(ROOT/'configs/standard.yaml')==release['formal_yaml_sha256']
    assert digest(ROOT/'build/standard')==release['formal_binary_sha256']
    cfg=yaml.safe_load((ROOT/'configs/standard.yaml').read_text())
    assert cfg['corner_refine_brightness_invariant_subpixel'] is False
    report={'schema':1,'stages':{},'physical_device_opened':False,'complete':False,
        'cadence_source_sha256':digest(ROOT/'data/field/tracker_observations.json')}
    historical_seeds=set()
    for name in ('centered-opportunity-development.json','centered-opportunity-holdout.json','centered-opportunity-stress.json',
                 'endpoint-deployment-images.json','endpoint-ablation.json','endpoint-formal-policy.json','fixed-target-selective-images.json'):
        d=json.loads((ROOT/'logs'/name).read_text())
        historical_seeds.update(r['seed'] for e in d['experiments'] for r in e['runs'])
    seeds=[s for ss,pp in SPLITS.values() for s in ss]
    assert len(seeds)==len(set(seeds)) and not set(seeds).intersection(historical_seeds)
    stages={};comparisons=[]
    expected_names={'development':{'uniform_baseline','baseline','fixed_6deg','time_margin'},
        'holdout':{'baseline','time_margin'},'stress':{'baseline_jitter_10','time_margin_jitter_10','baseline_jitter_40','time_margin_jitter_40'}}
    for stage in SPLITS:
        path=ROOT/'logs'/('recorded-cadence-'+stage+'.json');data=json.loads(path.read_text());stages[stage]=data
        assert data['complete'] and data['policy']==POLICY
        assert data['physical_device_opened'] is False and data['truth_supplied_to_predictor'] is False
        assert len(data['experiments'])==len(expected_names[stage]) and {e['name'] for e in data['experiments']}==expected_names[stage]
        for k,v in data['sha256'].items():assert digest(ROOT/k)==v,k
        if stage!='development':assert data['development_sha256']==digest(ROOT/'logs/recorded-cadence-development.json')
        summaries=[];events=busy=0
        for e in data['experiments']:
            assert e['summary']==metrics(e)
            case=e['scenario'];ctl=case['controller'];uniform=e['name']=='uniform_baseline'
            assert case['target']['ammo']==50 and case['target']['hp']==300 and case['target']['damage_per_hit']==10
            assert case['sampling']['duration_s']==60 and case['plant']['min_normal_speed_mps']==0
            assert case['observation']['mode']=='rendered_yolo26'
            assert not ctl['center_crossing_schedule'] and not ctl['prefer_smaller_radius']
            assert ctl['prediction_time_offset_s']==ctl['elevation_trim_deg']==0
            assert ctl['directional_prediction_offsets_s']=={'-1':0.,'1':0.} and ctl['apply_visual_residual_correction']
            assert ctl['center_phase_limit_deg']==(6. if e['name']=='fixed_6deg' else 0.)
            margin=POLICY['horizontal_time_margin_s'] if e['name'].startswith('time_margin') else 0.
            assert ctl['horizontal_time_margin_s']==margin
            assert case['sampling']['seeds']==SPLITS[stage][0]
            assert case['target']['initial_phases_rad']==[math.radians(p) for p in SPLITS[stage][1]]
            if uniform:assert 'capture_intervals_s' not in case['sampling']
            else:assert case['sampling']['capture_intervals_s']==intervals
            reference=next(x for x in data['experiments'] if x['name'].startswith('baseline') and x['scenario']['plant']==case['plant'])
            for key in ('target','plant'):assert case[key]==reference['scenario'][key]
            sampling=dict(case['sampling']);sampling.pop('capture_intervals_s',None)
            ref_sampling=dict(reference['scenario']['sampling']);ref_sampling.pop('capture_intervals_s',None)
            assert sampling==ref_sampling
            assert {k:v for k,v in case['observation'].items() if k!='preview_directory'}=={
                k:v for k,v in reference['scenario']['observation'].items() if k!='preview_directory'}
            assert len(e['runs'])==2*len(SPLITS[stage][0])*len(SPLITS[stage][1])
            per_run=[]
            for r,base in zip(e['runs'],reference['runs']):
                assert (r['seed'],r['direction'],r['initial_phase_rad'])==(base['seed'],base['direction'],base['initial_phase_rad'])
                assert r['recorded_cadence']==(not uniform)
                if not uniform:
                    dropped,n=check_clock(case,r,intervals);busy+=dropped;events+=n
                    common=min(len(r['sampling_events']),len(base['sampling_events']))
                    assert r['sampling_events'][:common]==base['sampling_events'][:common]
                else:assert r['sampling_events']==[] and r['busy_capture_drops']==0
                shots=r['shots'];assert len(shots)==r['issued']<=50
                assert r['valid_hits']==sum(s['valid_hit'] for s in shots)
                assert r['geometric_hits']==sum(s['geometric_hit'] for s in shots)
                assert r['budget_hit_fraction']==r['valid_hits']/50 and r['hp']==max(0,300-10*r['valid_hits'])
                hits=sorted(s['settle_s'] for s in shots if s['valid_hit'])
                assert r['kill_s']==(hits[29] if len(hits)>=30 else -1)
                for a,b in zip(shots,shots[1:]):assert b['issue_s']-a['issue_s']>=cfg['sentinel_fire_min_interval_ms']/1000-1e-9
                for s,bs in zip(shots,base['shots']):
                    assert 0<=s['capture_s']<=s['decision_s']+1e-9<=s['issue_s']+2e-9<=s['muzzle_s']+3e-9
                    assert s['issue_s']<60 and s['subframe_wait_s']==0 and s['visual_correction_applied']
                    assert 1<=s['processed_frame_sequence']<=r['processed_frames']
                    assert s['prediction_time_offset_s']==0 and s['horizontal_time_margin_s']==margin
                    if ctl['center_phase_limit_deg']:assert abs(math.degrees(s['estimated_phase_rad']))<=6+1e-9
                    if margin:
                        center=s['estimated_center_world_m'];distance=math.hypot(center[0],center[1])
                        tracking=math.remainder(s['issue_servo_yaw_rad']-s['issue_desired_yaw_rad'],2*math.pi)
                        usable=.135/2-distance*(abs(math.tan(tracking))+math.tan(math.radians(.2)))
                        room=(math.atan(usable/max(s['estimated_radius0_m'],s['estimated_radius1_m']))-
                              abs(math.remainder(s['estimated_phase_rad'],2*math.pi)))/abs(s['estimated_omega_rad_s'])
                        assert room>=margin-1e-10 and abs(room-s['horizontal_time_headroom_s'])<1e-10
                    else:assert s['horizontal_time_headroom_s'] is None
                    assert abs(s['physical_speed_mps']-bs['physical_speed_mps'])<1e-10
                    assert abs((s['muzzle_s']-s['issue_s'])-(bs['muzzle_s']-bs['issue_s']))<1e-10
                    for axis,field in (('yaw','world_yaw'),('elevation','world_elevation')):
                        assert abs((s[field]-s['muzzle_servo_'+axis+'_rad'])-(bs[field]-bs['muzzle_servo_'+axis+'_rad']))<1e-10
                per_run.append({'seed':r['seed'],'direction':r['direction'],'hits':r['valid_hits'],'issued':r['issued'],'kill_s':r['kill_s']})
            summaries.append({'name':e['name'],'summary':e['summary'],'per_run':per_run})
            if e['name'].startswith('time_margin'):
                comparisons.append({'stage':stage,'name':e['name'],'hit_delta':e['summary']['valid_hits']-reference['summary']['valid_hits'],
                    'nonregression_passed':nonregression(e,reference)})
        checked=analyze(data)
        report['stages'][stage]={'experiments':summaries,'independent_collision_shots':len(checked['shots']),
            'miss_axes':checked['experiments'],'sampling_events_verified':events,'busy_drops_verified':busy,
            'result_sha256':digest(path)}
    testout=(ROOT/'logs/recorded-cadence-tests.log').read_text()
    assert '100% tests passed, 0 tests failed out of 54' in testout
    last=ROOT/'build/native-offline/Testing/Temporary/LastTest.log'
    assert last.read_text().count('Test Passed.')==54 and 'Test Failed.' not in last.read_text()
    accepted=all(c['nonregression_passed'] for c in comparisons)
    accepted=accepted and next(c for c in comparisons if c['stage']=='development')['hit_delta']>0
    report.update(complete=True,comparisons=comparisons,offline_candidate_accepted=bool(accepted),deployed=False,
        formal_program_and_config_unchanged=True,ctest_passed=54,ctest_sha256=digest(last),
        auditor_sha256=digest(Path(__file__)),default_replay=replay_default())
    (ROOT/'logs/recorded-cadence-audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='stages'},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--default-replay-only',action='store_true')
    args=parser.parse_args()
    if args.default_replay_only:print(json.dumps(replay_default()))
    else:main()
