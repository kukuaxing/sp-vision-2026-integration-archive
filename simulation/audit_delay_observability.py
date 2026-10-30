#!/usr/bin/env python3
"""Truth-only diagnostic audit; never supplies a delay estimate to the controller."""
import copy
from concurrent.futures import ProcessPoolExecutor
import json
import math
from pathlib import Path
import statistics
import subprocess
import tempfile
from run_fixed_target import ROOT,digest
from study_recorded_cadence import run_case
from analyze_fixed_target_misses import crossings,analyze


def stats(values):
    x=sorted(values)
    if not x:return None
    return {'n':len(x),'mean':statistics.mean(x),'median':statistics.median(x),
        'std_population':statistics.pstdev(x),'p05':x[int(.05*(len(x)-1))],
        'p95':x[int(.95*(len(x)-1))],'min':x[0],'max':x[-1]}


def equal(a,b):
    if isinstance(a,bool) or a is None or isinstance(a,str):assert a==b
    elif isinstance(a,(float,int)):assert math.isfinite(a) and abs(a-b)<1e-9,(a,b)
    elif isinstance(a,list):
        assert len(a)==len(b)
        for x,y in zip(a,b):equal(x,y)
    elif isinstance(a,dict):
        assert set(a)==set(b)
        for k in a:equal(a[k],b[k])
    else:raise TypeError(type(a))


def score_grid_row(job):
    e,row=job;case=e['scenario'];plant=case['plant']
    assert plant['min_normal_speed_mps']==0
    per_run=[];hits=total=lost=gained=0;directions={-1:0,1:0};worst=1.
    for r in e['runs']:
        count=0
        for original in r['shots']:
            s=dict(original)
            z=(s['muzzle_s']-s['issue_s']-plant['muzzle_delay_s'])/plant['muzzle_delay_jitter_s']
            s['muzzle_s']=s['issue_s']+max(0.,plant['muzzle_delay_s']+row['physical_mean_shift_ms']*.001+z*row['physical_sigma_ms']*.001)
            cross=crossings(case,r,s)
            hit=bool(cross and cross['normalized_edge_distance']<=1+1e-8)
            count+=hit;total+=1;lost+=original['valid_hit'] and not hit;gained+=not original['valid_hit'] and hit
        hits+=count;directions[r['direction']]+=count;worst=min(worst,count/r['ammo_budget'])
        per_run.append({'seed':r['seed'],'direction':r['direction'],'initial_phase_rad':r['initial_phase_rad'],'hits':count})
    assert (hits,total,lost,gained,directions[-1],directions[1],worst)==(
        row['hits'],row['shots'],row['lost'],row['gained'],row['negative_hits'],row['positive_hits'],row['worst_fraction'])
    assert per_run==row['runs']
    return total


def compact(e):
    keys=('world_yaw','world_elevation','physical_speed_mps','muzzle_s','issue_s','valid_hit')
    return {'name':e['name'],'scenario':{'target':e['scenario']['target'],'plant':e['scenario']['plant']},
        'runs':[{**{k:r[k] for k in ('seed','direction','initial_phase_rad','omega_rad_s','ammo_budget')},
                 'shots':[{k:s[k] for k in keys} for s in r['shots']]} for r in e['runs']]}


def compatibility():
    receipt=ROOT/'logs/delay-observability-default-replay.json'
    raw=ROOT/'logs/delay-observability-default-replay-shots.json'
    names=('logs/recorded-cadence-stress.json','build/native-offline/bin/fixed_target_image_sim',
        'simulation/audit_delay_observability.py','simulation/study_recorded_cadence.py','configs/standard.yaml')
    inputs={k:digest(ROOT/k) for k in names}
    if receipt.exists():
        r=json.loads(receipt.read_text());assert r['inputs']==inputs and r['trace_sha256']==digest(raw)
        assert r['shots_verified']==100
        return r
    old=json.loads((ROOT/'logs/recorded-cadence-stress.json').read_text())
    old=next(e for e in old['experiments'] if e['name']=='baseline_jitter_10')
    case=copy.deepcopy(old['scenario']);case['sampling']['seeds']=[91001]
    case['observation']['preview_directory']='logs/fixed-target-images/delay-observability/default-replay'
    current=run_case(case,'finite_response_default','delay-observability')
    verified=0
    for r in current['runs']:
        prior=next(x for x in old['runs'] if (x['seed'],x['direction'])==(r['seed'],r['direction']))
        assert len(r['shots'])==len(prior['shots'])==50
        for a,b in zip(prior['shots'],r['shots']):
            equal(a,{k:b[k] for k in a});verified+=1
    raw.write_text(json.dumps(current,indent=2)+'\n')
    result={'inputs':inputs,'shots_verified':verified,'tolerance':1e-9,'trace_sha256':digest(raw),
        'scope':'historical finite-response compatibility, not new heldout validation'}
    receipt.write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    twins_path=ROOT/'logs/delay-observability-twins.json';twins=json.loads(twins_path.read_text())
    assert twins['complete'] and len(twins['experiments'])==2
    for k,v in twins['sha256'].items():assert digest(ROOT/k)==v,k
    a,b=twins['experiments'];assert [a['name'],b['name']]==['sigma_10ms','sigma_40ms']
    assert a['scenario']['sampling']==b['scenario']['sampling']
    assert a['scenario']['target']==b['scenario']['target']
    assert a['scenario']['controller']==b['scenario']['controller']
    pa,pb=dict(a['scenario']['plant']),dict(b['scenario']['plant'])
    assert pa.pop('muzzle_delay_jitter_s')==.01 and pb.pop('muzzle_delay_jitter_s')==.04 and pa==pb
    assert pa['servo_time_constant_s']==0 and twins['physical_device_opened'] is False
    assert twins['truth_supplied_to_predictor'] is False
    host_keys=('id','capture_s','decision_s','issue_s','subframe_wait_s','scheduled_capture_s','next_capture_s',
        'processed_frame_sequence','model_impact_s','selected_model_face','estimated_phase_rad','estimated_omega_rad_s',
        'estimated_center_world_m','estimated_radius0_m','estimated_radius1_m','issue_desired_yaw_rad',
        'issue_desired_elevation_rad','issue_servo_yaw_rad','issue_servo_elevation_rad','visual_residual',
        'model_point_world_m','model_aim_yaw_rad','visual_residual_correction_rad')
    comparison=[];flip_count=0;visual=[]
    for x,y in zip(a['runs'],b['runs']):
        assert (x['seed'],x['direction'])==(y['seed'],y['direction'])
        assert len(x['shots'])==len(y['shots'])==50
        common=min(len(x['frames']),len(y['frames']))
        for f,g in zip(x['frames'][:common],y['frames'][:common]):
            equal({k:v for k,v in f.items() if k!='hp'},{k:v for k,v in g.items() if k!='hp'})
        flips=0
        for s,t in zip(x['shots'],y['shots']):
            equal({k:s[k] for k in host_keys},{k:t[k] for k in host_keys})
            assert abs((s['muzzle_s']-s['issue_s']-.34)/.01-(t['muzzle_s']-t['issue_s']-.34)/.04)<1e-9
            assert s['physical_speed_mps']==t['physical_speed_mps']
            flips+=s['valid_hit']!=t['valid_hit'];visual.append(s['visual_residual'])
        flip_count+=flips
        comparison.append({'seed':x['seed'],'direction':x['direction'],'common_visual_frames':common,
            'shots_with_identical_host_state':50,'sigma10_hits':x['valid_hits'],'sigma40_hits':y['valid_hits'],'hit_outcomes_changed':flips})
    assert flip_count>0
    truth_check=analyze(twins);assert len(truth_check['shots'])==200
    report={'schema':1,'scope':'diagnostic, already-seen development data; no controller improvement claimed',
        'physical_device_opened':False,'truth_available_only_to_evaluator':True,'complete':False,
        'twins':comparison,'visual_metrics_degrees':{
            field:stats([math.degrees(q[field]) for q in visual if q[field] is not None])
            for field in ('bias_rad','scatter_rad','past_error_bound_rad')},'grid_sources':{},'timing_decomposition':[]}
    jobs=[];dependencies=set(twins['sha256'])|{'simulation/audit_delay_observability.py',
        'simulation/analyze_fixed_target_misses.py','simulation/src/fixed_target_counterfactual.cpp',
        'build/native-offline/bin/fixed_target_counterfactual',str(twins_path.relative_to(ROOT))}
    for part in ('holdout','stress'):
        source=ROOT/'logs'/('recorded-cadence-'+part+'.json');data=json.loads(source.read_text());assert data['complete']
        grid_path=ROOT/'logs'/('delay-budget-'+part+'.json');grid=json.loads(grid_path.read_text())
        assert len(grid['rows'])==20*len(data['experiments'])
        assert grid['zero_intervention_shots_verified']==sum(r['issued'] for e in data['experiments'] for r in e['runs'])
        with tempfile.TemporaryDirectory(prefix='rm-delay-grid-audit-') as tmp:
            out=Path(tmp)/'grid.json'
            subprocess.run([str(ROOT/'build/native-offline/bin/fixed_target_counterfactual'),str(source),str(out),'--delay-grid'],
                cwd=ROOT,check=True,timeout=180,stdout=subprocess.DEVNULL)
            assert json.loads(out.read_text())==grid
        compressed={e['name']:compact(e) for e in data['experiments']}
        jobs.extend((compressed[row['experiment']],row) for row in grid['rows'])
        report['grid_sources'][part]={'input_sha256':digest(source),'grid_sha256':digest(grid_path),
            'zero_intervention_shots':grid['zero_intervention_shots_verified'],'cells':len(grid['rows'])}
        dependencies.update((str(source.relative_to(ROOT)),str(grid_path.relative_to(ROOT))))
        for e in data['experiments']:
            # Independent truth only. A quarter-turn gauge does not identify
            # a physical face; this phase diagnostic is not online residual.
            phase_errors=[];delay_errors=[];arrival_errors=[];remainder_errors=[]
            nominal_delay=.330
            for r in e['runs']:
                for s in r['shots']:
                    c,p=s['estimated_center_world_m'],s['model_point_world_m']
                    normal=math.atan2(c[1]-p[1],c[0]-p[0])
                    err=math.remainder(normal-r['initial_phase_rad']-r['omega_rad_s']*s['model_impact_s'],math.pi/2)
                    phase_errors.append(1000*err/r['omega_rad_s'])
                    draw=s['muzzle_s']-s['issue_s']-e['scenario']['plant']['muzzle_delay_s']
                    delay_errors.append(1000*draw)
                    cross=crossings(e['scenario'],r,s)
                    if cross:
                        arrival=cross['time_s']-s['model_impact_s']
                        fixed=e['scenario']['plant']['muzzle_delay_s']-nominal_delay
                        residual=(cross['time_s']-s['muzzle_s'])-(s['model_impact_s']-s['issue_s']-nominal_delay)
                        assert abs(arrival-draw-fixed-residual)<1e-9
                        arrival_errors.append(arrival*1000);remainder_errors.append(residual*1000)
            report['timing_decomposition'].append({'source':part,'experiment':e['name'],
                'truth_model_phase_equivalent_ms':stats(phase_errors),'actual_delay_random_component_ms':stats(delay_errors),
                'fixed_true_mean_minus_nominal_ms':10.,'arrival_minus_model_ms':stats(arrival_errors),
                'remaining_flight_and_prediction_timebase_ms':stats(remainder_errors)})
    # Check every grid cell, including aggregate/per-run counters, in an
    # independent NumPy geometry implementation. No row selected for its score.
    with ProcessPoolExecutor(max_workers=2) as pool:
        independent=sum(pool.map(score_grid_row,jobs))
    report['independent_grid_shot_evaluations']=independent
    assert independent==40000
    # File-only malformed-input checks for the new evaluator branch.
    fixture=json.loads((ROOT/'logs/recorded-cadence-holdout.json').read_text())
    fixture['experiments']=fixture['experiments'][:1];fixture['experiments'][0]['runs']=fixture['experiments'][0]['runs'][:1]
    fixture['experiments'][0]['runs'][0]['shots']=fixture['experiments'][0]['runs'][0]['shots'][:1]
    with tempfile.TemporaryDirectory(prefix='rm-delay-grid-invalid-') as tmp:
        src,out=Path(tmp)/'source.json',Path(tmp)/'output.json'
        for invalid in ('incomplete','clipped_delay'):
            f=copy.deepcopy(fixture)
            if invalid=='incomplete':f['complete']=False
            else:
                shot=f['experiments'][0]['runs'][0]['shots'][0];shot['muzzle_s']=shot['issue_s']
            src.write_text(json.dumps(f))
            run=subprocess.run([str(ROOT/'build/native-offline/bin/fixed_target_counterfactual'),str(src),str(out),'--delay-grid'],
                capture_output=True,text=True,timeout=15)
            assert run.returncode!=0
        src.write_text(json.dumps(fixture))
        subprocess.run([str(ROOT/'build/native-offline/bin/fixed_target_counterfactual'),str(src),str(out)],
            cwd=ROOT,check=True,timeout=30,stdout=subprocess.DEVNULL)
        assert json.loads(out.read_text())['zero_intervention_shots_verified']==1
    release=json.loads((ROOT/'logs/simulation-deployment-validation.json').read_text())
    assert digest(ROOT/'configs/standard.yaml')==release['formal_yaml_sha256']
    assert digest(ROOT/'build/standard')==release['formal_binary_sha256']
    text=(ROOT/'logs/delay-observability-tests.log').read_text()
    assert '100% tests passed, 0 tests failed out of 54' in text
    last=ROOT/'build/native-offline/Testing/Temporary/LastTest.log'
    assert last.read_text().count('Test Passed.')==54
    report.update(complete=True,default_compatibility=compatibility(),formal_program_and_config_unchanged=True,
        deployed=False,ctest_passed=54,ctest_sha256=digest(last),
        sha256={k:digest(ROOT/k) for k in sorted(dependencies)})
    (ROOT/'logs/delay-observability-audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('sha256','timing_decomposition')},indent=2))


if __name__=='__main__':main()
