#!/usr/bin/env python3
"""Independent metadata replay decision and paired trajectory/ballistic audit."""
import copy
from concurrent.futures import ProcessPoolExecutor
import json
import math
from pathlib import Path
import re
import subprocess
import tempfile
from run_fixed_target import ROOT,digest
from audit_delay_observability import equal
from audit_timing_pipeline import independent_score
from study_frame_admission import CASES,SPLITS


def metadata_audit(events):
    seen=[];reference=None;last_now=None;skipped=0
    for e in events:
        now=round((e['decision_s']+1)*1e9);capture=round((e['observation_s']+1)*1e9)
        pair=(e['frame_id'],capture);ref=(e['generation'],e['boot'],e['epoch'])
        eligible=e['reference_valid'] and pair[0]!=0 and ref[0]!=0 and ref[1]!=0 and 0<capture<=now and now-capture<=100000000
        rejected=False
        if not eligible:seen=[];reference=None;last_now=None
        else:
            if ref!=reference or (last_now is not None and now<last_now):seen=[]
            reference=ref;last_now=now
            seen=[p for p in seen if now-p[1]<=100000000]
            rejected=pair in seen
            if not rejected:
                if seen and (pair[0]<=seen[-1][0] or pair[1]<=seen[-1][1]):seen=[]
                seen.append(pair);seen=seen[-64:]
        if not e['candidate']:rejected=False;seen=[];reference=None;last_now=None
        assert rejected==e['rejected'],e
        skipped+=rejected
    return skipped


def compatibility():
    source=ROOT/'logs/timing-pipeline-default-replay.json';old=json.loads(source.read_text())
    scenario=copy.deepcopy(old['scenario']);assert 'timing_pipeline' not in scenario
    scenario['observation']['preview_directory']='logs/fixed-target-images/frame-admission/default-compat'
    output=ROOT/'logs/frame-admission-default-replay.json';receipt=ROOT/'logs/frame-admission-default-compat.json'
    signature={str(source.relative_to(ROOT)):digest(source),
        'build/native-offline/bin/fixed_target_image_sim':digest(ROOT/'build/native-offline/bin/fixed_target_image_sim')}
    if receipt.exists():
        r=json.loads(receipt.read_text())
        if r.get('signature')==signature and r['output_sha256']==digest(output):return r
    with tempfile.TemporaryDirectory(prefix='rm-admission-compat-') as tmp:
        path=Path(tmp)/'scenario.json';path.write_text(json.dumps(scenario))
        with (ROOT/'logs/frame-admission-default-replay.log').open('w') as log:
            subprocess.run([str(ROOT/'build/native-offline/bin/fixed_target_image_sim'),str(ROOT/'configs/standard.yaml'),
                str(path),str(output)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=1200)
    current=json.loads(output.read_text());checked=0
    for a,b in zip(old['runs'],current['runs']):
        assert len(a['shots'])==len(b['shots'])==50
        for x,y in zip(a['shots'],b['shots']):equal(x,y);checked+=1
    assert checked==100
    result=dict(signature=signature,shots=checked,tolerance=1e-9,output_sha256=digest(output))
    receipt.write_text(json.dumps(result,indent=2)+'\n');return result


def recorded_metadata():
    files=[];rows=valid=duplicates=backwards=counter_only=damaged=0
    for path in sorted((ROOT/'logs/diagnostics').glob('vision-*.jsonl')):
        prior=None;seen=set();count=0
        for line in path.read_text().splitlines():
            if not line.strip():continue
            try:r=json.loads(line)
            except json.JSONDecodeError:damaged+=1;continue
            rows+=1;count+=1
            ident=r.get('capture_frame');t=r.get('time_s');pose=r.get('pose',{})
            if not isinstance(ident,int) or ident<=0 or not isinstance(t,(float,int)) or not math.isfinite(t):continue
            valid+=1;generation=pose.get('capture_generation')
            key=(generation,ident,t);duplicates+=key in seen;seen.add(key)
            if prior and prior[0]==generation:
                backwards+=t<=prior[2]
                counter_only+=ident==prior[1] and t!=prior[2]
            prior=(generation,ident,t)
        files.append(dict(path=str(path.relative_to(ROOT)),sha256=digest(path),rows=count))
    return dict(scope='existing sampled diagnostic rows, not all camera frames or proof of complete reference identity',
        files=files,rows=rows,rows_with_nonzero_frame_id=valid,exact_metadata_recurrences=duplicates,
        nonmonotonic_logged_times=backwards,repeated_id_with_changed_time=counter_only,damaged_lines=damaged,
        physical_device_opened=False,labels_independent=False)


def main():
    summary=[];jobs=[];metadata_count=skipped_count=identical_shots=0;holdout_nonregression=True
    for stage in SPLITS:
        manifest=json.loads((ROOT/'logs'/('frame-admission-'+stage+'.json')).read_text())
        assert manifest['complete'] and manifest['cases']==CASES and manifest['selected_before_results']
        assert not manifest['physical_device_opened']
        assert all(digest(ROOT/p)==value for p,value in manifest['sha256'].items())
        entries={(e['name'],e['candidate']):e for e in manifest['experiments']}
        names=tuple(CASES) if stage=='development' else ('clean','exact_duplicate','exact_reorder','new_frame_duplicate_stamp')
        assert set(entries)=={(n,c) for n in names for c in (False,True)}
        for name in names:
            pair=[]
            for candidate in (False,True):
                entry=entries[name,candidate];assert digest(ROOT/entry['raw'])==entry['sha256']
                jobs.append(entry['raw']);raw=json.loads((ROOT/entry['raw']).read_text());pair.append(raw)
                assert raw['scenario']['timing_pipeline']['faults']==CASES[name]
                assert raw['scenario']['controller']['reject_verified_replay']==candidate
                for r,s in zip(raw['runs'],entry['runs']):
                    assert r['issued']==len(r['shots'])==50 and r['hp']==0
                    events=r['timing_pipeline']['events'];admissions=r['timing_pipeline']['admission_events']
                    skipped=metadata_audit(admissions);assert skipped==s['skipped']
                    metadata_count+=len(admissions);skipped_count+=skipped
                    processed={round(e['delivery_s'],8):e for e in events}
                    for a in admissions:
                        if a['rejected']:
                            assert round(a['delivery_s'],8) not in processed
                            assert 0<=a['decision_s']-a['observation_s']<=.100000001
                    assert sum(e['fire'] for e in events)==50
                    ledger=set();previous=None
                    for e in events:
                        if e['reset']:assert e['residual_cycles']==0 and abs(e['correction_rad'])<1e-12
                        if e['qualified_fault']:assert not e['fire'] and not e['final_allowed']
                        if e['fire']:
                            assert e['capture_valid'] and e['capture_context'] and e['image_fresh'] and e['final_allowed']
                            key=tuple(e[k] for k in ('transmit_generation','track_id','residual_generation','selected_face','passage'))
                            assert key not in ledger;ledger.add(key)
                        previous=e
                    assert r['valid_hits']==s['hits']==sum(x['valid_hit'] for x in r['shots'])
            base,new=pair
            # New-frame clock faults and no-replay controls must be bit-for-bit
            # identical in shot state. A replay optimization is not clock repair.
            unchanged=name in ('clean','new_frame_duplicate_stamp','new_frame_back_stamp',
                'qualified_reference_change','qualified_image_expiry')
            for a,b in zip(base['runs'],new['runs']):
                assert a['direction']==b['direction']
                for x,y in zip(a['shots'],b['shots']):
                    if unchanged:equal(x,y);identical_shots+=1
                    assert abs((x['muzzle_s']-x['issue_s'])-(y['muzzle_s']-y['issue_s']))<1e-10
                    assert abs(x['physical_speed_mps']-y['physical_speed_mps'])<1e-10
                if stage=='holdout' and b['valid_hits']<a['valid_hits']:holdout_nonregression=False
            summary.append(dict(stage=stage,name=name,baseline=entries[name,False]['runs'],candidate=entries[name,True]['runs']))
    with ProcessPoolExecutor(max_workers=2) as pool:collisions=list(pool.map(independent_score,jobs))
    compat=compatibility()
    log=(ROOT/'logs/frame-admission-ctest.log').read_text()
    assert '100% tests passed, 0 tests failed out of 56' in log
    assert digest(ROOT/'configs/standard.yaml')=='e087e02c2c6ad257acef5a0808950a42f15445c4ed00714807023dad7427fd90'
    assert digest(ROOT/'build/standard')=='dbc3d740d18255383af4f211b9bb8b740bbee0c65d31b0d18c8a757f378d5cfe'
    check=subprocess.run(['python3','scripts/check_source_tree.py'],cwd=ROOT,capture_output=True,text=True)
    assert check.returncode==0,check.stdout+check.stderr
    result=dict(scope='exact-frame metadata replay candidate only, not a timestamp repair or full standard/UART test',
        physical_device_opened=False,formal_binary_and_config_unchanged=True,
        metadata_events_audited=metadata_count,rejected_replays=skipped_count,
        unchanged_control_shots=identical_shots,shots_independently_scored=sum(r['shots'] for r in collisions),
        summaries=summary,collision_checks=collisions,default_compatibility=compat,ctest_passed=56,
        image_holdout_per_run_hit_nonregression=holdout_nonregression,
        deployed=False,decision='offline_candidate_only; UART heartbeat/complete loop still unvalidated',
        recorded_metadata=recorded_metadata(),
        audit_sha256=digest(Path(__file__)),source_tree=check.stdout.strip())
    (ROOT/'logs/frame-admission-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('summaries','collision_checks','default_compatibility')},indent=2))


if __name__=='__main__':main()
