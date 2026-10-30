#!/usr/bin/env python3
"""Recorded-interval sensitivity + fixed estimated time-headroom policy, file-only."""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import math
from pathlib import Path
import subprocess
import tempfile
import yaml
from run_fixed_target import ROOT, digest, summarize

# Fixed before looking at this study's results. No fitting to development hits.
SPLITS = {'development': ([70001], [37.5]),
          'holdout': ([90001,90007,90011,90017,90019,90023], [12.5]),
          'stress': ([91001,91007], [67.5])}
POLICY = {'horizontal_time_margin_s': .020, 'angular_reserve_deg': .2,
          'assumed_armor_width_m': .135, 'estimated_radius': 'max of both fitted radii',
          'selected_before_development': True, 'reads_actual_muzzle_delay': False}


def metrics(e):
    s = summarize(e)
    s.update(last_issue_max_s=max((r['shots'][-1]['issue_s'] for r in e['runs'] if r['shots']), default=None),
        busy_capture_drops=sum(r['busy_capture_drops'] for r in e['runs']),
        phase_rejected_frames=sum(r['phase_rejected_frames'] for r in e['runs']),
        margin_rejected_frames=sum(r['margin_rejected_frames'] for r in e['runs']))
    return s


def run_case(case, name, stage):
    # Each independent run gets its own temporary scene/log. Two subprocesses
    # max; aggregation restores declared order, not completion order.
    keys = [(seed,direction,phase) for seed in case['sampling']['seeds']
            for direction in case['target']['directions'] for phase in case['target']['initial_phases_rad']]
    executable = ROOT/'build/native-offline/bin/fixed_target_image_sim'
    def one(index, key):
        seed,direction,phase = key
        sc = copy.deepcopy(case)
        sc['sampling'].update(seeds=[seed], capture_interval_offset=seed%len(sc['sampling'].get('capture_intervals_s',[]) or [0]))
        sc['target'].update(directions=[direction], initial_phases_rad=[phase])
        with tempfile.TemporaryDirectory(prefix='rm-recorded-cadence-') as tmp:
            src,out = Path(tmp)/'scenario.json',Path(tmp)/'output.json'
            src.write_text(json.dumps(sc))
            logpath = ROOT/'logs'/('recorded-cadence-'+stage+'-'+name+'-run'+str(index)+'.log')
            with logpath.open('w') as log:
                subprocess.run([str(executable),str(ROOT/'configs/standard.yaml'),str(src),str(out)],
                    cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=1200)
            data=json.loads(out.read_text())
        assert len(data['runs'])==1
        data['runs'][0]['cadence_interval_offset']=sc['sampling']['capture_interval_offset']
        return index,data
    finished={}
    with ThreadPoolExecutor(max_workers=2) as pool:
        futures=[pool.submit(one,i,key) for i,key in enumerate(keys)]
        for future in as_completed(futures):
            i,value=future.result();finished[i]=value
            print(stage,name,'runs',len(finished),'/',len(keys),flush=True)
    result=copy.deepcopy(finished[0])
    result.update(name=name,scenario=case,runs=[finished[i]['runs'][0] for i in range(len(keys))])
    result['summary']=metrics(result)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage',choices=SPLITS,required=True);args=parser.parse_args()
    stage=args.stage
    source=ROOT/'logs/endpoint-deployment-images.json'
    historical=ROOT/'data/field/tracker_observations.json'
    rows=json.loads(historical.read_text());times=[r['t'] for r in rows]
    intervals=[b-a for a,b in zip(times,times[1:])]
    assert len(intervals)>100 and all(math.isfinite(x) and .001<=x<=.5 for x in intervals)
    config=yaml.safe_load((ROOT/'configs/standard.yaml').read_text())
    assert config['corner_refine_brightness_invariant_subpixel'] is False
    prior=json.loads(source.read_text());assert prior['complete']
    base=copy.deepcopy(prior['experiments'][0]['scenario'])
    seeds,phases=SPLITS[stage]
    base['sampling'].update(seeds=seeds,capture_intervals_s=intervals)
    base['target']['initial_phases_rad']=[math.radians(x) for x in phases]
    base['controller'].update(center_crossing_schedule=False,center_phase_limit_deg=0.,
        horizontal_time_margin_s=0.,directional_prediction_offsets_s={'-1':0.,'1':0.},
        prediction_time_offset_s=0.,elevation_trim_deg=0.,prefer_smaller_radius=False,
        apply_visual_residual_correction=True)
    deps={'simulation/study_recorded_cadence.py','simulation/run_fixed_target.py',
        'tools/analyze_observation_timing.py','configs/standard.yaml','build/standard',
        'build/native-offline/bin/fixed_target_image_sim','assets/yolo26.xml','assets/yolo26.bin',
        str(source.relative_to(ROOT)),str(historical.relative_to(ROOT)),'tests/offline/CMakeLists.txt'}
    for folder in ('simulation/src','tasks/auto_aim','tools'):
        deps.update(str(p.relative_to(ROOT)) for p in (ROOT/folder).rglob('*') if p.suffix in ('.cpp','.hpp','.h'))
    result={'schema':1,'study':'recorded_cadence','stage':stage,'complete':False,'splits':SPLITS,
        'physical_device_opened':False,'truth_supplied_to_predictor':False,'policy':POLICY,
        'scope':'historical OBSERVATION interval replay as synthetic capture cadence; NOT measured capture age; actual YOLO26; not full standard/MCU',
        'cadence':{'frames':len(times),'source_duration_s':times[-1]-times[0],
            'source_sha256':digest(historical),'offset_rule':'seed modulo interval count; cyclic continuation',
            'processing_age_measured':False,'muzzle_delay_measured':False},
        'sha256':{k:digest(ROOT/k) for k in sorted(deps)},'experiments':[]}
    if stage!='development':
        dev=ROOT/'logs/recorded-cadence-development.json';d=json.loads(dev.read_text())
        assert d['complete'] and d['policy']==POLICY
        for k,v in d['sha256'].items():assert digest(ROOT/k)==v,k
        result['development_sha256']=digest(dev)
    policies=[('baseline',0.,0.)]
    if stage=='development':policies+=[('fixed_6deg',6.,0.)]
    policies+=[('time_margin',0.,POLICY['horizontal_time_margin_s'])]
    definitions=[(name,limit,margin,.025,True) for name,limit,margin in policies]
    if stage=='development':definitions.insert(0,('uniform_baseline',0.,0.,.025,False))
    if stage=='stress':definitions=[(name+'_jitter_'+str(round(jitter*1000)),limit,margin,jitter,True)
        for jitter in (.010,.040) for name,limit,margin in policies]
    for name,limit,margin,jitter,recorded in definitions:
        case=copy.deepcopy(base)
        if not recorded:case['sampling'].pop('capture_intervals_s')
        case['controller'].update(center_phase_limit_deg=limit,horizontal_time_margin_s=margin)
        case['plant']['muzzle_delay_jitter_s']=jitter
        case['observation']['preview_directory']='logs/fixed-target-images/recorded-cadence/'+stage+'/'+name
        e=run_case(case,name,stage)
        result['experiments'].append(e)
        (ROOT/'logs'/('recorded-cadence-'+stage+'.json')).write_text(json.dumps(result,indent=2)+'\n')
        print(stage,name,json.dumps(e['summary']),flush=True)
    result['complete']=True
    (ROOT/'logs'/('recorded-cadence-'+stage+'.json')).write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
