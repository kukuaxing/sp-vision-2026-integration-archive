#!/usr/bin/env python3
"""Evaluator-only uncertainty removals on frozen shot schedules/orientations."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
from analyze_fixed_target_misses import crossings


def score(data):
    assert data['complete']
    result={'schema':1,'scope':'evaluator-only error budget: frozen schedules and angles; not a deployable hit-rate improvement',
        'physical_device_opened':False,'truth_known_only_to_evaluator':True,'rows':[]}
    verified=0
    for e in data['experiments']:
        if e.get('condition','measured_follower')!='measured_follower':continue
        plant=e['scenario']['plant'];assert plant['min_normal_speed_mps']==0.
        dense=all('muzzle_servo_yaw_rad' in s for r in e['runs'] for s in r['shots'])
        treatments=['actual','mean_delay','nominal_speed','mean_delay_nominal_speed']
        if dense:treatments+=['zero_angular_spread','mean_delay_zero_angular_spread','all_random_removed']
        for treatment in treatments:
            total=hits=lost=gained=0;direction={-1:0,1:0};counts=[]
            for run in e['runs']:
                count=0
                for original in run['shots']:
                    shot=copy.copy(original)
                    if treatment.startswith('mean_delay') or treatment=='all_random_removed':
                        shot['muzzle_s']=shot['issue_s']+plant['muzzle_delay_s']
                    if 'nominal_speed' in treatment or treatment=='all_random_removed':
                        shot['physical_speed_mps']=plant['projectile_speed_mps']
                    if 'zero_angular_spread' in treatment or treatment=='all_random_removed':
                        shot['world_yaw']=shot['muzzle_servo_yaw_rad']+math.radians(plant['barrel_yaw_bias_deg'])
                        shot['world_elevation']=shot['muzzle_servo_elevation_rad']+math.radians(plant['barrel_pitch_bias_deg'])
                    cross=crossings(e['scenario'],run,shot)
                    hit=bool(cross and cross['normalized_edge_distance']<=1+1e-8)
                    if treatment=='actual':assert hit==original['valid_hit'];verified+=1
                    total+=1;count+=hit;lost+=original['valid_hit'] and not hit;gained+=not original['valid_hit'] and hit
                hits+=count;direction[run['direction']]+=count;counts.append(count)
            row={'experiment':e['name'],'treatment':treatment,'hits':hits,'shots':total,
                'negative_hits':direction[-1],'positive_hits':direction[1],
                'worst_hits_per_50':min(counts),'lost':lost,'gained':gained}
            result['rows'].append(row);print(json.dumps(row),flush=True)
    result['zero_intervention_shots_verified']=verified
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('input',type=Path)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=score(json.loads(a.input.read_text(encoding='utf-8')))
    result.update(input_sha256=hashlib.sha256(a.input.read_bytes()).hexdigest(),
                  evaluator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':main()
