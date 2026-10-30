#!/usr/bin/env python3
"""Post-hoc plane-crossing diagnostics. Never imported by the controller."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

def crossings(case,run,shot):
    target,plant=case['target'],case['plant']
    center=np.array(target['center_m']);omega=run['omega_rad_s']
    tilt=np.deg2rad(target['armor_pitch_deg'])
    speed=shot['physical_speed_mps'];yaw=shot['world_yaw'];pitch=shot['world_elevation']
    velocity=speed*np.array([np.cos(pitch)*np.cos(yaw),np.cos(pitch)*np.sin(yaw),np.sin(pitch)])
    def at(dt,face):
        phase=run['initial_phase_rad']+omega*(shot['muzzle_s']+dt)+face*np.pi/2
        c,s=np.cos(phase),np.sin(phase)
        point=np.expand_dims(dt,-1)*velocity
        point[...,2]-=.5*plant['gravity_mps2']*dt**2
        plate=np.stack([center[0]-target['radii_m'][face]*c,
                        center[1]-target['radii_m'][face]*s,
                        np.full_like(c,center[2]+(target['height_delta_m'] if face%2 else 0))],axis=-1)
        normal=np.stack([-np.cos(tilt)*c,-np.cos(tilt)*s,np.full_like(c,np.sin(tilt))],axis=-1)
        delta=point-plate
        return np.sum(delta*normal,axis=-1),delta,phase
    found=[]
    times=np.arange(0,plant['flight_timeout_s']+plant['collision_step_s']/2,plant['collision_step_s'])
    for face in range(4):
        distance,_,_=at(times,face)
        for i in np.flatnonzero((distance[:-1]>0)&(distance[1:]<=0)):
            lo,hi=times[i:i+2]
            for _ in range(35):
                mid=(lo+hi)/2
                if at(np.array(mid),face)[0]>0:lo=mid
                else:hi=mid
            dt=(lo+hi)/2;_,delta,phase=at(np.array(dt),face)
            u=delta@np.array([-np.sin(phase),np.cos(phase),0])
            v=delta@np.array([np.sin(tilt)*np.cos(phase),np.sin(tilt)*np.sin(phase),np.cos(tilt)])
            found.append({'face':face,'time_s':float(shot['muzzle_s']+dt),
                'u_m':float(u),'v_m':float(v),'phase_rad':float(np.remainder(phase+np.pi,2*np.pi)-np.pi),
                'normalized_edge_distance':float(max(abs(u)/(target['armor_width_m']/2),abs(v)/(target['armor_hit_height_m']/2)))})
    return min(found,key=lambda x:x['normalized_edge_distance']) if found else None

def analyze(data):
    rows=[]
    for e in data['experiments']:
        for r in e['runs']:
            for s in r['shots']:
                nearest=crossings(e['scenario'],r,s)
                assert bool(nearest and nearest['normalized_edge_distance']<=1+1e-8)==s['geometric_hit'],s['id']
                horizontal=nearest and abs(nearest['u_m'])>e['scenario']['target']['armor_width_m']/2
                vertical=nearest and abs(nearest['v_m'])>e['scenario']['target']['armor_hit_height_m']/2
                axis='hit' if s['valid_hit'] else ('both' if horizontal and vertical else
                    'horizontal_only' if horizontal else 'vertical_only' if vertical else 'unresolved')
                rows.append({'experiment':e['name'],'seed':r['seed'],'direction':r['direction'],'miss_axis':axis,
                    'initial_phase_rad':r['initial_phase_rad'],'id':s['id'],'valid_hit':s['valid_hit'],
                    'physical_delay_ms':1000*(s['muzzle_s']-s['issue_s']),
                    'predicted_vs_actual_arrival_ms':1000*(nearest['time_s']-s['model_impact_s']) if nearest else None,
                    'nearest_crossing':nearest})
    misses=[r for r in rows if not r['valid_hit']]
    groups={}
    for sign in (-1,1):
        selected=[r for r in misses if r['direction']==sign]
        groups[str(sign)]={'misses':len(selected),**{axis:sum(r['miss_axis']==axis for r in selected)
            for axis in ('horizontal_only','vertical_only','both','unresolved')}}
    cases={e['name']:{axis:sum(r['experiment']==e['name'] and r['miss_axis']==axis for r in rows)
        for axis in ('hit','horizontal_only','vertical_only','both','unresolved')} for e in data['experiments']}
    return {'scope':'post-hoc truth evaluation only, never available to controller','misses':len(misses),
        'direction':groups,'experiments':cases,'shots':rows}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('result',type=Path)
    p.add_argument('--output',type=Path,default=ROOT/'logs/fixed-target-miss-analysis.json');args=p.parse_args()
    result=analyze(json.loads(args.result.read_text(encoding='utf-8')))
    result['source_sha256']=hashlib.sha256(args.result.read_bytes()).hexdigest()
    result['analyzer_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='shots'}))

if __name__=='__main__':main()
