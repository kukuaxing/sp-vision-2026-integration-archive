#!/usr/bin/env python3
"""Summarize existing standard JSONL without camera, serial or control I/O."""
import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics


def stats(values):
    if not values:return None
    x=sorted(values)
    return {'n':len(x),'mean':statistics.fmean(x),'median':statistics.median(x),
            'p05':x[int((len(x)-1)*.05)],'p95':x[int((len(x)-1)*.95)]}


def finite_vector(value,size):
    return isinstance(value,list) and len(value)==size and all(
        isinstance(v,(int,float)) and math.isfinite(v) for v in value)


def analyze(paths,expected=None):
    groups=defaultdict(lambda:defaultdict(list));frames=invalid=0
    for path in paths:
        with path.open(encoding='utf-8') as handle:
            for line_no,line in enumerate(handle,1):
                if not line.strip():continue
                try:row=json.loads(line)
                except json.JSONDecodeError as e:raise ValueError(f'{path}:{line_no}: damaged JSON') from e
                frames+=1;obs=row.get('observation')
                if not isinstance(obs,dict) or not obs.get('pose_valid'):
                    invalid+=1;continue
                key=(row.get('build_source_sha256','unknown'),row.get('config_fnv1a64','unknown'),
                     row.get('endpoint_method','unknown'))
                group=groups[key];corners=obs.get('corners_px')
                if isinstance(corners,list) and len(corners)==4 and all(finite_vector(p,2) for p in corners):
                    for a,b in ((0,3),(1,2)):
                        group['lightbar_length_px'].append(math.dist(corners[a],corners[b]))
                    centers=[[(corners[a][k]+corners[b][k])*.5 for k in range(2)] for a,b in ((0,3),(1,2))]
                    group['lightbar_spacing_px'].append(math.dist(*centers))
                xyz=obs.get('xyz_gimbal_m')
                if finite_vector(xyz,3):
                    group['range_from_gimbal_m'].append(math.sqrt(sum(v*v for v in xyz)))
                    if expected is not None:
                        group['position_error_m'].append(math.dist(xyz,expected))
                        group['range_error_m'].append(math.sqrt(sum(v*v for v in xyz))-math.sqrt(sum(v*v for v in expected)))
                for key_name in ('reprojection_px','fixed_model_reprojection_px','confidence'):
                    value=obs.get(key_name)
                    if isinstance(value,(int,float)) and math.isfinite(value):group[key_name].append(value)
    return {'schema':1,'scope':'existing diagnostic files only; no device opened',
        'expected_static_position_gimbal_m':expected,
        'accuracy_measured':expected is not None and any(g.get('position_error_m') for g in groups.values()),
        'frames':frames,'missing_or_invalid_pose_frames':invalid,
        'source_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
        'groups':[{'build_source_sha256':key[0],'config_fnv1a64':key[1],'endpoint_method':key[2],
                   'statistics':{k:stats(v) for k,v in group.items()}} for key,group in groups.items()]}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('input',type=Path,nargs='+')
    p.add_argument('--expected-position-gimbal',type=float,nargs=3,metavar=('X','Y','Z'),
        help='Measured static plate center in gimbal frame, meters; never substitute rotating center position')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.expected_position_gimbal and not all(math.isfinite(v) for v in a.expected_position_gimbal):p.error('position must be finite')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    result=analyze(a.input,a.expected_position_gimbal)
    a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'frames':result['frames'],'groups':len(result['groups']),
                      'accuracy_measured':result['accuracy_measured']}))


if __name__=='__main__':main()
