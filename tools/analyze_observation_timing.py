#!/usr/bin/env python3
"""Analyze recorded observation cadence; never infer exposure or muzzle delay."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics


def stats(values):
    if not values:return None
    x=sorted(values)
    return {'n':len(x),'median':statistics.median(x),'p95':x[int(.95*(len(x)-1))],
            'p99':x[int(.99*(len(x)-1))],'max':x[-1]}


def analyze(path):
    rows=json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(rows,list):raise ValueError('expected recorded tracker observation array')
    times=[r.get('t') for r in rows]
    if not all(isinstance(t,(int,float)) and math.isfinite(t) for t in times):raise ValueError('invalid observation timestamp')
    differences=[b-a for a,b in zip(times,times[1:])]
    positive=[d for d in differences if d>0]
    return {'schema':1,'scope':'historical observation cadence only; not camera exposure/frame age or actuation latency',
        'input_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'frames':len(rows),
        'nonmonotonic_intervals':sum(d<=0 for d in differences),
        'duration_s':times[-1]-times[0] if len(times)>1 else None,
        'effective_observation_hz':(len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else None,
        'positive_interval_ms':stats([1000*d for d in positive]),
        'intervals_over_40ms':sum(d>.04 for d in positive),
        'intervals_over_100ms':sum(d>.1 for d in positive),
        'valid_pose_frames':sum(bool(r.get('pose_valid')) for r in rows),
        'frames_with_detections':sum(bool(r.get('detections')) for r in rows),
        'sensor_latency_measured':False,'muzzle_delay_measured':False}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('input',type=Path)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=analyze(a.input);a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result))


if __name__=='__main__':main()
