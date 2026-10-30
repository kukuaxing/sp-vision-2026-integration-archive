#!/usr/bin/env python3
"""Inspect image photometry around recorded detections, without pose truth/devices.

ROIs use already-output corners, so these are diagnostic thresholds, NOT an
exact reconstruction of the network-seed ROIs used by the original refiner.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import cv2
import numpy as np
import yaml


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def sample(image,point):
    x,y=map(float,point);ix,iy=math.floor(x),math.floor(y)
    if ix<0 or iy<0 or ix+1>=image.shape[1] or iy+1>=image.shape[0]:
        raise ValueError('profile touches image boundary')
    a,b=x-ix,y-iy
    return float((1-b)*((1-a)*image[iy,ix]+a*image[iy,ix+1])+
                 b*((1-a)*image[iy+1,ix]+a*image[iy+1,ix+1]))


def inspect(image,corners,config):
    p=np.asarray(corners,float)
    if p.shape!=(4,2) or not np.all(np.isfinite(p)):raise ValueError('four finite corners required')
    contrast=cv2.absdiff(image[:,:,0],image[:,:,2]);results=[]
    for first,last in ((0,3),(1,2)):
        top,bottom=p[first],p[last];center=(top+bottom)*.5;length=float(np.linalg.norm(bottom-top))
        if length<4:raise ValueError('bar shorter than refiner minimum')
        width=max(12.,length*config.get('corner_refine_roi_width_scale',.9))
        height=max(12.,length*config.get('corner_refine_roi_length_scale',1.8))
        x0=max(0,math.floor(center[0]-width/2));x1=min(image.shape[1],math.ceil(center[0]+width/2))
        y0=max(0,math.floor(center[1]-height/2));y1=min(image.shape[0],math.ceil(center[1]+height/2))
        if x1-x0<4 or y1-y0<4:raise ValueError('small or truncated ROI')
        blurred=cv2.GaussianBlur(contrast[y0:y1,x0:x1],(3,3),0)
        otsu,_=cv2.threshold(blurred,0,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)
        legacy=float(np.clip(.65*otsu+.35*config.get('threshold',100),25,230))
        axis=np.array([sample(blurred,top+(bottom-top)*i/40-np.array([x0,y0])) for i in range(41)])
        results.append(dict(length_px=length,otsu=float(otsu),legacy_threshold=legacy,
            candidate_threshold=max(1.,float(otsu)),axis_peak=float(max(axis)),axis_profile=axis.tolist(),
            legacy_over_axis_peak=legacy/max(1.,float(max(axis))),
            candidate_over_axis_peak=max(1.,float(otsu))/max(1.,float(max(axis))),
            legacy_axis_below_threshold=bool(max(axis)<legacy),
            candidate_axis_below_threshold=bool(max(axis)<max(1.,float(otsu)))))
    return results


def analyze(capture):
    capture=capture.resolve();rows=[json.loads(s) for s in capture.read_text().splitlines() if s.strip()]
    header=rows[0]
    if header.get('kind')!='manifest' or header.get('schema')!='sp-vision-events-1' or rows[-1].get('complete') is not True:
        raise ValueError('complete sp-vision-events-1 recording required')
    config=yaml.safe_load(header['configs'][0]['content_utf8'])
    images={r['data']['image_id']:r['data'] for r in rows if r['kind']=='image'}
    observations=[];skips=[];hashes={};cache={}
    for event in rows:
        if event['kind']!='detections':continue
        image_id=event['data']['image_id'];description=images[image_id]
        image_path=(capture.parent/description['path']).resolve()
        if capture.parent not in image_path.parents:raise ValueError('image is outside recording directory')
        if image_id not in cache:
            actual=digest(image_path);assert actual==description['sha256'],'recorded image hash mismatch'
            image=cv2.imread(str(image_path));assert image is not None
            assert image.shape[:2]==(description['height'],description['width'])
            hashes[image_id]=actual;cache[image_id]=image
        for index,obj in enumerate(event['data']['objects']):
            try:bars=inspect(cache[image_id],obj['corners_px'],config)
            except ValueError as e:
                skips.append(dict(image_id=image_id,object_index=index,reason=str(e)));continue
            observations.append(dict(image_id=image_id,object_index=index,label=obj['label'],
                confidence=obj['confidence'],bars=bars))
    bars=[b for r in observations for b in r['bars']]
    return dict(scope='existing decoded images and output detections; no independent position/corner/impact labels',
        accuracy_measured=False,physical_device_opened=False,
        implementation_sha256=digest(Path(__file__)),
        diagnostic_rois_are_not_original_network_rois=True,
        recording_sha256=digest(capture),config_content_sha256=hashlib.sha256(header['configs'][0]['content_utf8'].encode()).hexdigest(),
        image_sha256=hashes,recorded_frames=len(images),read_images=len(cache),
        inspected_objects=len(observations),inspected_bars=len(bars),skipped=skips,
        legacy_axis_below_threshold_bars=sum(b['legacy_axis_below_threshold'] for b in bars),
        candidate_axis_below_threshold_bars=sum(b['candidate_axis_below_threshold'] for b in bars),
        observations=observations,
        limitations=['unverified detections are not physical armor labels',
            'axis measurements are conditional on the existing corners',
            'small historical sample cannot establish a field failure cause'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();result=analyze(args.capture)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('observations','image_sha256','limitations')},indent=2))


if __name__=='__main__':main()
