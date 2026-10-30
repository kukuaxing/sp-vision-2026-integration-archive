#!/usr/bin/env python3
"""Replace synthetic corners with rendered YOLO observations under the same physical-error profile."""
import copy
import datetime as dt
import json
import math
from pathlib import Path
import subprocess
import tempfile
from run_fixed_target import ROOT,digest,summarize
from run_fixed_target_images import report


def main():
    geometry_path=ROOT/'logs/fixed-target-simulation.json'
    images_path=ROOT/'logs/fixed-target-image-simulation.json'
    geometry=json.loads(geometry_path.read_text());images=json.loads(images_path.read_text())
    reference=next(e for e in geometry['experiments'] if e['name']=='mixed_moderate')
    case=copy.deepcopy(reference['scenario'])
    case['target']['initial_phases_rad']=case['target']['initial_phases_rad'][:2]
    # Actual detector noise replaces the earlier independent synthetic corner
    # noise. Do not add truth-based perturbations to detector output corners.
    case['sampling']['keypoint_noise_px']=0
    case['observation']={'mode':'rendered_yolo26','number':4,'exposure_s':.006,'exposure_samples':5,
        'brightness':.65,'lens_blur_sigma_px':.6,'sensor_noise_stddev':2.,
        'preview_directory':'logs/fixed-target-images/mixed_moderate'}
    control=[r for r in reference['runs'] if r['initial_phase_rad']<=math.pi/4+1e-9]
    result={k:copy.deepcopy(v) for k,v in images.items() if k not in ('experiments','generated_utc')}
    result['generated_utc']=dt.datetime.now(dt.timezone.utc).isoformat()
    result['scope']='actual rendered YOLO input plus synthetic moderate physical/pose/timestamp/dropout uncertainties; not hardware validation'
    result['control']={'source':str(geometry_path.relative_to(ROOT)),
        'summary':summarize({'runs':control}),
        'difference':'synthetic 1px corners replaced by actual detector output on degraded rendered images; all physical settings retained'}
    result['sha256']['simulation/run_fixed_target_image_stress.py']=digest(Path(__file__))
    result['sha256'][str(geometry_path.relative_to(ROOT))]=digest(geometry_path)
    result['sha256'][str(images_path.relative_to(ROOT))]=digest(images_path)
    with tempfile.TemporaryDirectory(prefix='rm-image-stress-') as folder:
        src,out=Path(folder)/'scenario.json',Path(folder)/'result.json';src.write_text(json.dumps(case))
        with (ROOT/'logs/fixed-target-image-stress-console.log').open('w') as console:
            subprocess.run([str(ROOT/'build/native-offline/bin/fixed_target_image_sim'),str(ROOT/'configs/standard.yaml'),
                            str(src),str(out)],cwd=ROOT,stdout=console,stderr=subprocess.STDOUT,check=True,timeout=1200)
        experiment=json.loads(out.read_text());experiment.update(name='image_moderate',label='图像检测+中等物理误差')
    experiment['summary']=summarize(experiment)
    stats=[r['image_stats'] for r in experiment['runs']];visible=sum(s['visible_truth_faces'] for s in stats)
    experiment['image_summary']={'visible_truth_faces':visible,
        'correct_label_recall':sum(s['correct_label_faces'] for s in stats)/visible if visible else 0,
        'usable_pose_recall':sum(s['usable_pose_faces'] for s in stats)/visible if visible else 0,
        'detector_frames':sum(s['detector_frames'] for s in stats),
        'recall_scope':'includes configured 5% whole-frame dropout; not conditional detector-only recall'}
    result['experiments']=[experiment]
    path=ROOT/'logs/fixed-target-image-stress.json';path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    report(result,ROOT/'logs/fixed-target-image-stress.html')
    print(json.dumps({'image':experiment['summary'],'observation':experiment['image_summary'],'keypoint_control':result['control']['summary']}),flush=True)


if __name__=='__main__':main()
