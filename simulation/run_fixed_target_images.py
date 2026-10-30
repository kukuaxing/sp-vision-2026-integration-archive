#!/usr/bin/env python3
"""Run rendered image -> actual YOLO26 -> production prediction, file I/O only."""
import argparse
import base64
import copy
import datetime as dt
import html
import json
from pathlib import Path
import subprocess
import tempfile

from run_fixed_target import ROOT,digest,summarize


def report(data,path):
    rows=[];gallery=[]
    for e in data["experiments"]:
        s=e["summary"];m=e["image_summary"]
        rows.append(f'<tr><td>{html.escape(e["label"])}</td><td>{s["zero_hp_runs"]}/{s["runs"]}</td>'
                    f'<td>{s["valid_hits"]}/{s["budget_rounds"]}</td><td>{s["budget_hit_fraction"]:.1%}</td>'
                    f'<td>{m["correct_label_recall"]:.1%}</td><td>{m["usable_pose_recall"]:.1%}</td>'
                    f'<td>{m["detector_frames"]}</td></tr>')
        for r in e["runs"]:
            for preview in r["previews"]:
                img=ROOT/preview["path"]
                gallery.append(f'<figure data-case="{e["name"]}"><img loading="lazy" src="data:image/jpeg;base64,{base64.b64encode(img.read_bytes()).decode()}">'
                               f'<figcaption>{html.escape(e["label"])} · ω={r["omega_rad_s"]:+.2f} · 初相位{r["initial_phase_rad"]:.2f} · {preview["time_s"]:.1f}s</figcaption></figure>')
    options=''.join(f'<option value="{e["name"]}">{html.escape(e["label"])}</option>' for e in data["experiments"])
    details=html.escape(json.dumps({"cases":[{"name":e["name"],"scenario":e["scenario"],"runs":[{k:v for k,v in r.items() if k not in ("shots","frames","previews")} for r in e["runs"]]} for e in data["experiments"]],"sha256":data["sha256"]},ensure_ascii=False,indent=2))
    content='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>旋转靶 · 真实检测器图像实验</title>
<style>body{font:16px system-ui,"Microsoft YaHei";background:#101722;color:#e5edf8;max-width:1250px;margin:28px auto;padding:0 18px}h1{font-size:26px}p{line-height:1.7;color:#b8c8dc}.note{padding:12px 18px;background:#302921;border-left:3px solid #e8b672}table{border-collapse:collapse;width:100%;font-size:14px}th,td{border-bottom:1px solid #35465b;padding:12px;text-align:left}.scroll{overflow:auto}select{font:inherit;background:#25364b;color:#fff;padding:9px;margin:18px 0;border:1px solid #6b809c;border-radius:5px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px}figure{margin:0;background:#192638;padding:12px;border-radius:8px}img{width:100%;height:auto}figcaption{font-size:13px;color:#b8c8dc;margin-top:8px}pre{white-space:pre-wrap;overflow-wrap:anywhere;max-height:600px;overflow:auto}details{margin:20px 0;background:#192638;padding:14px}@media(max-width:800px){.grid{grid-template-columns:1fr}}</style>
<h1>固定旋转靶 · 真实 YOLO26 图像实验</h1><p class="note">输入已改为渲染RGB图像，运行当前CPU模型、颜色与角点精修，再进入现有解算/跟踪/预测。检测失败时不补真值角点。纹理、背景和误差分布是仿真假设；本页不是实车命中率，未运行电控与真实裁判。</p>
<p>统一300血、50发预算，至少30次合格几何命中完成；缺少发射资格的轮次按剩余预算计失败。召回率仅对55°内完整入画的灯条端点真值统计，画外/大斜角检出记为未匹配，不直接当误检。</p>
<div class="scroll"><table><thead><tr><th>实验</th><th>完成轮数</th><th>命中/预算</th><th>几何命中率</th><th>正确类别召回</th><th>可用角点召回</th><th>推理帧数</th></tr></thead><tbody>__ROWS__</tbody></table></div>
<p>绿色框是仅用于评估的几何真值，橙色框是检测器实际返回结果。图片是离散样本，不是完整视频；虚拟50FPS与实测推理耗时分开记录。</p><label>查看实验：<select id="pick">__OPTIONS__</select></label><div class="grid">__GALLERY__</div>
<details><summary>参数、逐轮结果、模型和源码指纹</summary><pre>__DETAILS__</pre></details>
<script>const pick=document.getElementById('pick');function show(){document.querySelectorAll('figure').forEach(x=>x.hidden=x.dataset.case!==pick.value)}pick.onchange=show;show();</script></html>'''
    path.write_text(content.replace('__ROWS__',''.join(rows)).replace('__OPTIONS__',options)
                    .replace('__GALLERY__',''.join(gallery)).replace('__DETAILS__',details),encoding='utf-8')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=['all','clear','exposure6ms','degraded','photo3','blank'],default='all')
    parser.add_argument('--quick',action='store_true',help='one direction/phase, 15s observation, for integration debugging')
    args=parser.parse_args()
    if args.case in ('all','photo3'):
        subprocess.run(['python3',str(ROOT/'simulation/prepare_fixed_target_texture.py')],cwd=ROOT,stdout=subprocess.DEVNULL,check=True)
    scenario=json.loads((ROOT/'simulation/fixed_target.json').read_text(encoding='utf-8'))
    scenario['target']['initial_phases_rad']=scenario['target']['initial_phases_rad'][:2]
    if args.quick:
        scenario['target']['directions']=[-1];scenario['target']['initial_phases_rad']=[0.0]
        scenario['sampling']['duration_s']=15
    observation={'mode':'rendered_yolo26','number':4,'exposure_s':0.,'exposure_samples':1,
                 'brightness':1.,'lens_blur_sigma_px':0.,'sensor_noise_stddev':0.}
    profiles=[('clear','清晰纹理',{}),('exposure6ms','6ms曝光积分',{'exposure_s':.006,'exposure_samples':5}),
              ('degraded','6ms曝光+降亮度+光学模糊',{'exposure_s':.006,'exposure_samples':5,'brightness':.65,
                 'lens_blur_sigma_px':.6,'sensor_noise_stddev':2.}),
              ('photo3','示例照片纹理·3号板',{'number':3,'texture_path':'data/fixed-target-input/photo-armor-three.png'}),
              ('blank','空白画面负对照',{'blank_scene':True})]
    result={'schema':1,'kind':'fixed_target_images','generated_utc':dt.datetime.now(dt.timezone.utc).isoformat(),
            'objective':{'hp':300,'ammo':50,'required_hits':30,'minimum_budget_hit_fraction':.6,'stop_on_truth_hp':False},
            'physical_device_opened':False,'truth_supplied_to_predictor':False,'ground_truth_fallback':False,
            'scope':'procedural rendered RGB -> actual YOLO26/color/corner-refiner -> production geometry/prediction helpers; not real camera/referee/MCU',
            'algorithm_parameters':{},'experiments':[],'sha256':{}}
    import yaml
    cfg=yaml.safe_load((ROOT/'configs/standard.yaml').read_text())
    result['algorithm_parameters']={k:cfg.get(k) for k in ('device','infer_threads','sentinel_fire_min_interval_ms','min_confidence',
        'yolo26_candidate_confidence','yolo26_model_path','high_speed_delay_time_cw','high_speed_delay_time_ccw')}
    files=['configs/standard.yaml','simulation/fixed_target.json','simulation/src/fixed_target_sim.cpp',
           'simulation/src/fixed_target_camera.hpp','simulation/src/fixed_target_renderer.hpp','simulation/src/fixed_target_world.hpp',
           'simulation/src/fixed_target_plant.hpp','simulation/run_fixed_target_images.py','simulation/run_fixed_target.py',
           'tasks/auto_aim/yolos/yolo26.cpp','tasks/auto_aim/adaptive_corner_refiner.cpp','tasks/auto_aim/solver.cpp',
           'tasks/auto_aim/tracker.cpp','tasks/auto_aim/target.cpp','tasks/auto_aim/aimer.cpp','tasks/auto_aim/high_speed_vision.hpp',
           'build/native-offline/bin/fixed_target_image_sim',cfg['yolo26_model_path']]
    model=Path(cfg['yolo26_model_path'])
    if model.suffix=='.xml':files.append(str(model.with_suffix('.bin')))
    if args.case in ('all','photo3'):files+=['data/fixed-target-input/photo-armor-three.png','data/fixed-target-input/texture-manifest.json',
                                          'assets/test.png','simulation/prepare_fixed_target_texture.py']
    result['sha256']={p:digest(ROOT/p) for p in files}
    console=ROOT/'logs/fixed-target-image-console.log'
    with tempfile.TemporaryDirectory(prefix='rm-fixed-images-') as folder,console.open('w') as log:
        for name,label,settings in profiles:
            if args.case not in ('all',name):continue
            case=copy.deepcopy(scenario);case['observation']={**observation,**settings,
                'preview_directory':f'logs/fixed-target-images/{name}'}
            if name=='blank':
                case['sampling']['duration_s']=10;case['target']['directions']=[-1];case['target']['initial_phases_rad']=[0.]
            src=Path(folder)/'scenario.json';out=Path(folder)/'result.json';src.write_text(json.dumps(case))
            log.write(name+'\n');log.flush()
            subprocess.run([str(ROOT/'build/native-offline/bin/fixed_target_image_sim'),str(ROOT/'configs/standard.yaml'),
                            str(src),str(out)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=1200)
            experiment=json.loads(out.read_text());experiment.update(name=name,label=label);experiment['summary']=summarize(experiment)
            stats=[r['image_stats'] for r in experiment['runs']];visible=sum(s['visible_truth_faces'] for s in stats)
            experiment['image_summary']={'visible_truth_faces':visible,'correct_label_recall':sum(s['correct_label_faces'] for s in stats)/visible if visible else 0,
                'usable_pose_recall':sum(s['usable_pose_faces'] for s in stats)/visible if visible else 0,'detector_frames':sum(s['detector_frames'] for s in stats)}
            result['experiments'].append(experiment)
            print(name,json.dumps({'shooting':experiment['summary'],'detection':experiment['image_summary']}),flush=True)
    output=ROOT/'logs/fixed-target-image-simulation.json';output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    report(result,ROOT/'logs/fixed-target-image-simulation.html');print('saved',output,flush=True)


if __name__=='__main__':main()
