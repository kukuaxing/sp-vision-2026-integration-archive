#!/usr/bin/env python3
"""Independent PnP and paired photometry diagnosis; no control imports/devices."""
import hashlib
import argparse
import itertools
import json
from pathlib import Path
import re
import subprocess
import sys
import cv2
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def stats(values):
    a=np.asarray(values,float)
    if not len(a):return None
    assert np.all(np.isfinite(a))
    return dict(n=len(a),mean=float(np.mean(a)),rmse=float(np.sqrt(np.mean(a*a))),
        median=float(np.median(a)),p95_abs=float(np.quantile(abs(a),.95)),max_abs=float(np.max(abs(a))))


def metrics(rows):
    valid=[r for r in rows if r['pose_valid']]
    lengths=[];spacing=[];inward=[]
    for r in rows:
        if not r['refined']:continue
        p=np.array(r['measured_corners']);t=np.array(r['truth_corners'])
        for a,b in ((0,3),(1,2)):
            axis=t[b]-t[a];length=np.linalg.norm(axis);axis/=length
            lengths.append(np.linalg.norm(p[b]-p[a])/length)
            inward.extend([(p[a]-t[a])@axis,-(p[b]-t[b])@axis])
        spacing.append(np.linalg.norm((p[1]+p[2]-p[0]-p[3])/2)/np.linalg.norm((t[1]+t[2]-t[0]-t[3])/2))
    return dict(n=len(rows),refined=sum(r['refined'] for r in rows),pose_valid=len(valid),
        x_error_m=stats([r['estimated_xyz'][0]-r['truth_xyz'][0] for r in valid]),
        bar_length_ratio=stats(lengths),bar_spacing_ratio=stats(spacing),inward_px=stats(inward),
        selected_threshold=stats([p['selected'] for r in rows for p in r['photometry']]),
        threshold_over_axis_peak=stats([p['selected_over_axis_peak'] for r in rows for p in r['photometry']]),
        axis_peak_below_threshold_bars=sum(p['axis_peak']<p['selected'] for r in rows for p in r['photometry']))


def analyze(require_regression=False):
    source=ROOT/'logs/photometry-geometry.json'
    data=json.loads(source.read_text());rows=data['rows']
    assert data['images']==1080 and len(rows)==2160 and not data['physical_device_opened']
    expected=set(itertools.product(('original','uniform_round','uniform_flat'),(2.,3.,4.),
        (-45.,-20.,0.,20.,45.),(-1,1),(.55,1.,1.45),(0.,.006),(0.,.6),(False,True)))
    def key(r):return tuple(r[k] for k in ('style','distance','angle_deg','direction','brightness','exposure_s','blur_px','candidate'))
    assert len({key(r) for r in rows})==len(rows) and {key(r) for r in rows}==expected
    bykey={key(r):r for r in rows}
    # Extend a fixture without altering its original behavior: compare old/new
    # sampled checksums AND all extracted points/pose/photometry metadata.
    before=json.loads((ROOT/'logs/photometry-default-before.json').read_text())
    after=json.loads((ROOT/'logs/photometry-default-after.json').read_text())
    assert before==after and before['images']==72
    for r in before['rows']:
        current=bykey[key(r)]
        assert {k:v for k,v in r.items() if k!='image_id'}=={k:v for k,v in current.items() if k!='image_id'}
    assert all(rows[i]['image_id']==rows[i+1]['image_id'] and
               rows[i]['pixel_checksum_fnv1a64']==rows[i+1]['pixel_checksum_fnv1a64'] for i in range(0,len(rows),2))
    cfg=yaml.safe_load((ROOT/'configs/standard.yaml').read_text())
    k=np.array(cfg['camera_matrix']).reshape(3,3);dist=np.array(cfg['distort_coeffs'])
    rc=np.array(cfg['R_camera2gimbal']).reshape(3,3);tc=np.array(cfg['t_camera2gimbal'])
    obj=np.array([[0,.0675,.028],[0,-.0675,.028],[0,-.0675,-.028],[0,.0675,-.028]],np.float32)
    errors=[]
    for r in rows:
        for p in r['photometry']:
            wanted=max(1.,p['otsu']) if r['candidate'] else np.clip(.65*p['otsu']+.35*cfg.get('threshold',100),25,230)
            assert abs(p['selected']-wanted)<1e-9
        if not r['pose_valid']:continue
        ok,rv,tv=cv2.solvePnP(obj,np.array(r['measured_corners'],np.float32),k,dist,flags=cv2.SOLVEPNP_IPPE)
        assert ok
        w,x,y,z=r['camera_world_quaternion_wxyz']
        rot=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        xyz=rot@(rc@tv.ravel()+tc)
        errors.append(float(np.linalg.norm(xyz-np.array(r['estimated_xyz']))))
    assert max(errors)<1e-4
    grouped=[]
    paired=[]
    for style,candidate in itertools.product(('original','uniform_round','uniform_flat'),(False,True)):
        selected=[r for r in rows if r['style']==style and r['candidate']==candidate]
        slices={}
        for field in ('direction','brightness','exposure_s','blur_px','distance'):
            slices[field]={str(value):metrics([r for r in selected if r[field]==value])
                           for value in sorted({r[field] for r in selected})}
        clean=[r for r in selected if r['exposure_s']==0 and r['blur_px']==0]
        clean_by_brightness={str(value):metrics([r for r in clean if r['brightness']==value]) for value in (.55,1.,1.45)}
        grouped.append(dict(style=style,candidate=candidate,all=metrics(selected),
            slices=slices,instant_sharp_by_brightness=clean_by_brightness))
    for style in ('original','uniform_round','uniform_flat'):
        pairs=[(r,bykey[key(r)[:-1]+(True,)]) for r in rows if r['style']==style and not r['candidate']]
        common=[(a,b) for a,b in pairs if a['pose_valid'] and b['pose_valid']]
        paired.append(dict(style=style,n=len(pairs),common_valid=len(common),
            legacy_only_valid=sum(a['pose_valid'] and not b['pose_valid'] for a,b in pairs),
            candidate_only_valid=sum(b['pose_valid'] and not a['pose_valid'] for a,b in pairs),
            neither_valid=sum(not a['pose_valid'] and not b['pose_valid'] for a,b in pairs),
            legacy=metrics([a for a,b in common]),candidate=metrics([b for a,b in common])))
    # With symmetric exposure and no sensor noise, changing rotation sign
    # reverses sample order only. Do not mistake these pairs for independent trials.
    sign_pairs=0;max_sign_corner_delta=0.
    for r in rows:
        if r['direction']!=-1:continue
        opposite=bykey[(r['style'],r['distance'],r['angle_deg'],1,r['brightness'],r['exposure_s'],r['blur_px'],r['candidate'])]
        assert r['refined']==opposite['refined'] and r['pose_valid']==opposite['pose_valid']
        if r['refined']:
            delta=float(np.max(abs(np.array(r['measured_corners'])-opposite['measured_corners'])))
            max_sign_corner_delta=max(max_sign_corner_delta,delta)
        sign_pairs+=1
    # Pixel checksum may differ by rounding: require geometry agreement, not identity.
    assert max_sign_corner_delta<.02
    names=['configs/standard.yaml','build/standard','simulation/src/fixed_target_renderer.hpp',
        'simulation/src/fixed_target_renderer_test.cpp','simulation/src/fixed_target_photometry_probe.cpp',
        'simulation/analyze_photometry.py','tasks/auto_aim/adaptive_corner_refiner.cpp',
        'tasks/auto_aim/adaptive_corner_refiner.hpp','build/native-offline/bin/fixed_target_photometry_probe',
        'tools/analyze_lightbar_photometry.py','tests/offline/test_lightbar_photometry.py']
    hashes={name:digest(ROOT/name) for name in names}
    assert hashes['configs/standard.yaml']=='e087e02c2c6ad257acef5a0808950a42f15445c4ed00714807023dad7427fd90'
    assert hashes['build/standard']=='dbc3d740d18255383af4f211b9bb8b740bbee0c65d31b0d18c8a757f378d5cfe'
    assert cfg['corner_refine_brightness_invariant_subpixel'] is False
    regression=None
    recorded=None
    if require_regression:
        log=(ROOT/'logs/photometry-ctest.log').read_text()
        match=re.search(r'100% tests passed, 0 tests failed out of (\d+)',log)
        assert match,'full current offline CTest has not passed'
        hygiene=subprocess.run([sys.executable,str(ROOT/'scripts/check_source_tree.py')],cwd=ROOT,capture_output=True,text=True)
        assert hygiene.returncode==0,hygiene.stdout+hygiene.stderr
        tests=subprocess.run([sys.executable,str(ROOT/'tests/offline/test_lightbar_photometry.py')],cwd=ROOT,capture_output=True,text=True)
        assert tests.returncode==0 and 'Ran 7 tests' in tests.stderr,tests.stderr
        sample=json.loads((ROOT/'logs/photometry-recorded.json').read_text())
        assert not sample['accuracy_measured'] and not sample['physical_device_opened']
        assert sample['implementation_sha256']==hashes['tools/analyze_lightbar_photometry.py']
        capture=ROOT/'data/replay/endpoint-baseline/events.jsonl'
        assert sample['recording_sha256']==digest(capture)
        recorded={k:sample[k] for k in ('recorded_frames','inspected_objects','inspected_bars',
            'legacy_axis_below_threshold_bars','candidate_axis_below_threshold_bars','accuracy_measured')}
        recorded['report_sha256']=digest(ROOT/'logs/photometry-recorded.json')
        regression=dict(ctest_passed=int(match[1]),source_tree=hygiene.stdout.strip(),
            new_python_tests=7,ctest_log_sha256=digest(ROOT/'logs/photometry-ctest.log'))
    result=dict(scope=data['scope'],source_sha256=digest(source),sha256=hashes,groups=grouped,paired_valid=paired,
        regression=regression,recorded_sample=recorded,
        checks=dict(complete_factorial=True,paired_images=1080,unchanged_default_images=72,
            independent_pnp_replays=len(errors),max_pnp_difference_m=max(errors),
            sign_pairs=sign_pairs,max_sign_corner_delta_px=max_sign_corner_delta,
            sign_pairs_not_independent_trials=True,formal_program_and_config_unchanged=True),
        limitations=['oracle ROIs, no detector/tracker/shot experiment',
            'artificial texture appearance, not measured camera photometry',
            'no sensor noise; direction pairs reuse symmetric exposure samples',
            'bundled endpoint flag changes threshold and subpixel extraction together'])
    (ROOT/'logs/photometry-analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    for group in grouped:
        print(group['style'],group['candidate'],json.dumps(group['all']))
        print('instant_sharp',json.dumps({b:{'x_rmse_m':m['x_error_m']['rmse'] if m['x_error_m'] else None,
            'length':m['bar_length_ratio']['mean'] if m['bar_length_ratio'] else None,
            'threshold':m['selected_threshold']['mean']} for b,m in group['instant_sharp_by_brightness'].items()}))
    print(json.dumps(result['checks']))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--require-regression',action='store_true')
    analyze(parser.parse_args().require_regression)
