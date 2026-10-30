#!/usr/bin/env python3
"""File-only prospective phase gating: development, frozen holdout, jitter stress."""
import argparse
import copy
import json
import math
from pathlib import Path
import subprocess
import tempfile
import yaml
from run_fixed_target import ROOT, digest, summarize

# Declared before running the development data. Do not retune on holdout/stress.
SPLITS = {
    'development': ([20011], [22.5]),
    'holdout': ([30011, 40009, 50021], [7.5, 52.5]),
    'stress': ([60013], [32.5]),
}


def metrics(e):
    s = summarize(e)
    s['last_issue_max_s'] = max((r['shots'][-1]['issue_s'] for r in e['runs'] if r['shots']), default=None)
    s['phase_rejected_frames'] = sum(r['phase_rejected_frames'] for r in e['runs'])
    return s


def development_eligible(s, baseline):
    return (s['full_budget_runs'] == s['runs'] and s['zero_hp_runs'] == s['runs']
            and s['valid_hits'] > baseline['valid_hits']
            and s['worst_run_budget_hit_fraction'] >= baseline['worst_run_budget_hit_fraction']
            and all(s['direction_summary'][str(d)]['valid_hits'] >=
                    baseline['direction_summary'][str(d)]['valid_hits'] for d in (-1, 1))
            and s['last_issue_max_s'] < 60)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage', choices=SPLITS, required=True)
    args = p.parse_args()
    stage = args.stage
    source = ROOT/'logs/endpoint-deployment-images.json'
    prior = json.loads(source.read_text())
    assert prior['complete']
    cfg = yaml.safe_load((ROOT/'configs/standard.yaml').read_text())
    assert cfg['corner_refine_brightness_invariant_subpixel'] is False
    base = copy.deepcopy(prior['experiments'][0]['scenario'])
    seeds, phases = SPLITS[stage]
    base['sampling']['seeds'] = seeds
    base['target']['initial_phases_rad'] = [math.radians(x) for x in phases]
    # Current formal prediction timing and historical visual correction, not
    # the old offline +20ms direction-specific calibration.
    base['controller'].update(center_crossing_schedule=False,
        directional_prediction_offsets_s={'-1': 0., '1': 0.}, prediction_time_offset_s=0.,
        elevation_trim_deg=0., prefer_smaller_radius=False, apply_visual_residual_correction=True)
    executable = ROOT/'build/native-offline/bin/fixed_target_image_sim'
    deps = {'configs/standard.yaml', 'simulation/study_centered_opportunities.py',
        'simulation/run_fixed_target.py', 'tests/offline/CMakeLists.txt',
        str(executable.relative_to(ROOT)), str(source.relative_to(ROOT)),
        'assets/yolo26.xml', 'assets/yolo26.bin', 'build/standard'}
    for folder in ('simulation/src', 'tasks/auto_aim', 'tools'):
        deps.update(str(x.relative_to(ROOT)) for x in (ROOT/folder).rglob('*')
                    if x.suffix in ('.cpp', '.hpp', '.h'))
    result = {'schema': 1, 'study': 'centered_opportunity', 'stage': stage,
        'scope': 'rendered YOLO26, formal prediction delays/residual; not full standard or hardware',
        'physical_device_opened': False, 'truth_supplied_to_predictor': False,
        'splits': SPLITS, 'assumed_jitter_not_measured': True, 'experiments': [],
        'complete': False, 'sha256': {k: digest(ROOT/k) for k in sorted(deps)}}
    development = ROOT/'logs/centered-opportunity-development.json'
    if stage == 'development':
        policies = [('baseline', 0.), ('phase_3deg', 3.), ('phase_6deg', 6.)]
    else:
        dev = json.loads(development.read_text())
        assert dev['complete']
        for name, value in dev['sha256'].items():
            assert digest(ROOT/name) == value, name
        result['development_sha256'] = digest(development)
        result['frozen_selection'] = dev['frozen_selection']
        policies = [('baseline', 0.), ('frozen_candidate', dev['frozen_selection']['center_phase_limit_deg'])]
    jitter_values = [.025] if stage != 'stress' else [.010, .040]
    for jitter in jitter_values:
        for name, limit in policies:
            case = copy.deepcopy(base)
            case['plant']['muzzle_delay_jitter_s'] = jitter
            case['controller']['center_phase_limit_deg'] = limit
            tag = name if stage != 'stress' else name + '_jitter_' + str(round(jitter*1000))
            case['observation']['preview_directory'] = 'logs/fixed-target-images/centered-opportunity/'+stage+'/'+tag
            with tempfile.TemporaryDirectory(prefix='rm-centered-opportunity-') as tmp:
                sc, out = Path(tmp)/'scenario.json', Path(tmp)/'output.json'
                sc.write_text(json.dumps(case))
                with (ROOT/'logs'/('centered-opportunity-'+stage+'-'+tag+'.log')).open('w') as log:
                    subprocess.run([str(executable), str(ROOT/'configs/standard.yaml'), str(sc), str(out)],
                        cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=3600)
                e = json.loads(out.read_text())
            e.update(name=tag, policy=name, center_phase_limit_deg=limit, summary=metrics(e))
            assert len(e['runs']) == len(seeds)*2*len(phases)
            result['experiments'].append(e)
            (ROOT/'logs'/('centered-opportunity-'+stage+'.json')).write_text(json.dumps(result, indent=2)+'\n')
            print(stage, tag, json.dumps(e['summary']), flush=True)
    if stage == 'development':
        baseline = result['experiments'][0]['summary']
        candidates = result['experiments'][1:]
        # Freeze one candidate even if it fails development; then report that
        # failure explicitly. No replacement after seeing holdout or stress.
        best = max(candidates, key=lambda e: (
            development_eligible(e['summary'], baseline), e['summary']['zero_hp_runs'],
            e['summary']['valid_hits'], e['summary']['worst_run_budget_hit_fraction'],
            -(e['summary']['max_kill_s'] or 999.), e['center_phase_limit_deg']))
        result['frozen_selection'] = {'name': best['name'],
            'center_phase_limit_deg': best['center_phase_limit_deg'],
            'development_eligible': development_eligible(best['summary'], baseline),
            'selection_uses_holdout': False}
        print('FROZEN', json.dumps(result['frozen_selection']), flush=True)
    result['complete'] = True
    (ROOT/'logs'/('centered-opportunity-'+stage+'.json')).write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
