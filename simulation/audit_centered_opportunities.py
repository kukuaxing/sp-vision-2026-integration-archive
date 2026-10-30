#!/usr/bin/env python3
"""Independent accounting/geometry and frozen-policy audit; never authorizes devices."""
import json
import math
from pathlib import Path
import copy
import subprocess
import tempfile
import argparse
import yaml
from analyze_fixed_target_misses import analyze
from run_fixed_target import ROOT, digest
from study_centered_opportunities import SPLITS, metrics, development_eligible


def default_replay():
    """Current zero-filter path must reproduce a historical 100-shot trace."""
    source = ROOT/'logs/endpoint-ablation.json'
    inputs = {name:digest(ROOT/name) for name in ('logs/endpoint-ablation.json',
        'configs/standard.yaml','build/native-offline/bin/fixed_target_image_sim',
        'simulation/audit_centered_opportunities.py')}
    receipt = ROOT/'logs/centered-opportunity-default-replay.json'
    raw = ROOT/'logs/centered-opportunity-default-replay-shots.json'
    if receipt.exists():
        cached = json.loads(receipt.read_text())
        assert cached['inputs'] == inputs, 'stale default replay receipt'
        assert cached['historical_shots_verified'] == 100 and cached['trace_sha256'] == digest(raw)
        return cached
    prior = json.loads(source.read_text())
    old = next(e for e in prior['experiments'] if e['name'] == 'measured_follower_baseline')
    case = copy.deepcopy(old['scenario'])
    assert case['controller'].get('center_phase_limit_deg', 0) == 0
    case['observation']['preview_directory'] = 'logs/fixed-target-images/centered-opportunity/default-replay'
    with tempfile.TemporaryDirectory(prefix='rm-centered-default-') as tmp:
        sc, out = Path(tmp)/'scenario.json', Path(tmp)/'output.json'
        sc.write_text(json.dumps(case))
        with (ROOT/'logs/centered-opportunity-default-replay.log').open('w') as log:
            subprocess.run([str(ROOT/'build/native-offline/bin/fixed_target_image_sim'),
                str(ROOT/'configs/standard.yaml'), str(sc), str(out)], cwd=ROOT,
                stdout=log, stderr=subprocess.STDOUT, check=True, timeout=900)
        current = json.loads(out.read_text())
    assert len(current['runs']) == len(old['runs']) == 2
    verified = 0
    for a,b in zip(old['runs'], current['runs']):
        assert len(a['shots']) == len(b['shots']) == 50
        for old_shot,new_shot in zip(a['shots'], b['shots']):
            for key,value in old_shot.items():
                if isinstance(value,(float,int)) and not isinstance(value,bool):
                    assert abs(value-new_shot[key]) < 1e-9, key
                else:
                    assert value == new_shot[key], key
            verified += 1
    raw.write_text(json.dumps(current, indent=2)+'\n')
    result = {'historical_shots_verified':verified, 'tolerance':1e-9,
        'source_sha256':digest(source), 'old_data_used_only_for_compatibility':True,
        'inputs':inputs, 'trace_sha256':digest(raw)}
    receipt.write_text(json.dumps(result, indent=2)+'\n')
    return result


def main():
    records = {}
    report = {'schema': 1, 'physical_device_opened': False, 'stages': {}, 'complete': False}
    release = json.loads((ROOT/'logs/simulation-deployment-validation.json').read_text())
    assert digest(ROOT/'configs/standard.yaml') == release['formal_yaml_sha256']
    assert digest(ROOT/'build/standard') == release['formal_binary_sha256']
    cfg = yaml.safe_load((ROOT/'configs/standard.yaml').read_text())
    assert cfg['corner_refine_brightness_invariant_subpixel'] is False
    interval = cfg['sentinel_fire_min_interval_ms']/1000
    expected_counts = {'development': 3, 'holdout': 2, 'stress': 4}
    all_seeds = [s for seeds, phases in SPLITS.values() for s in seeds]
    all_phases = [p for seeds, phases in SPLITS.values() for p in phases]
    assert len(all_seeds) == len(set(all_seeds)) and len(all_phases) == len(set(all_phases))
    # Known prior image studies must not supply a development or holdout seed.
    historical_seeds = set()
    for name in ('endpoint-deployment-images.json', 'endpoint-ablation.json',
                 'endpoint-formal-policy.json', 'fixed-target-selective-images.json',
                 'fixed-target-timing-image-holdout.json'):
        data = json.loads((ROOT/'logs'/name).read_text())
        historical_seeds.update(r['seed'] for e in data['experiments'] for r in e['runs'])
    assert not historical_seeds.intersection(all_seeds)
    for stage in SPLITS:
        path = ROOT/'logs'/('centered-opportunity-'+stage+'.json')
        data = json.loads(path.read_text())
        records[stage] = data
        assert data['complete'] and data['stage'] == stage
        assert len(data['experiments']) == expected_counts[stage]
        assert data['physical_device_opened'] is False and data['truth_supplied_to_predictor'] is False
        for name, value in data['sha256'].items():
            assert digest(ROOT/name) == value, name
        if stage != 'development':
            assert data['development_sha256'] == digest(ROOT/'logs/centered-opportunity-development.json')
            assert data['frozen_selection'] == records['development']['frozen_selection']
        stages = []
        for e in data['experiments']:
            assert e['summary'] == metrics(e)
            case, ctl = e['scenario'], e['scenario']['controller']
            assert case['target']['hp'] == 300 and case['target']['ammo'] == 50
            assert case['target']['damage_per_hit'] == 10
            assert case['sampling']['duration_s'] == 60
            assert case['observation']['mode'] == 'rendered_yolo26'
            assert not ctl['center_crossing_schedule'] and not ctl['prefer_smaller_radius']
            assert ctl['prediction_time_offset_s'] == ctl['elevation_trim_deg'] == 0
            assert ctl['directional_prediction_offsets_s'] == {'-1': 0., '1': 0.}
            assert ctl['apply_visual_residual_correction'] is True
            assert ctl['center_phase_limit_deg'] == e['center_phase_limit_deg']
            assert case['sampling']['seeds'] == SPLITS[stage][0]
            assert case['target']['initial_phases_rad'] == [math.radians(p) for p in SPLITS[stage][1]]
            assert len(e['runs']) == len(SPLITS[stage][0])*2*len(SPLITS[stage][1])
            baseline = next(x for x in data['experiments'] if x['policy'] == 'baseline'
                            and x['scenario']['plant'] == case['plant'])
            for key in ('sampling', 'target', 'plant'):
                assert case[key] == baseline['scenario'][key]
            assert case['plant']['min_normal_speed_mps'] == 0
            assert {k:v for k,v in case['observation'].items() if k != 'preview_directory'} == {
                k:v for k,v in baseline['scenario']['observation'].items() if k != 'preview_directory'}
            assert {k:v for k,v in ctl.items() if k != 'center_phase_limit_deg'} == {
                k:v for k,v in baseline['scenario']['controller'].items() if k != 'center_phase_limit_deg'}
            complete_in_time = 0
            per_run = []
            for r, b in zip(e['runs'], baseline['runs']):
                assert (r['seed'], r['direction'], r['initial_phase_rad']) == (b['seed'], b['direction'], b['initial_phase_rad'])
                shots = r['shots']
                assert len(shots) == r['issued'] <= 50
                assert r['valid_hits'] == sum(s['valid_hit'] for s in shots)
                assert r['geometric_hits'] == sum(s['geometric_hit'] for s in shots)
                assert r['hp'] == max(0, 300-10*r['valid_hits'])
                assert r['budget_hit_fraction'] == r['valid_hits']/50
                hits = sorted(s['settle_s'] for s in shots if s['valid_hit'])
                assert r['kill_s'] == (hits[29] if len(hits) >= 30 else -1)
                complete_in_time += sum(t <= 60 for t in hits) >= 30
                for s, bs in zip(shots, b['shots']):
                    assert s['prediction_time_offset_s'] == 0 and s['visual_correction_applied']
                    assert s['subframe_wait_s'] == 0
                    assert 0 <= s['capture_s'] <= s['decision_s']+1e-9
                    assert abs(s['issue_s']-s['decision_s']) < 1e-9
                    assert s['issue_s'] < 60 and s['muzzle_s'] >= s['issue_s']
                    if ctl['center_phase_limit_deg']:
                        assert abs(math.degrees(math.remainder(s['estimated_phase_rad'], 2*math.pi))) <= ctl['center_phase_limit_deg']+1e-9
                    # Same random physical perturbations by shot index; no
                    # fabricated paired camera samples after histories diverge.
                    assert abs(s['physical_speed_mps']-bs['physical_speed_mps']) < 1e-10
                    assert abs((s['muzzle_s']-s['issue_s'])-(bs['muzzle_s']-bs['issue_s'])) < 1e-10
                    for axis, field in (('yaw', 'world_yaw'), ('elevation', 'world_elevation')):
                        assert abs((s[field]-s['muzzle_servo_'+axis+'_rad'])-
                                   (bs[field]-bs['muzzle_servo_'+axis+'_rad'])) < 1e-10
                for a, bshot in zip(shots, shots[1:]):
                    assert bshot['issue_s']-a['issue_s'] >= interval-1e-9
                per_run.append({'seed': r['seed'], 'direction': r['direction'],
                    'phase_deg': math.degrees(r['initial_phase_rad']), 'issued': r['issued'],
                    'hits': r['valid_hits'], 'kill_s': r['kill_s']})
            stages.append({'name':e['name'], 'summary':e['summary'],
                'complete_within_60s_runs': int(complete_in_time), 'per_run':per_run})
        independent = analyze(data)
        report['stages'][stage] = {'experiments':stages, 'independent_collision_shots':len(independent['shots']),
            'miss_axes':independent['experiments'], 'result_sha256':digest(path)}
    dev = records['development']
    chosen = next(e for e in dev['experiments'] if e['name'] == dev['frozen_selection']['name'])
    assert dev['frozen_selection']['development_eligible'] == development_eligible(chosen['summary'], dev['experiments'][0]['summary'])
    accepted = dev['frozen_selection']['development_eligible']
    comparisons = []
    for stage in ('holdout', 'stress'):
        for candidate in (e for e in records[stage]['experiments'] if e['policy'] != 'baseline'):
            base = next(e for e in records[stage]['experiments'] if e['policy'] == 'baseline'
                        and e['scenario']['plant'] == candidate['scenario']['plant'])
            a,b = candidate['summary'],base['summary']
            nonregression = (a['valid_hits'] >= b['valid_hits'] and
                a['worst_run_budget_hit_fraction'] >= b['worst_run_budget_hit_fraction'] and
                a['zero_hp_runs'] == a['full_budget_runs'] == a['runs'] and
                a['max_kill_s'] is not None and a['max_kill_s'] <= 60 and
                all(a['direction_summary'][str(d)]['valid_hits'] >= b['direction_summary'][str(d)]['valid_hits'] for d in (-1,1)))
            accepted = accepted and nonregression
            comparisons.append({'stage':stage, 'name':candidate['name'], 'hit_delta':a['valid_hits']-b['valid_hits'],
                'nonregression_passed':nonregression})
    # Source tests belong to this run, not a previous LastTest.log snapshot.
    text = (ROOT/'logs/centered-opportunity-tests.log').read_text()
    assert '100% tests passed, 0 tests failed out of 54' in text
    last = ROOT/'build/native-offline/Testing/Temporary/LastTest.log'
    assert last.read_text().count('Test Passed.') == 54
    report.update(complete=True, ctest_passed=54, frozen_selection=dev['frozen_selection'],
        comparisons=comparisons, offline_candidate_accepted=bool(accepted),
        formal_program_and_config_unchanged=True, deployed=False,
        ctest_sha256=digest(last), auditor_sha256=digest(Path(__file__)))
    report['disabled_filter_replay'] = default_replay()
    out = ROOT/'logs/centered-opportunity-audit.json'
    out.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'stages'}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--default-replay-only', action='store_true')
    args = parser.parse_args()
    if args.default_replay_only:
        print(json.dumps(default_replay()))
    else:
        main()
