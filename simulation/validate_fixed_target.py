#!/usr/bin/env python3
"""Independent audit of result accounting, causal shot times and build identity."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeat",action="store_true")
    parser.add_argument("--result",type=Path,default=ROOT/"logs/fixed-target-simulation.json")
    parser.add_argument("--historical-contracts-only",action='store_true',
                        help='audit old numerical contracts while explicitly recording current source mismatches')
    args=parser.parse_args()
    result_path=args.result.resolve()
    data=json.loads(result_path.read_text(encoding="utf-8"))
    selective=data.get('comparison_type')=='selective_fire'
    # Runners checkpoint progress after each candidate. A partial file is useful
    # for inspection, but must never receive a completed-experiment audit.
    if selective:
        expected=({'baseline','elevation_only','selective'} if data['split']=='development geometry screening' else
                  {'baseline','elevation_only','visual_elevation'} if data['split']=='development image screening' else {'baseline','refined'})
        assert {e['name'] for e in data['experiments']}==expected and len(data['experiments'])==len(expected)
    elif data.get('frozen_directional_selection'):
        assert len(data['experiments'])==2
        assert {e['name'] for e in data['experiments']}=={'baseline','directional'}
    elif data.get('frozen_selection'):
        assert len(data['experiments'])==(6 if 'delay-sensitivity' in result_path.name else 2)
    elif 'timing-geometry-development' in result_path.name:
        assert 'selection' in data and len(data['experiments'])==8
    image_input=data.get("kind")=="fixed_target_images"
    if image_input:assert data["ground_truth_fallback"] is False
    assert data["objective"]=={"hp":300,"ammo":50,"required_hits":30,
                              "minimum_budget_hit_fraction":.6,"stop_on_truth_hp":False}
    assert not data["physical_device_opened"] and not data["truth_supplied_to_predictor"]
    total_runs=total_shots=0
    scheduled_shots=0
    for experiment in data["experiments"]:
        target=experiment["scenario"]["target"]
        assert target["hp"]==300 and target["ammo"]==50
        damage=target["damage_per_hit"]
        for run in experiment["runs"]:
            if image_input:
                assert run["observation_mode"]=="rendered_yolo26"
                stats=run["image_stats"]
                assert stats["detector_frames"]>0
                assert 0<=stats["usable_pose_faces"]<=stats["correct_label_faces"]<=stats["matched_faces"]<=stats["visible_truth_faces"]
                if experiment["name"]=="blank":
                    assert run["issued"]==run["valid_hits"]==stats["matched_faces"]==stats["visible_truth_faces"]==0
            total_runs+=1;total_shots+=run["issued"]
            shots=run["shots"]
            assert len(shots)==run["issued"]<=target["ammo"]
            assert [s["id"] for s in shots]==list(range(1,len(shots)+1))
            valid=sorted((s for s in shots if s["valid_hit"]),key=lambda s:s["settle_s"])
            assert len(valid)==run["valid_hits"]
            assert sum(s["geometric_hit"] for s in shots)==run["geometric_hits"]
            assert run["hp"]==max(0,target["hp"]-len(valid)*damage)
            assert run["geometric_hits_scored"]==min(30,len(valid))
            assert math.isclose(run["budget_hit_fraction"],len(valid)/50)
            if len(valid)>=30:
                assert math.isclose(run["kill_s"],valid[29]["settle_s"],abs_tol=1e-9)
                assert sum(s["cleared_hp"] for s in shots)==1
            else:assert run["kill_s"]==-1
            last_issue=-math.inf
            for shot in shots:
                assert shot["issue_s"]-last_issue>=data["algorithm_parameters"]["sentinel_fire_min_interval_ms"]/1000-1e-9
                last_issue=shot["issue_s"]
                assert shot["muzzle_s"]>=shot["issue_s"] and shot["settle_s"]>=shot["muzzle_s"]
                if 'subframe_wait_s' in shot:
                    wait=shot['subframe_wait_s'];scheduled_shots+=wait>1e-9
                    assert wait>=0 and wait<=1/experiment['scenario']['sampling']['fps']+1e-9
                    assert shot['decision_s']>=shot['capture_s']
                    assert math.isclose(shot['issue_s'],shot['decision_s']+wait,abs_tol=1e-9)
                    if wait>0:
                        assert shot['issue_s']-shot['capture_s']<=1/experiment['scenario']['sampling']['fps']+1e-9
                if selective:
                    ctl=experiment['scenario']['controller']
                    assert shot['elevation_trim_deg']==ctl['elevation_trim_deg']
                    assert shot['prefer_smaller_radius']==ctl['prefer_smaller_radius']
                    assert shot['visual_correction_applied']==ctl['apply_visual_residual_correction']
                    assert math.isfinite(shot['visual_residual_correction_rad'])
                    assert abs(shot['visual_residual_correction_rad'])<=.6*math.pi/180+1e-10
                    if ctl['prefer_smaller_radius']:
                        r0,r1=shot['estimated_radius0_m'],shot['estimated_radius1_m']
                        assert all(math.isfinite(r) and .05<r<.5 for r in (r0,r1))
                        if abs(r0-r1)>.003:
                            assert shot['selected_model_face']%2==(0 if r0<r1 else 1)
                assert shot["issue_s"]>=run["first_quality_ready_s"]>=run["first_fixed_ready_s"]>=0
                if shot["valid_hit"]:assert shot["geometric_hit"]
                if shot["geometric_hit"]:
                    assert 0<=shot["truth_hit_face"]<4
                    assert abs(shot["hit_u_m"])<=target["armor_width_m"]/2+1e-9
                    assert abs(shot["hit_v_m"])<=target["armor_hit_height_m"]/2+1e-9
        summary=experiment["summary"]
        assert summary["issued"]==sum(r["issued"] for r in experiment["runs"])
        assert summary["zero_hp_runs"]==sum(r["valid_hits"]>=30 for r in experiment["runs"])
    mismatched=[source for source,expected in data['sha256'].items() if sha(ROOT/source)!=expected]
    if not args.historical_contracts_only:assert not mismatched,mismatched
    paired_runs=0
    if 'timing-' in result_path.name or data.get('frozen_directional_selection') or selective:
        experiments=data['experiments']
        controls=[e for e in experiments if e['name'].startswith('baseline')]
        assert controls
        for experiment in experiments:
            if experiment in controls:continue
            control=next(e for e in controls if e['scenario']['plant']==experiment['scenario']['plant'])
            for section in ('target','sampling','plant'):
                assert control['scenario'][section]==experiment['scenario'][section],section
            assert len(control['runs'])==len(experiment['runs'])
            for a,b in zip(control['runs'],experiment['runs']):
                paired_runs+=1
                assert all(a[k]==b[k] for k in ('seed','direction','initial_phase_rad'))
                # Shot-indexed disturbances are shared, independent of trigger
                # time; do not improve a score by drawing easier ammunition.
                for x,y in zip(a['shots'],b['shots']):
                    assert math.isclose(x['physical_speed_mps'],y['physical_speed_mps'],abs_tol=1e-10)
                    assert math.isclose(x['muzzle_s']-x['issue_s'],y['muzzle_s']-y['issue_s'],abs_tol=1e-10)
        if data.get('split')=='heldout seeds and phases':
            development=json.loads((ROOT/'logs/fixed-target-timing-geometry-development.json').read_text())
            assert data['frozen_selection']==development['selection']
            old=development['experiments'][0]['runs'];new=experiments[0]['runs']
            assert not ({r['seed'] for r in old}&{r['seed'] for r in new})
            assert not ({r['initial_phase_rad'] for r in old}&{r['initial_phase_rad'] for r in new})
        if data.get('frozen_directional_selection'):
            development=json.loads((ROOT/data['selection_source']).read_text())
            for sign in (-1,1):
                best=max(development['experiments'],key=lambda e:sum(r['valid_hits'] for r in e['runs'] if r['direction']==sign))
                assert data['frozen_directional_selection'][str(sign)]==best['scenario']['controller']['prediction_time_offset_s']
            old=development['experiments'][0]['runs'];new=experiments[0]['runs']
            assert not ({r['seed'] for r in old}&{r['seed'] for r in new})
            assert not ({r['initial_phase_rad'] for r in old}&{r['initial_phase_rad'] for r in new})
            # The second validation is also independent of the original
            # geometry development cohort, not merely of the rejected test.
            assert not ({r['seed'] for r in new}&{27,83,149})
            for experiment in experiments:
                offsets=experiment['scenario']['controller']['directional_prediction_offsets_s']
                for run in experiment['runs']:
                    for shot in run['shots']:
                        # Selection must use the measured model rate, not the
                        # known world direction used only in report labels.
                        sign='1' if shot['estimated_omega_rad_s']>0 else '-1'
                        assert shot['prediction_time_offset_s']==offsets.get(sign,0.)
        if selective:
            original=json.loads((ROOT/data['calibration_source']).read_text())
            timing=json.loads((ROOT/data['timing_source']).read_text())
            assert data['fixed_directional_offsets_s']==timing['frozen_directional_selection']
            assert abs(data['calibration']['elevation_trim_deg'])<=.3
            assert data['calibration']['samples']>=500
            for experiment in experiments:
                ctl=experiment['scenario']['controller']
                assert ctl['directional_prediction_offsets_s']==data['fixed_directional_offsets_s']
                for run in experiment['runs']:
                    for shot in run['shots']:
                        sign='1' if shot['estimated_omega_rad_s']>0 else '-1'
                        assert shot['prediction_time_offset_s']==data['fixed_directional_offsets_s'][sign]
            if data['split']=='selective heldout seeds and phases':
                old=original['experiments'][0]['runs']+timing['experiments'][0]['runs']
                new=experiments[0]['runs']
                assert not ({r['seed'] for r in new}&({r['seed'] for r in old}|{27,83,149}))
                assert not ({r['initial_phase_rad'] for r in new}&({r['initial_phase_rad'] for r in old}|{0.,math.pi/4}))
                selection=data['frozen_policy']
                screen=json.loads((ROOT/selection['source']).read_text())
                best=max(screen['experiments'],key=lambda e:(e['summary']['zero_hp_runs'],e['summary']['valid_hits'],
                    e['summary']['worst_run_budget_hit_fraction'],-(e['summary']['max_kill_s'] or 999.)))
                assert selection['name']==best['name'] and selection['controller']==best['scenario']['controller']
                assert experiments[1]['scenario']['controller']==selection['controller']
                screen_runs=screen['experiments'][0]['runs']
                assert not ({r['seed'] for r in new}&{r['seed'] for r in screen_runs})
                assert not ({r['initial_phase_rad'] for r in new}&{r['initial_phase_rad'] for r in screen_runs})
    repeat=False
    if args.repeat:
        assert not image_input,"image timing fields are wall-clock samples; use image contract audit without --repeat"
        with tempfile.TemporaryDirectory(prefix="rm-fixed-audit-") as folder:
            scenario=Path(folder)/"scenario.json";output=Path(folder)/"result.json"
            baseline=data["experiments"][0]
            scenario.write_text(json.dumps(baseline["scenario"]),encoding="utf-8")
            subprocess.run([str(ROOT/"build/native-offline/bin/fixed_target_sim"),str(ROOT/"configs/standard.yaml"),
                            str(scenario),str(output)],cwd=ROOT,stdout=subprocess.DEVNULL,check=True,timeout=180)
            assert json.loads(output.read_text())["runs"]==baseline["runs"]
            repeat=True
    test_log=ROOT/"build/native-offline/Testing/Temporary/LastTest.log"
    log=test_log.read_text();passed=log.count("Test Passed.")
    assert passed>=54 and 'fixed_target_renderer_test' in log and not log.count("Test Failed.")
    manifest=ROOT/"build/runtime-source-manifest.txt"
    for line in manifest.read_text().splitlines():
        source,expected=line.rsplit(":",1);assert sha(ROOT/source)==expected,source
    # Read binary strings only: never launch the production robot executable.
    source_identity=sha(manifest)
    assert source_identity.encode() in (ROOT/"build/standard").read_bytes()
    audit={"schema":2,"runs_audited":total_runs,"shots_audited":total_shots,
           "input_mode":"actual_yolo26_images" if image_input else "synthetic_keypoints",
           "hp":300,"ammo":50,"required_hits":30,"accounting_and_causal_times_passed":True,
           "source_fingerprints_match":not mismatched,"deterministic_baseline_repeated":repeat,
           "ctest_passed":passed,"production_build_identity":source_identity,
           "production_binary_executed":False,"simulation_result_sha256":sha(result_path)}
    audit.update(scheduled_shots=scheduled_shots,paired_controller_runs_audited=paired_runs)
    audit.update(historical_contracts_only=args.historical_contracts_only,
                 current_source_mismatches=mismatched)
    audit.update(validator_sha256=sha(Path(__file__)),ctest_log_sha256=sha(test_log))
    audit["additional_simulation_source_sha256"]={str(p.relative_to(ROOT)):sha(p) for p in
        (ROOT/'simulation/src/fixed_target_camera.hpp',ROOT/'simulation/src/fixed_target_renderer.hpp',
         ROOT/'simulation/src/fixed_target_timing.hpp')}
    output=ROOT/'logs'/(result_path.stem.removesuffix('-simulation')+'-validation.json')
    output.write_text(json.dumps(audit,indent=2)+"\n")
    print(json.dumps(audit))


if __name__=="__main__":main()
