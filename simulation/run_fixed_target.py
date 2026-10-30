#!/usr/bin/env python3
"""Run the file-only production-geometry harness and create an offline HTML replay.

No camera, serial, ROS, service, mission, network or motor interfaces are opened.
The only production configuration read is configs/standard.yaml.
"""
import argparse
import copy
import datetime as dt
import hashlib
import json
import pathlib
import subprocess
import tempfile
import math

ROOT = pathlib.Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summarize(experiment):
    runs = experiment["runs"]
    issued = sum(r["issued"] for r in runs)
    collisions = sum(sum(s["geometric_hit"] for s in r["shots"]) for r in runs)
    valid=sum(r["valid_hits"] for r in runs)
    fractions=sorted(r["budget_hit_fraction"] for r in runs)
    return {
        "runs": len(runs), "issued": issued, "geometric_collisions": collisions,
        "damage_hits": sum(r["geometric_hits_scored"] for r in runs),
        "zero_hp_runs": sum(r["hp"] == 0 for r in runs),
        "geometric_collision_fraction": collisions / issued if issued else None,
        "valid_hits": valid, "budget_rounds":sum(r["ammo_budget"] for r in runs),
        "budget_hit_fraction":valid/sum(r["ammo_budget"] for r in runs),
        "worst_run_budget_hit_fraction":fractions[0],
        "p10_run_budget_hit_fraction":fractions[max(0,math.ceil(.1*len(fractions))-1)],
        "full_budget_runs":sum(r["issued"]==r["ammo_budget"] for r in runs),
        "direction_summary":{str(sign):{"runs":sum(r["direction"]==sign for r in runs),
            "complete":sum(r["hp"]==0 and r["direction"]==sign for r in runs),
            "valid_hits":sum(r["valid_hits"] for r in runs if r["direction"]==sign)} for sign in (-1,1)},
        "max_kill_s": max((r["kill_s"] for r in runs if r["kill_s"] >= 0), default=None),
    }


def render(result, path):
    template = (ROOT / "simulation/fixed_target_report.html").read_text(encoding="utf-8")
    # Do not permit a JSON string to close the script element in the template.
    data = json.dumps(result, ensure_ascii=False, separators=(",", ":")).replace("<", "\\u003c")
    path.write_text(template.replace("/*__SIM_DATA__*/null", data), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=pathlib.Path, default=ROOT / "build/native-offline/bin/fixed_target_sim")
    parser.add_argument("--scenario", type=pathlib.Path, default=ROOT / "simulation/fixed_target.json")
    parser.add_argument("--config", type=pathlib.Path, default=ROOT / "configs/standard.yaml")
    parser.add_argument("--output", type=pathlib.Path, default=ROOT / "logs/fixed-target-simulation.json")
    parser.add_argument("--report", type=pathlib.Path, default=ROOT / "logs/fixed-target-simulation.html")
    parser.add_argument("--suite", action="store_true", help="also test independent +/-80ms muzzle delay, jitter, and pixel noise")
    parser.add_argument("--stress", action="store_true", help="also test mixed synthetic errors with three independent seeds")
    parser.add_argument("--delay-sweep", action="store_true", help="sweep physical muzzle delay 250..410ms with fixed predictor")
    parser.add_argument("--seeds", help="comma separated random seeds; stress default:27,83,149")
    parser.add_argument("--report-only", action="store_true", help="render an existing --output without running C++")
    parser.add_argument("--muzzle-delay-ms", type=float, help="override physical plant only, not predictor")
    parser.add_argument("--jitter-ms", type=float)
    parser.add_argument("--noise-px", type=float)
    parser.add_argument("--fps", type=float)
    parser.add_argument("--duration", type=float)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    if args.report_only:
        result = json.loads(args.output.read_text(encoding="utf-8"))
    else:
        scenario = json.loads(args.scenario.read_text(encoding="utf-8"))
        if args.seeds:
            scenario["sampling"]["seeds"]=[int(x) for x in args.seeds.split(",")]
        for value, section, key, scale in [
            (args.muzzle_delay_ms, "plant", "muzzle_delay_s", .001),
            (args.jitter_ms, "plant", "muzzle_delay_jitter_s", .001),
            (args.noise_px, "sampling", "keypoint_noise_px", 1),
            (args.fps, "sampling", "fps", 1),
            (args.duration, "sampling", "duration_s", 1),
        ]:
            if value is not None:
                scenario[section][key] = value * scale
        cases = [("baseline", "理想基线", scenario)]
        if args.suite:
            for name, label, section, key, value in [
                ("delay_minus_80ms", "实际出膛提前 80ms", "plant", "muzzle_delay_s", scenario["plant"]["muzzle_delay_s"]-.08),
                ("delay_plus_80ms", "实际出膛滞后 80ms", "plant", "muzzle_delay_s", scenario["plant"]["muzzle_delay_s"]+.08),
                ("jitter_15ms", "出膛时刻随机抖动 σ=15ms", "plant", "muzzle_delay_jitter_s", .015),
                ("noise_1px", "角点独立高斯噪声 σ=1px", "sampling", "keypoint_noise_px", 1.0),
            ]:
                case = copy.deepcopy(scenario)
                case[section][key] = value
                cases.append((name, label, case))
        stress_seeds=[int(x) for x in args.seeds.split(",")] if args.seeds else [27,83,149]
        if args.stress:
            profiles=[
                ("servo_noise", "有限云台+噪声/丢帧", {"keypoint_noise_px":1.0,"frame_dropout_probability":.05},
                 {"servo_time_constant_s":.06,"muzzle_delay_jitter_s":.015,"speed_stddev_mps":.2,
                  "yaw_spread_stddev_deg":.1,"pitch_spread_stddev_deg":.12}),
                ("mixed_moderate", "组合误差·中等假设", {"keypoint_noise_px":1.0,"frame_dropout_probability":.05,
                 "frame_age_jitter_s":.004,"timestamp_bias_s":.005,"pose_yaw_bias_deg":.15},
                 {"servo_time_constant_s":.06,"muzzle_delay_s":.34,"muzzle_delay_jitter_s":.025,
                  "speed_stddev_mps":.3,"yaw_spread_stddev_deg":.12,"pitch_spread_stddev_deg":.2,
                  "barrel_yaw_bias_deg":.1,"barrel_pitch_bias_deg":.08}),
                ("mixed_hard", "组合误差·较强假设", {"keypoint_noise_px":1.5,"frame_dropout_probability":.15,
                 "frame_age_jitter_s":.008,"timestamp_bias_s":.01,"pose_yaw_bias_deg":.3},
                 {"servo_time_constant_s":.12,"muzzle_delay_s":.35,"muzzle_delay_jitter_s":.04,
                  "speed_stddev_mps":.7,"yaw_spread_stddev_deg":.3,"pitch_spread_stddev_deg":.55,
                  "barrel_yaw_bias_deg":.15,"barrel_pitch_bias_deg":.12}),
                ("hard_low_spread", "较强假设·只减小弹丸散布", {"keypoint_noise_px":1.5,"frame_dropout_probability":.15,
                 "frame_age_jitter_s":.008,"timestamp_bias_s":.01,"pose_yaw_bias_deg":.3},
                 {"servo_time_constant_s":.12,"muzzle_delay_s":.35,"muzzle_delay_jitter_s":.04,
                  "speed_stddev_mps":.7,"yaw_spread_stddev_deg":.1,"pitch_spread_stddev_deg":.15,
                  "barrel_yaw_bias_deg":.15,"barrel_pitch_bias_deg":.12}),
                ("hard_low_jitter", "较强假设·只减小时序抖动", {"keypoint_noise_px":1.5,"frame_dropout_probability":.15,
                 "frame_age_jitter_s":.008,"timestamp_bias_s":.01,"pose_yaw_bias_deg":.3},
                 {"servo_time_constant_s":.12,"muzzle_delay_s":.35,"muzzle_delay_jitter_s":.01,
                  "speed_stddev_mps":.7,"yaw_spread_stddev_deg":.3,"pitch_spread_stddev_deg":.55,
                  "barrel_yaw_bias_deg":.15,"barrel_pitch_bias_deg":.12}),
            ]
            for name,label,sampling,plant in profiles:
                case=copy.deepcopy(scenario);case["sampling"].update(sampling,seeds=stress_seeds);case["plant"].update(plant)
                case["provenance"]["stress_profile"]="Synthetic assumed distributions for sensitivity/ablation; NOT measured real hardware. No learned tuning deployed."
                cases.append((name,label,case))
            legacy=copy.deepcopy(next(case for name,label,case in cases if name=="servo_noise"))
            legacy["controller"]={"retain_short_loss_context":False}
            cases.append(("short_loss_legacy","相同误差·旧短丢帧调用对照",legacy))
        if args.delay_sweep:
            for ms in range(250,411,20):
                case=copy.deepcopy(scenario);case["plant"]["muzzle_delay_s"]=ms/1000
                case["sampling"]["seeds"]=stress_seeds
                cases.append((f"sweep_{ms}ms",f"物理延迟扫描 {ms}ms",case))
        result = {"schema": 3, "generated_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
                  "scope": "synthetic keypoints, production prediction helpers, uncertainty and bounded servo; full50 budget, HP300; not full robot",
                  "physical_device_opened": False, "truth_supplied_to_predictor": False,
                  "experiments": [], "sha256": {}}
        required=math.ceil(scenario["target"]["hp"]/scenario["target"]["damage_per_hit"])
        result["objective"]={"hp":scenario["target"]["hp"],"ammo":scenario["target"]["ammo"],
            "required_hits":required,"minimum_budget_hit_fraction":required/scenario["target"]["ammo"],
            "stop_on_truth_hp":False}
        result["independent_shot_probability_illustration"]={"assumption":"illustrative independent identical hit probability, not empirical confidence",
            "probability_of_at_least_required_hits":{str(p):sum(math.comb(scenario["target"]["ammo"],k)*p**k*(1-p)**(scenario["target"]["ammo"]-k)
                for k in range(required,scenario["target"]["ammo"]+1)) for p in (.6,.7,.8)}}
        for source in [args.config, args.scenario, args.executable,
                       ROOT / "simulation/src/fixed_target_sim.cpp", ROOT / "simulation/src/fixed_target_world.hpp",
                       ROOT / "simulation/src/fixed_target_plant.hpp",ROOT / "simulation/run_fixed_target.py",
                       ROOT / "simulation/fixed_target_report.html",
                       ROOT / "tasks/auto_aim/tracker.cpp", ROOT / "tasks/auto_aim/solver.cpp",
                       ROOT / "tasks/auto_aim/target.cpp", ROOT / "tasks/auto_aim/aimer.cpp",
                       ROOT / "tasks/auto_aim/fixed_center_model.hpp", ROOT / "tasks/auto_aim/high_speed_vision.hpp",
                       ROOT / "tasks/auto_aim/face_prediction_residual.hpp", ROOT / "tasks/auto_aim/rotation_fire_window.hpp"]:
            result["sha256"][str(source.relative_to(ROOT)) if source.is_relative_to(ROOT) else str(source)] = digest(source)
        # Read for audit only. These are not altered or passed back into the plant.
        import yaml
        config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
        result["algorithm_parameters"] = {key: config.get(key) for key in (
            "xuc_high_speed_strategy", "high_speed_delay_time_cw", "high_speed_delay_time_ccw",
            "sentinel_fire_min_interval_ms", "yaw_offset", "pitch_offset",
            "xuc_direct_yaw_impact_correction_deg", "xuc_direct_yaw_boresight_correction_deg",
            "xuc_high_speed_prediction_quality_max_error_deg")}
        with tempfile.TemporaryDirectory(prefix="rm-fixed-target-") as temporary:
            folder = pathlib.Path(temporary)
            for name, label, case in cases:
                scenario_path, output_path = folder / "scenario.json", folder / "result.json"
                scenario_path.write_text(json.dumps(case, ensure_ascii=False), encoding="utf-8")
                completed = subprocess.run([str(args.executable.resolve()), str(args.config.resolve()),
                                            str(scenario_path), str(output_path)],
                                           cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                           text=True, timeout=180)
                if completed.returncode:
                    raise RuntimeError(f"{name}: harness failed ({completed.returncode})\n{completed.stdout[-4000:]}")
                experiment = json.loads(output_path.read_text(encoding="utf-8"))
                experiment.update(name=name, label=label)
                experiment["summary"] = summarize(experiment)
                result["experiments"].append(experiment)
                print(name, json.dumps(experiment["summary"], ensure_ascii=False), flush=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    render(result, args.report)
    print("JSON:", args.output)
    print("HTML:", args.report)


if __name__ == "__main__":
    main()
