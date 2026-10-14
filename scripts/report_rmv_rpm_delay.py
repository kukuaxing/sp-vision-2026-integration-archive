#!/usr/bin/env python3
"""Offline per-shot wheel-drop timing; never opens a device or changes control."""
import argparse
import bisect
import json
import math
import statistics
from pathlib import Path


def distribution(values):
    if not values:
        return None
    return dict(samples=len(values), mean_ms=statistics.mean(values),
                median_ms=statistics.median(values), min_ms=min(values), max_ms=max(values),
                stdev_ms=statistics.stdev(values) if len(values)>1 else 0.)


def analyze(rows, min_drop_rpm=50., pair_gap_ms=15.):
    if not math.isfinite(min_drop_rpm) or min_drop_rpm<=0 or not 0<=pair_gap_ms<=100:
        raise ValueError("Invalid drop threshold / pair gap")
    commands={}; events={}; samples=[]
    for r in rows:
        event=r.get("event")
        if event=="command_issued":
            commands[r["request_id"]]=r
        elif event=="shot_event":
            events.setdefault(r["request_id"],{})[r["shot_event"]]=r
        elif event=="wheel_sample":
            rpm=r.get("rpm",[])
            ages=r.get("feedback_age_us")
            fresh=ages is None or (len(ages)>=2 and max(ages[:2])<=20000)
            if fresh and len(rpm)>=2 and r.get("valid_flags",0)&3==3 and all(isinstance(x,(int,float)) and math.isfinite(x) for x in rpm[:2]):
                samples.append(r)
    samples.sort(key=lambda r:r["mcu_us"])
    samples=list({r["mcu_us"]:r for r in samples}.values())
    times=[r["mcu_us"] for r in samples]
    deltas=[(b-a)/1000 for a,b in zip(times,times[1:]) if b>a]
    result=[]
    for ident, command in sorted(commands.items()):
        shot=events.get(ident,{})
        row=dict(request_id=ident, status="missing_shot_markers", evidence="wheel-speed drop proxy",
                 actual_projectile_exit_detected=False,
                 clock_uncertainty_us=command.get("clock_uncertainty_us"))
        result.append(row)
        if 0 not in shot or 1 not in shot:
            continue
        accepted=shot[0]["mcu_us"]; started=shot[1]["mcu_us"]
        completed=shot.get(2,{}).get("mcu_us")
        if completed:
            trace=[r for r in samples if started-10000<=r["mcu_us"]<=completed+15000 and "feeder" in r]
            if trace:
                first=min(trace,key=lambda r:abs(r["mcu_us"]-started))
                last=min(trace,key=lambda r:abs(r["mcu_us"]-(completed+5000)))
                row["feeder_diagnostics"]={
                    "software_start_to_completion_ms":(completed-started)/1000,
                    "position_change_counts_approx":last["feeder"]["position_counts"]-first["feeder"]["position_counts"],
                    "final_target_error_counts":last["feeder"]["target_counts"]-last["feeder"]["position_counts"],
                    "rpm_min":min(r["rpm"][2] for r in trace),
                    "rpm_max":max(r["rpm"][2] for r in trace),
                    "peak_command_current_native":max(abs(r["feeder"]["command_current_native"]) for r in trace),
                    "peak_feedback_current_native":max(abs(r["feeder"]["feedback_current_native"]) for r in trace),
                    "states":sorted(set(r["feeder"]["state"] for r in trace)),
                    "motor_modes":sorted(set(r["feeder"]["motor_mode"] for r in trace)),
                    "unexpected_idle_samples":sum(r["feeder"]["state"]==3 and r["feeder"]["motor_mode"]==6
                        and started<=r["mcu_us"]<completed-20000 for r in trace),
                    "faults":sorted(set(r["feeder"]["fault"] for r in trace)),
                    "feedback_max_age_us":max(r["feedback_age_us"][2] for r in trace)}
        before=samples[bisect.bisect_left(times,accepted-300000):bisect.bisect_left(times,accepted-10000)]
        if len(before)<20 or before[-1]["mcu_us"]-before[0]["mcu_us"]<150000:
            row["status"]="insufficient_baseline";continue
        baseline=[statistics.median(abs(r["rpm"][k]) for r in before) for k in (0,1)]
        sigma=[1.4826*statistics.median(abs(abs(r["rpm"][k])-baseline[k]) for r in before) for k in (0,1)]
        thresholds=[max(min_drop_rpm,6*s) for s in sigma]
        row.update(baseline_rpm=baseline, baseline_noise_sigma_rpm=sigma, drop_threshold_rpm=thresholds)
        if any(b<6200 or b>6800 for b in baseline) or any(s>50 for s in sigma):
            row["status"]="unstable_baseline";continue
        next_times=[c["mcu_us"] for i,c in commands.items() if i>ident and c.get("mcu_us",0)>accepted]
        end=min(started+1500000,completed if completed else started+1500000,
                min(next_times)-10000 if next_times else started+1500000)
        after=samples[bisect.bisect_left(times,started):bisect.bisect_right(times,end)]
        row["status"]="no_paired_drop"
        left=[];right=[]
        # Two consecutive samples (5–15 ms apart) suppress isolated noise spikes.
        for a,b in zip(after,after[1:]):
            gap=b["mcu_us"]-a["mcu_us"]
            if not 0<gap<=15000:continue
            for k,candidates in ((0,left),(1,right)):
                if baseline[k]-abs(a["rpm"][k])>=thresholds[k] and baseline[k]-abs(b["rpm"][k])>=thresholds[k]:
                    index=bisect.bisect_left(times,a["mcu_us"])
                    previous=samples[index-1] if index else None
                    # Require a new crossing, not a previous shot's recovery tail.
                    if previous is None or a["mcu_us"]-previous["mcu_us"]>20000 or baseline[k]-abs(previous["rpm"][k])<thresholds[k]:
                        candidates.append(a)
        pairs=[(l,r) for l in left for r in right if abs(l["mcu_us"]-r["mcu_us"])<=pair_gap_ms*1000]
        if not pairs:
            continue
        onsets=sorted(set(max(l["mcu_us"],r["mcu_us"]) for l,r in pairs))
        row["paired_drop_onsets_mcu_us"]=onsets
        if len(onsets)>1:
            row["status"]="multiple_paired_drops";continue
        l,r=min(pairs,key=lambda pair:max(x["mcu_us"] for x in pair))
        onset=max(l["mcu_us"],r["mcu_us"])
        prior_index=bisect.bisect_left(times,min(l["mcu_us"],r["mcu_us"]))-1
        prior=times[prior_index] if prior_index>=0 else None
        if prior is None or onset-prior>20000:
            row["status"]="sample_gap_at_drop";continue
        issued=command.get("mcu_us",0)
        row.update(status="paired_drop", drop_mcu_us=onset,
                   left_drop_mcu_us=l["mcu_us"], right_drop_mcu_us=r["mcu_us"],
                   start_to_drop_ms=(onset-started)/1000,
                   accepted_to_drop_ms=(onset-accepted)/1000,
                   command_to_drop_ms=(onset-issued)/1000 if issued else None,
                   sampling_onset_bracket_mcu_us=[prior,onset],
                   sample_bracket_width_ms=(onset-prior)/1000,
                   wheel_rpm_at_onset=[l["rpm"][0],r["rpm"][1]],
                   peak_drop_rpm=[max((baseline[k]-abs(x["rpm"][k]) for x in after),default=0) for k in (0,1)],
                   feeder_completed=2 in shot)
        row["early_drop_possible_power_coupling"]=row["start_to_drop_ms"]<15
        if not row.get("feeder_completed") or any(e in shot for e in (4,5,6)):
            row["status"]="shot_not_completed"
    detected=[r for r in result if r["status"]=="paired_drop"]
    usable=[r for r in detected if not r["early_drop_possible_power_coupling"]]
    return dict(measurement="wheel-contact latency proxy; not muzzle-exit detection",
                expected_rate_hz=200, actual_sample_period_ms=distribution(deltas),
                sample_count=len(samples), command_count=len(commands), detected_count=len(detected),
                usable_timing_count=len(usable),
                missing_or_uncertain_count=len(result)-len(detected), shots=result,
                command_to_drop=distribution([r["command_to_drop_ms"] for r in usable if r["command_to_drop_ms"] is not None]),
                accepted_to_drop=distribution([r["accepted_to_drop_ms"] for r in usable]),
                start_to_drop=distribution([r["start_to_drop_ms"] for r in usable]),
                possible_power_coupling_count=sum(r["early_drop_possible_power_coupling"] for r in detected),
                command_to_drop_excluding_early_drops=distribution([r["command_to_drop_ms"] for r in detected
                    if not r["early_drop_possible_power_coupling"] and r["command_to_drop_ms"] is not None]),
                start_to_drop_followup_shots=distribution([r["start_to_drop_ms"] for r in usable if r["request_id"]!=1]),
                actual_projectile_exit_delay_measured=False, automatic_parameter_update=False,
                limitations=["A common power dip or motor-controller filtering can shift wheel feedback.",
                             "5 ms sample spacing and clock uncertainty limit temporal precision.",
                             "The projectile still travels from wheel contact to muzzle exit."])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log",type=Path)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--min-drop-rpm",type=float,default=50.)
    args=parser.parse_args()
    rows=[json.loads(line) for line in args.log.read_text(encoding="utf-8").splitlines() if line.strip()]
    report=analyze(rows,args.min_drop_rpm)
    output=args.output or args.log.parent/"rpm-delay-report.json"
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print("\n轮速掉速测量（毫秒；表示接触代理时间，不是已测出膛）")
    print("发次  指令→掉速  接受→掉速  拨弹启动→掉速  结果")
    for r in report["shots"]:
        def number(key):
            v=r.get(key);return f"{v:.1f}" if v is not None else "—"
        flag="（过早掉速，可能供电影响）" if r.get("early_drop_possible_power_coupling") else ""
        print(f'{r["request_id"]:>3}  {number("command_to_drop_ms"):>10}  {number("accepted_to_drop_ms"):>10}  {number("start_to_drop_ms"):>14}  {r["status"]}{flag}')
    print(f'检测到 {report["detected_count"]}/{report["command_count"]} 发；缺失/不确定 {report["missing_or_uncertain_count"]} 发')
    print(f'可纳入时间统计 {report["usable_timing_count"]} 发；多次掉速、跨周期残留及过早掉速不纳入均值')
    for label,key in (("指令→掉速","command_to_drop"),("拨弹启动→掉速","start_to_drop")):
        s=report[key]
        if s:print(f'{label}：平均 {s["mean_ms"]:.1f} ms，中位 {s["median_ms"]:.1f} ms，范围 {s["min_ms"]:.1f}–{s["max_ms"]:.1f} ms')
    print("报告:",output)


if __name__=="__main__":
    main()
