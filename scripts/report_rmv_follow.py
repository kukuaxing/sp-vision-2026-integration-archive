#!/usr/bin/env python3
"""Summarize a bounded snapshot of a field log; never opens robot devices.

Command-minus-joint errors describe the software servo loop, not measured shot
accuracy. Old logs may lack ready reasons; report that limitation explicitly.
"""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
import statistics


def distribution(values):
    xs = sorted(x for x in values if math.isfinite(x))
    if not xs:
        return None
    return dict(min=xs[0], median=statistics.median(xs),
                p90=xs[int((len(xs)-1)*.9)], max=xs[-1])


def report(path):
    # Freeze the byte boundary so a running logger cannot extend this read.
    with path.open('rb') as stream:
        data = stream.read(path.stat().st_size)
    rows = []
    malformed = 0
    for line in data.splitlines():
        try:
            row = json.loads(line)
        except (ValueError, UnicodeError):
            malformed += 1
            continue
        if isinstance(row, dict) and 'host_us' in row:
            rows.append(row)
    if not rows:
        raise ValueError('No frame rows in the selected log')
    active = [r for r in rows if r.get('mode') in (2,3) and r.get('owner') == 2]
    valid = [r for r in active if r.get('sent_goal_valid')]
    intervals = [(b['host_us']-a['host_us'])/1000 for a,b in zip(valid,valid[1:])]
    tail = [r for r in active if r['host_us'] >= active[-1]['host_us']-5000000] if active else []
    err = [((r['sent_goal_joint_deg'][0]-r['actual_joint_deg'][0]+180)%360-180,
            r['sent_goal_joint_deg'][1]-r['actual_joint_deg'][1]) for r in valid]
    fields = ['ready', 'image_pose_valid', 'current_pose_valid', 'target_valid',
              'goal_valid', 'aim_submitted', 'sent_goal_valid']
    counts = {k: sum(bool(r.get(k)) for r in active) for k in fields}
    clipped = [r for r in valid if abs(r['unclipped_goal_joint_deg'][1]-
                                     r['sent_goal_joint_deg'][1]) > .001]
    grouped = collections.defaultdict(list)
    for r in valid:
        grouped[int((r['host_us']-rows[0]['host_us'])/1e6)].append(r)
    windows = []
    for second, rs in sorted(grouped.items()):
        errors = [((r['sent_goal_joint_deg'][0]-r['actual_joint_deg'][0]+180)%360-180,
                   r['sent_goal_joint_deg'][1]-r['actual_joint_deg'][1]) for r in rs]
        windows.append({'second_from_log_start': second, 'rows': len(rs),
                        'median_command_error_deg': [statistics.median(e[i] for e in errors)
                                                     for i in range(2)]})
    rotating=[r for r in active if isinstance(r.get('rotation'),dict)]
    rotation_report={
        'rows':len(rotating),
        'modes':dict(collections.Counter(r['rotation'].get('mode') for r in rotating)),
        'reasons':dict(collections.Counter(r['rotation'].get('reason') for r in rotating)),
        'phase_locked_rows':sum(bool(r['rotation'].get('phase_locked')) for r in rotating),
        'qualified_rows':sum(bool(r['rotation'].get('qualified')) for r in rotating),
        'fire_gate_reasons':dict(collections.Counter(r.get('fire_gate_reason','not_recorded') for r in rotating)),
        'physical_fire_permission_rows':sum(bool(r.get('fire_permission')) for r in rotating),
        'fire_gate_reasons_with_permission':dict(collections.Counter(r.get('fire_gate_reason','not_recorded')
            for r in rotating if r.get('fire_permission'))),
        'fire_requests':len({r['shot_request_id'] for r in rotating if r.get('shot_request_id')}),
        'software_feeder_end_ids':sorted({r['shot_event']['request_id'] for r in rotating if
            r.get('shot_event',{}).get('event')==2}),
        'hit_rate_verified':False,
        'event_evidence_note':'Feeder start/end are motor/software events, not measured projectile exits or target hits.'}
    return {'input_log': str(path), 'snapshot_bytes': len(data),'rotation':rotation_report,
            'snapshot_sha256': hashlib.sha256(data).hexdigest(),
            'frame_rows': len(rows), 'malformed_or_partial_lines': malformed,
            'duration_seconds': (rows[-1]['host_us']-rows[0]['host_us'])/1e6,
            'remote_mode_rows': len(active), 'active_counts': counts,
            'active_fractions': {k: v/len(active) if active else None for k, v in counts.items()},
            'control_not_ready_reasons': dict(collections.Counter(
                r.get('control_ready_reason', 'not_recorded') for r in active if not r.get('ready'))),
            'image_age_ms': distribution([r['image_age_us']/1000 for r in active]),
            'status_age_ms': distribution([r['status_age_us']/1000 for r in active if r.get('status_age_us') is not None]),
            'joint_age_ms': distribution([r['joint_age_us']/1000 for r in active if r.get('joint_age_us') is not None]),
            'valid_command_interval_ms': distribution(intervals),
            'valid_command_intervals_over_ttl': sum(x>60 for x in intervals),
            'status_target_expired_rows': sum(bool(r.get('constraints',0)&8) for r in active),
            'mcu_fault_rows': sum(bool(r.get('mcu_fault_mask',0)) for r in active),
            'actual_joint_deg': [distribution([r['actual_joint_deg'][i] for r in active]) for i in range(2)],
            'last_five_seconds': {'rows':len(tail),
                'actual_joint_deg':[distribution([r['actual_joint_deg'][i] for r in tail]) for i in range(2)],
                'requested_joint_deg':[distribution([r['unclipped_goal_joint_deg'][i] for r in tail if r.get('goal_valid')]) for i in range(2)],
                'abs_requested_minus_joint_deg':[distribution([abs(r['command_error_deg'][i]) for r in tail if 'command_error_deg' in r]) for i in range(2)]},
            'abs_command_minus_joint_deg': [distribution([abs(e[i]) for e in err]) for i in range(2)],
            'pitch_requested_beyond_limit_rows': len(clipped),
            'pitch_clip_fraction_of_valid_commands': len(clipped)/len(valid) if valid else None,
            'small_yaw_actual_deg_during_valid_commands': distribution([r['small_yaw_joint_deg'] for r in valid]),
            'status_constraint_counts': dict(collections.Counter(r['constraints'] for r in valid)),
            'valid_command_seconds': windows,
            'limits': ['Mixed fixed and moving target run; static accuracy not isolated.',
                       'Software command/joint differences are not measured bore or shot accuracy.',
                       'STATUS constraints describe the last reported adoption; they can lag this image.',
                       'Expired STATUS rows are repeated observations, not distinct expiry events.',
                       'P command clipping does not mean the physical joint reached the boundary.']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = report(args.log)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k: result[k] for k in ('frame_rows', 'remote_mode_rows', 'active_counts',
                                           'control_not_ready_reasons', 'pitch_requested_beyond_limit_rows')},
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
