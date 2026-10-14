#!/usr/bin/env python3
"""Describe saved RMV telemetry validity. Offline only; never opens hardware."""
import argparse
import json
import math
from pathlib import Path

SENSORS = {0: '相机支架', 1: '小 Y IMU', 2: '大 Y IMU'}
AXES = {0: '小 Y', 1: 'P', 2: '大 Y'}
FAULTS = ['IMU 失效', '小 Y 故障', 'P 故障', '大 Y 故障', '遥控失联',
          '远程失联', '回零失败', '硬限位触发', '控制超时', '发射机构故障', '底盘故障']


def finite_vector(values, length):
    return (isinstance(values, (list, tuple)) and len(values) == length and
            all(isinstance(x, (int, float)) and math.isfinite(x) for x in values))


def build_report(summary):
    status = summary.get('last_status') or {}
    poses = summary.get('last_pose') or {}
    joints = summary.get('last_joints') or []
    warnings = []
    sensors = []
    for source, name in SENSORS.items():
        pose = poses.get(str(source), poses.get(source))
        if pose is None:
            sensors.append({'sensor_id': source, 'name': name, 'received': False})
            warnings.append(name + '：未收到该来源的数据')
            continue
        flags = pose.get('flags', 0)
        q = pose.get('quaternion_wxyz', [])
        norm = math.hypot(*q) if finite_vector(q, 4) else None
        if norm is not None and not math.isfinite(norm):
            norm = None
        valid = bool(flags & 1) and norm is not None and abs(norm-1) <= .01
        sensors.append({'sensor_id': source, 'name': name, 'received': True,
                        'quaternion_valid': valid, 'quaternion_norm': norm,
                        'gyro_valid': bool(flags & 2) and finite_vector(pose.get('gyro_rad_s'), 3),
                        'acceleration_valid': bool(flags & 4) and finite_vector(pose.get('acceleration_m_s2'), 3),
                        'time_basis': 'MCU 接收时间' if flags & 32 else
                                      '映射到 MCU 的采样时间' if flags & 8 else '无有效采样时间',
                        'time_flags_conflict': bool(flags & 8 and flags & 32)})
        if not valid:
            warnings.append(name + '：四元数无效，不应按完整姿态使用')
        if flags & 8 and flags & 32:
            warnings.append(name + '：时间有效标志互相冲突')
    axes = []
    for source, name in AXES.items():
        joint = next((entry for entry in joints if entry.get('axis_id') == source), None)
        if joint is None:
            axes.append({'axis_id': source, 'name': name, 'received': False})
            warnings.append(name + '：未收到关节数据')
            continue
        bits = joint.get('state_bits', 0)
        online = bool(bits & 1)
        fault = joint.get('fault_code')
        axes.append({'axis_id': source, 'name': name, 'received': True, 'online': online,
                     'angle_valid': bool(bits & 8), 'velocity_valid': bool(bits & 16),
                     'feedback_age_us': joint.get('feedback_age_us'), 'fault_code': fault})
        if not online or fault != 0:
            warnings.append(name + '：反馈离线或有故障；需结合电源和接线状态解释')
    mask = status.get('fault_mask', 0)
    capabilities = (summary.get('hello') or {}).get('capabilities')
    first, last = summary.get('diagnostics_first'), summary.get('diagnostics')
    drops = ((last[5]-first[5]) & 0xffffffff if first and last and
             len(first) == len(last) == 10 else None)
    return {'communication_passed': bool(summary.get('hello') and status and
                not any(summary.get(k) for k in ('crc_errors', 'format_errors',
                    'parser_timeouts', 'capture_error', 'restore_error'))),
            'sensors': sensors, 'axes': axes,
            'camera_pose_advertised': bool(capabilities & 1) if capabilities is not None else None,
            'reported_faults': [name for bit, name in enumerate(FAULTS) if mask & (1 << bit)],
            'mcu_tx_dropped_frames_delta': drops, 'warnings': warnings,
            'note': '通信完整性与数据有效性分开判断；故障可能锁存，反馈离线也可能是未上电。'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path)
    args = parser.parse_args()
    print(json.dumps(build_report(json.loads(args.summary.read_text(encoding='utf-8'))),
                     ensure_ascii=False, indent=2, allow_nan=False))
