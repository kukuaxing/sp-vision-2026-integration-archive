#!/usr/bin/env python3
"""Inspect deployment without opening a camera/serial device or starting control."""
from pathlib import Path
import argparse, hashlib, json, os, subprocess
import yaml


def protocol_report(config, calibration):
    board = str(config.get('board_protocol', '')).strip().upper() or 'UNKNOWN'
    runtime = 'RMV' if board == 'RMV' and config.get('rmv_debug_enabled', False) else (('SENTINEL' if board == 'SENTINEL' else 'SP') if config.get('xuc_enable', False) else 'CBOARD_CAN')
    if calibration.get('calibration_pose_source') == 'rmv':
        pose_reader = 'RMV'
    elif calibration.get('calibration_pose_source') == 'hipnuc':
        pose_reader = 'HI91'
    elif calibration.get('xuc_enable', False):
        pose_reader = 'SP'
    elif calibration.get('require_cboard', True):
        pose_reader = 'CBOARD_CAN'
    else:
        pose_reader = 'NONE'
    blockers = []
    warnings = []
    if board == 'UNKNOWN':
        blockers.append('尚未声明下位机协议；请核对当前固件后设置 standard.yaml 的 board_protocol。')
    elif board != runtime:
        blockers.append(f'下位机配置协议为 {board}，正式 standard 使用 {runtime}；需完成接口适配后才能启动自瞄。')
    if board == 'SENTINEL' and config.get('xuc_serial_baud') != 921600:
        blockers.append('SENTINEL 固定使用 921600 波特率。')
    pose_matches = board != 'UNKNOWN' and board == pose_reader
    if not pose_matches:
        warnings.append(f'标定姿态接收器使用 {pose_reader}，与声明的下位机协议 {board} 不匹配；不能按旧流程采集外参姿态。')
    return {
        'runtime_protocol': runtime,
        'board_protocol': board,
        'board_protocol_source': 'configs/standard.yaml declaration; no live handshake',
        'runtime_protocol_compatible': not blockers,
        'calibration_pose_protocol': pose_reader,
        'calibration_pose_protocol_compatible': pose_matches,
        'calibration_pose_verified': bool(calibration.get('calibration_pose_verified', False)),
        'calibration_raw_pose_verified': bool(calibration.get('calibration_raw_pose_verified', False)),
        'calibration_raw_capture_permitted': pose_matches and pose_reader == 'RMV' and bool(calibration.get('calibration_raw_pose_verified', False)),
        'runtime_blockers': blockers,
        'warnings': warnings,
    }


def main():
    import grp

    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--require-hardware', action='store_true')
    parser.add_argument('--require-runtime', action='store_true',
                        help='Require hardware and a declared firmware protocol compatible with standard')
    args = parser.parse_args()
    errors = []
    manifest = json.loads((root/'docs/artifacts.json').read_text())
    for item in manifest['files']:
        p = root/item['path']
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest() != item['sha256']:
            errors.append('Missing or changed resource: '+item['path'])
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = ':'.join(['/opt/MVS/lib/64',str(root/'io/hikrobot/lib/amd64'),str(root/'io/mindvision/lib/amd64'),'/opt/ros/humble/lib',env.get('LD_LIBRARY_PATH','')])
    for name in ['standard','capture','calibrate_camera','calibrate_handeye','calibrate_robotworld_handeye','verify_handeye','xuc_board_probe']:
        binary = root/'build'/name
        if not binary.is_file() or not os.access(binary,os.X_OK):
            errors.append('Missing executable: '+name)
            continue
        result = subprocess.run(['ldd',str(binary)],env=env,capture_output=True,text=True,timeout=10)
        if result.returncode or 'not found' in result.stdout:
            errors.append('Unresolved runtime libraries: '+name+'\n'+result.stdout+result.stderr)
    gui = {'ready': False}
    capture_binary = root/'build/capture'
    if capture_binary.is_file():
        # Discover support using help first: an older capture must never treat
        # an unknown diagnostic flag as a request to open physical devices.
        help_result = subprocess.run([str(capture_binary), '--help'], env=env, capture_output=True, text=True, timeout=5)
        if help_result.returncode == 0 and 'gui-info' in help_result.stdout:
            gui_result = subprocess.run([str(capture_binary), '--gui-info'], env=env, capture_output=True, text=True, timeout=5)
            if gui_result.returncode == 0 and gui_result.stdout.startswith('GUI '):
                gui.update(json.loads(gui_result.stdout[4:]), ready=True)
            else:
                errors.append('Capture GUI backend unavailable: '+gui_result.stdout+gui_result.stderr)
        else:
            errors.append('Capture lacks the device-free GUI check; rebuild capture.')
    software_ready = not errors
    config = yaml.safe_load((root/'configs/standard.yaml').read_text())
    calibration = yaml.safe_load((root/'configs/calibration.yaml').read_text())
    protocols = protocol_report(config, calibration)
    if protocols['runtime_protocol'] == 'SENTINEL':
        binary = root/'build/standard'
        help_result = subprocess.run([str(binary),'--help'],env=env,capture_output=True,text=True,timeout=5)
        if help_result.returncode or 'sentinel-check' not in help_result.stdout:
            protocols['runtime_blockers'].append('standard 缺少 Sentinel 固定帧入口；请重新编译。')
        else:
            check = subprocess.run([str(binary),str(root/'configs/standard.yaml'),'--sentinel-check'],env=env,capture_output=True,text=True,timeout=5)
            if check.returncode:
                protocols['runtime_blockers'].append('Sentinel 静态配置检查失败：'+check.stdout+check.stderr)
        protocols['runtime_protocol_compatible'] = not protocols['runtime_blockers']
    if protocols['runtime_protocol'] == 'RMV':
        # Check help before the static flag so an old binary cannot open devices.
        binary = root/'build/standard'
        help_result = subprocess.run([str(binary),'--help'],env=env,capture_output=True,text=True,timeout=5)
        if help_result.returncode or 'rmv-check' not in help_result.stdout:
            protocols['runtime_blockers'].append('standard 缺少 RMV 调试入口；请重新编译。')
        else:
            check = subprocess.run([str(binary),str(root/'configs/standard.yaml'),'--rmv-check'],env=env,capture_output=True,text=True,timeout=5)
            if check.returncode:
                protocols['runtime_blockers'].append('RMV 静态配置检查失败：'+check.stdout+check.stderr)
        protocols['runtime_protocol_compatible'] = not protocols['runtime_blockers']
    groups = [grp.getgrgid(gid).gr_name for gid in os.getgroups()]
    serial = Path(config['rmv_port'] if protocols['runtime_protocol']=='RMV' else config['xuc_serial_port'])
    usb = subprocess.run(['lsusb'],capture_output=True,text=True,timeout=10)
    hardware = {'serial_path':str(serial),'serial_present':serial.exists(),
                'serial_accessible':serial.exists() and os.access(serial,os.R_OK|os.W_OK),
                'hikrobot_usb_present':config['vid_pid'].lower() in usb.stdout.lower()}
    hardware_ready = all(hardware[k] for k in ['serial_present','serial_accessible','hikrobot_usb_present'])
    if (args.require_hardware or args.require_runtime) and not hardware_ready:
        if not hardware['serial_present']:
            errors.append('未连接下位机 USB 串口：'+str(serial))
        elif not hardware['serial_accessible']:
            errors.append('没有串口读写权限；请注销桌面后重新登录以刷新 dialout 组。')
        if not hardware['hikrobot_usb_present']:
            errors.append('未检测到 HikRobot 相机（USB '+str(config['vid_pid'])+'）。')
    if args.require_runtime:
        errors.extend(protocols['runtime_blockers'])
    result = {'project':str(root),'software_ready':software_ready,'capture_gui':gui,'groups':groups,'hardware':hardware,
              'hardware_ready':hardware_ready, **protocols,
              'runtime_ready':software_ready and hardware_ready and protocols['runtime_protocol_compatible'],
              'control_enabled':False if protocols['runtime_protocol']=='RMV' else bool(config['xuc_allow_control']),
              'control_requires_explicit_rmv_follow': protocols['runtime_protocol']=='RMV',
              'shoot_enabled':False if protocols['runtime_protocol']=='RMV' else bool(config['xuc_allow_shoot']),
              'auto_fire_enabled':bool(config.get('auto_fire',False)),
              'errors':errors,'note':'Inspection only; protocol is declared, not live-verified. No device was opened. Readiness does not certify mounting, calibration or robot commissioning.'}
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
