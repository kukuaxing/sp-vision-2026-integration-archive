#!/usr/bin/env python3
"""Bounded USB camera capture using installed MVS SDK; no serial or control IO."""
from pathlib import Path
import argparse
import ctypes as ct
import json
import os
import sys
import time
import yaml

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault('MVCAM_COMMON_RUNENV', '/opt/MVS/lib')
sys.path.insert(0, '/opt/MVS/Samples/64/Python/MvImport')
from MvCameraControl_class import MvCamera
from CameraParams_header import MV_CC_DEVICE_INFO_LIST, MV_CC_DEVICE_INFO, MV_FRAME_OUT, MVCC_ENUMVALUE, MVCC_INTVALUE_EX, MVCC_FLOATVALUE, MV_SAVE_IMAGE_TO_FILE_PARAM_EX, MV_Image_Png
from CameraParams_const import MV_USB_DEVICE, MV_ACCESS_Exclusive

config = yaml.safe_load((ROOT / 'configs/standard.yaml').read_text())
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--fps', type=float, help='Temporary frame-rate override; restored after capture')
parser.add_argument('--seconds', type=float, default=6)
parser.add_argument('--max-frames', type=int, default=300)
args = parser.parse_args()
if not 0 < args.seconds <= 120 or not 1 <= args.max_frames <= 20000:
    parser.error('seconds must be in (0,120], max-frames in [1,20000]')
if args.fps is not None:
    if not 0 < args.fps <= 500:
        parser.error('fps must be in (0,500]')
    config['camera_fps'] = args.fps
stamp = time.strftime('%Y%m%d_%H%M%S')
out = ROOT / 'data/camera-check' / stamp
out.mkdir(parents=True, exist_ok=False)
log = ROOT / 'logs/camera-check' / (stamp + '.json')
log.parent.mkdir(parents=True, exist_ok=True)
report = {'devices': [], 'frames': 0, 'frame_gaps': 0, 'errors': [], 'restore_errors': [], 'sample': None}
cam = None
created = opened = grabbing = False
saved = []
sample_buffer = sample_params = None


def check(code, operation):
    if code != 0:
        raise RuntimeError(f'{operation}: SDK error 0x{code & 0xffffffff:08x}')


def text(chars):
    return bytes(chars).split(b'\0', 1)[0].decode('utf-8', errors='replace')


def set_node(name, kind, value):
    types = {'enum': (MVCC_ENUMVALUE, 'MV_CC_GetEnumValue', 'MV_CC_SetEnumValue', 'nCurValue'),
             'int': (MVCC_INTVALUE_EX, 'MV_CC_GetIntValueEx', 'MV_CC_SetIntValueEx', 'nCurValue'),
             'float': (MVCC_FLOATVALUE, 'MV_CC_GetFloatValue', 'MV_CC_SetFloatValue', 'fCurValue'),
             'bool': (ct.c_bool, 'MV_CC_GetBoolValue', 'MV_CC_SetBoolValue', 'value')}
    typ, getter, setter, field = types[kind]
    old = typ()
    check(getattr(cam, getter)(name, old), 'Read '+name)
    previous = getattr(old, field)
    if isinstance(value, str):
        check(cam.MV_CC_SetEnumValueByString(name, value), 'Set '+name)
    else:
        check(getattr(cam, setter)(name, value), 'Set '+name)
    saved.append((setter, name, previous))


try:
    check(MvCamera.MV_CC_Initialize(), 'SDK initialization')
    report['sdk_version'] = hex(MvCamera.MV_CC_GetSDKVersion())
    devices = MV_CC_DEVICE_INFO_LIST()
    check(MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE, devices), 'USB enumeration')
    chosen = None
    for i in range(devices.nDeviceNum):
        info = ct.cast(devices.pDeviceInfo[i], ct.POINTER(MV_CC_DEVICE_INFO)).contents
        details = {'model': text(info.SpecialInfo.stUsb3VInfo.chModelName), 'serial': text(info.SpecialInfo.stUsb3VInfo.chSerialNumber)}
        report['devices'].append(details)
        if details['serial'] == 'DA4302248':
            chosen = info
    if chosen is None:
        raise RuntimeError('Expected Hikrobot DA4302248 not enumerated; no automatic reconnect attempted')
    cam = MvCamera()
    check(cam.MV_CC_CreateHandle(chosen), 'Create handle'); created = True
    check(cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0), 'Open camera'); opened = True
    for name, kind, value in [
        ('TriggerMode', 'enum', 0), ('ExposureAuto', 'enum', 0), ('GainAuto', 'enum', 0),
        ('Width', 'int', config['camera_width']), ('Height', 'int', config['camera_height']),
        ('PixelFormat', 'enum', config['hikrobot_pixel_format']),
        ('ExposureTime', 'float', config['exposure_ms']*1000), ('Gain', 'float', config['gain']),
        ('BalanceWhiteAuto', 'enum', 0), ('AcquisitionFrameRateEnable', 'bool', True),
        ('AcquisitionFrameRate', 'float', config['camera_fps']),
    ]:
        set_node(name, kind, value)
    report['requested'] = {k: config[k] for k in ['camera_width', 'camera_height', 'hikrobot_pixel_format', 'exposure_ms', 'gain', 'camera_fps']}
    check(cam.MV_CC_StartGrabbing(), 'Start acquisition'); grabbing = True
    started = time.monotonic()
    first = last = previous = None
    while report['frames'] < args.max_frames and time.monotonic()-started < args.seconds:
        frame = MV_FRAME_OUT()
        check(cam.MV_CC_GetImageBuffer(frame, 1000), 'Get frame')
        try:
            now = time.monotonic()
            number = frame.stFrameInfo.nFrameNum
            if previous is not None:
                report['frame_gaps'] += max(0, ((number-previous) & 0xffffffff)-1)
            previous = number
            if first is None: first = now
            last = now
            report['frames'] += 1
            report['actual_dimensions'] = [frame.stFrameInfo.nWidth, frame.stFrameInfo.nHeight]
            if report['frames'] == 1:
                # Own a raw copy; PNG encoding and disk IO must not stall capture.
                sample_buffer = (ct.c_ubyte * frame.stFrameInfo.nFrameLen)()
                ct.memmove(sample_buffer, frame.pBufAddr, frame.stFrameInfo.nFrameLen)
                sample_params = MV_SAVE_IMAGE_TO_FILE_PARAM_EX()
                sample_params.enPixelType = frame.stFrameInfo.enPixelType
                sample_params.nWidth, sample_params.nHeight = report['actual_dimensions']
                sample_params.nDataLen = frame.stFrameInfo.nFrameLen
                sample_params.pData = ct.cast(sample_buffer, ct.POINTER(ct.c_ubyte))
        finally:
            check(cam.MV_CC_FreeImageBuffer(frame), 'Release frame')
    report['host_receive_fps'] = (report['frames']-1)/(last-first) if first is not None and last > first else None
    report['capture_seconds'] = time.monotonic()-started
    check(cam.MV_CC_StopGrabbing(), 'Stop acquisition')
    grabbing = False
    if sample_params is not None:
        sample = out / 'sample.png'
        sample_params.enImageType = MV_Image_Png
        sample_params.pcImagePath = ct.create_string_buffer(str(sample).encode())
        sample_params.iMethodValue = 1
        check(cam.MV_CC_SaveImageToFileEx(sample_params), 'Save sample')
        report['sample'] = str(sample)
except Exception as exc:
    report['errors'].append(str(exc))
finally:
    if grabbing:
        code = cam.MV_CC_StopGrabbing()
        if code: report['restore_errors'].append(f'StopGrabbing: {code:#x}')
    if opened:
        for setter, name, value in reversed(saved):
            code = getattr(cam, setter)(name, value)
            if code: report['restore_errors'].append(f'{name}: {code:#x}')
        cam.MV_CC_CloseDevice()
    if created: cam.MV_CC_DestroyHandle()
    MvCamera.MV_CC_Finalize()
    log.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print('Report:', log)
    print(json.dumps(report, ensure_ascii=False, indent=2))
raise SystemExit(1 if report['errors'] or report['restore_errors'] or not report['sample'] else 0)
