#!/usr/bin/env python3
"""Supervised Sentinel V2 trial and bounded direction probe; receive-only by default."""
import argparse
from collections import deque
import hashlib
import json
import math
import os
import signal
import statistics
import struct
import sys
import time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.dont_write_bytecode = True
sys.path.append(str(ROOT))
from field_support import ReceiveFanout, persist_sequence, atomic_json, verify_bundle
WIRE_LIMIT = .40
PROBE_VY = .40
PROBE_SECONDS = 1.0
NEGATIVE_PROBE_VY = -.38
NEGATIVE_PROBE_SECONDS = .60
PROBE_POSITION_RADIUS = .04
FORWARD_SPEED_VY = -.60
FORWARD_SPEED_SECONDS = 2.0
FORWARD_SPEED_POSITION_RADIUS = .15
FORWARD_SPEED_WIRE_LIMIT = .60
FORWARD_RAMP_VY = -2.0
FORWARD_RAMP_SECONDS = 1.0
FORWARD_RAMP_PULSE_SECONDS = 2.5
FORWARD_RAMP_POSITION_RADIUS = .60
FORWARD_RAMP_WIRE_LIMIT = 2.0
FORWARD_RAMP_MAX_LINEAR_SPEED = .30
POSITIVE_X_RAMP_VX = 2.0
POSITIVE_X_RAMP_SECONDS = 1.0
POSITIVE_X_RAMP_PULSE_SECONDS = 2.0
POSITIVE_X_RAMP_POSITION_RADIUS = .35
POSITIVE_X_RAMP_MAX_LINEAR_SPEED = .30
POSITIVE_X_RAMP_MAX_BODY_FORWARD = .025
BACKWARD_RAMP_VY = 2.0
BACKWARD_RAMP_SECONDS = 1.0
BACKWARD_RAMP_PULSE_SECONDS = 2.0
BACKWARD_RAMP_POSITION_RADIUS = .40
BACKWARD_RAMP_MAX_LINEAR_SPEED = .30
BACKWARD_RAMP_MAX_BODY_LEFT = .025
BACKWARD_RAMP_MAX_POSITIVE_FORWARD = .015
RAMP_PROFILES = ('forward_speed_ramp', 'positive_x_ramp', 'negative_x_ramp', 'backward_speed_ramp')
TRIAL_DISTANCE = .20
TRIAL_AMPLITUDE = .38
TRIAL_SLOW_AMPLITUDE = .20
TRIAL_SLOW_AT = .16
TRIAL_STOP_AT = .195
TRIAL_TIMEOUT_S = 20.
TRIAL_CROSS_LIMIT = .025
TRIAL_YAW_LIMIT = .10
TRIAL_HEIGHT_LIMIT = .03
DIRECTIONS = {'forward':(1.,0.), 'backward':(-1.,0.),
              'left':(0.,1.), 'right':(0.,-1.)}
sys.path.insert(0, str(ROOT / 'sentinel_translation'))
from serial_transport import SerialTransport
from localization.frame_math import quat_matrix


def check_odometry_source_age(ros_now_ns, stamp_ns):
    """Check source age at use time, including after a message was received."""
    age_ns = ros_now_ns - stamp_ns
    if not -50_000_000 <= age_ns <= 180_000_000:
        raise RuntimeError('odometry source timestamp is stale or too far in the future')
    return age_ns * 1e-9


def crc16(data):
    crc = 0xffff
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


def nav_frame(boot, seq, vx=0., enabled=True, vy=0.,wire_limit=WIRE_LIMIT):
    if wire_limit not in (WIRE_LIMIT,FORWARD_SPEED_WIRE_LIMIT,FORWARD_RAMP_WIRE_LIMIT):
        raise ValueError('only the fixed 0.40, 0.60 or 2.0 wire limit is supported')
    if (not 0 < boot <= 0xffffffff or not all(math.isfinite(v) for v in (vx,vy))
            or math.hypot(vx,vy) > wire_limit):
        raise ValueError('invalid boot or trial speed')
    if not enabled and (vx != 0 or vy != 0):
        raise ValueError('revoke must have zero velocity')
    payload = struct.pack('<IIHBBfff', 0, 0, 100, int(enabled), 0, vx, vy, 0.)
    frame = struct.pack('<2sBBHHI', b'\xa6\x6a', 2, 0x11, 24, seq & 0xffff, boot) + payload
    return frame + struct.pack('<H', crc16(frame))


class Stream:
    def __init__(self):
        self.buf = bytearray()
        self.count = 0
        self.bad_crc = 0
        self.boot = None
        self.state = None
        self.status_at = 0.
        self.status_seq = None
        self.reject = None

    def feed(self, data, now):
        self.buf.extend(data)
        while len(self.buf) >= 14:
            pos = self.buf.find(b'\xa6\x6a')
            if pos < 0:
                self.buf[:] = self.buf[-1:]
                return
            if pos:
                del self.buf[:pos]
            if len(self.buf) < 14:
                return
            _, version, kind, size, seq, boot = struct.unpack_from('<2sBBHHI', self.buf)
            if version != 2 or size > 128:
                del self.buf[0]
                continue
            total = 14 + size
            if len(self.buf) < total:
                return
            frame = bytes(self.buf[:total])
            if crc16(frame[:-2]) != struct.unpack_from('<H', frame, total - 2)[0]:
                self.bad_crc += 1
                del self.buf[0]
                continue
            del self.buf[:total]
            if (kind, size) not in ((1, 32), (2, 20), (0x81, 12)):
                continue
            if not boot:
                raise RuntimeError('MCU boot_id is zero')
            if self.boot is not None and boot != self.boot:
                raise RuntimeError('MCU restarted; no automatic resume')
            self.boot = boot
            self.count += 1
            if kind == 0x81:
                if self.status_seq is not None:
                    diff = (seq - self.status_seq) & 0xffff
                    if diff == 0 or diff >= 32768:
                        raise RuntimeError('MCU status sequence did not advance')
                self.status_seq = seq
                self.state = frame[18]
                self.reject = frame[19]
                self.status_at = now


def pose_rotation(pose):
    if len(pose)>=10:
        return quat_matrix(pose[6:10])
    # Old receipts did not retain a quaternion. Preserve their explicit planar
    # interpretation; never invent their missing roll/pitch during a replay.
    c,s=math.cos(pose[3]),math.sin(pose[3])
    return np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])


def body_displacement(origin,pose):
    return pose_rotation(origin).T@(np.asarray(pose[:3])-np.asarray(origin[:3]))


def relative(origin, pose):
    p=body_displacement(origin,pose)
    rotation=pose_rotation(origin).T@pose_rotation(pose)
    return float(p[0]),float(p[1]),math.atan2(rotation[1,0],rotation[0,0])


def direction_components(forward, left, direction='forward'):
    """Project initial body axes onto the requested direction and its left."""
    ux,uy = DIRECTIONS[direction]
    return ux*forward+uy*left, -uy*forward+ux*left


def validate_forward_axis_alignment(alignment):
    """Only the current, independently checked forward wire axis is usable."""
    if not isinstance(alignment,dict) or alignment.get('command_strategy') != 'forward_axis_only':
        raise ValueError('an explicit forward_axis_only strategy is required')
    if alignment.get('forward_direction_confirmed') is not True:
        raise ValueError('physical forward direction not confirmed')
    if alignment.get('lateral_direction_confirmed') is not False:
        raise ValueError('unverified lateral direction must be explicitly disabled')
    if alignment.get('body_to_wire_xy') != [[0,0],[-1,0]]:
        raise ValueError('forward-only mapping must be [[0,0],[-1,0]]')


def trial_parameters(direction='forward',alignment=None):
    if direction not in DIRECTIONS:
        raise ValueError('unknown trial direction')
    parameters = {'direction':direction, 'target_distance_m':TRIAL_DISTANCE,
                  'amplitude':TRIAL_AMPLITUDE, 'slow_amplitude':TRIAL_SLOW_AMPLITUDE,
                  'slow_at_m':TRIAL_SLOW_AT, 'timeout_s':TRIAL_TIMEOUT_S,
                  'wire_limit':WIRE_LIMIT}
    if alignment is not None:
        validate_forward_axis_alignment(alignment)
        if direction != 'forward':
            raise ValueError('only a forward trial is currently verified')
        parameters.update(command_strategy='forward_axis_only',lateral_direction_confirmed=False)
    return parameters


def trial_plan_matches(plan, direction, source_sha256, alignment_sha256,alignment=None):
    """Every direction needs an exact, reviewed plan; legacy plans fail closed."""
    expected = dict(trial_parameters(direction,alignment), source_sha256=source_sha256,
                    alignment_sha256=alignment_sha256)
    return isinstance(plan,dict) and all((plan.get(k) is v if isinstance(v,bool) else plan.get(k)==v)
                                         for k,v in expected.items())


def probe_parameters(profile):
    """Fixed wire probes do not assume or confirm a physical body direction."""
    if profile == 'positive_y':
        vy,seconds = PROBE_VY,PROBE_SECONDS
    elif profile == 'negative_y':
        vy,seconds = NEGATIVE_PROBE_VY,NEGATIVE_PROBE_SECONDS
    elif profile == 'forward_speed':
        return {'profile':profile, 'wire_vx':0., 'wire_vy':FORWARD_SPEED_VY,
                'pulse_s':FORWARD_SPEED_SECONDS,
                'position_radius_guard_m':FORWARD_SPEED_POSITION_RADIUS,
                'wire_limit':FORWARD_SPEED_WIRE_LIMIT}
    elif profile == 'forward_speed_ramp':
        return {'profile':profile, 'wire_vx':0., 'target_wire_vy':FORWARD_RAMP_VY,
                'ramp_s':FORWARD_RAMP_SECONDS, 'pulse_s':FORWARD_RAMP_PULSE_SECONDS,
                'position_radius_guard_m':FORWARD_RAMP_POSITION_RADIUS,
                'wire_limit':FORWARD_RAMP_WIRE_LIMIT,
                'max_linear_speed_m_s':FORWARD_RAMP_MAX_LINEAR_SPEED}
    elif profile in ('positive_x_ramp', 'negative_x_ramp'):
        return {'profile':profile, 'target_wire_vx':POSITIVE_X_RAMP_VX * (-1 if profile == 'negative_x_ramp' else 1), 'wire_vy':0.,
                'ramp_s':POSITIVE_X_RAMP_SECONDS, 'pulse_s':POSITIVE_X_RAMP_PULSE_SECONDS,
                'position_radius_guard_m':POSITIVE_X_RAMP_POSITION_RADIUS,
                'wire_limit':FORWARD_RAMP_WIRE_LIMIT,
                'max_linear_speed_m_s':POSITIVE_X_RAMP_MAX_LINEAR_SPEED,
                'max_body_forward_m':POSITIVE_X_RAMP_MAX_BODY_FORWARD}
    elif profile == 'backward_speed_ramp':
        return {'profile':profile, 'wire_vx':0., 'target_wire_vy':BACKWARD_RAMP_VY,
                'ramp_s':BACKWARD_RAMP_SECONDS, 'pulse_s':BACKWARD_RAMP_PULSE_SECONDS,
                'position_radius_guard_m':BACKWARD_RAMP_POSITION_RADIUS,
                'wire_limit':FORWARD_RAMP_WIRE_LIMIT,
                'max_linear_speed_m_s':BACKWARD_RAMP_MAX_LINEAR_SPEED,
                'max_body_left_m':BACKWARD_RAMP_MAX_BODY_LEFT,
                'max_positive_body_forward_m':BACKWARD_RAMP_MAX_POSITIVE_FORWARD}
    else:
        raise ValueError('unknown direction probe profile')
    return {'profile':profile, 'wire_vx':0., 'wire_vy':vy, 'pulse_s':seconds,
            'position_radius_guard_m':PROBE_POSITION_RADIUS}


def probe_plan_matches(plan, profile, source_sha256,alignment_sha256=None):
    expected = dict(probe_parameters(profile), source_sha256=source_sha256)
    if profile in ('forward_speed',*RAMP_PROFILES):
        if (not isinstance(alignment_sha256,str) or len(alignment_sha256)!=64
                or any(c not in '0123456789abcdef' for c in alignment_sha256)):
            return False
        expected['alignment_sha256'] = alignment_sha256
    return isinstance(plan,dict) and all(plan.get(k)==v for k,v in expected.items())


def probe_command(profile,elapsed):
    """Return wire amplitudes; ramp rate is not a calibrated SI acceleration."""
    if not math.isfinite(elapsed) or elapsed < 0:
        raise ValueError('probe elapsed time must be finite and nonnegative')
    parameters=probe_parameters(profile)
    if elapsed >= parameters['pulse_s']:
        return 0.,0.
    if profile in ('forward_speed_ramp','backward_speed_ramp'):
        return parameters['wire_vx'],parameters['target_wire_vy']*min(elapsed/parameters['ramp_s'],1.)
    if profile in ('positive_x_ramp', 'negative_x_ramp'):
        return parameters['target_wire_vx']*min(elapsed/parameters['ramp_s'],1.),parameters['wire_vy']
    return parameters['wire_vx'],parameters['wire_vy']


class AngularWindow:
    def __init__(self):
        self.samples = deque(maxlen=3)
        self.last_stamp = None

    def update(self,stamp,wz):
        if stamp != self.last_stamp:
            if self.last_stamp is not None and stamp-self.last_stamp > 220_000_000:
                self.samples.clear()
            self.samples.append(wz)
            self.last_stamp = stamp
        return statistics.median(self.samples) if len(self.samples)==3 else 0.


def guard(origin, pose, angular_rate=None, direction='forward',profile=None):
    if profile not in (None,'forward_speed_ramp','backward_speed_ramp'):
        raise ValueError('unknown forward guard profile')
    if profile == 'backward_speed_ramp':
        along_limit=BACKWARD_RAMP_POSITION_RADIUS
        linear_limit=BACKWARD_RAMP_MAX_LINEAR_SPEED
        reverse_limit=-BACKWARD_RAMP_MAX_POSITIVE_FORWARD
        cross_limit=BACKWARD_RAMP_MAX_BODY_LEFT
    else:
        along_limit=FORWARD_RAMP_POSITION_RADIUS if profile=='forward_speed_ramp' else .23
        linear_limit=FORWARD_RAMP_MAX_LINEAR_SPEED if profile=='forward_speed_ramp' else .10
        reverse_limit=-.015
        cross_limit=TRIAL_CROSS_LIMIT
    forward, lateral, yaw = relative(origin, pose)
    along,cross = direction_components(forward,lateral,direction)
    if along < reverse_limit:
        raise RuntimeError('reverse motion detected')
    if along > along_limit:
        raise RuntimeError('travel limit exceeded')
    height=float(body_displacement(origin,pose)[2])
    if abs(cross) > cross_limit or abs(yaw) > TRIAL_YAW_LIMIT or abs(height) > TRIAL_HEIGHT_LIMIT:
        raise RuntimeError(f'deviation: cross={cross:.6f} m, body_left={lateral:.6f} m, '
                           f'yaw={yaw:.6f} rad, height={height:.6f} m')
    if (pose[4] > linear_limit or abs(pose[5]) > .60
            or abs(pose[5] if angular_rate is None else angular_rate) > .15):
        raise RuntimeError(f'measured speed exceeded trial limit: linear={pose[4]:.6f}, angular={pose[5]:.6f}')
    return forward, lateral, yaw


def final_target_check(origin, pose, direction='forward'):
    """Validate the stopped endpoint without interrupting zero-speed handling."""
    forward,left,yaw=relative(origin,pose)
    along,cross=direction_components(forward,left,direction)
    height=float(body_displacement(origin,pose)[2])
    reasons=[]
    if not all(math.isfinite(v) for v in (along,cross,yaw,height)):
        reasons.append('non-finite final displacement')
    if not .18 <= along <= .22:
        reasons.append('final along distance outside acceptance')
    if abs(cross) > TRIAL_CROSS_LIMIT:
        reasons.append('final cross deviation exceeded limit')
    if abs(yaw) > TRIAL_YAW_LIMIT:
        reasons.append('final yaw deviation exceeded limit')
    if abs(height) > TRIAL_HEIGHT_LIMIT:
        reasons.append('final height deviation exceeded limit')
    return {'accepted':not reasons,'along_m':along,'cross_m':cross,
            'yaw_rad':yaw,'body_up_m':height,'rejection_reasons':reasons,
            'limits':{'along_min_m':.18,'along_max_m':.22,
                      'max_cross_m':TRIAL_CROSS_LIMIT,'max_yaw_rad':TRIAL_YAW_LIMIT,
                      'max_body_up_m':TRIAL_HEIGHT_LIMIT}}


def probe_guard(origin, pose, angular_rate=None,profile='positive_y'):
    forward,lateral,yaw = relative(origin,pose)
    parameters=probe_parameters(profile)
    radius=parameters['position_radius_guard_m']
    linear_limit=parameters.get('max_linear_speed_m_s',.10)
    if math.hypot(forward,lateral) > radius or abs(yaw) > .08 or abs(body_displacement(origin,pose)[2]) > .03:
        raise RuntimeError('direction probe displacement/rotation limit')
    if (pose[4] > linear_limit or abs(pose[5]) > .60
            or abs(pose[5] if angular_rate is None else angular_rate) > .15):
        raise RuntimeError(f'direction probe measured speed limit: linear={pose[4]:.6f}, angular={pose[5]:.6f}')
    if profile in ('positive_x_ramp', 'negative_x_ramp') and abs(forward) > parameters['max_body_forward_m']:
        raise RuntimeError('positive wire-X probe body forward drift exceeded limit')
    if profile in ('forward_speed','forward_speed_ramp'):
        guard(origin,pose,angular_rate,'forward',
              'forward_speed_ramp' if profile=='forward_speed_ramp' else None)
    elif profile == 'backward_speed_ramp':
        guard(origin,pose,angular_rate,'backward','backward_speed_ramp')
    return forward,lateral,yaw


class StopWindow:
    """Only distinct, healthy odometry samples can establish a quiet interval."""
    LIMITS = {'window_s':1., 'min_samples':10, 'position_diameter_m':.008,
              'max_speed_m_s':.03, 'mean_speed_m_s':.015, 'max_yaw_change_rad':.02,
              'max_yaw_rate_rad_s':.04, 'max_arrival_gap_s':.22}

    def __init__(self):
        self.last_time = None
        self.last_stamp = None
        self.samples = deque()
        self.confirmed = False

    def reset(self):
        self.samples.clear()
        self.confirmed = False

    def update(self, now, stamp, pose):
        if self.last_time is not None and now-self.last_time > self.LIMITS['max_arrival_gap_s']:
            self.reset()
        if stamp == self.last_stamp:
            return
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.reset()
        self.last_stamp,self.last_time = stamp,now
        if pose[4] >= self.LIMITS['max_speed_m_s'] or abs(pose[5]) >= self.LIMITS['max_yaw_rate_rad_s']:
            self.reset()
            return
        self.samples.append((now,pose))
        while len(self.samples)>1 and now-self.samples[1][0] >= self.LIMITS['window_s']:
            self.samples.popleft()
        self.confirmed = self.diagnostics()['confirmed']

    @classmethod
    def criteria(cls):
        return dict(cls.LIMITS)

    def diagnostics(self):
        """Describe the same retained raw samples and the exact acceptance gates."""
        positions = [p[:3] for _,p in self.samples]
        diameter = max((math.dist(a,b) for a in positions for b in positions),default=0.)
        yaw_range = max((abs(math.remainder(p[3]-self.samples[0][1][3],2*math.pi))
                         for _,p in self.samples),default=0.)
        mean_speed = sum(p[4] for _,p in self.samples)/len(self.samples) if self.samples else None
        span = self.samples[-1][0]-self.samples[0][0] if self.samples else 0.
        checks = {'sample_count':len(self.samples)>=self.LIMITS['min_samples'],
                  'window_span':span>=self.LIMITS['window_s'],
                  'position_diameter':diameter<=self.LIMITS['position_diameter_m'],
                  'yaw_change':yaw_range<=self.LIMITS['max_yaw_change_rad'],
                  'mean_speed':mean_speed is not None and mean_speed<self.LIMITS['mean_speed_m_s']}
        return {'sample_count':len(self.samples), 'span_s':span,
                'position_diameter_m':diameter, 'max_yaw_change_rad':yaw_range,
                'mean_speed_m_s':mean_speed,
                'max_speed_m_s':max((p[4] for _,p in self.samples),default=None),
                'max_abs_yaw_rate_rad_s':max((abs(p[5]) for _,p in self.samples),default=None),
                'checks':checks, 'failure_reasons':[key for key,passed in checks.items() if not passed],
                'confirmed':all(checks.values())}


def body_to_wire(forward,left,alignment):
    if alignment.get('command_strategy') == 'forward_axis_only':
        validate_forward_axis_alignment(alignment)
        if left != 0:
            raise ValueError('lateral command is not verified')
        vx,vy = 0.,-forward
        if not all(math.isfinite(v) for v in (vx,vy)) or math.hypot(vx,vy)>WIRE_LIMIT:
            raise ValueError('mapped wire command outside bounds')
        return vx,vy
    if alignment.get('command_strategy') is not None:
        raise ValueError('unknown command strategy')
    if alignment.get('forward_direction_confirmed') is not True:
        raise ValueError('physical forward direction not confirmed')
    matrix = alignment['body_to_wire_xy']
    vx = matrix[0][0]*forward+matrix[0][1]*left
    vy = matrix[1][0]*forward+matrix[1][1]*left
    if not all(math.isfinite(v) for v in (vx,vy)) or math.hypot(vx,vy)>WIRE_LIMIT:
        raise ValueError('mapped wire command outside bounds')
    return vx,vy


def trial_command(origin,pose,alignment,direction='forward'):
    forward,left,yaw=relative(origin,pose)
    along,cross=direction_components(forward,left,direction)
    amplitude=TRIAL_SLOW_AMPLITUDE if along>TRIAL_SLOW_AT else TRIAL_AMPLITUDE
    if alignment.get('command_strategy') == 'forward_axis_only':
        trial_parameters(direction,alignment)
        # Cross/yaw remain abort conditions. Until the lateral axis is checked,
        # neither side correction nor yaw-based command rotation is permitted.
        return body_to_wire(amplitude,0.,alignment)
    correction=max(-.08,min(.08,-4.*cross))
    ux,uy=DIRECTIONS[direction]
    initial_forward=ux*amplitude-uy*correction
    initial_left=uy*amplitude+ux*correction
    # Reference line is fixed in the start body frame; rotate its command into
    # the current body frame. This is translation only, with no chassis yaw.
    current_forward=math.cos(yaw)*initial_forward+math.sin(yaw)*initial_left
    current_left=-math.sin(yaw)*initial_forward+math.cos(yaw)*initial_left
    return body_to_wire(current_forward,current_left,alignment)


def forward_command(origin,pose,alignment):
    return trial_command(origin,pose,alignment,'forward')


def record_zero_request(report, latest, sequence, now=time.monotonic, wall=time.time):
    """Retain the last received odometry at a host request, including faults."""
    requested=now()
    pose=latest.get('pose')
    marker={'request_unix_s':wall(), 'request_monotonic_s':requested,
            'nav_sequence':sequence, 'odom_stamp_ns':latest.get('stamp'),
            'odom_pose':list(pose) if pose is not None else None,
            'odom_arrival_age_s':requested-latest['odom_at'] if pose is not None else None,
            'odom_fault':latest.get('fault'),
            'meaning':'host zero request and last received odometry; not an MCU acknowledgement'}
    report.setdefault('first_zero_request',marker)
    return marker


def record_zero_host_write(report, request, now=time.monotonic, wall=time.time):
    report.setdefault('first_zero_host_write',dict(request,
        host_write_completed_unix_s=wall(), host_write_completed_monotonic_s=now(),
        meaning='zero frame fully accepted by host os.write; no UART drain or MCU acknowledgement'))


def is_stopping_zero(report,vx,vy):
    return report.get('phase')=='stopping' and report.get('movement_attempted') is True and vx==0 and vy==0


def stopping_displacement(origin, zero_pose, final_pose, direction='forward'):
    delta=body_displacement(origin,final_pose)-body_displacement(origin,zero_pose)
    along,cross=direction_components(float(delta[0]),float(delta[1]),direction)
    return {'body_forward_m':float(delta[0]), 'body_left_m':float(delta[1]),
            'body_up_m':float(delta[2]), 'along_m':along, 'cross_m':cross,
            'reference':'initial body axes; last odometry at first host zero request to last observed pose',
            'meaning':'odometry displacement estimate; not wheel feedback or independently measured stopping distance'}


STOP_RELEASE_MIN_S = 1.2
STOP_RELEASE_MAX_S = 3.


def stop_observation_criteria():
    return dict(StopWindow.criteria(), zero_hold_s=3., release_observe_min_s=STOP_RELEASE_MIN_S,
                release_observe_max_s=STOP_RELEASE_MAX_S,
                meaning='stability of estimated odometry; not an MCU wheel-speed acknowledgement')


def bounded_stop(send_zero, observe, now=time.monotonic, sleep=time.sleep, hold_s=3.):
    """Keep trying zero despite bad pose or motion guards; never resumes movement.

    observe must validate freshness and return (stamp, pose). A failed zero write
    is latched; this function never reopens a broken transport. Releasing NAV is
    permitted only after the zero hold and a measured quiet window.
    """
    begin = now()
    window = StopWindow()
    result = {'zero_attempts':0,'zero_delivered':0,'errors':[],
              'revoke_sent':False,'confirmed':False,'samples':[],
              'release_samples':[], 'release_window_checks':[],
              'release_observe_min_s':STOP_RELEASE_MIN_S, 'release_observe_max_s':STOP_RELEASE_MAX_S,
              'release_observe_duration_s':0., 'release_final_window':None,
              'release_end_reason':'hold_not_quiet'}
    write_failed = False
    while now()-begin < hold_s:
        result['zero_attempts'] += 1
        try:
            send_zero(True)
            result['zero_delivered'] += 1
        except Exception as error:
            write_failed = True
            if str(error) not in result['errors']:
                result['errors'].append(str(error))
        try:
            stamp,pose = observe()
            observed_at = now()
            window.update(observed_at,stamp,pose)
            result['samples'].append([observed_at-begin,stamp,list(pose)])
        except Exception as error:
            window.reset()
            if str(error) not in result['errors']:
                result['errors'].append(str(error))
        sleep(.02)
    result['hold_duration_s'] = now()-begin
    result['hold_final_window'] = window.diagnostics()
    if write_failed:
        result['release_end_reason'] = 'zero_write_failed'
    # Lease expiry returns control to the RC even if we cannot confirm stopping.
    # Do not explicitly hand over sooner by sending revoke on a failed stop.
    if window.confirmed and not write_failed:
        released = None
        final_window = StopWindow()
        try:
            send_zero(False)
            result['revoke_sent'] = True
            released = now()
            deadline = released+STOP_RELEASE_MAX_S
            result['release_end_reason'] = 'deadline_without_quiet_window'
            while now() < deadline:
                stamp,pose = observe()
                observed_at = now()
                elapsed = observed_at-released
                final_window.update(observed_at,stamp,pose)
                result['release_samples'].append({'elapsed_s':elapsed,'monotonic_s':observed_at,
                    'stamp_ns':stamp,'pose':list(pose),'error':None})
                result['release_window_checks'].append(dict(final_window.diagnostics(),elapsed_s=elapsed))
                if STOP_RELEASE_MIN_S <= elapsed <= STOP_RELEASE_MAX_S and final_window.confirmed:
                    result['confirmed'] = True
                    result['release_end_reason'] = 'current_quiet_window'
                    break
                remaining = deadline-now()
                if remaining > 0:
                    sleep(min(.02,remaining))
        except Exception as error:
            result['errors'].append(str(error))
            result['release_end_reason'] = 'observation_error' if released is not None else 'revoke_send_error'
            if released is not None:
                observed_at = now()
                result['release_samples'].append({'elapsed_s':observed_at-released,
                    'monotonic_s':observed_at,'stamp_ns':None,'pose':None,'error':str(error)})
                result['release_window_checks'].append(dict(final_window.diagnostics(),
                    elapsed_s=observed_at-released,error=str(error)))
        finally:
            if released is not None:
                result['release_observe_duration_s'] = now()-released
                result['release_final_window'] = final_window.diagnostics()
    return result


def stopped_trial_outcome(pending_outcome,stop_confirmed,target_accepted=False):
    if not stop_confirmed:
        return 'STOP_UNCONFIRMED_OPERATOR_STOP_REQUIRED'
    if pending_outcome == 'probe_finished_pending_stop':
        return 'direction_probe_completed_mapping_unverified'
    if pending_outcome == 'forward_speed_finished_pending_stop':
        return 'forward_speed_probe_completed'
    if pending_outcome == 'forward_speed_ramp_finished_pending_stop':
        return 'forward_speed_ramp_completed'
    if pending_outcome == 'lateral_work_finished_pending_stop':
        return 'lateral_work_completed_mapping_unverified'
    if pending_outcome == 'backward_speed_ramp_finished_pending_stop':
        return 'backward_speed_ramp_completed'
    return ('completed' if pending_outcome=='target_reached_pending_stop' and target_accepted
            else 'stopped_without_target_acceptance')


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute-20cm', action='store_true')
    mode.add_argument('--probe-axis-y', action='store_true', help='one +vy pulse, <=1.0 s; no 20 cm goal')
    mode.add_argument('--probe-negative-y', action='store_true',
                      help='one fixed -vy 0.38 pulse, <=0.60 s; direction remains unverified')
    mode.add_argument('--probe-forward-speed', action='store_true',
                      help='verified forward -vy 0.60 pulse, <=2.0 s; no 20 cm goal')
    mode.add_argument('--probe-forward-speed-ramp', action='store_true',
                      help='verified forward wire ramp 0 to -2.0 in 1.0 s, <=2.5 s total; no distance goal')
    mode.add_argument('--probe-positive-x-ramp', action='store_true',
                      help='candidate +wire-X work probe, <=2.0 s; cap +2.0, 0.35m software trigger; no SI calibration')
    mode.add_argument('--probe-backward-speed-ramp', action='store_true',
                      help='backward +wire-Y ramp to +2.0 in 1.0 s, <=2.0 s total; no distance goal')
    mode.add_argument('--probe-negative-x-ramp', dest='probe_positive_x_ramp',
                      action='store_const', const='negative',
                      help='candidate -wire-X work probe; direction not accepted')
    parser.add_argument('--entry-id')
    parser.add_argument('--direction', choices=tuple(DIRECTIONS), default='forward',
                        help='one bounded 20 cm translation; requires --execute-20cm')
    parser.add_argument('--site-ready', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.direction != 'forward' and not args.execute_20cm:
        parser.error('--direction requires --execute-20cm')
    return args


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    args = parse_arguments()
    verify_bundle()
    if args.output.exists():
        raise SystemExit('output exists; choose a new receipt')
    ramping = (args.probe_forward_speed_ramp or args.probe_positive_x_ramp
               or args.probe_backward_speed_ramp)
    probing = args.probe_axis_y or args.probe_negative_y or args.probe_forward_speed or ramping
    probe_profile = ('backward_speed_ramp' if args.probe_backward_speed_ramp else
                     (('negative_x_ramp' if args.probe_positive_x_ramp == 'negative' else 'positive_x_ramp') if args.probe_positive_x_ramp else
                     ('forward_speed_ramp' if args.probe_forward_speed_ramp else
                      ('forward_speed' if args.probe_forward_speed else
                       ('negative_y' if args.probe_negative_y else 'positive_y')))))
    probe = probe_parameters(probe_profile) if probing else None
    wire_limit = (FORWARD_RAMP_WIRE_LIMIT if ramping else
                  (FORWARD_SPEED_WIRE_LIMIT if args.probe_forward_speed else WIRE_LIMIT))
    moving = args.execute_20cm or probing
    if moving and not args.site_ready:
        raise SystemExit('site readiness is required for motion')
    fault_latch = ROOT / 'logs/forward_trial_fault.json'
    latch = json.loads(fault_latch.read_text(encoding='utf-8')) if fault_latch.exists() else {}
    if args.execute_20cm and latch.get('forward_trial_authorized') is not True:
        raise SystemExit('20 cm trial requires a reviewed one-shot plan after the previous fault')
    if probing and latch.get('direction_probe_authorized') is not True:
        raise SystemExit('A reviewed one-shot direction probe plan is required')
    if moving:
        owned_plan = latch.get('direction_probe_plan' if probing else 'forward_trial_plan', {})
        if (not args.entry_id or owned_plan.get('entry_id') != args.entry_id
                or latch.get('session_paused') is not False or latch.get('blocked') is not False):
            raise SystemExit('candidate needs the current operator-owned one-shot entry')
    if probing:
        plan = latch.get('direction_probe_plan',{})
        alignment_sha256 = None
        if args.probe_forward_speed or ramping:
            alignment_path=ROOT/'direction_alignment.json'
            alignment=json.loads(alignment_path.read_text(encoding='utf-8'))
            trial_parameters('forward',alignment)
            alignment_sha256=hashlib.sha256(alignment_path.read_bytes()).hexdigest()
        if not probe_plan_matches(plan,probe_profile,
                hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),alignment_sha256):
            raise SystemExit('Prepared probe limits/source do not match this script')
    if args.execute_20cm:
        alignment = json.loads((ROOT/'direction_alignment.json').read_text(encoding='utf-8'))
        trial_parameters(args.direction,alignment)
        body_to_wire(0.,0.,alignment)
        plan = latch.get('forward_trial_plan',{})
        if not trial_plan_matches(plan,args.direction,
                hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                hashlib.sha256((ROOT/'direction_alignment.json').read_bytes()).hexdigest(),alignment):
            raise SystemExit('Reviewed one-shot plan does not match direction/limits/script/mapping')
    baseline = json.loads((ROOT / '固件基准核对.json').read_text(encoding='utf-8-sig'))
    for f in baseline['files']:
        actual = hashlib.sha256((ROOT.parent / 'firmware/sentry' / f['path']).read_bytes()).hexdigest()
        if actual != f['remote_sha256']:
            raise SystemExit('firmware source changed: ' + f['path'])
    # Root is needed only for a complete /proc serial-owner audit. Drop to the
    # workspace owner before ROS discovery so DDS uses the existing user's IPC.
    transport = SerialTransport('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', 921600)
    transport.open()
    if os.geteuid() == 0:
        import pwd
        account = pwd.getpwuid(ROOT.stat().st_uid)
        if account.pw_uid == 0:
            transport.close()
            raise SystemExit('navigation directory must belong to the robot user')
        os.initgroups(account.pw_name, account.pw_gid)
        os.setgid(account.pw_gid)
        os.setuid(account.pw_uid)
    os.environ['ROS_LOG_DIR'] = str(ROOT / 'logs/forward_trial_ros')
    rclpy.init(args=[])
    node = rclpy.create_node('navigation_forward_trial')
    latest = {'pose': None, 'odom_at': 0., 'stamp': 0, 'status': None, 'status_at': 0., 'fault': None}
    poses = []
    source_poses = []

    def odom(m):
        ns = m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec
        p, q = m.pose.pose.position, m.pose.pose.orientation
        v, w = m.twist.twist.linear, m.twist.twist.angular
        vals = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z]
        if (m.header.frame_id != 'odom' or m.child_frame_id != 'base_link'
                or not all(math.isfinite(x) for x in vals)
                or abs(sum(x*x for x in [q.x,q.y,q.z,q.w])-1) > .001
                or ns <= latest['stamp']):
            latest['fault'] = 'invalid odometry/frame or non-increasing time'
            return
        try:
            check_odometry_source_age(node.get_clock().now().nanoseconds, ns)
        except RuntimeError as error:
            latest['fault'] = str(error)
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        pose = (p.x,p.y,p.z,yaw,math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),w.z,q.x,q.y,q.z,q.w)
        if latest['pose'] is not None and math.dist(pose[:3], latest['pose'][:3]) > .035:
            latest['fault'] = 'odometry position jump'
        latest.update(pose=pose, odom_at=time.monotonic(), stamp=ns)
        poses.append((time.monotonic(), pose))
        source_poses.append((time.monotonic(),ns,pose))

    def status(m):
        try:
            latest.update(status=json.loads(m.data), status_at=time.monotonic())
        except ValueError:
            latest['fault'] = 'invalid localization status'

    subs = [node.create_subscription(Odometry, '/navigation/odom', odom, qos_profile_sensor_data),
            node.create_subscription(String, '/navigation/localization_status', status, qos_profile_sensor_data)]
    stream = Stream()
    fanout = ReceiveFanout(stream)
    seq_path = ROOT / 'logs/forward_trial_sequence.json'
    seq = json.loads(seq_path.read_text(encoding='utf-8'))['next_sequence'] if seq_path.exists() else 0
    report = {'requested_forward_m':TRIAL_DISTANCE if args.execute_20cm and args.direction=='forward' else None,
              'requested_distance_m':TRIAL_DISTANCE if args.execute_20cm else None,
              'direction':args.direction if args.execute_20cm else None,
              'mode': ('backward_speed_ramp_probe' if args.probe_backward_speed_ramp else
                       ('lateral_work_probe' if args.probe_positive_x_ramp else
                       ('forward_speed_ramp_probe' if args.probe_forward_speed_ramp else
                       ('forward_speed_probe' if args.probe_forward_speed else
                       (('negative_y_probe' if args.probe_negative_y else 'axis_y_probe') if probing
                        else ('execute' if args.execute_20cm else 'receive_only')))))),
              'started_unix': time.time(), 'outcome': 'preflight_failed', 'movement_attempted': False,
              'phase':'preflight',
              'obstacle_avoidance': 'local operator clearance; no automatic obstacle guard',
              'velocity_scale': f'unmeasured; wire vector capped at {wire_limit:.2f}; not calibrated SI velocity',
              'physical_distance_accuracy_validated': False, 'trace': [],
              'trace_columns':['elapsed_s','body_forward_m','body_left_m','relative_yaw_rad',
                               'measured_speed_m_s','wire_vx','wire_vy','along_m','cross_m']}
    if probing:
        report['probe_plan'] = latch['direction_probe_plan']
        report['probe_profile'] = probe_profile
    if args.probe_forward_speed or ramping:
        report['direction_alignment']=alignment
        report['alignment_sha256']=alignment_sha256
        report['position_limit_meaning']=f"{probe['position_radius_guard_m']:.2f} m is an odometry software trigger, not a physical hard travel limit"
    if ramping:
        report['wire_write_timeline']=[]
        report['ramp_units']='wire amplitude per second; not a calibrated SI acceleration'
    if args.probe_positive_x_ramp:
        report['wire_x_physical_direction']='candidate negative wire-X; physical direction unaccepted' if args.probe_positive_x_ramp == 'negative' else 'positive wire-X previously observed left; working point unaccepted'
        report['positive_x_body_forward_guard_m']=probe['max_body_forward_m']
        report['stopping_distance_direction']='right' if args.probe_positive_x_ramp == 'negative' else 'left'
    if args.probe_backward_speed_ramp:
        report['intended_body_direction']='backward; no lateral mapping activated'
        report['stopping_distance_direction']='backward'
        report['trace_direction_reference']='backward along=-body_forward, cross=-body_left; body columns retain initial body axes'
        report['backward_body_guard_m']={'max_body_left':probe['max_body_left_m'],
                                       'max_positive_body_forward':probe['max_positive_body_forward_m']}
    if args.execute_20cm:
        report['direction_alignment'] = alignment
        report['forward_trial_plan'] = latch['forward_trial_plan']
        report['trial_parameters'] = trial_parameters(args.direction,alignment)
        report['distance_reference']='full initial base_link 3D orientation; not world XY projection'
    report['angular_motion_guard'] = {'median_source_samples':3,'median_limit_rad_s':.15,
        'instantaneous_limit_rad_s':.60,'absolute_yaw_limit_rad':.08 if probing else .10,
        'raw_angular_rates_retained_in_samples':True}
    report['stop_observation_criteria'] = stop_observation_criteria()
    origin = None
    stop_confirmed = False

    def tick(check_fault=True):
        rclpy.spin_once(node, timeout_sec=.005)
        data = transport.read_available(16384)
        fanout.feed(data, time.monotonic())
        if check_fault and fanout.error:
            raise RuntimeError('chassis observation failed: '+fanout.error)
        if check_fault and latest['fault']:
            raise RuntimeError(latest['fault'])

    def health(auto=False):
        now = time.monotonic()
        if stream.boot is None or stream.state is None or now-stream.status_at > .15:
            raise RuntimeError('MCU status missing or stale')
        if latest['pose'] is None or now-latest['odom_at'] > .22:
            raise RuntimeError('odometry missing or stale')
        check_odometry_source_age(node.get_clock().now().nanoseconds, latest['stamp'])
        if (latest['status'] is None or now-latest['status_at'] > 1.6
                or not latest['status'].get('fresh') or latest['status'].get('fault')):
            raise RuntimeError('localization status not healthy')
        if latest['fault']:
            raise RuntimeError(latest['fault'])
        if len(node.get_publishers_info_by_topic('/navigation/odom')) != 1:
            raise RuntimeError('expected exactly one odometry source')
        if auto and not stream.state & 0x80:
            raise RuntimeError('physical AUTO is not active')

    def send(vx, enabled=True, vy=0.):
        nonlocal seq
        zero_request = (record_zero_request(report,latest,seq)
                        if is_stopping_zero(report,vx,vy) else None)
        write_marker=None
        if ramping:
            requested=time.monotonic()
            elapsed=requested-report['motion_begin_monotonic_s']
            planned=probe_command(probe_profile,max(0.,elapsed))
            write_marker={'request_monotonic_s':requested,'elapsed_s':elapsed,'nav_sequence':seq,
                          'phase':report['phase'],'planned_wire_vx':planned[0],'planned_wire_vy':planned[1],
                          'submitted_wire_vx':vx,'submitted_wire_vy':vy,'submitted_wire_wz':0.,
                          'nav_enabled':enabled,'host_transport_submitted':False,
                          'host_os_write_completed':False,
                          'meaning':'host submission and OS write completion; not MCU acknowledgement or motor feedback'}
            report['wire_write_timeline'].append(write_marker)
        frame = nav_frame(stream.boot, seq, vx, enabled, vy,wire_limit=wire_limit)
        seq = (seq+1) & 0xffff
        # Persist before sending so a process interruption cannot replay a seq.
        persist_sequence(seq_path, seq, stream.boot)
        if write_marker is not None:
            write_marker['host_submit_monotonic_s']=time.monotonic()
        submitted=transport.submit(frame, stop=(vx==0 and vy==0))
        if write_marker is not None:
            write_marker.update(host_transport_submitted=submitted,
                                host_transport_submission_monotonic_s=time.monotonic())
        if not transport.pump(timeout_s=.04):
            raise RuntimeError('serial write incomplete')
        if write_marker is not None:
            write_marker.update(host_os_write_completed=submitted,
                                host_write_completed_monotonic_s=time.monotonic())
        if zero_request is not None:
            record_zero_host_write(report,zero_request)

    def interrupted(signum, frame):
        raise KeyboardInterrupt('signal '+str(signum))

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    try:
        # Discard queued host input once, before establishing this session's boot.
        # A boot change after this point remains a hard fault.
        import termios
        termios.tcflush(transport._fd, termios.TCIFLUSH)
        begin = time.monotonic()
        while time.monotonic()-begin < 2.5:
            tick()
        health()
        recent = [p for t,p in poses if t > time.monotonic()-1.5]
        if len(recent) < 10 or any(p[4] > .04 or abs(p[5]) > .06 for p in recent):
            raise RuntimeError('robot is not sufficiently still before start')
        if max(math.dist(recent[0][:3], p[:3]) for p in recent) > .015:
            raise RuntimeError('preflight position is not stable')
        report.update(boot_id=stream.boot, mcu_state=stream.state, mcu_last_reject=stream.reject,
                      odometry=list(latest['pose']), received_frames=stream.count)
        if stream.state & 0x60:
            raise RuntimeError('an existing navigation lease is active')
        if not moving:
            report['outcome'] = 'receive_only_passed'
        else:
            health(auto=True)
            if args.probe_backward_speed_ramp and not fanout.attitude.unique_sample_fresh(time.monotonic()):
                raise RuntimeError('Fresh independent chassis 0x02 observation required before backward retry')
            origin = latest['pose']
            report['origin'] = list(origin)
            if probing:
                latch['direction_probe_authorized'] = False
                latch['probe_consumed_unix'] = time.time()
                atomic_json(fault_latch, latch)
            if args.execute_20cm:
                latch['forward_trial_authorized'] = False
                latch['forward_trial_consumed_unix'] = time.time()
                atomic_json(fault_latch, latch)
            begin = time.monotonic()
            report['motion_begin_monotonic_s']=begin
            report['phase']='motion'
            report['movement_attempted'] = True
            if ramping:
                send(0.,vy=0.)
            last_send = begin if ramping else -1.
            first_nonzero = None
            angular_window = AngularWindow()
            while True:
                tick()
                health(auto=True)
                now = time.monotonic()
                if now-last_send >= .100 and last_send >= 0:
                    raise RuntimeError('100ms command lease gap; no stale motion resume')
                angular_rate=angular_window.update(latest['stamp'],latest['pose'][5])
                if probing:
                    forward,lateral,yaw=probe_guard(origin,latest['pose'],angular_rate,probe_profile)
                    if args.probe_backward_speed_ramp:
                        along,cross=direction_components(forward,lateral,'backward')
                    else:
                        along,cross=None,None
                else:
                    forward,lateral,yaw=guard(origin,latest['pose'],angular_rate,args.direction)
                    along,cross=direction_components(forward,lateral,args.direction)
                if probing and now-begin >= probe['pulse_s']:
                    report['outcome'] = ('backward_speed_ramp_finished_pending_stop' if args.probe_backward_speed_ramp else
                                         ('lateral_work_finished_pending_stop' if args.probe_positive_x_ramp else
                                         ('forward_speed_ramp_finished_pending_stop' if args.probe_forward_speed_ramp
                                          else ('forward_speed_finished_pending_stop' if args.probe_forward_speed
                                                else 'probe_finished_pending_stop'))))
                    break
                if args.execute_20cm and along >= TRIAL_STOP_AT:
                    report['outcome'] = 'target_reached_pending_stop'
                    break
                if now-begin > TRIAL_TIMEOUT_S:
                    raise RuntimeError('20 second trial deadline')
                if first_nonzero and now-first_nonzero > .3 and (stream.state & 0x60) != 0x60:
                    raise RuntimeError('MCU navigation lease not fresh/permitted')
                if args.execute_20cm and now-begin > 4 and along < .005:
                    raise RuntimeError('no progress along requested direction; no automatic speed increase')
                if now-last_send >= .02:
                    # Wire values are deliberately not interpreted as calibrated m/s.
                    if probing:
                        vx,vy = probe_command(probe_profile,now-begin)
                    else:
                        vx,vy = trial_command(origin,latest['pose'],alignment,args.direction)
                    send(vx,vy=vy)
                    if vx!=0 or vy!=0:
                        first_nonzero = first_nonzero or now
                    last_send = now
                    report['trace'].append([now-begin,forward,lateral,yaw,latest['pose'][4],vx,vy,along,cross])
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
        if report['movement_attempted']:
            report['outcome'] = 'aborted_pending_stop'
    finally:
        if report['movement_attempted']:
            report['phase']='stopping'
        if report['movement_attempted'] and not transport.is_open:
            report['outcome'] = 'STOP_UNCONFIRMED_OPERATOR_STOP_REQUIRED'
        if report['movement_attempted'] and transport.is_open:
            try:
                def stop_observe():
                    tick(check_fault=False)
                    health()
                    return latest['stamp'],latest['pose']
                report['stopping'] = bounded_stop(lambda enabled:send(0.,enabled), stop_observe)
                stop_confirmed = report['stopping']['confirmed']
                report['final_relative'] = list(relative(origin, latest['pose']))
                report['final_pose'] = list(latest['pose'])
                if args.execute_20cm:
                    target_check=final_target_check(origin,latest['pose'],args.direction)
                    report['final_target_acceptance']=target_check
                    report['final_direction_relative']={k:target_check[k]
                                                        for k in ('along_m','cross_m','yaw_rad','body_up_m')}
                report['outcome'] = stopped_trial_outcome(report['outcome'],stop_confirmed,
                    report.get('final_target_acceptance',{}).get('accepted',False))
            except BaseException as error:
                report['stop_error'] = str(error) or type(error).__name__
                stop_confirmed = False
                report['outcome'] = 'STOP_UNCONFIRMED_OPERATOR_STOP_REQUIRED'
                # Do not abort zero attempts merely because pose checks failed.
                try:
                    for _ in range(10):
                        if not transport.is_open:
                            break
                        send(0.)
                        time.sleep(.02)
                except BaseException as last_error:
                    report['last_zero_attempt_error'] = str(last_error)
        if latest['pose'] is not None:
            report['last_observed_pose'] = list(latest['pose'])
            if origin is not None:
                report['last_observed_relative'] = list(relative(origin, latest['pose']))
                if args.execute_20cm:
                    along,cross=direction_components(*report['last_observed_relative'][:2],direction=args.direction)
                    report['last_observed_direction_relative']={'along_m':along,'cross_m':cross,
                                                                'yaw_rad':report['last_observed_relative'][2]}
                first_zero=report.get('first_zero_request')
                if first_zero and first_zero['odom_pose'] is not None:
                    report['stopping_displacement_estimate']=dict(
                        stopping_displacement(origin,first_zero['odom_pose'],latest['pose'],
                            ('right' if args.probe_positive_x_ramp == 'negative' else 'left')
                            if args.probe_positive_x_ramp else
                            ('backward' if args.probe_backward_speed_ramp else args.direction)),
                        from_odom_stamp_ns=first_zero['odom_stamp_ns'],to_odom_stamp_ns=latest['stamp'],
                        request_odom_arrival_age_s=first_zero['odom_arrival_age_s'],
                        request_odom_fault=first_zero['odom_fault'],stop_observed_in_odometry=stop_confirmed)
        report['observed_odometry_samples'] = [[t,list(p)] for t,p in poses]
        report['observed_odometry_source_samples'] = [[t,ns,list(p)] for t,ns,p in source_poses]
        report.update(stop_observed_in_odometry=stop_confirmed,
                      final_mcu_state=stream.state,final_mcu_reject=stream.reject,
                      received_frames=stream.count, crc_errors=stream.bad_crc, finished_unix=time.time())
        transport.close()
        report['serial'] = transport.status()
        report['chassis_attitude'] = fanout.result(report.get('motion_begin_monotonic_s'))
        if report['movement_attempted']:
            latch.update(blocked=report['outcome']!='completed',receipt=str(args.output),reason=report['outcome'],
                         time_unix=time.time(),direction_probe_authorized=False,forward_trial_authorized=False)
            atomic_json(fault_latch, latch)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        summary = {k:v for k,v in report.items() if k not in ('trace','observed_odometry_samples','stopping',
                                                            'wire_write_timeline','chassis_attitude','observed_odometry_source_samples')}
        if 'stopping' in report:
            summary['stopping'] = {k:v for k,v in report['stopping'].items()
                                   if k not in ('samples','release_samples','release_window_checks')}
        print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['outcome'] in ('receive_only_passed','completed',
                                    'direction_probe_completed_mapping_unverified',
                                    'forward_speed_probe_completed','forward_speed_ramp_completed',
                                    'lateral_work_completed_mapping_unverified',
                                    'backward_speed_ramp_completed') else 2


if __name__ == '__main__':
    raise SystemExit(main())
