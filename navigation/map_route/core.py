"""Planar route feedback in registered course coordinates; no ROS or device IO."""
from dataclasses import dataclass
import math


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


@dataclass(frozen=True)
class State:
    x: float
    y: float
    yaw: float
    forward_speed: float
    left_speed: float
    yaw_rate: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('Nonfinite pose or velocity')


class Tracker:
    """Absolute endpoints, signed along speed, independent lateral and yaw feedback.

    Speed calibration is approximate; simulation does not validate physical tracking.
    Reaching an endpoint enters a separate, stationary stop observation in the host.
    """
    def __init__(self, points, max_speed=.45):
        if not .1 <= max_speed <= .45:
            raise ValueError('This entry supports at most 0.45 m/s')
        self.points = tuple(tuple(p) for p in points)
        if len(self.points) != 3 or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in self.points):
            raise ValueError('Exactly three finite planar waypoints required')
        if self.points[0] != (0., 0.):
            raise ValueError('Route must start at registered P00')
        self.segments = []
        for a, b in zip(self.points, self.points[1:]):
            d = (b[0]-a[0], b[1]-a[1]); length = math.hypot(*d)
            if not .5 <= length <= 1.5:
                raise ValueError('Unreviewed segment length')
            self.segments.append((a, b, (d[0]/length, d[1]/length), length))
        self.max_speed = max_speed
        self.index = 0
        self.speed = 0.
        self.wz = 0.

    def advance(self):
        if self.index >= len(self.segments):
            raise ValueError('Route already complete')
        self.index += 1
        self.speed = self.wz = 0.

    def step(self, s, dt, source_age):
        if not 0 < dt <= .1 or not -.05 <= source_age <= .30:
            raise ValueError('Control interval or source age invalid')
        if self.index >= len(self.segments):
            raise ValueError('Route already complete')
        a, b, u, length = self.segments[self.index]
        n = (-u[1], u[0]); delta = (s.x-a[0], s.y-a[1])
        along = delta[0]*u[0] + delta[1]*u[1]
        cross = delta[0]*n[0] + delta[1]*n[1]
        c, sn = math.cos(s.yaw), math.sin(s.yaw)
        vx = c*s.forward_speed-sn*s.left_speed
        vy = sn*s.forward_speed+c*s.left_speed
        along_speed = vx*u[0]+vy*u[1]
        remaining = length-along
        distance = math.hypot(s.x-b[0], s.y-b[1])
        if abs(cross) > .08 or along < -.10 or along > length+.12 or abs(wrap(s.yaw)) > math.radians(12):
            raise RuntimeError('Route cross/along/heading bound exceeded')
        if math.hypot(s.forward_speed, s.left_speed) > .8 or abs(s.yaw_rate) > .6:
            raise RuntimeError('Measured planar speed/yaw rate exceeded')
        reached = remaining <= .025 and abs(cross) <= .025
        # Reserve source latency and assumed actuator delay before braking.
        room = max(0., remaining-.025-max(0., along_speed)*(max(0., source_age)+.12))
        # A finite 0.16 m/s terminal approach avoids asymptotic creeping and
        # the chassis low-command dead zone observed in earlier field trials.
        desired = min(self.max_speed, max(.16, math.sqrt(2*.45*room))) if remaining > .025 else 0.
        self.speed = clamp(desired, max(0., self.speed-.65*dt), self.speed+.45*dt)
        if remaining <= .025:
            self.speed = 0.
        # Metric lateral authority: 4 cm error requests 8 cm/s correction.
        lateral = clamp(-2.*cross, -.16, .16)
        cvx = self.speed*u[0] + lateral*n[0]
        cvy = self.speed*u[1] + lateral*n[1]
        forward = c*cvx+sn*cvy
        left = -sn*cvx+c*cvy
        wire_vx = left/.095
        wire_vy = -forward/(.0885 if forward >= 0 else .095)
        # Firmware positive wz turns right, decreasing the ROS yaw angle.
        target_wz = clamp(4.*wrap(s.yaw)+s.yaw_rate, -.30, .30)
        self.wz = clamp(target_wz, self.wz-dt, self.wz+dt)
        total = abs(wire_vx)+abs(wire_vy)
        scale = min(1., (2000./159.-abs(self.wz))/total) if total else 1.
        wire_vx *= scale; wire_vy *= scale
        if reached:
            wire_vx = wire_vy = self.wz = self.speed = 0.
        return dict(segment=self.index+1, along_m=along, cross_m=cross, remaining_m=remaining,
                    endpoint_error_m=distance, along_speed_m_s=along_speed,
                    heading_error_deg=math.degrees(wrap(s.yaw)), reached=reached,
                    wire_vx=wire_vx, wire_vy=wire_vy, wire_wz=self.wz,
                    planned_speed_m_s=self.speed, source_age_s=source_age)


class SourceGuard:
    """Latch one localization identity; faults are terminal for an active run."""
    def __init__(self, *, check_pose_jumps=True):
        self.identity = None
        self.last_stamp = None
        self.last_state = None
        self.check_pose_jumps = check_pose_jumps

    def check(self, identity, stamp_ns, now_ns, received_age, state, healthy):
        if not healthy or not identity or any(v is None for v in identity):
            raise RuntimeError('Localization not healthy')
        age = (now_ns-stamp_ns)/1e9
        if not -.05 <= age <= .30 or not 0 <= received_age <= .30:
            raise RuntimeError('Localization stale or future dated')
        if self.identity is not None and identity != self.identity:
            raise RuntimeError('Localization/map/registration identity changed; no resume')
        if self.last_stamp is not None:
            if stamp_ns < self.last_stamp:
                raise RuntimeError('Localization time reversed')
            if stamp_ns == self.last_stamp and (state.x,state.y,state.yaw) != (self.last_state.x,self.last_state.y,self.last_state.yaw):
                raise RuntimeError('Pose changed without source time advancing')
            if self.check_pose_jumps and stamp_ns > self.last_stamp:
                dt = (stamp_ns-self.last_stamp)/1e9
                old = self.last_state
                if math.hypot(state.x-old.x, state.y-old.y) > .035+.8*dt:
                    raise RuntimeError('Map pose discontinuity')
                if abs(wrap(state.yaw-old.yaw)) > .025+.6*dt:
                    raise RuntimeError('Map heading discontinuity')
        self.identity = identity
        self.last_stamp = stamp_ns
        self.last_state = state
        return age
