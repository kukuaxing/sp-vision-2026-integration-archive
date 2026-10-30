"""Translate the old measured rpm command into the newly flashed firmware units."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class NativeProfile:
    gain: float = 3859.9
    yaw_lever_m: float = 0.
    yaw_sign: float = 1.
    motor_rpm_bound: float = 2000.

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):raise ValueError('Nonfinite firmware parameter')
        if self.gain<=0 or self.yaw_lever_m<0 or abs(self.yaw_sign)!=1 or not 0<self.motor_rpm_bound<=6000:
            raise ValueError('Invalid firmware parameter')

    def convert(self, vx, vy, wz, *, preview=False):
        if not all(math.isfinite(v) for v in (vx,vy,wz)):raise ValueError('Nonfinite command')
        rx,ry,rz=(v*159. for v in (vx,vy,wz))
        if abs(rx)+abs(ry)+abs(rz)>self.motor_rpm_bound+1e-9:
            raise ValueError(f'Motor rpm sum exceeds configured {self.motor_rpm_bound:g} rpm ceiling')
        available=self.yaw_lever_m>0
        if not available and abs(wz)>1e-6 and not preview:
            raise RuntimeError('Flashed firmware yaw lever is zero; cannot send heading correction')
        return dict(wire_vx=rx/self.gain,wire_vy=ry/self.gain,
                    wire_wz=rz/(self.gain*self.yaw_lever_m*self.yaw_sign) if available else 0.,
                    heading_control_available=available,requested_motor_rpm=[rx,ry,rz],
                    calibration_basis='Preserve old measured motor targets; new SI calibration not validated')

    @classmethod
    def from_config(cls, config):
        return cls(gain=config['gain_rpm_per_unit'],yaw_lever_m=config['yaw_lever_m'],yaw_sign=config['yaw_sign'],
                   motor_rpm_bound=config.get('motor_rpm_ceiling',2000.))


def validate_flashed_profile(profile,review):
    if (profile.get('source_confirmed_flashed_by_user') is not True or review.get('user_confirmed_flashed') is not True
        or profile.get('yaw_lever_m',0)<=0 or abs(profile.get('yaw_sign',0))!=1
        or any(review.get(k)!=profile.get(k) for k in ['yaw_lever_m','yaw_sign','gain_rpm_per_unit'])):
        raise RuntimeError('Corrected flashed firmware profile has not been confirmed and reviewed')
    return NativeProfile.from_config(profile)
