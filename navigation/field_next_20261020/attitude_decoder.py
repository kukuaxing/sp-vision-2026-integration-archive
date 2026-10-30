"""Observe V2 0x02 bytes supplied by the existing serial owner. No I/O here.

This module is an offline candidate. It does not open a port, send a command,
alter a guard, or grant motion. MCU time is UART arrival time, not sensor time.
"""
from __future__ import annotations

import math
import struct


def crc16(data):
    value=0xffff
    for byte in data:
        value^=byte
        for _ in range(8):
            value=(value>>1)^(0x8408 if value&1 else 0)
    return value


class ChassisAttitudeDecoder:
    def __init__(self):
        self.buffer=bytearray()
        self.crc_errors=0
        self.last_host=None
        self.previous=None
        self.last_unique_host=None
        self.segment=0
        self.relative_yaw=0.
        self.records=[]

    def feed(self,data:bytes,host_monotonic_s:float):
        """Call with bytes already read by the sole owner; preserve returned rows."""
        now=float(host_monotonic_s)
        if not math.isfinite(now) or now<0 or (self.last_host is not None and now<self.last_host):
            raise ValueError('Host receive time must be finite and monotonic.')
        self.last_host=now
        self.buffer.extend(data)
        output=[]
        while len(self.buffer)>=14:
            start=self.buffer.find(b'\xa6\x6a')
            if start<0:
                self.buffer[:]=self.buffer[-1:]
                break
            if start:del self.buffer[:start]
            if len(self.buffer)<14:break
            _,version,kind,size,seq,boot=struct.unpack_from('<2sBBHHI',self.buffer)
            if version!=2 or size>128:
                del self.buffer[0]
                continue
            total=14+size
            if len(self.buffer)<total:break
            frame=bytes(self.buffer[:total])
            if crc16(frame[:-2])!=struct.unpack_from('<H',frame,total-2)[0]:
                self.crc_errors+=1
                del self.buffer[0]
                continue
            del self.buffer[:total]
            if (kind,size)!=(2,20):continue
            imu_id,flags,epoch,time_us,roll,pitch,yaw=struct.unpack_from('<BBHIfff',frame,12)
            row=dict(host_receive_monotonic_s=now,boot_id=boot,sequence=seq,imu_id=imu_id,
                     flags=flags,epoch=epoch,mcu_uart_arrival_us=time_us,
                     roll_rad=roll,pitch_rad=pitch,yaw_rad=yaw,raw_frame_hex=frame.hex(),
                     orientation_valid=bool(flags&1),gyro_valid=bool(flags&2),
                     sensor_axes_verified=bool(flags&8),sensor_sample_time_verified=bool(flags&4),
                     accepted=False,reason=None,relative_yaw_rad=None,
                     angle_difference_rate_rad_s=None,measured_gyro_rate_rad_s=None)
            self._observe(row)
            self.records.append(row)
            output.append(row)
        return output

    def _observe(self,row):
        previous=self.previous
        reason=None
        if not row['boot_id'] or row['imu_id']!=2 or row['flags']&~0x0f:
            reason='invalid chassis schema'
        elif row['gyro_valid']:
            reason='unexpected chassis gyro-valid flag for audited firmware'
        elif not row['orientation_valid'] or not all(math.isfinite(row[k]) for k in ('roll_rad','pitch_rad','yaw_rad')):
            reason='orientation invalid/non-finite'
        if reason:
            row['reason']=reason
            self.previous=None
            self.last_unique_host=None
            return
        reset=previous is None
        if previous:
            if row['boot_id']!=previous['boot_id']:
                reason,reset='boot changed; start independent observation interval',True
            elif row['epoch']!=previous['epoch']:
                reason,reset='IMU epoch changed; start independent observation interval',True
            else:
                delta_seq=(row['sequence']-previous['sequence'])&0xffff
                delta_us=(row['mcu_uart_arrival_us']-previous['mcu_uart_arrival_us'])&0xffffffff
                angles=('roll_rad','pitch_rad','yaw_rad')
                if not 0<delta_seq<32768:
                    row['reason']='duplicate/out-of-order frame sequence'
                    return
                if delta_us==0:
                    unchanged=all(row[k]==previous[k] for k in angles)
                    row['reason']='repeated IMU arrival timestamp' if unchanged else 'changed angles at repeated timestamp'
                    if not unchanged:
                        self.previous=None
                        self.last_unique_host=None
                    # Repeated telemetry is retained but cannot rejuvenate unique-sample freshness.
                    return
                if delta_us>=0x80000000 or delta_us>250000 or row['host_receive_monotonic_s']-previous['host_receive_monotonic_s']>.25:
                    reason,reset='source/receive gap or MCU time regression; start independent interval',True
                else:
                    dyaw=math.remainder(row['yaw_rad']-previous['yaw_rad'],2*math.pi)
                    self.relative_yaw+=dyaw
                    row['angle_difference_rate_rad_s']=dyaw/(delta_us*1e-6)
        if reset:
            self.segment+=1
            self.relative_yaw=0.
        row.update(accepted=True,reason=reason or ('interval baseline' if reset else 'distinct orientation sample'),
                   observation_interval=self.segment,relative_yaw_rad=self.relative_yaw)
        self.previous=row
        self.last_unique_host=row['host_receive_monotonic_s']

    def unique_sample_fresh(self,now,max_age_s=.15):
        if not math.isfinite(now) or not math.isfinite(max_age_s) or max_age_s<=0:return False
        return self.last_unique_host is not None and 0<=now-self.last_unique_host<=max_age_s

