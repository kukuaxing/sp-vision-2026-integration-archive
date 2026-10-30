"""Distance speed shaping and measured-sign heading feedback; no device I/O.

Only the forward ceiling and positive-wz sign have physical evidence. The
braking curve and mixed-axis response remain design assumptions.
"""
import math
import struct

DIRECTIONS={'forward':(1.,0.),'backward':(-1.,0.),'left':(0.,1.),'right':(0.,-1.)}
WIRE_LIMIT=2000./159.
TURN_LIMIT=.30

def finite(*values):
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
        raise ValueError('Finite numeric inputs required')

def quantize(v):
    raw=struct.pack('<f',v);rounded=struct.unpack('<f',raw)[0]
    if abs(rounded)>abs(v):
        rounded=struct.unpack('<f',struct.pack('<I',struct.unpack('<I',raw)[0]-1))[0]
    return rounded

def policy():
    return dict(schema='distance_profile_with_heading_v1',wire_l1_ceiling=WIRE_LIMIT,
        wheel_mix_motor_rpm_ceiling=2000,turn_wire_cap=TURN_LIMIT,
        heading_kp=4.,heading_kd=1.,heading_deadband_rad=.004,
        positive_wire_wz='right; decreasing ROS yaw; physically observed',
        speed_ceiling_m_s=1.10,acceleration_assumption_m_s2=.55,deceleration_assumption_m_s2=.60,
        forward_wire_speed_estimate=.0885,other_direction_wire_speed_assumption=.095,
        field_validated=False,combined_translation_rotation_validated=False)

def command(*,direction,raw_amplitude,ramp,task_yaw_rad,cross_m,
            travelled,remaining,speed,yaw_rate,previous_wz,dt,heading_error_rad=None):
    finite(raw_amplitude,ramp,task_yaw_rad,cross_m,travelled,remaining,speed,yaw_rate,previous_wz,dt)
    if direction not in DIRECTIONS or not 0<=ramp<=1 or min(travelled,remaining,speed)<0 or not 0<dt<=.1:
        raise ValueError('Invalid profile input')
    if not 0<=raw_amplitude<=2 or abs(previous_wz)>TURN_LIMIT:
        raise ValueError('Invalid seed/previous turn')
    # Reserve distance for the allowed source age before constructing braking cap.
    room=max(0.,remaining-.10-speed*.18)
    target=min(1.10,math.sqrt(.18**2+2*.55*travelled),math.sqrt(.18**2+2*.60*room))
    if remaining<=.015:target=0.
    scale=.0885 if direction=='forward' else .095
    amplitude=target/scale*ramp
    uf,ul=DIRECTIONS[direction]
    correction=max(-.20,min(.20,-8.*math.copysign(max(abs(cross_m)-.003,0.),cross_m)))
    tf,tl=amplitude*uf-correction*ul*ramp,amplitude*ul+correction*uf*ramp
    co,si=math.cos(task_yaw_rad),math.sin(task_yaw_rad)
    vx,vy=-si*tf+co*tl,-(co*tf+si*tl)
    heading_error=task_yaw_rad if heading_error_rad is None else heading_error_rad
    finite(heading_error)
    error=math.copysign(max(abs(heading_error)-.004,0.),heading_error)
    desired=max(-TURN_LIMIT,min(TURN_LIMIT,4.*error+yaw_rate))
    wz=max(previous_wz-1.*dt,min(previous_wz+1.*dt,desired))
    if target==0:wz=0.;vx=vy=0.
    # Sum is a conservative bound on every mecanum wheel's x/y/z mix.
    available=WIRE_LIMIT-abs(wz)
    ratio=min(1.,available/(abs(vx)+abs(vy))) if vx or vy else 1.
    vx,vy,wz=map(quantize,(vx*ratio,vy*ratio,wz))
    return dict(wire_vx=vx,wire_vy=vy,wire_wz=wz,target_speed_m_s=target,
        task_yaw_rad=task_yaw_rad,heading_error_rad=heading_error,cross_m=cross_m,remaining_m=remaining,
        wire_l1_after_scale=abs(vx)+abs(vy)+abs(wz),mixed_axis_physically_validated=False)

def nav_frame(boot,sequence,vx,vy,wz,enabled,crc16):
    finite(vx,vy,wz)
    if type(boot) is not int or not 0<boot<=0xffffffff or type(sequence) is not int or not 0<=sequence<=65535:
        raise ValueError('Invalid boot/sequence')
    if type(enabled) is not bool or (not enabled and (vx or vy or wz)):
        raise ValueError('Disabled frame must be zero')
    vx,vy,wz=map(quantize,(vx,vy,wz))
    if abs(wz)>TURN_LIMIT or abs(vx)+abs(vy)+abs(wz)>WIRE_LIMIT:
        raise ValueError('Wheel mix or turn limit exceeded')
    payload=struct.pack('<IIHBBfff',0,0,100,int(enabled),0,vx,vy,wz)
    prefix=struct.pack('<2sBBHHI',b'\xa6\x6a',2,0x11,24,sequence,boot)+payload
    return prefix+struct.pack('<H',crc16(prefix))
