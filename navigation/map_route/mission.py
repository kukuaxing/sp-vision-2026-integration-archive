"""Whole-course holonomic path following and task/home state machine. No IO."""
from collections import deque
import math
from core import clamp, wrap


def length(points):
    return sum(math.dist(a,b) for a,b in zip(points,points[1:]))


def simplify(points, tolerance=.025):
    """Ramer-Douglas-Peucker; bounded error to the original taught polyline."""
    if len(points)<=2:return [list(p) for p in points]
    a,b=points[0],points[-1];dx,dy=b[0]-a[0],b[1]-a[1];den=dx*dx+dy*dy
    distances=[]
    for p in points[1:-1]:
        f=clamp(((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den,0.,1.) if den else 0.
        distances.append(math.hypot(p[0]-a[0]-f*dx,p[1]-a[1]-f*dy))
    error=max(distances,default=0.)
    if error<=tolerance:return [list(a),list(b)]
    i=distances.index(error)+1
    return simplify(points[:i+1],tolerance)[:-1]+simplify(points[i:],tolerance)


class Path:
    def __init__(self, points):
        self.points=[tuple(p) for p in points]
        if not 2<=len(points)<=500 or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError('Invalid global path')
        self.distances=[0.]
        for a,b in zip(self.points,self.points[1:]):
            d=math.dist(a,b)
            if not .001<=d<=8.:raise ValueError('Degenerate or oversized global path segment')
            self.distances.append(self.distances[-1]+d)
        self.total=self.distances[-1]

    def point(self, progress):
        progress=clamp(progress,0.,self.total)
        for i in range(len(self.points)-1):
            if self.distances[i+1]>=progress:
                a,b=self.points[i:i+2];f=(progress-self.distances[i])/(self.distances[i+1]-self.distances[i])
                return (a[0]+(b[0]-a[0])*f,a[1]+(b[1]-a[1])*f)
        return self.points[-1]

    def project(self, xy, previous):
        # Bounded arc window prevents jumping to an adjacent corridor or later leg.
        options=[]
        for i,(a,b) in enumerate(zip(self.points,self.points[1:])):
            lo=max(self.distances[i],previous-.12);hi=min(self.distances[i+1],previous+.50)
            if hi<lo:continue
            d=self.distances[i+1]-self.distances[i];u=((b[0]-a[0])/d,(b[1]-a[1])/d)
            s=clamp(self.distances[i]+(xy[0]-a[0])*u[0]+(xy[1]-a[1])*u[1],lo,hi)
            p=(a[0]+u[0]*(s-self.distances[i]),a[1]+u[1]*(s-self.distances[i]))
            options.append((math.dist(xy,p),s,i,p,u))
        if not options:raise RuntimeError('No local path projection')
        return min(options,key=lambda v:v[0])


class Follower:
    def __init__(self,points,max_speed=.45,*,motor_rpm_bound=2000.,acceleration=.45,deceleration=.65,jerk_limit=6.,corner_speed=.18,corner_brake_margin=.10):
        self.path=Path(points);self.progress=0.;self.vx=self.vy=self.wz=0.
        if not .16<=max_speed<=1.80:raise ValueError('Unreviewed mission speed')
        self.max_speed=max_speed
        self.motor_rpm_bound=motor_rpm_bound
        if not all(math.isfinite(v) and v>0 for v in (acceleration,deceleration,jerk_limit)):
            raise ValueError('Invalid motion profile')
        if not math.isfinite(motor_rpm_bound) or not 0<motor_rpm_bound<=6000.:
            raise ValueError('Invalid motor rpm bound')
        self.acceleration=acceleration;self.deceleration=deceleration
        self.jerk_limit=jerk_limit;self.ax=self.ay=0.
        self.terminal_settling=False
        if not math.isfinite(corner_speed) or not .05<=corner_speed<=.45:
            raise ValueError('Invalid corner speed')
        self.corner_speed=corner_speed
        if not math.isfinite(corner_brake_margin) or not .04<=corner_brake_margin<=.20:
            raise ValueError('Invalid corner braking margin')
        self.corner_brake_margin=corner_brake_margin

    def _smooth_velocity(self,desired,dt):
        # A velocity error servo with bounded vector acceleration and jerk.
        # The response time starts easing acceleration before reaching cruise.
        error=(desired[0]-self.vx,desired[1]-self.vy)
        slowing=error[0]*self.vx+error[1]*self.vy<0.
        bound=self.deceleration if slowing else self.acceleration
        wanted=(error[0]/.15,error[1]/.15)
        norm=math.hypot(*wanted)
        if norm>bound:wanted=tuple(v*bound/norm for v in wanted)
        delta=(wanted[0]-self.ax,wanted[1]-self.ay);norm=math.hypot(*delta)
        limit=self.jerk_limit*dt
        if norm>limit:delta=tuple(v*limit/norm for v in delta)
        self.ax+=delta[0];self.ay+=delta[1]
        old=(self.vx,self.vy)
        self.vx+=self.ax*dt;self.vy+=self.ay*dt
        speed=math.hypot(self.vx,self.vy)
        if speed>self.max_speed:
            # The speed envelope takes priority over smoothing during saturation.
            self.vx*=self.max_speed/speed;self.vy*=self.max_speed/speed
            self.ax=(self.vx-old[0])/dt;self.ay=(self.vy-old[1])/dt

    def step(self,s,dt,source_age,stamp_ns=None):
        if not 0<dt<=.1 or not -.05<=source_age<=.30:raise ValueError('Invalid timing')
        _,arc,index,nearest,u=self.path.project((s.x,s.y),self.progress)
        dx,dy=s.x-nearest[0],s.y-nearest[1]
        error=abs(u[0]*dy-u[1]*dx)
        self.progress=max(self.progress,arc)
        remaining=self.path.total-self.progress
        end_distance=math.dist((s.x,s.y),self.path.points[-1])
        c,sn=math.cos(s.yaw),math.sin(s.yaw)
        measured=(c*s.forward_speed-sn*s.left_speed,sn*s.forward_speed+c*s.left_speed)
        along_speed=measured[0]*u[0]+measured[1]*u[1]
        room=max(0.,remaining-.025-max(0.,along_speed)*(max(0.,source_age)+.12))
        target_speed=min(self.max_speed,max(.16,math.sqrt(2*self.acceleration*room)))
        # Slow before real bends. Small noisy RDP vertices alone do not stop the car.
        bend_preview=(self.max_speed**2-self.corner_speed**2)/(2*self.acceleration)+self.corner_brake_margin+max(0.,along_speed)*(max(0.,source_age)+.12)
        for j in range(max(1,index),len(self.path.points)-1):
            ahead=self.path.distances[j]-self.progress
            if ahead<-.30:continue
            if ahead>bend_preview:break
            a,b,d=self.path.points[j-1:j+2]
            u1=(b[0]-a[0],b[1]-a[1]);u2=(d[0]-b[0],d[1]-b[1])
            cosine=(u1[0]*u2[0]+u1[1]*u2[1])/(math.hypot(*u1)*math.hypot(*u2))
            angle=math.acos(clamp(cosine,-1.,1.))
            if angle>math.radians(35):
                bend_room=(max(0.,ahead-self.corner_brake_margin-max(0.,along_speed)*(max(0.,source_age)+.12))
                           if ahead>=0 else max(0.,-ahead))
                target_speed=min(target_speed,math.sqrt(self.corner_speed**2+2*self.acceleration*bend_room))
        # Blend the direction over the next 8 cm of the reviewed polyline.
        # Starting at the current arc anticipates actuator response at a bend.
        before=self.path.point(self.progress)
        after=self.path.point(self.progress+.08)
        direction=(after[0]-before[0],after[1]-before[1]);norm=math.hypot(*direction)
        if norm<1e-6:direction=u
        else:direction=(direction[0]/norm,direction[1]/norm)
        desired=(direction[0]*target_speed+2.*(nearest[0]-s.x),
                 direction[1]*target_speed+2.*(nearest[1]-s.y))
        # Blend into metric endpoint feedback, eliminating the 0.16 m/s
        # forward crawl and the following hard stop at the arrival radius.
        terminal=clamp((.50-remaining)/.30,0.,1.)
        terminal=terminal*terminal*(3.-2.*terminal)
        goal=self.path.points[-1]
        lead=min(.20,max(0.,source_age)+.05)
        endpoint=(3.*(goal[0]-s.x-measured[0]*lead),3.*(goal[1]-s.y-measured[1]*lead))
        # Once slowly inside the endpoint, finish braking despite centimetre
        # localization noise. Re-enable feedback after a clear displacement.
        if end_distance>.035:self.terminal_settling=False
        predicted_endpoint_error=math.hypot(*endpoint)/3.
        if (remaining<.06 and end_distance<=.035 and predicted_endpoint_error<=.035 and
            math.hypot(self.vx,self.vy)<=.12 and math.hypot(s.forward_speed,s.left_speed)<=.12):
            self.terminal_settling=True
        if self.terminal_settling:endpoint=(0.,0.)
        desired=tuple((1.-terminal)*v+terminal*g for v,g in zip(desired,endpoint))
        speed_cap=self.max_speed
        if remaining<.80:
            # Closed-form braking envelope includes source age, the velocity
            # servo response allowance, and a jerk ramp allowance. This prevents
            # accelerating again too late in a short final segment.
            reaction=max(0.,source_age)+.15+.5*self.deceleration/self.jerk_limit
            at=self.deceleration*reaction
            room=max(0.,end_distance-.02)
            speed_cap=min(speed_cap,max(.12,math.sqrt(at*at+2.*self.deceleration*room)-at))
        size=math.hypot(*desired)
        if size>speed_cap:desired=tuple(v*speed_cap/size for v in desired)
        previous_velocity=(self.vx,self.vy);previous_acceleration=(self.ax,self.ay)
        self._smooth_velocity(desired,dt)
        # A final exact zero must respect this tick's acceleration/jerk and
        # the following zero-command tick, rather than wait for a tiny speed.
        zero_acceleration=tuple(-v/dt for v in previous_velocity)
        zero_safe=(math.hypot(*previous_velocity)<=min(.003,self.jerk_limit*dt*dt) and
                   math.dist(zero_acceleration,previous_acceleration)<=self.jerk_limit*dt and
                   math.hypot(*zero_acceleration)<=self.deceleration)
        reached=(remaining<.06 and end_distance<=.035 and predicted_endpoint_error<=.035 and
                 self.terminal_settling and zero_safe and
                 math.hypot(s.forward_speed,s.left_speed)<=.06)
        f=c*self.vx+sn*self.vy;l=-sn*self.vx+c*self.vy
        raw_x=l/.095;raw_y=-f/(.0885 if f>=0 else .095)
        want=clamp(4.*wrap(s.yaw)+s.yaw_rate,-.30,.30)
        self.wz=clamp(want,self.wz-dt,self.wz+dt)
        scale=min(1.,(self.motor_rpm_bound/159-abs(self.wz))/(abs(raw_x)+abs(raw_y))) if raw_x or raw_y else 1.
        if reached:raw_x=raw_y=self.wz=self.vx=self.vy=self.ax=self.ay=0.
        return dict(wire_vx=raw_x*scale,wire_vy=raw_y*scale,wire_wz=self.wz,reached=reached,
                    progress_m=self.progress,remaining_path_m=remaining,endpoint_error_m=end_distance,
                    lateral_error_m=error,path_segment=index,planned_speed_m_s=math.hypot(self.vx,self.vy),
                    planned_acceleration_m_s2=math.hypot(self.ax,self.ay),terminal_blend=terminal,
                    planned_world_vx_m_s=self.vx*scale,planned_world_vy_m_s=self.vy*scale,
                    endpoint_speed_cap_m_s=speed_cap,endpoint_predicted_error_m=predicted_endpoint_error,
                    terminal_settling=self.terminal_settling)


def stopped(window,*,fast=False):
    if fast:
        if not window:return False
        window=[row for row in window if row[0]>=window[-1][0]-.50]
    if len(window)<(4 if fast else 8) or window[-1][0]-window[0][0]<(0.35 if fast else .9):return False
    states=[v[1] for v in window];speed=[math.hypot(s.forward_speed,s.left_speed) for s in states]
    return (max(speed)<=(.06 if fast else .03) and sum(speed)/len(speed)<=(.03 if fast else .015)
        and max(math.dist((a.x,a.y),(b.x,b.y)) for a in states for b in states)<=(.03 if fast else .012)
        and max(abs(wrap(a.yaw-b.yaw)) for a in states for b in states)<=(.035 if fast else .02)
        and max(abs(s.yaw_rate) for s in states)<=(.08 if fast else .04))


class Mission:
    def __init__(self,plan,*,observe_only=False,external_task=False):
        out=plan['outbound_waypoints_m'];back=plan['return_waypoints_m']
        if math.dist(out[0],(0.,0.))>1e-8 or math.dist(back[-1],out[0])>1e-8 or math.dist(back[0],out[-1])>1e-8:
            raise ValueError('Outbound/return endpoints do not form a P00 round trip')
        if back!=list(reversed(out)):raise ValueError('Return must reverse the reviewed outbound path')
        if 'task_goal_m' in plan and math.dist(out[-1],plan['task_goal_m'])>1e-9:
            raise ValueError('Task label and path endpoint differ')
        self.points=out;self.return_points=back;self.max_speed=plan['speed_ceiling_m_s']
        self.dwell=plan['task_dwell_s']
        if not 0<=self.dwell<=60:raise ValueError('Invalid task dwell')
        self.follower_options=dict(motor_rpm_bound=plan.get('motor_rpm_ceiling',2000.),
                                   acceleration=plan.get('acceleration_m_s2',.45),
                                   deceleration=plan.get('deceleration_m_s2',.65),
                                   jerk_limit=plan.get('jerk_limit_m_s3',6.),
                                   corner_speed=plan.get('corner_speed_m_s',.18),
                                   corner_brake_margin=plan.get('corner_brake_margin_m',.10))
        self.follower=Follower(out,self.max_speed,**self.follower_options)
        self.phase='OUTBOUND';self.phase_since=None;self.window=deque();self.last_stamp=None
        self.task_reached=False;self.home_reached=False;self.events=[]
        self.observe_only=observe_only
        self.external_task=external_task

    def enter(self,phase,now):
        self.phase=phase;self.phase_since=now;self.window.clear();self.last_stamp=None
        self.events.append(dict(phase=phase,monotonic_s=now))

    def step(self,s,dt,source_age,stamp_ns,now,*,task_complete=False):
        if self.phase_since is None:self.phase_since=now
        phase=self.phase;command=dict(wire_vx=0.,wire_vy=0.,wire_wz=0.,enabled=phase not in ('RELEASING','DONE'))
        if phase in ('OUTBOUND','RETURNING'):
            command.update(self.follower.step(s,dt,source_age,stamp_ns))
            if command['reached'] and not self.observe_only:
                self.enter('STOP_TASK' if phase=='OUTBOUND' else 'STOP_HOME',now)
        elif phase!='DONE':
            goal=self.points[-1] if phase in ('STOP_TASK','TASK_DWELL') else self.points[0]
            error=math.dist((s.x,s.y),goal)
            if stamp_ns!=self.last_stamp:self.window.append((now,s));self.last_stamp=stamp_ns
            while len(self.window)>1 and now-self.window[0][0]>1.15:self.window.popleft()
            stable=stopped(self.window,fast=self.external_task)
            if phase=='STOP_TASK' and stable:
                self.task_reached=True;self.enter('TASK_DWELL',now)
            elif phase=='TASK_DWELL':
                if (task_complete if self.external_task else now-self.phase_since>=self.dwell) and stable:
                    self.follower=Follower(self.return_points,self.max_speed,**self.follower_options)
                    self.enter('RETURNING',now)
            elif phase=='STOP_HOME' and stable:
                self.home_reached=True;self.enter('RELEASING',now)
            elif phase=='RELEASING' and now-self.phase_since>=(.35 if self.external_task else 1.2) and stable:
                self.enter('DONE',now)
            command.update(endpoint_error_m=error)
        command.update(phase=self.phase,task_reached=self.task_reached,home_reached=self.home_reached)
        # Disable on the transition into release, not one command later.
        if self.phase in ('RELEASING','DONE'):command['enabled']=False
        return command
