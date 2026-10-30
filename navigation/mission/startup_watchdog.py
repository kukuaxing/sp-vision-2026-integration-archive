"""Pure startup/session decisions; no ROS, serial or camera operations."""
import math

IDLE_PHASES={'WAIT_READY','READY_WAIT_BOTH_UP','STOPPED'}


def interrupted_route(marker,boot_id):
    return marker.get('boot_id')==boot_id and marker.get('active') is True


class SessionWatchdog:
    def __init__(self,started_s,boot_id,*,grace_s=20.,loss_s=5.):
        if not all(math.isfinite(v) for v in (started_s,grace_s,loss_s)) or min(grace_s,loss_s)<=0:
            raise ValueError('Invalid session watchdog timeouts')
        self.started_s=started_s;self.boot_id=boot_id
        self.grace_s=grace_s;self.loss_s=loss_s
        self.activity_seen=False;self.unhealthy_since=None

    def inspect(self,now,vision,manager,command,*,children_ok=True,devices_ok=True):
        current=(manager.get('boot_id')==self.boot_id and
                 self.started_s<=manager.get('monotonic_s',-1)<=now)
        phase=manager.get('phase') if current else None
        cmd_current=self.started_s*1e9<=command.get('monotonic_ns',-1)<=now*1e9
        cmd_owns=bool(vision.get('vision_instance')) and command.get('vision_instance')==vision.get('vision_instance')
        if (phase and phase not in IDLE_PHASES) or (cmd_current and cmd_owns and command.get('nav_enabled')):
            self.activity_seen=True
        if phase=='FAULT':return 'latch','Mission fault: '+str(manager.get('error'))
        if not children_ok:return self.failure('Mission/vision child exited')
        if not devices_ok:return self.failure('Camera/serial disappeared or was re-enumerated')
        reasons=[];ns=now*1e9
        for key,limit in [('monotonic_ns',1e9),('frame_ns',1e9)]:
            stamp=vision.get(key,0)
            if not isinstance(stamp,int) or stamp<=0 or not 0<=ns-stamp<=limit:reasons.append('stale '+key)
        if not vision.get('link_fresh'):reasons.append('MCU stream unavailable')
        if not vision.get('pose_fresh'):reasons.append('MCU pose unavailable')
        if reasons:
            if self.unhealthy_since is None:self.unhealthy_since=now
            if now-self.started_s>=self.grace_s and now-self.unhealthy_since>=self.loss_s:
                return self.failure('; '.join(reasons))
            return 'running','Waiting for first/progressing camera and MCU streams'
        self.unhealthy_since=None
        return 'running','Vision streams fresh; route readiness is checked separately'

    def failure(self,reason):return ('latch' if self.activity_seen else 'retry'),reason
