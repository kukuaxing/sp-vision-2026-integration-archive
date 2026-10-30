"""Mission gates and return policy, pure logic with a monotonic clock."""


class StartGate:
    def __init__(self):
        self.armed=False;self.previous_auto=None;self.ready_since=None

    def update(self,now,ready,auto,fresh):
        if not fresh or not ready:
            self.armed=False;self.ready_since=None;self.previous_auto=auto if fresh else None
            return False
        if self.ready_since is None:self.ready_since=now
        if not auto and now-self.ready_since>=1.0:self.armed=True
        start=self.armed and self.previous_auto is False and auto
        self.previous_auto=auto
        if start:self.armed=False
        return start


class ReturnPolicy:
    def __init__(self,baseline,generation,boot,*,minimum=30,maximum=50,lost_s=5.):
        if type(minimum) is not int or type(maximum) is not int or not 0<=minimum<maximum<=1000 or not 0<lost_s<=60:
            raise ValueError('Invalid competition return policy')
        if type(baseline) is not int or baseline<0 or baseline+maximum>0xffffffff:
            raise ValueError('Invalid step counter baseline')
        self.baseline=baseline;self.generation=generation;self.boot=boot
        self.minimum=minimum;self.maximum=maximum;self.lost_s=lost_s
        self.last_count=baseline;self.threshold_since=None;self.reason=None;self.task_since=None

    def update(self,now,status,*,task=False):
        if status['mcu_boot']!=self.boot or status['count_generation']!=self.generation:
            raise RuntimeError('MCU/host counter generation changed')
        count=status['fire_requests_sent']
        if type(count) is not int or count<self.last_count:
            raise RuntimeError('Host request counter reversed')
        self.last_count=count;used=count-self.baseline
        if task and self.task_since is None:self.task_since=now
        if used>=self.maximum:self.reason='sent_50_requests'
        elif task and used>self.minimum:
            if self.threshold_since is None:self.threshold_since=now
            last=max(self.threshold_since,self.task_since,status['armor_last_ns']*1e-9)
            if now-last>self.lost_s:self.reason='armor_lost_over_5s_after_30_requests'
        return used,self.reason


def vision_health(v,now_ns):
    """Both transport and source times must progress; stale images are faults, not armor loss."""
    if not v or v.get('schema')!=2 or v.get('count_source')!='host_tx':
        raise RuntimeError('Waiting for host-request-count vision status')
    for name,timeout in [('monotonic_ns',200000000),('frame_ns',300000000)]:
        stamp=v.get(name,0)
        if type(stamp) is not int or stamp<=0 or not 0<=now_ns-stamp<=timeout:
            raise RuntimeError('Stale/future '+name)
    if not v.get('link_fresh') or not v.get('pose_fresh'):
        raise RuntimeError('MCU link/pose unavailable')
    count=v.get('fire_requests_sent')
    if type(count) is not int or not 0<=count<=0xffffffff or v.get('count_generation')!=0:
        raise RuntimeError('Invalid host request counter')
    if v.get('mcu_reject')!=0:raise RuntimeError('MCU command rejected')
    return bool(v['mcu_state']&0x80)
