"""Production DebugLink against a private pseudoterminal; no physical devices."""
import dataclasses, math, os, pty, select, struct, subprocess, sys, time
from tools.rmv.protocol import Frame, StreamDecoder, encode, LAYOUTS, JOINT_HEADER, JOINT_ENTRY
from tools.rmv.simulator import Board

class BigYawBoard(Board):
    """Private extension fixture; two simulated moving axes are big Y and P.

    Wire axis 0 is a separate held small Y, never driven by the moving axes.
    The real MCU core separately tests capability/mask/policy and freshness.
    """
    CAPS=11291
    def __init__(self, kind='follow'):
        super().__init__()
        self.kind=kind
        self.origin=self.now
        self.axis_mask=0
        self.shot_events=[]
        self.request_count=0
        self.rpm_profile=False
        self.profile_requests=[]
        self.last_wheel_emit=0
        self.parameters[0x1100].current=-.72
        self.parameters[0x1101].current=.45
    def _dispatch(self, req):
        if req.msg_type==4 and self.kind.startswith('fire_batch_rpm'):
            entries=[struct.unpack_from('<HHI',req.payload,4+i*8) for i in range(req.payload[0])]
            enabled=entries==[(0x101,20,0),(0x102,25,1),(0x103,25,0),(0x104,0,0),(0x109,200,0)]
            assert enabled or entries==[(0x101,20,0),(0x102,100,1),(0x103,100,0),(0x104,20,0),(0x109,0,0)]
            self.profile_requests.append(enabled)
            if self.kind=='fire_batch_rpm_reject':return self._ack(req,7)
            if enabled:assert 62*20+98*25+142*25+106*200+70+54==28564
            self.rpm_profile=enabled
            for msg,hz,source in entries:
                if msg in (0x101,0x102,0x103):self.subscriptions[(msg,source)]=hz
            return self._ack(req,0)
        if req.msg_type==0x203 and self.kind.startswith('fire'):
            values=struct.unpack(LAYOUTS['SHOOT_REQUEST'][1],req.payload)
            if not self.physical_permit:return self._ack(req,7)
            if values[4]!=self.last_accepted:return self._ack(req,11)
            assert values[1]==1 and values[4]==self.last_accepted
            assert self.now<values[5]<=self.now+100000
            if self.kind.startswith('fire_batch'):
                self.request_count+=1
                assert self.request_count<=20
                assert not self.shot_events, 'overlapping feeder requests'
                request_id=values[0]
                def event(code,reason=0):
                    return struct.pack(LAYOUTS['SHOT_EVENT'][1],request_id,code,1 if code==2 else 0,reason,self.now,values[4])
                if self.kind=='fire_batch_reject':
                    self.shot_events.append((self.now,event(4,7)))
                    return self._ack(req,7)
                if self.kind!='fire_batch_lost':
                    if self.kind=='fire_batch_rpm':
                        self.shot_events.append((self.now,event(0)))
                    self.shot_events.extend([(self.now+5000,event(1)),(self.now+30000,event(2))])
            return self._ack(req,0)
        if req.msg_type==0x200:
            mode,axes,reserved,epoch=struct.unpack('<BBHI',req.payload)
            if mode==3 or (mode==2 and self.kind.startswith('fire')):
                assert axes==6,'must select big Y + P'
                req=dataclasses.replace(req,payload=struct.pack('<BBHI',3 if self.kind.startswith('fire') else mode,3,reserved,epoch))
                self.axis_mask=6
        return super()._dispatch(req)
    def _aim(self, req):
        v=list(struct.unpack(LAYOUTS['AIM_SETPOINT'][1],req.payload))
        assert v[3]==2 and v[4]==2 and self.axis_mask==6
        if v[5]&6:
            assert self.kind in ('trajectory','trajectory_current') and v[5]==7 and self.CAPS&(1<<15)
            assert abs(v[8])<=2 and abs(v[9])<=1.5 and max(map(abs,v[10:]))<=100
            v[5]=1;v[8:]=[0.,0.,0.,0.]  # Shared motion fixture has no trajectory extension.
        v[4]=0  # Delegate the shared expiry/range/motion model only.
        return super()._aim(dataclasses.replace(req,payload=struct.pack(LAYOUTS['AIM_SETPOINT'][1],*v)))
    def telemetry(self, force=False):
        out=[]
        for f in StreamDecoder().feed(super().telemetry(force)):
            if f.msg_type==0x101 and self.kind.startswith('fire'):
                p=bytearray(f.payload)
                if p[0]==3:p[0]=2
                window=self.now-self.origin>.8e6 if self.kind.startswith('fire_batch') else .8e6<self.now-self.origin<1.6e6
                p[7]=int(self.kind not in ('fire_no_rc','fire_batch_no_rc') and window and self.physical_permit)
                f=dataclasses.replace(f,payload=bytes(p))
            if f.msg_type==0x103:
                entries=f.payload[16:]
                big=bytearray(entries[:32]);big[0]=2
                if self.kind=='fire_batch_quantized_velocity':
                    struct.pack_into('<f',big,8,-.03296661376953125)
                if self.kind=='invalid_joint' and self.now-self.origin>1400000:big[1]&=~8
                bits=0x3D|(2 if self.mode in (2,3) else 0)
                held=JOINT_ENTRY.pack(0,bits,0,-.2,0.,-.2,0.,0.,0.,0)
                f=dataclasses.replace(f,payload=JOINT_HEADER.pack(3,0,1,self.now-5000,self.epoch)+held+entries[32:]+big)
            elif f.msg_type==0x102:
                if self.kind=='pose_stall' and self.now-self.origin>1400000:continue
                # IMU/joint samples precede packet transmission, as on hardware.
                lag=15000 if self.kind=='lagging_pose' else 5000
                p=bytearray(f.payload);struct.pack_into('<Q',p,8,self.now-lag)
                f=dataclasses.replace(f,payload=bytes(p))
            out.append(encode(f))
        emit_wheel=(not self.kind.startswith('fire_batch_rpm') or
                    self.now-self.last_wheel_emit>=(5000 if self.rpm_profile else 50000))
        if self.kind.startswith('fire') and emit_wheel:
            self.last_wheel_emit=self.now
            stamp=self.now-200000 if self.kind=='fire_stale_state' else self.now
            rpm=float('nan') if self.kind=='fire_invalid_rpm' else 6500. if self.kind.startswith('fire_batch') else 2500.
            if self.kind=='fire_batch_quantized_velocity':rpm=6400.
            right=-rpm if self.kind.startswith('fire_batch') else -2500.
            state=struct.pack(LAYOUTS['SHOOT_STATE'][1],max(0,stamp),7,0,rpm,right,0.,0.,0xffffffff,0.,0.,0,0,0)
            if self.rpm_profile:
                state=struct.pack('<QIIfffiiifiiIIIBBHII',self.now,self.request_count,7,
                    rpm,right,0.,-1234,-43364,-42130,-40.,-900,-890,1000,1000,1000,3,2,0,4,0)
                assert len(state)==76
                out.append(encode(self._frame(0x109,state)))
            else:out.append(encode(self._frame(0x104,state)))
        for stamp,payload in self.shot_events:
            if stamp<=self.now:out.append(encode(self._frame(0x105,payload)))
        self.shot_events=[x for x in self.shot_events if x[0]>self.now]
        return b''.join(out)

def run(binary,kind,limit):
    master,slave=pty.openpty(); path=os.ttyname(slave)
    board=BigYawBoard(kind); board.physical_permit=False
    if kind.startswith('fire'):board.CAPS|=128
    if kind in ('trajectory','trajectory_current'):board.CAPS|=1<<15
    decoder=StreamDecoder(); seen=[]; valid_goals=[]; joint_goals=[]
    start=time.monotonic(); mcu_origin=board.now; ready_once=False
    mode=kind if kind.startswith(('fire','trajectory')) else 'readonly' if kind=='readonly' else ('negative' if kind=='negative' else 'follow')
    child=subprocess.Popen([binary,'--pty-offline',path,mode,str(limit)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    def adjust(raw):
        result=[]
        for f in StreamDecoder().feed(raw):
            if f.msg_type==0x8001:
                v=list(struct.unpack(LAYOUTS['HELLO_REPLY'][1],f.payload));v[2]=2026100605 if kind.startswith('fire') and kind!='fire_old' else 2026100603 if kind=='previous_firmware' else 2026100604
                if kind=='trajectory':v[2]=2026100701
                if kind.startswith('fire_batch'):v[2]=2026100702 if kind=='fire_batch_old' else 2026100703
                if kind.startswith('fire_batch_rpm') or kind=='fire_batch_current':v[2]=2026100708
                if kind=='fire_batch_rpm_old':v[2]=2026100703
                if kind=='trajectory_current':v[2]=2026100708
                f=dataclasses.replace(f,payload=struct.pack(LAYOUTS['HELLO_REPLY'][1],*v))
            result.append(encode(f))
        return b''.join(result)
    try:
        while child.poll() is None:
            now=time.monotonic(); elapsed=now-start
            assert elapsed<7,'peer hung'
            if elapsed>.3 and not ready_once:
                board.physical_permit=True;board.mode=0;ready_once=True
            if kind=='epoch' and elapsed>1.4:board.epoch=2
            if kind in ('revoke','fire_revoke','fire_batch_revoke') and elapsed>1.4:board.physical_permit=False
            board.advance(max(0,mcu_origin+int((now-start)*1e6)-board.now))
            output=board.telemetry()
            if select.select([master],[],[],.002)[0]:
                raw=os.read(master,4096)
                for f in decoder.feed(raw):
                    seen.append((elapsed,f.msg_type))
                    if f.msg_type==0x201:
                        v=struct.unpack(LAYOUTS['AIM_SETPOINT'][1],f.payload)
                        assert v[3]==2 and v[4]==2
                        if kind in ('trajectory','trajectory_current'):
                            assert v[5] in (0,7)
                            if v[5]:assert any(v[8:])
                        else:
                            assert not(v[5]&~1)
                            assert not any(v[8:]),'legacy feedforward must stay disabled'
                        if v[5]&1:
                            if limit:assert max(abs(v[6]),abs(v[7]))<=math.radians(limit)+1e-6,'anchor drift escaped configured limit'
                            assert -.72-1e-6<=v[7]<=.45+1e-6,'pitch hard stop escaped'
                            assert abs(v[6])<=math.pi+1e-6,'big yaw must be a circular angle'
                            joint_goals.append((v[6],v[7]))
                            valid_goals.append(elapsed)
                # Stamp TIME_REPLY at receipt, rather than before select waited.
                board.advance(max(0,mcu_origin+int((time.monotonic()-start)*1e6)-board.now))
                output+=adjust(board.receive(raw))
            if output:os.write(master,output)
        out,err=child.communicate(timeout=2)
        # Child exit may precede reading the destructor's final STOP from the
        # PTY. Drain remaining bytes before asserting its transmitted commands.
        while select.select([master],[],[],.05)[0]:
            for f in decoder.feed(os.read(master,4096)):
                seen.append((time.monotonic()-start,f.msg_type))
        if kind.startswith('fire'):
            requests=[when for when,t in seen if t==0x203]
            if kind.startswith('fire_batch'):
                if kind=='fire_batch_rpm_old':
                    assert child.returncode==1 and 'requires firmware 2026100704' in out,(out,err)
                    assert not any(t in (0x200,0x201,0x203) for _,t in seen)
                elif kind=='fire_batch_rpm_reject':
                    assert child.returncode==1 and 'subscription rejected' in out,(out,err)
                    assert not any(t in (0x200,0x201,0x203) for _,t in seen),seen
                elif kind=='fire_batch_old':
                    assert child.returncode==1 and 'Shot test requires' in out,(out,err)
                    assert not requests
                else:
                    assert child.returncode==0,(out,err)
                    if kind in ('fire_batch','fire_batch_current','fire_batch_quantized_velocity','fire_batch_rpm'):
                        assert len(requests)==20 and 'BATCH_DONE completed=20 shots=20' in out,(out,err,requests)
                        if kind=='fire_batch_rpm':
                            assert board.profile_requests==[True,False] and not board.rpm_profile
                            assert 'RPM_RESTORED' in out,(out,err)
                    elif kind in ('fire_batch_lost','fire_batch_reject'):assert len(requests)==1,(out,err,requests)
                    elif kind=='fire_batch_no_rc':assert not requests
                    elif kind=='fire_batch_revoke':assert requests and all(t<1.52 for t in requests),requests
                    assert len(requests)<=20
                    assert any(t==0x202 for _,t in seen),'batch exit must STOP'
                print('PASS',kind,'requests=',len(requests),out.strip())
                return
            if kind=='fire_old':
                assert child.returncode==1 and 'Fire requires' in out,(out,err)
                assert not any(t in (0x200,0x201,0x203) for _,t in seen),seen
            else:
                assert child.returncode==0 and valid_goals,(out,err)
                if kind in ('fire_no_rc','fire_stale_state','fire_invalid_rpm'):assert not requests,seen
                else:
                    assert requests,(out,err)
                    assert all(.79<when<1.72 for when in requests),requests
                    if kind=='fire_revoke':assert all(when<1.52 for when in requests),requests
                assert any(t==0x202 for _,t in seen),'exit did not stop joint VISION'
            print('PASS',kind,'requests=',len(requests),out.strip())
            return
        assert not any(t in (0x203,0x204,0x205,0x302,0x303,0x304) for _,t in seen),seen
        if kind=='readonly':
            assert child.returncode==0,(out,err)
            assert all(t in (1,2,3) for _,t in seen),seen
            assert not valid_goals
        elif kind=='trajectory_old':
            assert child.returncode==1 and 'Trajectory control requires' in out,(out,err)
            assert not any(t in (0x200,0x201,0x203) for _,t in seen),seen
        elif kind=='epoch':
            assert child.returncode==1 and 'reference changed' in out,(out,err)
            assert valid_goals
        else:
            assert child.returncode==0,(out,err)
            assert valid_goals and 'ready=1' in out,(out,err)
            assert not any(t in (0x200,0x201) for when,t in seen if when<.3),'manual RC allowed output'
            if kind=='revoke':assert not any(t==0x201 for when,t in seen if when>1.52),'RC revoke failed'
            if kind in ('invalid_joint','pose_stall'):
                assert not any(t==0x201 for when,t in seen if when>1.56),'invalid/stale feedback allowed output'
            if kind=='lagging_pose':
                assert len(valid_goals)>120,'packet phase still starves target transmission'
                assert 'used_prior_pair=1' in out,'phase fixture did not exercise retained pairs'
            assert any(t==0x202 for _,t in seen),'exit did not stop remote control'
        if kind not in ('readonly','trajectory_old'):
            assert joint_goals
            magnitude=math.radians(limit) if limit else 1.
            expected=-magnitude if kind=='negative' else magnitude
            if kind in ('trajectory','trajectory_current'):assert abs(joint_goals[0][0]-.2)<1e-6 and abs(joint_goals[0][1]-.1)<1e-6
            else:
                assert abs(joint_goals[0][0]-expected)<1e-6
                assert abs(joint_goals[0][1]-max(-.72,min(expected,.45)))<1e-6
        print('PASS',kind,limit,'degrees;',len(valid_goals),'valid goals;',out.strip())
    finally:
        if child.poll() is None:child.kill();child.wait()
        os.close(master);os.close(slave)

if __name__=='__main__':
    for limit in (0,3,30):
        for kind in ('readonly','follow','negative','revoke','epoch'):run(sys.argv[1],kind,limit)
    for kind in ('lagging_pose','invalid_joint','pose_stall','previous_firmware'):run(sys.argv[1],kind,0)
    for kind in ('fire','fire_no_rc','fire_stale_state','fire_invalid_rpm','fire_revoke','fire_old'):run(sys.argv[1],kind,0)
    for kind in ('trajectory','trajectory_old','trajectory_current'):run(sys.argv[1],kind,0)
    for kind in ('fire_batch','fire_batch_current','fire_batch_quantized_velocity','fire_batch_reject','fire_batch_lost','fire_batch_no_rc','fire_batch_revoke','fire_batch_old','fire_batch_rpm','fire_batch_rpm_reject','fire_batch_rpm_old'):run(sys.argv[1],kind,0)
