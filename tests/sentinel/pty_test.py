# Private PTYs only. Independent struct/CRC peer for production XucSender.
import errno,math,os,pty,select,struct,subprocess,sys,tempfile,time
from pathlib import Path

def crc16(data):
    c=0xffff
    for b in data:
        c^=b
        for _ in range(8):c=(c>>1)^(0x8408 if c&1 else 0)
    return c

def frame(kind,seq,boot,payload):
    wire=struct.pack('<2sBBHHI',b'\xa6\x6a',2,kind,len(payload),seq&65535,boot)+payload
    return wire+struct.pack('<H',crc16(wire))

def run(binary,control,friction,shoot,require_verified=True):
    master,slave=pty.openpty();os.set_blocking(master,False)
    with tempfile.TemporaryDirectory(prefix='sentinel-v2-pty-') as folder:
        cfg=Path(folder)/'peer.yaml'
        cfg.write_text(f'''board_protocol: SENTINEL
xuc_enable: true
xuc_serial_port: "{os.ttyname(slave)}"
xuc_serial_baud: 921600
xuc_allow_control: {str(control).lower()}
xuc_allow_shoot: {str(shoot).lower()}
xuc_imu_yaw_sign: -1
xuc_imu_pitch_sign: 1
sentinel_allow_friction: {str(friction).lower()}
sentinel_require_verified_pose: {str(require_verified).lower()}
sentinel_swap_roll_pitch: false
sentinel_tx_hz: 50
''')
        process=subprocess.Popen([binary,str(cfg)]+([] if require_verified else ['reuse']),stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        start=time.monotonic();next_tx=.1;index=0;rx=bytearray();out=[];accepted={0x11223344:65530,0x55667788:100}
        references={}
        try:
            while process.poll() is None:
                t=time.monotonic()-start
                if t>8:raise AssertionError('production link hung')
                if t>=next_tx:
                    boot=0x11223344 if t<2.25 else 0x55667788
                    stamp=(0xffffff00+int(t*1e6))&0xffffffff
                    if 1.3<=t<1.65:stamp=1300000
                    flags=0 if .8<=t<1 else (3 if t<.35 or not require_verified else 15)
                    identity=2 if 2<=t<2.25 else 1;epoch=1 if t<2.9 else 2
                    p=struct.pack('<BBHI6f',identity,flags,epoch,stamp,*[math.radians(x) for x in (22,11,179)],1,2,3)
                    g=frame(1,index,boot,p);references[(boot,index&65535)]=(epoch,stamp)
                    c=frame(2,index,boot,struct.pack('<BBHI3f',2,1,1,stamp,0,0,0))
                    st=frame(0x81,index,boot,struct.pack('<IHBBHH',0,accepted[boot],4 if 1.05<=t<1.25 else 0x84,0,0,0))
                    wire=g+c+(b'' if 3.1<=t<3.45 or 2.25<=t<2.30 else st)
                    if index==0:
                        bad=bytearray(g);bad[20]^=1;os.write(master,b'\xff\x00'+bad)
                    os.write(master,wire[:7]);time.sleep(.001);os.write(master,wire[7:])
                    index+=1;next_tx=t+.02
                if select.select([master],[],[],.002)[0]:
                    try:rx.extend(os.read(master,4096))
                    except OSError as e:
                        if e.errno!=errno.EIO:raise
                    while len(rx)>=38:
                        b=bytes(rx[:38]);del rx[:38]
                        magic,ver,kind,size,seq,boot=struct.unpack_from('<2sBBHHI',b)
                        assert (magic,ver,kind,size)==(b'\xa6\x6a',2,0x10,24)
                        assert crc16(b[:-2])==struct.unpack_from('<H',b,36)[0]
                        session,epoch,refseq,stamp,valid,flags,res,dy,dp=struct.unpack_from('<IHHIHBBff',b,12)
                        assert session==res==0
                        accepted[boot]=seq
                        out.append((time.monotonic()-start,flags,seq,boot,dy,dp))
                        if flags:
                            assert flags&3==3 and valid==100
                            assert references[(boot,refseq)]==(epoch,stamp)
                            assert math.isclose(dy,math.radians(2),abs_tol=1e-5)
                            assert math.isclose(dp,math.radians(1.5),abs_tol=1e-5)
                            assert bool(flags&4)==friction
                            assert not flags&8 or (friction and shoot)
                        else:assert dy==dp==0
            output=process.communicate(timeout=3)[0].decode(errors='replace')
            assert process.returncode==0,output
            if not control:assert not out,'receive-only wrote data'
            else:
                moving=[x for x in out if x[1]];assert 30<len(moving)<190
                assert not any((require_verified and t<.34) or .86<t<.98 or 1.12<t<1.23 or 1.45<t<1.63 or 2.06<t<2.23 or 2.48<t<2.63 or 3.24<t<3.43 for t,*_ in moving)
                assert any(.4<t<.7 for t,*_ in moving) and any(3.5<t<3.65 for t,*_ in moving)
                assert any(flags==0 for _,flags,*_ in out)
                assert any(flags&8 for _,flags,*_ in out)==(friction and shoot)
                first_old=next(x for x in out if x[3]==0x11223344);assert first_old[2]==65531
                first_new=next(x for x in out if x[3]==0x55667788);assert first_new[2]==101
            print(f'PASS V2 production PTY control={control} friction={friction} shoot={shoot}: frames={len(out)}; references, units, seq/boot, invalid/frozen IMU, flags, stale status, stop/recovery')
        finally:
            if process.poll() is None:process.kill();process.wait()
            os.close(master);os.close(slave)
def run_startup_cache(binary):
    """Old 01/02/81 group followed by a live stream over half a seq cycle away."""
    master,slave=pty.openpty();os.set_blocking(master,False)
    with tempfile.TemporaryDirectory(prefix='sentinel-startup-cache-') as folder:
        cfg=Path(folder)/'peer.yaml'
        cfg.write_text(f'''board_protocol: SENTINEL
xuc_enable: true
xuc_serial_port: "{os.ttyname(slave)}"
xuc_serial_baud: 921600
xuc_allow_control: true
xuc_allow_shoot: false
xuc_imu_yaw_sign: -1
xuc_imu_pitch_sign: 1
sentinel_allow_friction: false
sentinel_require_verified_pose: false
sentinel_tx_hz: 50
''')
        process=subprocess.Popen([binary,str(cfg),'startup'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        start=time.monotonic();next_tx=.1;index=-1;rx=bytearray();out=[];refs={};boot=2243602295
        try:
            while process.poll() is None:
                t=time.monotonic()-start
                if t>8:raise AssertionError('startup cache peer hung')
                if t>=next_tx:
                    seq=3631 if index<0 else 40527+index
                    stamp=72634445 if index<0 else 810550719+index*20000
                    p=struct.pack('<BBHI6f',1,3,1,stamp,*[math.radians(x) for x in (22,11,179)],1,2,3)
                    g=frame(1,seq,boot,p);refs[seq]=(1,stamp)
                    c=frame(2,seq,boot,struct.pack('<BBHI3f',2,1,1,stamp,0,0,0))
                    # STOP in old bytes prohibits commands before recovery. The
                    # first new status after confirmation must reseed TX at 901.
                    st=frame(0x81,seq,boot,struct.pack('<IHBBHH',0,123 if index<0 else 900,0 if index<0 else 0x80,0,0xffff,0))
                    if index<0:
                        bad=bytearray(g);bad[20]^=1;os.write(master,bad)
                    os.write(master,g+c+st)
                    next_tx=.3 if index<0 else t+.02;index+=1
                if select.select([master],[],[],.002)[0]:
                    try:rx.extend(os.read(master,4096))
                    except OSError as error:
                        if error.errno!=errno.EIO:raise
                    while len(rx)>=38:
                        b=bytes(rx[:38]);del rx[:38]
                        assert b[:4]==b'\xa6\x6a\x02\x10' and crc16(b[:-2])==struct.unpack_from('<H',b,36)[0]
                        seq,wireboot=struct.unpack_from('<HI',b,6);assert wireboot==boot
                        session,epoch,refseq,stamp,valid,flags,res,dy,dp=struct.unpack_from('<IHHIHBBff',b,12)
                        out.append((time.monotonic()-start,seq,flags,refseq))
                        if flags:
                            assert flags==3 and session==res==0 and valid==100
                            assert refseq>=40528 and refs[refseq]==(epoch,stamp)
                            assert math.isclose(dy,math.radians(2),abs_tol=1e-5)
                            assert math.isclose(dp,math.radians(1.5),abs_tol=1e-5)
                        else:assert dy==dp==0
            output=process.communicate(timeout=3)[0].decode(errors='replace')
            assert process.returncode==0,output
            moving=[item for item in out if item[2]]
            assert len(moving)>60 and moving[0][1]==901,moving[:3]
            assert moving[0][0]>=.3 and moving[0][3]>=40528
            assert any(t>3.4 for t,*_ in moving),'must remain recovered after ordinary updates'
            print(f'PASS V2 production PTY: stale 01/02/81 seq3631 ->40527/40528, source advances, pose recovers, next status reseeds TX901 ({len(out)} frames)')
        finally:
            if process.poll() is None:process.kill();process.wait()
            os.close(master);os.close(slave)

def run_wheel_hold(binary):
    master,slave=pty.openpty();os.set_blocking(master,False)
    with tempfile.TemporaryDirectory(prefix='sentinel-wheel-hold-') as folder:
        cfg=Path(folder)/'peer.yaml'
        cfg.write_text(f"""board_protocol: SENTINEL
xuc_enable: true
xuc_serial_port: "{os.ttyname(slave)}"
xuc_serial_baud: 921600
xuc_allow_control: true
xuc_allow_shoot: true
xuc_imu_yaw_sign: -1
xuc_imu_pitch_sign: 1
sentinel_allow_friction: true
sentinel_require_verified_pose: false
sentinel_friction_lost_timeout_ms: 5000
sentinel_tx_hz: 50
""")
        process=subprocess.Popen([binary,str(cfg),'friction'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        start=time.monotonic();next_tx=.1;index=0;rx=bytearray();out=[];refs={}
        try:
            while process.poll() is None:
                t=time.monotonic()-start
                if t>10:raise AssertionError('wheel hold peer hung')
                if t>=next_tx:
                    stamp=int(t*1e6);identity=2 if 6.4<t<6.5 else 1
                    p=struct.pack('<BBHI6f',identity,3,1,stamp,*[math.radians(x) for x in (22,11,179)],1,2,3)
                    g=frame(1,index,0x11223344,p);refs[index&65535]=stamp
                    st=frame(0x81,index,0x11223344,struct.pack('<IHBBHH',0,0,0x80,0,0xffff,0))
                    if index==0:
                        bad=bytearray(g);bad[20]^=1;os.write(master,bad)
                    os.write(master,g+st);index+=1;next_tx=t+.02
                if select.select([master],[],[],.002)[0]:
                    rx.extend(os.read(master,4096))
                    while len(rx)>=38:
                        b=bytes(rx[:38]);del rx[:38]
                        assert b[:4]==b'\xa6\x6a\x02\x10' and crc16(b[:-2])==struct.unpack_from('<H',b,36)[0]
                        session,epoch,refseq,stamp,valid,flags,res,dy,dp=struct.unpack_from('<IHHIHBBff',b,12)
                        assert flags in (0,5) and dy==dp==0 # no target/control motion or fire
                        if flags:assert session==res==0 and epoch==1 and valid==100 and refs[refseq]==stamp
                        out.append((time.monotonic()-start,flags))
            # Drain the destructor's explicit stop even when process exit raced the poll.
            while select.select([master],[],[],0)[0]:
                chunk=os.read(master,4096)
                if not chunk:break
                rx.extend(chunk)
                while len(rx)>=38:
                    b=bytes(rx[:38]);del rx[:38]
                    assert crc16(b[:-2])==struct.unpack_from('<H',b,36)[0]
                    out.append((time.monotonic()-start,b[26]))
            output=process.communicate(timeout=2)[0].decode(errors='replace')
            assert process.returncode==0,output
            spinning=[t for t,flags in out if flags==5]
            assert not any(t<.3 or 5.75<t<5.9 or 6.3<t<6.6 for t in spinning),out
            assert any(.4<t<.65 for t in spinning) and any(5.15<t<5.4 for t in spinning),out
            assert any(6.02<t<6.15 for t in spinning) and any(6.7<t<6.85 for t in spinning),out
            assert out[-1][1]==0,'destructor must bypass the five-second hold'
            assert sum(flags==0 for _,flags in out)>=3 # loss expiry, explicit shutdown, destructor
            print(f'PASS real sender: detection starts wheel-only lease; 5s loss expiry; reacquire; explicit exit; destructor; no gimbal/fire ({len(out)} frames)')
        finally:
            if process.poll() is None:process.kill();process.wait()
            os.close(master);os.close(slave)

def run_send_results(binary, cadence=False, interval_ms=500):
    """A real production fire frame survives a same-tick ordinary TX throttle."""
    master,slave=pty.openpty();os.set_blocking(master,False)
    with tempfile.TemporaryDirectory(prefix='sentinel-send-result-') as folder:
        cfg=Path(folder)/'peer.yaml'
        interval_line='' if interval_ms is None else f'sentinel_fire_min_interval_ms: {interval_ms}'
        cfg.write_text(f'''board_protocol: SENTINEL
xuc_enable: true
xuc_serial_port: "{os.ttyname(slave)}"
xuc_serial_baud: 921600
xuc_allow_control: true
xuc_allow_shoot: true
xuc_imu_yaw_sign: -1
xuc_imu_pitch_sign: 1
sentinel_allow_friction: true
sentinel_require_verified_pose: false
sentinel_tx_hz: 50
{interval_line}
''')
        process=subprocess.Popen([binary,str(cfg),'cadence' if cadence else 'sendresult'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        start=time.monotonic();next_tx=.1;seq=1;rx=bytearray();out=[]
        try:
            while process.poll() is None:
                t=time.monotonic()-start
                if t>5:raise AssertionError('send-result peer hung')
                if t>=next_tx:
                    p=struct.pack('<BBHI6f',1,3,1,int(t*1e6),*[math.radians(x) for x in (22,11,179)],1,2,3)
                    st=struct.pack('<IHBBHH',0,900,0x84,0,0xffff,0)
                    os.write(master,frame(1,seq,0x11223344,p)+frame(0x81,seq,0x11223344,st))
                    seq+=1;next_tx=t+.02
                if select.select([master],[],[],.001)[0]:
                    rx.extend(os.read(master,4096))
                    while len(rx)>=38:
                        b=bytes(rx[:38]);del rx[:38]
                        assert b[:4]==b'\xa6\x6a\x02\x10'
                        assert crc16(b[:-2])==struct.unpack_from('<H',b,36)[0]
                        out.append((struct.unpack_from('<H',b,6)[0],b[26]))
            output=process.communicate(timeout=2)[0].decode(errors='replace')
            assert process.returncode==0,output
            if cadence:
                flags=[f for _,f in out]
                assert 'SUMMARY cadence=1' in output,output
                effective_interval=1000 if interval_ms is None else interval_ms
                minimum,maximum=(6,8) if effective_interval==300 else (3,5) if effective_interval==500 else (3,4) if effective_interval==600 else (2,3)
                assert minimum<=sum(bool(f&8) for f in flags)<=maximum,(out,output)
                assert f'interval_ms={effective_interval}' in output,output
                assert all(not(a&8 and b&8) for a,b in zip(flags,flags[1:])),out
                assert 7 in flags and 0 in flags,(out,output)
                print(f'PASS production cadence: >={effective_interval}ms between successful fire writes, held/chattering requests, clear fire between shots, STOP keeps cooldown, control/friction continue; explicit_config={interval_ms is not None}')
            else:
                assert [flags for _,flags in out]==[7,15,7,0],(out,output)
                assert [s for s,_ in out]==[901,902,903,904],out
                print('PASS production send results: ordinary throttle, immediate fire TX, expired fire cleared, explicit stop, capture-pose history')
        finally:
            if process.poll() is None:process.kill();process.wait()
            os.close(master);os.close(slave)

if __name__=='__main__':
    for options in ((False,False,False),(True,False,True),(True,True,False),(True,True,True)):
        run(str(Path(sys.argv[1]).resolve()),*options)

    run(str(Path(sys.argv[1]).resolve()),True,True,True,False)

    run_startup_cache(str(Path(sys.argv[1]).resolve()))

    run_wheel_hold(str(Path(sys.argv[1]).resolve()))

    run_send_results(str(Path(sys.argv[1]).resolve()))
    run_send_results(str(Path(sys.argv[1]).resolve()),cadence=True)
    run_send_results(str(Path(sys.argv[1]).resolve()),cadence=True,interval_ms=600)
    run_send_results(str(Path(sys.argv[1]).resolve()),cadence=True,interval_ms=300)
    run_send_results(str(Path(sys.argv[1]).resolve()),cadence=True,interval_ms=1000)
    run_send_results(str(Path(sys.argv[1]).resolve()),cadence=True,interval_ms=None)
