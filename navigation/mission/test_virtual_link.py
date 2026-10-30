#!/usr/bin/env python3
"""Exercise real XucSender/bridge over a PTY, with no physical device access."""
from pathlib import Path
import json,os,pty,select,struct,subprocess,tempfile,time,tty


def crc(data):
    c=0xffff
    for byte in data:
        c^=byte
        for _ in range(8):c=(c>>1)^ (0x8408 if c&1 else 0)
    return c


def frame(typ,seq,payload):
    b=struct.pack('<2sBBHHI',b'\xa6\x6a',2,typ,len(payload),seq,123)+payload
    return b+struct.pack('<H',crc(b))


def run(binary):
    with tempfile.TemporaryDirectory(prefix='rm27-mission-pty-') as d:
        root=Path(d);master,slave=pty.openpty();port=os.ttyname(slave)
        tty.setraw(slave) # Disable PTY echo before the production serial opener runs.
        os.set_blocking(master,False)
        config=root/'config.yaml'
        config.write_text(f'''board_protocol: SENTINEL
xuc_enable: true
xuc_serial_port: {port}
xuc_serial_baud: 921600
xuc_allow_control: true
xuc_allow_shoot: true
sentinel_allow_friction: true
sentinel_require_verified_pose: false
sentinel_fire_min_interval_ms: 200
''')
        env=dict(os.environ,RM27_MISSION_DIR=d)
        log=(root/'probe.log').open('w')
        child=subprocess.Popen([binary,str(config)],env=env,stdout=log,stderr=subprocess.STDOUT)
        begin=time.monotonic();seq=0;pending=bytearray();frames=[];last_cmd=0;status={}
        try:
            while time.monotonic()-begin<14.5:
                now=time.monotonic();elapsed=now-begin
                us=int(elapsed*1e6)&0xffffffff
                # Boot already AUTO must NOT authorize aim or navigation by itself.
                g=struct.pack('<BBHIffffff',1,11,1,us,0,0,0,0,0,0)
                link=struct.pack('<IHBBHH',0,0,0x80,0,0,0)
                # No 0x83 at startup, then deliberately misleading MCU encoder telemetry.
                extra=b''
                if elapsed>=3:
                    m=struct.pack('<BBHIIII f HH',1,0,0,7,999999,us,0,100.,0,0)
                    extra=frame(0x83,seq,m)
                os.write(master,frame(1,seq,g)+frame(0x81,seq,link)+extra);seq=(seq+1)&65535
                try:status=json.loads((root/'vision.json').read_text())
                except (OSError,ValueError):pass
                # 1..2 NAV only; task then reaches 50 actual host fire writes at ~12s.
                # Stop publishing at 13s: independent IPC watchdog must revoke NAV.
                if status and 1<elapsed<13 and now-last_cmd>.02:
                    task=elapsed>=2
                    j=dict(schema=2,count_source='host_tx',monotonic_ns=time.monotonic_ns(),vision_instance=status['vision_instance'],mcu_boot=123,
                        nav_enabled=True,task_enabled=task,vx=0. if task else .1,vy=0.,wz=0.,
                        count_limit=50,count_generation=0)
                    tmp=root/'command.tmp';tmp.write_text(json.dumps(j));tmp.replace(root/'command.json');last_cmd=now
                try:pending.extend(os.read(master,65536))
                except BlockingIOError:pass
                while len(pending)>=12:
                    assert pending[:2]==b'\xa6\x6a','Interleaved or corrupt serial frame'
                    n=struct.unpack_from('<H',pending,4)[0]+14
                    if len(pending)<n:break
                    b=bytes(pending[:n]);del pending[:n]
                    assert crc(b[:-2])==struct.unpack_from('<H',b,n-2)[0]
                    frames.append((elapsed,b[3],b[12:-2]))
                if child.poll() is not None:raise RuntimeError('Probe exited: '+(root/'probe.log').read_text())
                time.sleep(.015)
            def aim_on(t):return [p[14] for e,k,p in frames if k==0x10 and t[0]<e<t[1] and p[14]]
            assert not frames or not [f for f in frames if f[0]<.9], 'Boot AUTO produced output'
            assert not aim_on((1.1,1.9)), 'Aim/friction enabled during outbound'
            assert any(k==0x11 and p[10]==1 for e,k,p in frames if 1.1<e<1.9)
            assert any(v&8 for v in aim_on((2.5,3.9))), 'Task did not permit qualified aim/fire'
            fires=[(e,p) for e,k,p in frames if k==0x10 and p[14]&8]
            assert len(fires)==50, f'Expected 50 successful host fire writes, got {len(fires)}'
            assert status['count_source']=='host_tx' and status['fire_requests_sent']==50
            assert not aim_on((12.6,14.4)), 'Host count ceiling/stale IPC retained aim permission'
            assert any(k==0x11 and p[10]==0 for e,k,p in frames if 13.05<e<13.6),'IPC loss did not revoke NAV'
            print(json.dumps(dict(result='PASS',frames=len(frames),physical_device_opened=False,
                checks=['boot_AUTO_no_output','outbound_NAV_without_0x83','task_AIM_FIRE',
                        'MCU_count_ignored','exactly_50_host_fire_writes','host_count_50_gates_AIM',
                        'stale_IPC_revokes_NAV','serialized_CRC_frames'])))
        finally:
            child.terminate()
            try:child.wait(timeout=2)
            except subprocess.TimeoutExpired:child.kill();child.wait()
            os.close(master);os.close(slave);log.close()


if __name__=='__main__':
    import sys
    run(sys.argv[1])
