"""Bounded attach-only shooter inspection. No halt, reset, writes or UART commands.
Run with the existing local pyOCD virtualenv; operator starts any physical test.
The exact running image is read and hashed before interpreting RAM addresses.
"""
import argparse
import hashlib
import json
import struct
import time
from pathlib import Path
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target
from pyocd.probe.pydapaccess.interface.hidapi_backend import HidApiUSB
from pyocd.probe.pydapaccess.dap_access_api import DAPAccessIntf

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=5)
    parser.add_argument('--interval', type=float, default=.05)
    parser.add_argument('--stop-on-auto-exit', action='store_true')
    parser.add_argument('--output', default='logs/deployment/sentinel-shooter-readonly.jsonl')
    args = parser.parse_args()
    assert 1 <= args.seconds <= 180 and .02 <= args.interval <= 2
    output = (ROOT / args.output).resolve()
    assert output.is_relative_to((ROOT / 'logs').resolve())
    layout = json.loads((ROOT / 'logs/deployment/sentinel-shooter-probe-layout.json').read_text())
    image = (ROOT / 'firmware/sentry/build/STM32F405.bin').read_bytes()
    assert hashlib.sha256(image).hexdigest() == layout['binary_sha256']
    assert hashlib.sha256((ROOT / 'firmware/sentry/build/STM32F405.elf').read_bytes()).hexdigest() == layout['elf_sha256']
    def bounded_read(self):
        data = bytes(self.device.read(self.packet_size, self.DEFAULT_USB_TIMEOUT_MS))
        if not data: raise DAPAccessIntf.DeviceError('HID timeout; no automatic reconnect')
        return data
    HidApiUSB.read = bounded_read
    probe = ConnectHelper.choose_probe(unique_id='cmsisdap:ATK 20190528', blocking=False)
    assert probe is not None, 'Expected local probe is not attached'
    probe.create_associated_board = lambda: None
    session = Session(probe, options={'target_override':'stm32f405rg', 'frequency':250000,
        'connect_mode':'attach', 'auto_unlock':False, 'resume_on_disconnect':False, 'no_config':True,
        'cmsis_dap.limit_packets':True, 'cmsis_dap.deferred_transfers':False,
        'cache.enable_memory':False, 'cache.enable_register':False})
    field = layout['fields']
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', encoding='utf-8') as stream, session:
        target = session.target
        assert target.get_state() == Target.State.RUNNING
        uid = bytes(target.read_memory_block8(0x1fff7a10,12)).hex()
        assert uid == layout['expected_uid']
        live = bytearray()
        for offset in range(0,len(image),256):
            live.extend(target.read_memory_block8(0x08000000+offset,min(256,len(image)-offset)))
        assert hashlib.sha256(live).hexdigest() == layout['binary_sha256'], 'Running firmware differs; addresses cannot be used'
        pointers = list(target.read_memory_block32(field['shooter_pointers']['address'],2))
        pointers.append(target.read32(field['feeder_pointer']['address']))
        assert all(0x20000000 <= p <= 0x20020000-408 for p in pointers)
        ids = [target.read32(p) for p in pointers]
        meta = {'kind':'metadata','read_only':True,'halt_reset_or_actuator_write':False,
            'verified_image_sha256':layout['binary_sha256'],'uid':uid,'motor_addresses':pointers,
            'motor_ids':ids,'snapshot_atomic':False,'started_local':time.strftime('%Y-%m-%d %H:%M:%S')}
        stream.write(json.dumps(meta)+'\n');stream.flush();print(json.dumps(meta),flush=True)
        start = time.monotonic(); count=0; seen_auto=False; exit_samples=0
        while time.monotonic()-start < args.seconds:
            tick0 = target.read32(field['tick']['address'])
            mode = target.read8(field['mode']['address'])
            shooter = bytes(target.read_memory_block8(field['shooter']['address'],40))
            motors=[]
            for pointer in pointers:
                data=bytes(target.read_memory_block8(pointer+72,136))
                tail=bytes(target.read_memory_block8(pointer+360,8))
                value=lambda fmt,offset:struct.unpack_from('<'+fmt,data,offset-72)[0]
                motors.append({'last_feedback_ms':value('I',72),'feedback_received':value('B',76),
                    'generation':value('I',88),'decoded_generation':value('I',92),
                    'current':value('i',100),'speed_rpm':value('i',104),'setcurrent':value('i',108),
                    'target_rpm':value('i',112),'sum_angle':value('i',128),'temperature':value('i',188),
                    'has_feedback':value('B',206),'angle_latched':value('B',207),
                    'target_angle':struct.unpack_from('<f',tail)[0],
                    'angle':struct.unpack_from('<f',tail,4)[0]})
            aim={key:target.read8(field[key]['address']) for key in ('aim_control','aim_valid','aim_friction','aim_fire')}
            tick1=target.read32(field['tick']['address'])
            mode_end=target.read8(field['mode']['address'])
            for motor in motors:
                motor['feedback_age_upper_ms']=(tick1-motor['last_feedback_ms'])&0xffffffff
                motor['fresh']=bool(motor['feedback_received'] and motor['feedback_age_upper_ms']<=100)
            shoot_rpm=struct.unpack_from('<h',shooter,12)[0]
            wheels_ready=bool(all(m['fresh'] and m['has_feedback'] and m['temperature']<=70 for m in motors[:2])
                and motors[0]['speed_rpm']>=shoot_rpm*.8 and motors[1]['speed_rpm']<=-shoot_rpm*.8)
            feeder_ready=bool(motors[2]['fresh'] and motors[2]['has_feedback'] and motors[2]['angle_latched'])
            row={'kind':'sample','elapsed_s':round(time.monotonic()-start,4),'mode':mode,'mode_end':mode_end,
                'tick_start':tick0,'tick_end':tick1,'span_ms':(tick1-tick0)&0xffffffff,
                'auto_shoot':shooter[4],'request_level':shooter[5],'openRub':shooter[6],
                'shoot_speed':shoot_rpm,'trig_pre':shooter[21],
                'feed_permission_was_ready':shooter[26],'feed_rearm_required':shooter[27],
                'wheels_ready_observed':wheels_ready,'feeder_ready_observed':feeder_ready,'aim':aim,
                'motors':dict(zip(('left','right','feeder'),motors))}
            stream.write(json.dumps(row)+'\n');stream.flush();count+=1
            if count==1 or count%50==0:
                print(json.dumps({k:row[k] for k in ('elapsed_s','mode','openRub','auto_shoot','wheels_ready_observed','feeder_ready_observed')}
                    | {'rpm':[m['speed_rpm'] for m in motors]}),flush=True)
            if mode==2 and mode_end==2: seen_auto=True
            exit_samples=exit_samples+1 if seen_auto and mode==9 and mode_end==9 else 0
            if args.stop_on_auto_exit and exit_samples>=2: break
            time.sleep(args.interval)
        assert target.get_state()==Target.State.RUNNING
        print(json.dumps({'status':'completed_read_only','samples':count,'path':str(output)}),flush=True)

if __name__=='__main__':main()
