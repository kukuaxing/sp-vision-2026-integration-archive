"""Attach-only postflash STOP check. No reset, halt, flash or control writes."""
import hashlib
import json
from pathlib import Path
import struct
import time
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target
from pyocd.probe.pydapaccess.interface.hidapi_backend import HidApiUSB
from pyocd.probe.pydapaccess.dap_access_api import DAPAccessIntf

ROOT=Path(__file__).resolve().parents[1]
layout=json.loads((ROOT/'logs/deployment/nav-turn-postflash-layout.json').read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(ROOT/'firmware/sentry/build/STM32F405.bin')==layout['binary_sha256']
assert sha(ROOT/'firmware/sentry/build/STM32F405.elf')==layout['elf_sha256']
def bounded_read(self):
    data=bytes(self.device.read(self.packet_size,self.DEFAULT_USB_TIMEOUT_MS))
    if not data:raise DAPAccessIntf.DeviceError('Probe timeout; no automatic reconnect')
    return data
HidApiUSB.read=bounded_read
report={'read_only':True,'reset_requested':False,'physical_tests_started':False,
    'binary_sha256':layout['binary_sha256'],'samples':[],'passed':False}
try:
    probe=ConnectHelper.choose_probe(unique_id='cmsisdap:ATK 20190528',blocking=False)
    assert probe is not None,'Expected probe unavailable'
    probe.create_associated_board=lambda:None
    with Session(probe,options={'target_override':'stm32f405rg','frequency':250000,
        'connect_mode':'attach','auto_unlock':False,'resume_on_disconnect':False,
        'no_config':True,'cmsis_dap.limit_packets':True,'cmsis_dap.deferred_transfers':False,
        'cache.enable_memory':False,'cache.enable_register':False}) as session:
        target=session.target
        assert target.get_state()==Target.State.RUNNING
        assert bytes(target.read_memory_block8(0x1FFF7A10,12)).hex()==layout['expected_uid']
        for _ in range(10):
            row={}
            for name,field in layout['fields'].items():
                raw=bytes(target.read_memory_block8(field['address'],field['bytes']))
                row[name]=(struct.unpack('<f',raw)[0] if name in ('nav_turn_rpm','nav_wz')
                    else int.from_bytes(raw,'little',signed=name in ('speedx','speedy','speedz')))
            report['samples'].append(row)
            assert row['mode']==9 and row['boot']!=0
            assert row['speedx']==row['speedy']==row['speedz']==row['nav_turn_rpm']==0
            assert row['nav_enable']==0
            time.sleep(.1)
        assert len({s['boot'] for s in report['samples']})==1
        assert all(b['tick']>a['tick'] for a,b in zip(report['samples'],report['samples'][1:]))
        assert target.get_state()==Target.State.RUNNING
        report['passed']=True
except BaseException as exc:
    report['error']=repr(exc)
    raise
finally:
    (ROOT/'logs/deployment/nav-turn-postflash-health.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
