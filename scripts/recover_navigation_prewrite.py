"""Resume only the identified old STOP firmware after the recorded pre-write failure."""
import json
from pathlib import Path
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target
from pyocd.probe.pydapaccess.interface.hidapi_backend import HidApiUSB
from pyocd.probe.pydapaccess.dap_access_api import DAPAccessIntf

ROOT=Path(__file__).resolve().parents[1]
failed=json.loads((ROOT/'data/flash/20261023_030647/result.json').read_text())
assert failed['status']=='failed_do_not_assume_running' and failed['preflash_mode']==9
assert 'original_sha256' not in failed and 'Probe HID reply timeout' in failed['error']
layout=json.loads((ROOT/'logs/deployment/nav-turn-preflash-layout.json').read_text())
def bounded_read(self):
    data=bytes(self.device.read(self.packet_size,self.DEFAULT_USB_TIMEOUT_MS))
    if not data:raise DAPAccessIntf.DeviceError('Probe timeout; no automatic reconnect')
    return data
HidApiUSB.read=bounded_read
probe=ConnectHelper.choose_probe(unique_id='cmsisdap:ATK 20190528',blocking=False)
assert probe is not None
probe.create_associated_board=lambda:None
with Session(probe,options={'target_override':'stm32f405rg','frequency':250000,
    'connect_mode':'attach','auto_unlock':False,'resume_on_disconnect':False,'no_config':True,
    'cmsis_dap.limit_packets':True,'cmsis_dap.deferred_transfers':False,
    'cache.enable_memory':False,'cache.enable_register':False}) as session:
    target=session.target
    assert bytes(target.read_memory_block8(0x1FFF7A10,12)).hex()==failed['uid']
    assert target.read8(layout['addresses']['mode'])==9
    state=target.get_state()
    assert state in (Target.State.HALTED,Target.State.RUNNING)
    if state==Target.State.HALTED:target.resume()
    assert target.get_state()==Target.State.RUNNING
    report={'old_firmware_resumed':True,'previous_state':str(state),'mode':9,
        'flash_erased':False,'flash_written':False,'reset_requested':False}
    (ROOT/'logs/deployment/nav-turn-prewrite-recovery.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))
