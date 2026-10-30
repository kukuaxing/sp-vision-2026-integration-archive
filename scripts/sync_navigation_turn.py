"""Synchronize only the reviewed navigation firmware files after verified flash."""
import base64
import hashlib
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]
flash=json.loads((ROOT/'data/flash/20261023_031047/result.json').read_text())
health=json.loads((ROOT/'logs/deployment/nav-turn-postflash-health.json').read_text())
assert flash['status']=='flashed_verified_reset' and health['passed']
assert flash['image_sha256']==health['binary_sha256']
report_path=ROOT/'logs/sentinel-v2-validation.json'
report=json.loads(report_path.read_text())
assert report['binary_sha256']==flash['image_sha256']
report.update(flashed=True,deployed=True,firmware_readback='Complete 128KiB application region verified; unwritten tail preserved',
    flash_result='data/flash/20261023_031047/result.json',postflash_report='logs/deployment/nav-turn-postflash-health.json',
    boot_id=health['samples'][-1]['boot'],STOP_confirmed=True)
report_path.write_text(json.dumps(report,indent=2)+'\n')
old={'RC.cpp':'2824b94ac658dff1a9b9f8c13f72759521326b0e15a383ce06138f921e9ef2bb',
     'RC.h':'cf13e51fe84faaedce8dd62de74bd462786f34c779759ab9dc1e670f8f1563b4',
     'xuc.h':'c5d97341a81d1f8ea49680a77520c505bac3bcb1be3b4f4d91bfb3b0025ff286'}
payload={}
for name,expected in old.items():
    rel='firmware/sentry/STM32F405/'+name
    data=(ROOT/rel).read_bytes()
    payload[rel]={'old':expected,'new':hashlib.sha256(data).hexdigest(),'data':base64.b64encode(data).decode()}
code='PAYLOAD='+repr(payload)+'\nREPORT='+repr(report)+'\n'+r'''
import base64,fcntl,hashlib,json,os
from pathlib import Path
root=Path('/home/rcsdart/rm27').resolve()
nav=root/'navigation'
with (nav/'logs/field_trial_entry.lock').open('a') as lock:
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    for p in (nav/'logs').glob('*_latch.json'):
        value=json.loads(p.read_text())
        assert not any(v is True for k,v in value.items() if k.endswith('_authorized')),str(p)
    for rel,item in PAYLOAD.items():
        path=(root/rel).resolve()
        assert path.is_relative_to(root/'firmware/sentry/STM32F405')
        assert hashlib.sha256(path.read_bytes()).hexdigest() in (item['old'],item['new']),rel+' changed independently'
        assert hashlib.sha256(base64.b64decode(item['data'])).hexdigest()==item['new']
    for rel,item in PAYLOAD.items():
        path=root/rel
        data=base64.b64decode(item['data'])
        path.write_bytes(data)
        assert hashlib.sha256(path.read_bytes()).hexdigest()==item['new']
    receipt={'firmware_binary_sha256':REPORT['binary_sha256'],'boot_id':REPORT['boot_id'],
        'source_files':{rel:item['new'] for rel,item in PAYLOAD.items()},
        'motion_started':False,'old_navigation_review_not_reauthorized':True}
    (root/'logs/deployment/nav-turn-firmware-sync.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(receipt))
'''
result=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=6','-o','ConnectionAttempts=1','rm27','python3 -'],
    input=code,text=True,encoding='utf-8',capture_output=True,timeout=20)
assert result.returncode==0,result.stderr+result.stdout
(ROOT/'logs/deployment/nav-turn-firmware-sync.json').write_text(result.stdout,encoding='utf-8')
print(result.stdout)
