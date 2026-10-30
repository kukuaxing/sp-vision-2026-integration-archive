"""Prepare build-bound navigation-turn validation and SWD layouts; no device I/O."""
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT=Path(__file__).resolve().parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
image=ROOT/'firmware/sentry/build/STM32F405.bin'
elf=image.with_suffix('.elf')
old=json.loads((ROOT/'logs/deployment/sentinel-shooter-probe-layout.json').read_text())
assert old['binary_sha256']=='c0b7b2887e9dcf3fb72d5b9da3113c41b34a91ab0f6c31bed6ef2039768f74b6'
pre={'binary_sha256':old['binary_sha256'],'binary_size':old['binary_size'],
    'fields':{'mode':{'size_bytes':old['fields']['mode']['bytes']}},
    'addresses':{'mode':old['fields']['mode']['address']},'mode_enum':{'STOP':9,'RESET':3}}
(ROOT/'logs/deployment/nav-turn-preflash-layout.json').write_text(json.dumps(pre,indent=2)+'\n')
fields={'mode':'ctrl.mode','speedx':'ctrl.chassis.speedx','speedy':'ctrl.chassis.speedy',
    'speedz':'ctrl.chassis.speedz','nav_turn_rpm':'rc.nav_turn_rpm','boot':'xuc.boot_id',
    'tick':'xTickCount','nav_enable':'xuc.nav_enable','nav_wz':'xuc.nav_wz'}
gdb=Path('C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-gdb.exe')
args=[str(gdb),'-batch',str(elf)]
for name,expr in fields.items():
    args+=['-ex',f'printf "FIELD {name} %u %u\\n", (unsigned)&({expr}), (unsigned)sizeof({expr})']
result=subprocess.run(args,capture_output=True,text=True,check=True)
layout={name:{'address':int(address),'bytes':int(size)}
    for name,address,size in re.findall(r'FIELD (\w+) (\d+) (\d+)',result.stdout)}
assert set(layout)==set(fields)
post={'binary_sha256':sha(image),'binary_size':image.stat().st_size,'elf_sha256':sha(elf),
      'expected_uid':'1b0037000647353330303436','fields':layout}
(ROOT/'logs/deployment/nav-turn-postflash-layout.json').write_text(json.dumps(post,indent=2)+'\n')
ctest='D:/Dev/VSBuildTools/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/ctest.exe'
test=subprocess.run([ctest,'--test-dir',str(ROOT/'build/native-offline'),'-C','Release',
    '-R','^sentinel_firmware_','--output-on-failure'],capture_output=True,text=True)
(ROOT/'logs/nav-turn-host-tests.log').write_text(test.stdout+test.stderr)
assert test.returncode==0 and '100% tests passed' in test.stdout
assert sha(image)==post['binary_sha256']
report={'binary_sha256':sha(image),'binary_bytes':image.stat().st_size,'control_enabled':True,
    'arm_link':'passed','native_firmware_variants':'4/4 passed; production ApplyNav and parser',
    'navigation_turn_enabled':True,'turn_units':'raw amplitude * 159 motor rpm; physical left/right sign unmeasured',
    'turn_limit_motor_rpm':200,'turn_step_motor_rpm_per_5ms_cycle':10,
    'physical_tests_started':False,'turn_direction_physically_verified':False,
    'flashed':False,'deployed':False,'firmware_readback':'pending',
    'previous_binary_sha256':pre['binary_sha256'],
    'source_sha256':{name:sha(ROOT/'firmware/sentry/STM32F405'/name) for name in ('RC.cpp','RC.h','xuc.h')},
    'validation_scope':'Navigation turn hookup only; no aim/fire control changes or motion test',
    'regressions':['positive/negative turn','finite extreme clamp','slew','translation preserved',
        'NaN/Inf rejection','enabled zero','withdrawal','lease timeout/manual fallback',
        'StopChassis','non-AUTO','remote loss']}
(ROOT/'logs/sentinel-v2-validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
