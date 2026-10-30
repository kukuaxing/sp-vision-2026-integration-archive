"""Explicit STOP-only sequence boot maintenance; never submit a serial frame."""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import time
import uuid
from live import Wire, ROOT, HERE, checked_sources
from firmware import validate_flashed_profile


def binding_context(old, observed, profile, review):
    if (type(old.get('boot_id')) is not int or old['boot_id']<=0 or type(old.get('next_sequence')) is not int
        or not 0<=old['next_sequence']<=65535):raise ValueError('Malformed existing sequence journal')
    if (observed['state']!=0 or observed['reject']!=0 or observed['frames']<100 or observed['crc_errors']!=0
        or observed['age_s']>.20 or type(observed['boot']) is not int or not 0<observed['boot']<=0xffffffff):
        raise RuntimeError('A stable STOP heartbeat without active aim/navigation is required')
    validate_flashed_profile(profile,review)
    return dict(boot_id=observed['boot'],next_sequence=old['next_sequence'])


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bind',action='store_true');a=p.parse_args()
    checked_sources()
    profile=json.loads((HERE/'firmware_profile.json').read_text(encoding='utf-8'))
    review=json.loads((HERE/'firmware_source_review.json').read_text(encoding='utf-8'))
    wire=Wire();directory=None
    try:
        wire.open_and_drop_privileges()
        before=wire.journal.read_bytes();old=json.loads(before)
        until=time.monotonic()+2.
        while time.monotonic()<until:wire.receive();time.sleep(.005)
        s=wire.stream
        observed=dict(boot=s.boot,state=s.state,reject=s.reject,frames=s.count,crc_errors=s.bad_crc,
                      age_s=time.monotonic()-s.status_at)
        result=dict(mode='bind' if a.bind else 'check_only',observed=observed,previous=old,
                    serial_frames_sent=wire.io.frames_sent,journal_changed=False,motion_authorized=False)
        if a.bind:
            context=binding_context(old,observed,profile,review)
            if wire.io.frames_sent!=0:raise RuntimeError('Receive-only maintenance unexpectedly wrote serial data')
            if wire.journal.read_bytes()!=before:raise RuntimeError('Sequence journal changed during observation')
            directory=ROOT/'logs'/('map_route_boot_binding_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
            directory.mkdir()
            # A durable evidence copy is a log artifact, not another source tree.
            (directory/'previous_sequence.json').write_bytes(before)
            wire.support.atomic_json(directory/'intent.json',dict(previous=old,new=context,observed=observed,
                firmware_review_sha256=hashlib.sha256((HERE/'firmware_source_review.json').read_bytes()).hexdigest(),
                serial_frames_sent=0,motion_authorized=False,next_sequence_reset=False))
            if wire.journal.read_bytes()!=before:raise RuntimeError('Journal changed before binding commit')
            if context!=old:
                wire.support.atomic_json(wire.journal,context)
                result['journal_changed']=True
            if json.loads(wire.journal.read_bytes())!=context:raise RuntimeError('Journal post-check failed')
            result.update(current=context,next_sequence_preserved=True,postcheck_passed=True)
            wire.support.atomic_json(directory/'receipt.json',result)
        print(json.dumps(result,indent=2,ensure_ascii=False))
        if directory:print(str(directory))
    finally:
        error=wire.close()
        if error:raise RuntimeError(error)


if __name__=='__main__':
    try:main()
    except (RuntimeError,ValueError,OSError) as e:
        print('只读通信／绑定未完成：'+str(e),file=sys.stderr);sys.exit(2)
