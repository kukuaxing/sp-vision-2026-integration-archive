#!/usr/bin/env python3
"""Paired real-file detector regression; no ground-truth accuracy assertion."""
import copy
import json
from pathlib import Path
import subprocess
import tempfile
import yaml
from run_fixed_target import ROOT,digest


def main():
    config=yaml.safe_load((ROOT/'configs/standard.yaml').read_text())
    result={'scope':'real recorded video/blank regression, CPU1; no independent pose/label/impact truth',
        'physical_device_opened':False,'accuracy_measured':False,'experiments':[]}
    with tempfile.TemporaryDirectory(prefix='rm-endpoint-replay-') as tmp:
        for enabled in (False,True):
            name='candidate' if enabled else 'baseline';cfg=copy.deepcopy(config)
            cfg['corner_refine_brightness_invariant_subpixel']=enabled
            path=Path(tmp)/'config.yaml';path.write_text(yaml.safe_dump(cfg,sort_keys=False))
            report=ROOT/'logs/replay'/('endpoint-'+name+'.json')
            with (ROOT/'logs'/('endpoint-recorded-'+name+'.log')).open('w') as log:
                subprocess.run(['python3','scripts/replay_detector.py','--input','data/offline-input/test.mp4',
                    '--config',str(path),'--output','data/replay/endpoint-'+name,'--report',str(report),
                    '--max-frames','16','--stride','5','--repeat-check'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=300)
            # The normalized recording is the stable public format. Counts are
            # observational only; a higher count is not an accuracy result.
            capture=ROOT/'data/replay'/('endpoint-'+name)/'events.jsonl'
            rows=[json.loads(s) for s in capture.read_text().splitlines()]
            result['experiments'].append({'name':name,'report_sha256':digest(report),
                'capture_sha256':digest(capture),'event_rows':len(rows),
                'repeat_check_report':json.loads(report.read_text())})
        with (ROOT/'logs/endpoint-recorded-blank.log').open('w') as log:
            subprocess.run(['python3','scripts/replay_detector.py','--blank','--config',str(path),
                '--output','data/replay/endpoint-blank','--report','logs/replay/endpoint-blank.json',
                '--repeat-check'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=300)
    result['blank_and_repeat_checks_passed']=True
    (ROOT/'logs/endpoint-recorded-validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print('PASS paired real-file repeat checks and candidate blank negative control')


if __name__=='__main__':main()
