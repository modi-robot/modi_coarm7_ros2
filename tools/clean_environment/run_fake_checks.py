#!/usr/bin/env python3
"""Run only fake transport, in the container's private network namespace."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

p=argparse.ArgumentParser();p.add_argument('--output',required=True);args=p.parse_args()
out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
with (out/'fake_launch.log').open('w') as log:
    launch=subprocess.Popen(['ros2','run','coarm7_bringup','start_coarm7.py','--mode','fake','--no-rviz'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        deadline=time.monotonic()+70
        while 'CoArm7 startup: Ready:' not in (out/'fake_launch.log').read_text():
            if launch.poll() is not None:raise RuntimeError('Project startup exited before readiness; inspect fake_launch.log')
            if time.monotonic()>deadline:raise RuntimeError('Project startup readiness timed out; inspect fake_launch.log')
            time.sleep(.2)
        for name in ['validate_offline','validate_ptp']:
            with (out/(name+'.log')).open('w') as stream:
                result=subprocess.run(['ros2','run','coarm7_bringup',name+'.py','--report',str(out/(name+'.json'))],stdout=stream,stderr=subprocess.STDOUT,timeout=240)
            if result.returncode:raise RuntimeError(name+' failed; inspect '+str(out/(name+'.log')))
    finally:
        log.flush()
        shutdown_log_offset=(out/'fake_launch.log').stat().st_size
        try:os.killpg(launch.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:launch.wait(timeout=40)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid,signal.SIGTERM)
            try:launch.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGKILL);launch.wait()
summary={}
for name in ['validate_offline','validate_ptp']:
    report=json.loads((out/(name+'.json')).read_text());summary[name]={'status':report['status'],'checks':len(report['tests'])}
launch_log=(out/'fake_launch.log').read_bytes()
summary['runtime_child_crash_recorded']=b'process has died' in launch_log[:shutdown_log_offset]
summary['shutdown_child_crash_recorded']=b'process has died' in launch_log[shutdown_log_offset:]
summary['status']=('failed' if summary['runtime_child_crash_recorded'] else
                   'passed_with_shutdown_issue' if summary['shutdown_child_crash_recorded'] else 'passed')
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))

if summary['runtime_child_crash_recorded']:raise SystemExit(1)
