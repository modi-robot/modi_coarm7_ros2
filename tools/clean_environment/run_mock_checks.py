#!/usr/bin/env python3
"""Mock-only integration. Run in an isolated no-CAN environment."""
import argparse,json,os,signal,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
with (out/'launch.log').open('w') as log:
 process=subprocess.Popen(['ros2','launch','coarm7_bringup','mock.launch.py','rviz:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  with (out/'validation.log').open('w') as stream:
   result=subprocess.run(['ros2','run','coarm7_bringup','validate_mock.py','--report',str(out/'report.json')],stdout=stream,stderr=subprocess.STDOUT,timeout=300)
  if result.returncode:raise RuntimeError('Mock validation failed; see validation.log')
 finally:
  try:os.killpg(process.pid,signal.SIGINT)
  except ProcessLookupError:pass
  try:process.wait(timeout=40)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
print((out/'report.json').read_text())
