#!/usr/bin/env python3
"""Audit SDK ELF dependency closure without loading it or opening a CAN socket."""
import argparse
import json
from pathlib import Path
import re
import subprocess

def elf(path):
    header=subprocess.check_output(['readelf','-h',str(path)],text=True,stderr=subprocess.STDOUT)
    machine=re.search(r'Machine:\s*(.*)',header).group(1).strip()
    dynamic=subprocess.check_output(['readelf','-d',str(path)],text=True,stderr=subprocess.STDOUT)
    return machine,re.findall(r'\(NEEDED\).*?\[(.*?)\]',dynamic)

def audit(sdk_root,sysroot='/'):
    sdk=Path(sdk_root).expanduser().resolve();system=Path(sysroot).expanduser().resolve()
    library=sdk/'lib/libcanevo_sdk_x86.so'
    errors=[]
    for name in ['modi_bus_canevo.h','modi_joint_canevo.h','modi_tool_io_canevo.h','modi_regen_canevo.h']:
        if not (sdk/'include'/name).is_file():errors.append('Missing public header: '+name)
    if not (sdk/'lib/cmake/canevo_sdk_x86/canevo_sdk_x86Config.cmake').is_file():errors.append('Missing exported CMake package')
    if not library.is_file():return {'status':'failed','errors':errors+['Missing SDK library'],'missing':[],'objects':[]}
    machine,_=elf(library)
    architecture='arm64' if machine=='AArch64' else 'x86_64' if 'X86-64' in machine else 'unsupported'
    triplet='aarch64-linux-gnu' if architecture=='arm64' else 'x86_64-linux-gnu'
    search=[sdk/'lib',sdk/'lib/runtime',system/'opt/ros/humble/lib',system/('usr/lib/'+triplet),system/('lib/'+triplet),system/'lib',system/'usr/lib']
    pending=[library];seen=set();objects=[];missing=set()
    while pending:
        path=pending.pop();key=str(path.resolve())
        if key in seen:continue
        seen.add(key)
        try:other,deps=elf(path)
        except (OSError,subprocess.CalledProcessError) as error:errors.append(str(error));continue
        if other!=machine:errors.append(f'Architecture mismatch: {path.name}: {other}, expected {machine}')
        objects.append({'file':str(path),'machine':other,'needed':deps})
        for name in deps:
            candidates=[path.parent/name,path.parent/'runtime'/name,*[directory/name for directory in search]]
            resolved=next((p for p in candidates if p.is_file()),None)
            if resolved is None:missing.add(name)
            else:pending.append(resolved)
    if missing:errors.append('Unresolved SDK runtime dependencies: '+', '.join(sorted(missing)))
    return {'status':'failed' if errors else 'passed','architecture':architecture,'sdk_root':str(sdk),'sysroot':str(system),'errors':errors,'missing':sorted(missing),'objects':objects}

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--sdk-root',required=True);parser.add_argument('--sysroot',default='/');parser.add_argument('--report')
    args=parser.parse_args();result=audit(args.sdk_root,args.sysroot)
    if args.report:Path(args.report).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='objects'},indent=2))
    raise SystemExit(0 if result['status']=='passed' else 1)
if __name__=='__main__':main()
