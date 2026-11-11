#!/usr/bin/env python3
"""Fetch only the pinned official binary SDK; never compile SDK implementation."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import stat
import tempfile
import urllib.request
import zipfile

def fetch(architecture, destination, archive=None):
    lock=json.loads((Path(__file__).resolve().parents[1]/'dependencies/sdk.lock.json').read_text())
    asset=lock['assets'][architecture]
    destination=Path(destination).expanduser().resolve()
    destination.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='coarm7-sdk-') as temporary:
        source=Path(archive).expanduser().resolve() if archive else Path(temporary)/asset['filename']
        if archive is None:
            request=urllib.request.Request(asset['url'],headers={'User-Agent':'coarm7-sdk-fetch'})
            with urllib.request.urlopen(request,timeout=60) as response,source.open('wb') as output:
                shutil.copyfileobj(response,output)
        digest=hashlib.sha256(source.read_bytes()).hexdigest()
        if digest!=asset['sha256']:
            raise RuntimeError(f'SDK checksum mismatch: expected {asset["sha256"]}, got {digest}')
        with zipfile.ZipFile(source) as bundle:
            names=bundle.infolist()
            top={PurePosixPath(item.filename).parts[0] for item in names if item.filename}
            if len(top)!=1:raise RuntimeError('SDK archive must contain one package directory')
            target=destination/next(iter(top))
            if target.exists():raise RuntimeError(f'Destination already exists: {target}')
            for item in names:
                path=PurePosixPath(item.filename)
                if path.is_absolute() or '..' in path.parts or '\\' in item.filename or stat.S_ISLNK(item.external_attr>>16):
                    raise RuntimeError(f'Unsafe SDK archive member: {item.filename}')
            # Extract to a staging directory, then move the complete verified tree.
            staging=Path(temporary)/'extracted';bundle.extractall(staging)
            shutil.move(str(staging/next(iter(top))),str(target))
        print(target)
        return target

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--architecture',choices=['x86_64','arm64'],required=True)
    parser.add_argument('--destination',required=True)
    parser.add_argument('--archive',help='Use an existing official ZIP, with the same checksum validation')
    args=parser.parse_args()
    try:fetch(args.architecture,args.destination,args.archive)
    except Exception as error:parser.exit(1,str(error)+'\n')
if __name__=='__main__':main()
