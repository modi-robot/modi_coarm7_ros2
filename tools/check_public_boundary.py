#!/usr/bin/env python3
"""检查公开候选目录：私有实现、二进制依赖、机器配置和越界链接。"""
import argparse
import json
import re
from pathlib import Path
import xml.etree.ElementTree as ET

EXCLUDED = {'.git', '__pycache__', '.pytest_cache'}
GENERATED = {'build', 'install', 'log', 'artifacts', 'deps', 'vendor', '.coarm7-deps'}
PRIVATE = {'canevo_impl.cpp', 'canevo_impl.hpp', 'modi_bus_canevo.cpp',
           'modi_joint_canevo.cpp', 'modi_tool_io_canevo.cpp', 'modi_regen_canevo.cpp',
           'SDK_LOCAL_PATCH.diff', 'PUBLIC_EVIDENCE.json', 'id_rsa', 'id_ed25519'}
LOCAL_PATH = re.compile(r'/home/(?:fan|linux)(?:/|\b)|[A-Za-z]:[\\/]Users[\\/]')
PRIVATE_IP = re.compile(r'(?<![\d.])192\.168\.\d{1,3}\.\d{1,3}(?![\d.])')
KEY = re.compile(r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----')
BINARY = re.compile(r'\.(?:so(?:\.\d+)*|a|o|zip|deb|pcap|pcapng|tgz|tar(?:\.gz|\.xz)?)$')
LEGACY_DEPS = {'modi_tools', 'modi_robot', 'modi_robot_hardware', 'modi_system', 'modi_canevo_sdk'}

def check(root):
    root = Path(root).resolve()
    errors = []
    files = 0
    for file in root.rglob('*'):
        rel = file.relative_to(root)
        if any(part in EXCLUDED for part in rel.parts):
            continue
        if file.is_symlink():
            try:
                file.resolve(strict=True).relative_to(root)
            except (ValueError, OSError, RuntimeError):
                errors.append('External or invalid source symlink: ' + str(rel))
            # Do not read a possibly private target through a link.
            continue
        if file.is_dir():
            if file.name in GENERATED:
                errors.append('Generated/dependency directory: ' + str(rel))
            continue
        if not file.is_file():
            continue
        files += 1
        if file.name in PRIVATE:
            errors.append('Private SDK/evidence file: ' + str(rel))
        if file.name == '.env' or file.name.startswith('.env.') or file.name == 'hardware_reviewed.yaml':
            errors.append('Local machine configuration: ' + str(rel))
        if BINARY.search(file.name):
            errors.append('Binary dependency/build/capture artifact: ' + str(rel))
        # README images are published assets; reject disguised binaries.
        if file.suffix.lower() in {'.gif', '.png'}:
            with file.open('rb') as stream:
                header = stream.read(8)
            valid = (header[:6] in (b'GIF87a', b'GIF89a') if file.suffix.lower() == '.gif'
                     else header == b'\x89PNG\r\n\x1a\n')
            if rel.parts[:2] != ('docs', 'images') or not valid:
                errors.append('Invalid or misplaced README image: ' + str(rel))
            continue
        # STL meshes are intentionally published robot assets, not SDK binaries.
        if file.suffix.lower() == '.stl':
            continue
        try:
            text = file.read_text(encoding='utf-8')
        except UnicodeError:
            errors.append('Unexpected binary file: ' + str(rel))
            continue
        if any(pattern.search(text) for pattern in (LOCAL_PATH, PRIVATE_IP, KEY)):
            errors.append('Machine address/path or private key in: ' + str(rel))
        if file.name == 'package.xml':
            try:
                dependencies = {e.text for e in ET.fromstring(text) if 'depend' in e.tag}
                if dependencies & LEGACY_DEPS:
                    errors.append('Legacy system manifest dependency: ' + str(rel))
            except ET.ParseError:
                errors.append('Invalid package manifest: ' + str(rel))
    cmake_path = root / 'coarm7_hardware/CMakeLists.txt'
    cmake = cmake_path.read_text() if cmake_path.is_file() else ''
    if 'vendor/canevo_sdk' in cmake:
        errors.append('Build still references a vendored private SDK')
    resolver = root / 'coarm7_hardware/cmake/CoArm7SDK.cmake'
    sdk_cmake = cmake + (resolver.read_text() if resolver.is_file() else '')
    if 'find_package(canevo_sdk_x86' not in sdk_cmake:
        errors.append('Public SDK package interface missing')
    return {'status': 'failed' if errors else 'passed', 'files_checked': files,
            'errors': errors, 'note': 'Static checks do not certify licensing or detect every possible secret.'}

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', nargs='?', default=Path(__file__).resolve().parents[1])
    result = check(parser.parse_args().root)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    raise SystemExit(bool(result['errors']))
