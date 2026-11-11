"""Source-boundary and SDK archive integrity checks; no network or CAN access."""
import hashlib
import importlib.util
import json
from pathlib import Path
import zipfile
import pytest

ROOT=Path(__file__).resolve().parents[1]
def module(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'tools'/(name+'.py'))
    obj=importlib.util.module_from_spec(spec);spec.loader.exec_module(obj);return obj
fetch=module('fetch_sdk');boundary=module('check_public_boundary')

def fixture_archive(tmp_path,monkeypatch,member='sdk/include/public.h',symlink=False,bad_hash=False):
    archive=tmp_path/'sdk.zip'
    with zipfile.ZipFile(archive,'w') as bundle:
        info=zipfile.ZipInfo(member)
        if symlink:info.create_system=3;info.external_attr=0o120777<<16
        bundle.writestr(info,'verified bytes')
    fake=tmp_path/'repository';(fake/'tools').mkdir(parents=True);(fake/'dependencies').mkdir()
    digest=hashlib.sha256(archive.read_bytes()).hexdigest()
    lock={'assets':{'x86_64':{'filename':'sdk.zip','sha256':'0'*64 if bad_hash else digest}}}
    (fake/'dependencies/sdk.lock.json').write_text(json.dumps(lock))
    monkeypatch.setattr(fetch,'__file__',str(fake/'tools/fetch_sdk.py'))
    return archive

def test_checksum_failure_cannot_extract(tmp_path,monkeypatch):
    archive=fixture_archive(tmp_path,monkeypatch,bad_hash=True)
    with pytest.raises(RuntimeError,match='checksum mismatch'):fetch.fetch('x86_64',tmp_path/'output',archive)
    assert not list((tmp_path/'output').iterdir())

def test_path_escape_cannot_extract(tmp_path,monkeypatch):
    archive=fixture_archive(tmp_path,monkeypatch,'sdk/../../escaped')
    with pytest.raises(RuntimeError,match='Unsafe SDK archive'):fetch.fetch('x86_64',tmp_path/'output',archive)
    assert not (tmp_path/'escaped').exists()

def test_symlink_cannot_extract(tmp_path,monkeypatch):
    archive=fixture_archive(tmp_path,monkeypatch,symlink=True)
    with pytest.raises(RuntimeError,match='Unsafe SDK archive'):fetch.fetch('x86_64',tmp_path/'output',archive)

def test_verified_archive_and_existing_prefix_protection(tmp_path,monkeypatch):
    archive=fixture_archive(tmp_path,monkeypatch)
    prefix=fetch.fetch('x86_64',tmp_path/'output',archive)
    assert (prefix/'include/public.h').read_text()=='verified bytes'
    with pytest.raises(RuntimeError,match='already exists'):fetch.fetch('x86_64',tmp_path/'output',archive)

def minimal_tree(tmp_path):
    root=tmp_path/'public';(root/'coarm7_hardware').mkdir(parents=True)
    (root/'coarm7_hardware/CMakeLists.txt').write_text('find_package(canevo_sdk_x86 CONFIG REQUIRED)\n')
    return root

def test_private_implementation_cannot_enter_public_tree(tmp_path):
    root=minimal_tree(tmp_path)
    assert boundary.check(root)['status']=='passed'
    (root/'coarm7_hardware/canevo_impl.cpp').write_text('private')
    assert boundary.check(root)['status']=='failed'

def test_source_symlink_cannot_bypass_boundary(tmp_path):
    root=minimal_tree(tmp_path);(tmp_path/'private.cpp').write_text('private')
    (root/'source.cpp').symlink_to(tmp_path/'private.cpp')
    assert boundary.check(root)['status']=='failed'


def test_official_lock_matches_exported_provenance_and_minimum_version():
    lock=json.loads((ROOT/'dependencies/sdk.lock.json').read_text())
    provenance=json.loads((ROOT/'coarm7_hardware/SDK_PROVENANCE.json').read_text())
    assert provenance['public_provider']==lock
    assert lock['version']=='0.0.4' and lock['release']=='v0.0.4'
    assert 'include(cmake/CoArm7SDK.cmake)' in (ROOT/'coarm7_hardware/CMakeLists.txt').read_text()
    resolver = (ROOT/'coarm7_hardware/cmake/CoArm7SDK.cmake').read_text()
    assert 'EXACT CONFIG' in resolver and 'SHA256=${_coarm7_sha}' in resolver
    for arch in ['x86_64','arm64']:
        asset=lock['assets'][arch]
        assert f'linux_{arch}_' in asset['filename']
        assert asset['url'].endswith('/v0.0.4/'+asset['filename'])
        assert len(asset['sha256'])==64


@pytest.mark.parametrize('name', ['.env', 'hardware_reviewed.yaml', 'sdk.zip', 'libtest.so.1', 'trace.pcap'])
def test_machine_and_binary_artifacts_rejected(tmp_path, name):
    root = minimal_tree(tmp_path)
    (root / name).write_text('not public')
    assert boundary.check(root)['status'] == 'failed'


def test_local_address_and_generated_output_rejected(tmp_path):
    root = minimal_tree(tmp_path)
    (root / 'install').mkdir()
    (root / 'note.txt').write_text('.'.join(['192', '168', '2', '1']))
    result = boundary.check(root)
    assert any('Generated/' in error for error in result['errors'])
    assert any('Machine address' in error for error in result['errors'])


def test_stl_asset_allowed_and_private_dependency_rejected(tmp_path):
    root = minimal_tree(tmp_path)
    (root / 'model.STL').write_bytes(bytes([0xff, 0, 0xff]))
    assert boundary.check(root)['status'] == 'passed'
    (root / 'coarm7_hardware/package.xml').write_text('<package><depend>modi_tools</depend></package>')
    assert boundary.check(root)['status'] == 'failed'


# Exercise the real CMake resolver with tiny local, hash-locked binary packages.
# These tests never download from the network or execute an SDK library.
import os
import subprocess
import shutil


def cmake_sdk_fixture(tmp_path, arch='x86_64'):
    root = tmp_path / 'source'
    (root / 'coarm7_hardware/cmake').mkdir(parents=True)
    (root / 'dependencies').mkdir()
    shutil.copyfile(ROOT/'coarm7_hardware/cmake/CoArm7SDK.cmake', root/'coarm7_hardware/cmake/CoArm7SDK.cmake')
    prefix = tmp_path/'sdk'
    cmake = prefix/'lib/cmake/canevo_sdk_x86';cmake.mkdir(parents=True)
    (prefix/'include').mkdir();(prefix/'share').mkdir()
    (prefix/'include/public.h').write_text('fixture')
    (prefix/'share/version.txt').write_text('0.0.4')
    (prefix/'CHANGELOG.md').write_text('fixture')
    elf=bytearray(20);elf[:6]=b'\x7fELF\x02\x01';elf[18:20]=(183 if arch=='arm64' else 62).to_bytes(2,'little')
    (prefix/'lib/libcanevo_sdk_x86.so').write_bytes(elf)
    (cmake/'canevo_sdk_x86Config.cmake').write_text(
        'add_library(canevo_sdk_x86::canevo_sdk_x86 SHARED IMPORTED)\n'
        'set_target_properties(canevo_sdk_x86::canevo_sdk_x86 PROPERTIES IMPORTED_LOCATION_RELEASE "${CMAKE_CURRENT_LIST_DIR}/../../libcanevo_sdk_x86.so")\n')
    (cmake/'canevo_sdk_x86ConfigVersion.cmake').write_text(
        'set(PACKAGE_VERSION 0.0.4)\nif(PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)\n'
        'set(PACKAGE_VERSION_COMPATIBLE TRUE)\nset(PACKAGE_VERSION_EXACT TRUE)\nendif()\n')
    archive=tmp_path/'official.zip'
    with zipfile.ZipFile(archive,'w') as bundle:
        for file in prefix.rglob('*'):
            if file.is_file():bundle.write(file,'sdk/'+str(file.relative_to(prefix)))
    asset={'url':archive.as_uri(),'sha256':hashlib.sha256(archive.read_bytes()).hexdigest()}
    (root/'dependencies/sdk.lock.json').write_text(json.dumps({'version':'0.0.4','assets':{'x86_64':asset,'arm64':asset}}))
    (root/'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.19)\nproject(sdk_test LANGUAGES NONE)\ninclude(coarm7_hardware/cmake/CoArm7SDK.cmake)\n')
    return root,prefix


def run_sdk_configure(root,tmp_path,processor='x86_64',extra=()):
    env=os.environ.copy()
    for key in ['CMAKE_PREFIX_PATH','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH']:env.pop(key,None)
    return subprocess.run(['cmake','-S',str(root),'-B',str(tmp_path/'build'),
        '-DCMAKE_SYSTEM_NAME=Linux','-DCMAKE_SYSTEM_PROCESSOR='+processor,
        '-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF',*extra],capture_output=True,text=True,env=env,timeout=25)


@pytest.mark.parametrize('arch,processor',[('x86_64','x86_64'),('arm64','aarch64')])
def test_cmake_download_reconfigure_and_install(tmp_path,arch,processor):
    root,prefix=cmake_sdk_fixture(tmp_path,arch)
    result=run_sdk_configure(root,tmp_path,processor)
    assert result.returncode==0,result.stdout+result.stderr
    assert 'fetching official SDK' in result.stdout
    result=run_sdk_configure(root,tmp_path,processor,['-DFETCHCONTENT_FULLY_DISCONNECTED=ON'])
    assert result.returncode==0,result.stdout+result.stderr
    assert 'fetching official SDK' not in result.stdout
    subprocess.run(['cmake','--install',str(tmp_path/'build'),'--prefix',str(tmp_path/'installed')],check=True,capture_output=True)
    assert (tmp_path/'installed/lib/libcanevo_sdk_x86.so').read_bytes()==(prefix/'lib/libcanevo_sdk_x86.so').read_bytes()


def test_cmake_reuses_external_sdk_without_download(tmp_path):
    root,prefix=cmake_sdk_fixture(tmp_path)
    result=run_sdk_configure(root,tmp_path,extra=['-DCOARM7_DOWNLOAD_SDK=OFF','-Dcanevo_sdk_x86_DIR='+str(prefix/'lib/cmake/canevo_sdk_x86')])
    assert result.returncode==0,result.stdout+result.stderr
    assert 'fetching official SDK' not in result.stdout
    assert not (tmp_path/'build/_deps').exists()


def test_cmake_rejects_wrong_architecture(tmp_path):
    root,prefix=cmake_sdk_fixture(tmp_path)
    result=run_sdk_configure(root,tmp_path,'aarch64',['-Dcanevo_sdk_x86_DIR='+str(prefix/'lib/cmake/canevo_sdk_x86')])
    assert result.returncode!=0 and 'architecture mismatch' in result.stderr


def test_cmake_rejects_unsupported_target(tmp_path):
    root,_=cmake_sdk_fixture(tmp_path)
    result=run_sdk_configure(root,tmp_path,'riscv64')
    assert result.returncode!=0 and 'Unsupported SDK target' in result.stderr


def test_cmake_rejects_bad_checksum(tmp_path):
    root,_=cmake_sdk_fixture(tmp_path)
    path=root/'dependencies/sdk.lock.json';lock=json.loads(path.read_text())
    lock['assets']['x86_64']['sha256']='0'*64;path.write_text(json.dumps(lock))
    result=run_sdk_configure(root,tmp_path)
    assert result.returncode!=0 and 'SHA256' in result.stdout+result.stderr


def test_readme_gif_assets_allowed_but_disguised_binary_rejected(tmp_path):
    root = minimal_tree(tmp_path)
    images = root/'docs/images';images.mkdir(parents=True)
    gif = images/'demo.gif';gif.write_bytes(b'GIF89a' + bytes(20))
    assert boundary.check(root)['status'] == 'passed'
    gif.write_bytes(b'\x7fELF' + bytes(20))
    assert boundary.check(root)['status'] == 'failed'


def test_readme_png_logo_allowed_but_disguised_binary_rejected(tmp_path):
    root = minimal_tree(tmp_path)
    images = root / 'docs/images'
    images.mkdir(parents=True)
    logo = images / 'modi.png'
    logo.write_bytes(b'\x89PNG\r\n\x1a\n' + bytes(20))
    assert boundary.check(root)['status'] == 'passed'
    logo.write_bytes(b'\x7fELF' + bytes(20))
    assert boundary.check(root)['status'] == 'failed'
