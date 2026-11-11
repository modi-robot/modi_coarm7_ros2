"""Affinity-only tests: no ROS, CAN, SDK, or real-time scheduling."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from unittest.mock import Mock
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/affinity_guard.py'
spec = importlib.util.spec_from_file_location('affinity_guard', SCRIPT)
guard = importlib.util.module_from_spec(spec); spec.loader.exec_module(guard)


def test_probe_can_expand_narrow_parent_mask_and_restore(monkeypatch):
    state = {0,1,2,3,4,5}; original = state.copy()
    def set_mask(_, mask): state.clear(); state.update(mask)
    monkeypatch.setattr(guard.os, 'sched_getaffinity', lambda _: set(state))
    monkeypatch.setattr(guard.os, 'sched_setaffinity', set_mask)
    report = guard.probe({4,5,6,7}, 3)
    assert report['kernel_binding_verified'] and state == original


def test_silent_cpuset_intersection_is_rejected_and_original_restored(monkeypatch):
    allowed = {0,1,2,3,4,5}; state = allowed.copy()
    def set_mask(_, mask): state.clear(); state.update(set(mask) & allowed)
    monkeypatch.setattr(guard.os, 'sched_getaffinity', lambda _: set(state))
    monkeypatch.setattr(guard.os, 'sched_setaffinity', set_mask)
    with pytest.raises(RuntimeError, match='filtered'): guard.probe({4,5,6,7}, -1)
    assert state == allowed


def test_kernel_rejection_preserves_errno_and_never_executes(monkeypatch):
    monkeypatch.setattr(guard.os, 'sched_getaffinity', lambda _: {0})
    def reject(_, mask):
        if mask != {0}: raise PermissionError(1, 'Operation not permitted')
    monkeypatch.setattr(guard.os, 'sched_setaffinity', reject)
    execute = Mock(); monkeypatch.setattr(guard.os, 'execvp', execute)
    with pytest.raises(RuntimeError, match='kernel rejected'):
        guard.main(['--cpus','7','--','should-never-execute'])
    execute.assert_not_called()


def test_probe_only_never_executes_command():
    with pytest.raises(ValueError, match='cannot execute'):
        guard.main(['--probe-only','--','should-never-execute'])


def test_control_cpu_can_be_outside_process_default(monkeypatch):
    state = {0,1}; masks=[]
    def set_mask(_, mask): masks.append(set(mask)); state.clear(); state.update(mask)
    monkeypatch.setattr(guard.os, 'sched_getaffinity', lambda _: set(state))
    monkeypatch.setattr(guard.os, 'sched_setaffinity', set_mask)
    guard.probe({4,5,6,7},3)
    assert masks == [{4,5,6,7},{3},{0,1}]


def test_real_child_expands_inherited_mask_before_exec():
    allowed = sorted(os.sched_getaffinity(0))
    if len(allowed)<2: pytest.skip('requires two allowed CPUs')
    cpus = ','.join(map(str, allowed[:2]))
    code = "import os,sys; os.sched_setaffinity(0,{int(sys.argv[1])}); os.execv(sys.executable,[sys.executable,*sys.argv[2:]])"
    command = [sys.executable,'-c',code,str(allowed[0]),str(SCRIPT),'--cpus',cpus,'--',
               sys.executable,'-c','import os,json; print(json.dumps(sorted(os.sched_getaffinity(0))))']
    result = subprocess.run(command,capture_output=True,text=True,timeout=10,check=True)
    assert json.loads(result.stdout.splitlines()[-1]) == allowed[:2]
    assert sorted(os.sched_getaffinity(0)) == allowed


def test_sdk_threads_are_probed_outside_process_mask_and_restored(monkeypatch):
    state = {0, 1}; masks = []
    def set_mask(_, mask): masks.append(set(mask)); state.clear(); state.update(mask)
    monkeypatch.setattr(guard.os, 'sched_getaffinity', lambda _: set(state))
    monkeypatch.setattr(guard.os, 'sched_setaffinity', set_mask)
    report = guard.probe({4, 5, 6, 7}, 3, [2, 7])
    assert masks == [{4, 5, 6, 7}, {3}, {2}, {7}, {0, 1}]
    assert report['sdk_cpus'] == [2, 7] and state == {0, 1}
