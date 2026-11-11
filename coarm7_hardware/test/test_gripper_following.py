"""Gripper-only tracking model and durable passive trace regression checks."""
import importlib.util
import math
from pathlib import Path
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
LIMITS = yaml.safe_load((ROOT.parent / 'coarm7_moveit_config/config/joint_limits.yaml').read_text())
spec = importlib.util.spec_from_file_location('gripper_trace', ROOT.parent / 'coarm7_diagnostics/scripts/gripper_trace.py')
trace_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trace_module)


def trapezoid(distance, velocity, acceleration):
    ramp = min(velocity / acceleration, math.sqrt(distance / acceleration))
    peak = acceleration * ramp
    cruise = max(0.0, (distance - peak * ramp) / peak)
    duration = 2 * ramp + cruise
    def position(t):
        if t <= 0: return 0.0
        if t < ramp: return 0.5 * acceleration * t * t
        if t < ramp + cruise: return 0.5 * acceleration * ramp * ramp + peak * (t-ramp)
        if t < duration: return distance - 0.5 * acceleration * (duration-t)**2
        return distance
    return position, duration


def run_plant(velocity, acceleration, distance=0.85536, direction=1):
    # Model bounds, not a claim of calibrated physical dynamics: actuator 0.18
    # rad/s, commands 20 Hz, feedback 20 Hz and 100 ms aggregate observation delay.
    # The old 0.5/0.3 profile must reproduce abort + measured-hold reversal.
    dt = 0.005
    reference, duration = trapezoid(distance, velocity, acceleration)
    start = 0.0 if direction == 1 else distance
    position = target = feedback = start
    history = [(-1.0, start), (0.0, start)]
    observed = [(0.0, start)]
    aborted = False
    hold = None
    maximum_error = 0.0
    path = []
    for tick in range(1, int((duration + 2.0) / dt) + 1):
        now = tick * dt
        requested = start + direction * reference(now) if hold is None else hold
        if tick % 10 == 0:
            feedback = next(q for t, q in reversed(history) if t <= now - 0.1 + 1e-10)
            observed.append((now, feedback))
        error = abs(requested - feedback)
        maximum_error = max(maximum_error, error)
        if hold is None and error > 0.15:
            aborted = True
            hold = feedback
            requested = hold
        if tick % 10 == 0:
            target = requested
        position += max(-0.18 * dt, min(0.18 * dt, target-position))
        history.append((now, position))
        path.append(position)
    return {'aborted': aborted, 'error': maximum_error, 'path': path,
            'goal': start + direction * distance, 'duration': duration}


def test_old_profile_reproduces_following_abort_and_rebound():
    result = run_plant(0.5, 0.6 * 0.5)
    assert result['aborted']
    assert max(result['path']) - result['path'][-1] > 0.005


@pytest.mark.parametrize('side', ['L', 'R'])
@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('distance', [0.12, 0.5, 0.85536])
def test_matched_profile_completes_without_abort_or_direction_reversal(side, direction, distance):
    limits = LIMITS['joint_limits'][side + '_4C2_r_1_joint']
    result = run_plant(limits['max_velocity'], limits['max_acceleration'] * 0.5,
                       distance, direction)
    assert not result['aborted']
    assert result['error'] < 0.04
    assert abs(result['path'][-1] - result['goal']) < 0.01
    assert all(direction * (b-a) >= -1e-10 for a, b in zip(result['path'], result['path'][1:]))


def test_master_and_mimic_gripper_limits_match_and_tolerances_remain_enabled():
    grippers = [v for name, v in LIMITS['joint_limits'].items() if '_4C2_' in name]
    assert len(grippers) == 12
    assert all(v['max_velocity'] == 0.12 and v['max_acceleration'] == 0.12 for v in grippers)
    controllers = yaml.safe_load((ROOT.parent / 'coarm7_bringup/config/controllers_driver.yaml').read_text())
    for side, name in [('L', 'left'), ('R', 'right')]:
        parameters = controllers[name + '_gripper_controller']['ros__parameters']
        assert parameters['goal_tolerance'] == 0.01
        assert parameters['feedback_timeout'] == 0.5
        assert parameters['stall_timeout'] > parameters['feedback_timeout']


def sample(t, controller='right_gripper_controller', reference=0.5, feedback=0.3):
    return {'controller': controller, 'received_monotonic_s': t,
            'reference_position_rad': [reference], 'feedback_position_rad': [feedback]}


def test_abort_window_keeps_before_and_after_reference_and_feedback():
    trace = trace_module.TraceWindows()
    for i in range(61): trace.sample(sample(i / 10))
    trace.status('right_gripper_controller', 'goal', 2, 5.9, 100)
    event = trace.status('right_gripper_controller', 'goal', 6, 6.0, 200)
    assert event['previous_status'] == 2
    # Capture the post-abort reference jump separately from continued feedback.
    trace.sample(sample(6.1, reference=0.3, feedback=0.36))
    trace.sample(sample(8.9, reference=0.3, feedback=0.3))
    assert trace.ready(8.99) == []
    windows = trace.ready(9.0)
    assert len(windows) == 1
    assert windows[0]['samples'][0]['received_monotonic_s'] == 4.0
    assert windows[0]['samples'][-2]['reference_position_rad'] == [0.3]
    assert windows[0]['samples'][-2]['feedback_position_rad'] == [0.36]
    assert windows[0]['post_window_complete']


def test_repeated_terminal_status_does_not_retrigger_and_goals_are_independent():
    trace = trace_module.TraceWindows()
    trace.sample(sample(1))
    assert trace.status('right_gripper_controller', 'a', 6, 2, 20)
    assert trace.status('right_gripper_controller', 'a', 6, 2.5, 25) is None
    assert trace.status('left_gripper_controller', 'b', 5, 3, 30)
    windows = trace.ready(6)
    assert len(windows) == 2
    assert {w['event']['status_name'] for w in windows} == {'aborted', 'canceled'}


def test_trace_memory_is_bounded_and_shutdown_marks_incomplete_post_window():
    trace = trace_module.TraceWindows(capacity=10, pending_limit=1)
    for i in range(100): trace.sample(sample(i / 100))
    assert len(trace.history) == 10
    trace.status('right_gripper_controller', 'a', 6, 1, 10)
    event = trace.status('left_gripper_controller', 'b', 6, 1.1, 11)
    assert event['window_skipped']
    assert trace.overflow == 1
    windows = trace.ready(1.2, shutdown=True)
    assert not windows[0]['post_window_complete']


def test_saved_fault_window_survives_later_activity_and_retention_never_deletes(tmp_path):
    writer = trace_module.WindowWriter(tmp_path, maximum=1, free_bytes=0)
    trace = trace_module.TraceWindows()
    trace.sample(sample(1))
    trace.status('right_gripper_controller', 'a', 6, 2, 20)
    window = trace.ready(5)[0]
    assert writer.write(window)
    path = next(tmp_path.glob('*.json'))
    original = path.read_bytes()
    trace.status('left_gripper_controller', 'b', 5, 6, 60)
    assert not writer.write(trace.ready(9)[0])
    assert path.read_bytes() == original
    assert len(list(tmp_path.glob('*.json'))) == 1
    writer.close()


def test_state_alias_prefers_populated_reference_or_legacy_fields():
    from types import SimpleNamespace
    full = SimpleNamespace(positions=[0.3])
    empty = SimpleNamespace(positions=[])
    old = SimpleNamespace(reference=empty, desired=full, feedback=empty, actual=full)
    new = SimpleNamespace(reference=full, desired=empty, feedback=full, actual=empty)
    for msg in (old, new):
        assert trace_module.trajectory_point(msg, 'reference', 'desired') is full
        assert trace_module.trajectory_point(msg, 'feedback', 'actual') is full


def test_state_alias_rejects_empty_points():
    from types import SimpleNamespace
    assert trace_module.trajectory_point(SimpleNamespace(), 'reference', 'desired') is None
    assert trace_module.trajectory_point(SimpleNamespace(reference=SimpleNamespace(positions=[])),
                                  'reference', 'desired') is None
