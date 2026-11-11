"""Startup-only tests. No CAN sockets, SDK calls, ROS nodes or real processes."""
import copy
import importlib.util
from pathlib import Path
import signal
import subprocess
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2] / 'coarm7_hardware'
spec = importlib.util.spec_from_file_location('coarm7_startup', ROOT.parent / 'coarm7_bringup/scripts/start_coarm7.py')
startup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(startup)


@pytest.fixture
def fixture(tmp_path):
    config = yaml.safe_load((ROOT / 'config/offline.yaml').read_text())
    path = tmp_path / 'robot.yaml'
    path.write_text(yaml.safe_dump(config))
    return config, path


def options(fixture, *extra, hardware=False):
    config, path = fixture
    if hardware:
        config.update(transport='sdk', commissioning_confirmed=True, deactivate_policy='disable', gripper_read_only=False)
        path.write_text(yaml.safe_dump(config))
    return startup.resolve_options(['--mode', 'hardware' if hardware else 'fake', '--config', str(path), *extra], ROOT)


def test_defaults_are_fake_and_diagnostics_are_optional():
    args, config = startup.resolve_options(['--no-rviz'], ROOT)
    assert args.mode == 'fake' and config['transport'] == 'fake'
    assert args.activate and args.start_controllers
    assert not any([args.initialize_can, args.can_capture, args.usb_capture, args.gripper_trace])
    assert args.control_cpu == -1 and args.process_cpus == ''


@pytest.mark.parametrize('flag', ['--initialize-can', '--can-capture', '--usb-capture'])
def test_fake_rejects_all_can_and_usb_access(fixture, flag):
    with pytest.raises(ValueError, match='Fake mode cannot'):
        options(fixture, flag)


def test_fake_rejects_hardware_config_even_when_reviewed(fixture):
    config, path = fixture
    config.update(transport='sdk', commissioning_confirmed=True)
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match='transport'):
        options(fixture)


@pytest.mark.parametrize('changes', [dict(commissioning_confirmed=False), dict(deactivate_policy='pending'), dict(gripper_read_only='false')])
def test_hardware_requires_reviewed_typed_configuration(fixture, changes):
    config, path = fixture
    config.update(transport='sdk', commissioning_confirmed=True, deactivate_policy='disable', gripper_read_only=False)
    config.update(changes); path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError):
        startup.resolve_options(['--mode', 'hardware', '--config', str(path)], ROOT)


def test_existing_config_can_hold_optional_runtime_and_diagnostic_sections(fixture):
    config, path = fixture
    config['startup'] = dict(control_cpu=-1, remote_rviz=True, rviz=False)
    config['diagnostics'] = dict(gripper_trace=True, directory='logs')
    path.write_text(yaml.safe_dump(config))
    args, _ = options(fixture, '--rviz', '--no-gripper-trace')
    assert args.rviz and args.remote_rviz and not args.gripper_trace
    assert args.diagnostics_dir == path.parent / 'logs'


@pytest.mark.parametrize('value', ['false', 1, None])
def test_yaml_boolean_options_are_strictly_typed(fixture, value):
    config, path = fixture
    config['diagnostics'] = dict(can_capture=value)
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match='boolean'): options(fixture)


def ready_link():
    return dict(ifname='can0', flags=['UP'], mtu=72, txqlen=64,
                linkinfo=dict(info_kind='can', info_data=dict(state='ERROR-ACTIVE', restart_ms=1000,
                              bittiming=dict(bitrate=1000000), data_bittiming=dict(bitrate=5000000))))


def test_can_ready_check_accepts_existing_profile_and_rejects_active_mismatch(fixture):
    args, _ = options(fixture)
    link = ready_link()
    assert startup.can_state(link, 'can0', args.can_settings) == 'ready'
    link['linkinfo']['info_data']['bittiming']['bitrate'] = 500000
    with pytest.raises(ValueError, match='Refusing to reset'):
        startup.can_state(link, 'can0', args.can_settings)


def test_both_buses_are_checked_before_any_interface_changes(fixture):
    args, config = options(fixture, '--initialize-can', hardware=True)
    session = startup.Session(args, config, ROOT)
    session.inspect_can = Mock(side_effect=['down', ValueError('can1 mismatched')])
    session.command = Mock()
    with pytest.raises(ValueError): session.initialize_can()
    session.command.assert_not_called()


def test_can_initialization_only_raises_down_interfaces_and_never_takes_bus_down(fixture):
    args, config = options(fixture, '--initialize-can', hardware=True)
    session = startup.Session(args, config, ROOT)
    session.handover = Mock()
    session.inspect_can = Mock(side_effect=['down', 'ready', 'down', 'ready', 'ready', 'ready'])
    session.command = Mock()
    session.initialize_can()
    commands = [call.args[0] for call in session.command.call_args_list]
    assert len(commands) == 4 and commands[0] == ['sudo', '-v']
    assert all('down' not in command for command in commands)
    assert all('can1' not in command for command in commands)
    assert 'up' in commands[-1]


def test_ready_can_requires_neither_reset_nor_sudo_without_usb(fixture):
    args, config = options(fixture, hardware=True)
    session = startup.Session(args, config, ROOT)
    session.inspect_can = Mock(return_value='ready'); session.command = Mock(); session.handover = Mock()
    session.initialize_can()
    session.command.assert_not_called()


def test_down_can_without_explicit_initialization_fails_without_writes(fixture):
    args, config = options(fixture, hardware=True)
    session = startup.Session(args, config, ROOT)
    session.inspect_can = Mock(return_value='down'); session.command = Mock()
    with pytest.raises(RuntimeError, match='explicitly'): session.initialize_can()
    session.command.assert_not_called()


def lifecycle(fixture, *, read_only=False, extra=()):
    args, config = options(fixture, *extra, hardware=True)
    config['gripper_read_only'] = read_only
    session = startup.Session(args, config, ROOT)
    session.command = Mock(); session.state = Mock(); session.handover = Mock(); session.ready_can = Mock(); session.wait_planning_limits = Mock()
    return session


def test_lifecycle_order_and_both_arm_and_gripper_controllers(fixture):
    session = lifecycle(fixture)
    session.lifecycle()
    commands = [call.args[0] for call in session.command.call_args_list]
    assert '--configure' in commands[0]
    assert 'joint_state_broadcaster' in commands[1]
    assert '--activate' in commands[2]
    assert commands[3][4:8] == ['left_arm_controller', 'right_arm_controller', 'left_gripper_controller', 'right_gripper_controller']
    assert '--activate-as-group' in commands[3]
    assert [c.args[0] for c in session.state.call_args_list] == ['inactive', 'inactive', 'active', 'active']
    assert all('send_goal' not in c and 'pub' not in c and 'clear_drive_faults' not in c for c in commands)


def test_read_only_grippers_are_not_started(fixture):
    session = lifecycle(fixture, read_only=True); session.lifecycle()
    command = session.command.call_args_list[-1].args[0]
    assert 'left_arm_controller' in command and 'right_arm_controller' in command
    assert not any('gripper_controller' in part for part in command)


@pytest.mark.parametrize('flag', ['--no-activate', '--no-start-controllers'])
def test_explicit_feedback_or_hold_mode_does_not_start_motion_controllers(fixture, flag):
    session = lifecycle(fixture, extra=[flag]); session.lifecycle()
    commands = [call.args[0] for call in session.command.call_args_list]
    assert not any('left_arm_controller' in c for c in commands)
    assert any('--activate' in c for c in commands) == (flag == '--no-start-controllers')


def test_configure_or_feedback_failure_never_proceeds_to_activation(fixture):
    session = lifecycle(fixture)
    session.command.side_effect = subprocess.CalledProcessError(1, 'configure')
    with pytest.raises(subprocess.CalledProcessError): session.lifecycle()
    assert session.command.call_count == 1
    session = lifecycle(fixture); session.state.side_effect = RuntimeError('bad feedback')
    with pytest.raises(RuntimeError): session.lifecycle()
    assert session.command.call_count == 1


def test_controller_manager_collision_is_rejected_before_can_access(fixture):
    args, config = options(fixture, hardware=True)
    session = startup.Session(args, config, ROOT)
    session.handover = Mock(); session.command = Mock(return_value=SimpleNamespace(stdout='/controller_manager/list_hardware_components\n'))
    session.initialize_can = Mock(); session.cleanup = Mock()
    try:
        with pytest.raises(RuntimeError, match='already running'): session.run()
        session.initialize_can.assert_not_called()
    finally:
        if session.lock is not None: import os; os.close(session.lock)


def test_legacy_process_is_only_a_configurable_handover_guard(fixture):
    args, config = options(fixture, '--conflicting-process', 'modi_system_node', hardware=True)
    session = startup.Session(args, config, ROOT)
    session.command = Mock(return_value=SimpleNamespace(stdout='modi_system_nod\n'))
    with pytest.raises(RuntimeError, match='handover'): session.handover()
    assert session.command.call_args.args[0] == ['ps', '-eo', 'comm=']


def test_diagnostic_failure_policy_is_independent_and_explicit(fixture):
    args, config = options(fixture)
    session = startup.Session(args, config, ROOT)
    session.diagnostic_error('usb_capture', RuntimeError('not installed'))
    assert not args.usb_capture
    args.diagnostics_required = True
    with pytest.raises(RuntimeError, match='not installed'):
        session.diagnostic_error('usb_capture', RuntimeError('not installed'))


def test_capture_filters_derive_tool_reply_ids_from_config(fixture):
    _, config = options(fixture)
    assert startup.capture_filters(config) == ['can0,03F:C00007FF,080:C00007C0,628:C00007FF,#1FFFFFFF',
                                             'can1,03F:C00007FF,080:C00007C0,629:C00007FF,#1FFFFFFF']


def test_launch_command_preserves_explicit_affinity_and_independent_package(fixture):
    args, config = options(fixture, '--remote-rviz', '--no-rviz', '--control-cpu', '-1', hardware=True)
    args.process_cpus = '4-7'; args.control_cpu = 3
    session = startup.Session(args, config, ROOT)
    command = session.launch_command()
    assert command[1].endswith('affinity_guard.py')
    assert command[2:6] == ['--cpus', '4-7', '--control-cpu', '3']
    assert '--' in command
    assert 'hardware.launch.py' in command and 'control_cpu:=3' in command
    assert not any('modi_tools' in item or 'modi_robot' in item for item in command)


def test_fake_launch_uses_staged_fake_component_without_hardware_launch(fixture):
    args, config = options(fixture)
    session = startup.Session(args, config, ROOT)
    assert session.component == 'CoArm7HardwareOffline'
    assert 'offline.launch.py' in session.launch_command()
    assert 'staged:=true' in session.launch_command()


def test_cleanup_keeps_capture_until_control_exit_and_releases_lock(fixture):
    args, config = options(fixture)
    session = startup.Session(args, config, ROOT)
    session.launch = Mock(); capture = Mock(); session.capture = [capture]
    session.stop = Mock(); session.cleanup()
    assert session.stop.call_args_list[0].args == (session.launch, signal.SIGINT, 30)
    assert session.stop.call_args_list[1].args == (capture, signal.SIGTERM, 8)


def test_fake_default_does_not_require_can_tools_or_sudo(fixture, monkeypatch):
    args, config = options(fixture)
    session = startup.Session(args, config, ROOT)
    session.command = Mock()
    monkeypatch.setattr(startup.shutil, 'which', lambda name: (_ for _ in ()).throw(AssertionError('tool probe')))
    session.diagnostic_preflight()
    session.command.assert_not_called()


def test_no_hidden_automatic_fault_clear_or_reset_entry():
    script = (ROOT.parent / 'coarm7_bringup/scripts/start_coarm7.py').read_text()
    assert 'clear_drive_faults' not in script
    assert "'down']" not in script
    assert 'pkill' not in script.replace('never global pkill/drive reset.', '')


def test_optional_recording_directory_error_does_not_block_normal_startup(fixture, monkeypatch):
    args, config = options(fixture, '--gripper-trace')
    session = startup.Session(args, config, ROOT)
    session.diagnostic_script = Mock(return_value=Path(__file__))
    def denied(*args, **kwargs): raise PermissionError('read-only directory')
    with monkeypatch.context() as scoped:
        scoped.setattr(startup.Path, 'mkdir', denied)
        session.diagnostic_preflight()
    assert not args.gripper_trace


def test_online_isolated_cpus_not_rejected_by_inherited_affinity(fixture, monkeypatch):
    monkeypatch.setattr(startup, 'online_cpus', lambda: set(range(8)))
    monkeypatch.setattr(startup.os, 'sched_getaffinity', lambda _: set(range(6)))
    args, _ = options(fixture, '--process-cpus', '4-7', '--control-cpu', '3')
    assert args.process_cpus == '4-7'


@pytest.mark.parametrize('flags', [('--process-cpus', '6-7'), ('--control-cpu', '7')])
def test_offline_cpu_is_still_rejected(fixture, monkeypatch, flags):
    monkeypatch.setattr(startup, 'online_cpus', lambda: set(range(6)))
    with pytest.raises(ValueError, match='offline or absent'): options(fixture, *flags)


def test_affinity_rejection_happens_before_can_or_launch(fixture):
    args, config = options(fixture, hardware=True)
    session = startup.Session(args, config, ROOT)
    session.handover = Mock()
    session.command = Mock(return_value=SimpleNamespace(stdout=''))
    session.affinity_preflight = Mock(side_effect=RuntimeError('binding rejected'))
    session.initialize_can = Mock(); session.diagnostic_preflight = Mock(); session.lifecycle = Mock()
    with pytest.raises(RuntimeError, match='binding rejected'): session.run()
    session.initialize_can.assert_not_called()
    session.diagnostic_preflight.assert_not_called()
    session.lifecycle.assert_not_called()
    assert session.launch is None


def test_preflight_preserves_kernel_error_detail(fixture):
    args, config = options(fixture)
    args.process_cpus = '4-7'
    session = startup.Session(args, config, ROOT)
    session.command = Mock(side_effect=subprocess.CalledProcessError(1, 'probe', stderr='requested [4,5,6,7], actual [4,5]'))
    with pytest.raises(RuntimeError, match=r'actual \[4,5\]'): session.affinity_preflight()


def test_default_affinity_never_spawns_a_binding_probe(fixture):
    args, config = options(fixture)
    session = startup.Session(args, config, ROOT)
    session.command = Mock()
    session.affinity_preflight()
    session.command.assert_not_called()


def test_usb_auth_failure_stops_before_ros_launch_or_can_access(fixture, monkeypatch):
    args, config = options(fixture, '--usb-capture', '--diagnostics-required', hardware=True)
    session = startup.Session(args, config, ROOT)
    session.handover = Mock(); session.affinity_preflight = Mock()
    session.command = Mock(return_value=SimpleNamespace(stdout=''))
    session.diagnostic_script = Mock(return_value=Path('/unused'))
    session.initialize_can = Mock(); session.lifecycle = Mock()
    module = {'resolve_adapter': lambda **kw: (1, 5), 'passive_cpu': lambda *a: 0,
              'open_usbmon': Mock(side_effect=RuntimeError('authorization denied'))}
    monkeypatch.setattr(startup.runpy, 'run_path', lambda path: module)
    spawn = Mock(side_effect=AssertionError('No background or ROS process may start'))
    monkeypatch.setattr(startup.subprocess, 'Popen', spawn)
    with pytest.raises(RuntimeError, match='authorization denied'): session.run()
    session.initialize_can.assert_not_called(); session.lifecycle.assert_not_called(); spawn.assert_not_called()


def test_sdk_cpu_is_checked_against_online_before_hardware(fixture, monkeypatch):
    config, path = fixture
    config['sdk_tx'] = dict(cpu=7, priority=90)
    monkeypatch.setattr(startup, 'online_cpus', lambda: {0, 1, 2, 3})
    with pytest.raises(ValueError, match='sdk_tx.cpu'): options(fixture, hardware=True)


def test_reviewed_cpu_policy_and_period_are_preserved(fixture, monkeypatch):
    config, path = fixture
    config['sdk_rx'] = dict(cpu=2, priority=90)
    config['sdk_tx'] = dict(cpu=7, priority=90)
    monkeypatch.setattr(startup, 'online_cpus', lambda: set(range(8)))
    args, loaded = options(fixture, '--process-cpus', '4-7', '--control-cpu', '3', hardware=True)
    session = startup.Session(args, loaded, ROOT)
    assert args.sdk_cpus == [2, 7] and args.process_cpus == '4-7' and args.control_cpu == 3
    assert session.usb_avoid_cpus() == [2, 3, 7]
    assert loaded['timing']['period_us'] == 5000
    assert session.affinity_command()[-4:] == ['--probe-cpu', '2', '--probe-cpu', '7']


def test_failed_limit_ack_never_activates_hardware(fixture):
    session = lifecycle(fixture)
    session.wait_planning_limits.side_effect = RuntimeError('limit mismatch')
    with pytest.raises(RuntimeError, match='limit mismatch'): session.lifecycle()
    commands = [call.args[0] for call in session.command.call_args_list]
    assert len(commands) == 1 and '--configure' in commands[0]
    assert not any('--activate' in command for command in commands)
