"""CoArm7 MoveIt with an explicitly staged CanEvo hardware lifecycle."""
import atexit
import json
from pathlib import Path
import runpy
import select
import subprocess
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration


TIMESTAMP_HELPER = Path('/usr/local/sbin/coarm7-sync-timestamp-lease')


def start_timestamp_lease(control_node):
    logger = get_logger('coarm7_hardware')
    if not TIMESTAMP_HELPER.is_file():
        logger.warning('SYNC hardware TX auto-enable helper is absent. Collecting available timestamps.')
        return []
    try:
        process = subprocess.Popen(
            ['/usr/bin/sudo', '-n', str(TIMESTAMP_HELPER)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, start_new_session=True,
        )
    except OSError as error:
        logger.warning(f'Cannot start SYNC hardware TX helper. Collecting available timestamps: {error}')
        return []

    def release(*_):
        if process.stdin.closed:
            return
        process.stdin.close()
        try:
            result = process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            logger.error('SYNC hardware TX helper did not confirm restoration. Check CAN timestamp modes.')
            return
        if result == 0:
            logger.info('Original CAN hardware TX timestamp settings restored.')
        elif result != 1:
            logger.error('SYNC hardware TX helper could not verify restoration. Check CAN timestamp modes.')

    try:
        readable, _, _ = select.select([process.stdout], [], [], 3)
        ready = bool(readable) and process.stdout.read(1) == b'1'
    except BaseException:
        release()
        raise
    finally:
        process.stdout.close()

    if not ready:
        release()
        if not readable or process.returncode != 1:
            raise RuntimeError('SYNC hardware TX setup did not confirm a safe state')
        logger.warning('SYNC hardware TX helper unavailable. Collecting available timestamps.')
        return []

    atexit.register(release)
    logger.info('Hardware TX timestamp lease active on can0 and can1.')
    return [RegisterEventHandler(OnProcessExit(target_action=control_node, on_exit=release))]


def setup(context):
    pkg = Path(get_package_share_directory('coarm7_bringup'))
    hardware = Path(get_package_share_directory('coarm7_hardware'))
    stack = runpy.run_path(str(pkg / 'launch/stack.py'))
    config_file = Path(LaunchConfiguration('hardware_config').perform(context)).expanduser().resolve(strict=True)
    requested = LaunchConfiguration('limits_session_dir').perform(context)
    temporary = None if requested else tempfile.TemporaryDirectory(prefix='coarm7-limits-')
    session = Path(requested or temporary.name).resolve(strict=True)
    report_path, ready_path = session / 'effective_limits.yaml', session / 'planning_ready.json'
    if report_path.exists() or ready_path.exists():
        raise RuntimeError('Limit synchronization requires a fresh session directory')
    robot = stack['description'](config_file, transport='sdk', effective_limits_file=report_path)
    try:
        control_cpu = int(LaunchConfiguration('control_cpu').perform(context))
    except ValueError as exc:
        raise RuntimeError('control_cpu must be a CPU index or -1 for no pinning') from exc
    if control_cpu < -1 or (
        control_cpu >= 0 and not Path(f'/sys/devices/system/cpu/cpu{control_cpu}').is_dir()
    ):
        raise RuntimeError('control_cpu must name an existing CPU or be -1 for no pinning')
    remote_env = None
    cleanup = []
    if temporary:
        cleanup.append(RegisterEventHandler(OnShutdown(on_shutdown=lambda *_: temporary.cleanup())))
    if IfCondition(LaunchConfiguration('remote_rviz')).evaluate(context):
        transport = runpy.run_path(str(pkg / 'launch/rviz_transport.py'))
        profiles = tempfile.TemporaryDirectory(prefix='coarm7-rviz-server-')
        remote_env = transport['server_environment'](Path(profiles.name))
        cleanup.append(RegisterEventHandler(OnShutdown(
            on_shutdown=lambda *_: profiles.cleanup())))
        get_logger('coarm7_hardware').info('RViz DDS profiles prepared; MoveIt listener starts after effective limit synchronization')
    controls = stack['control_nodes'](
        robot, pkg / 'config/controllers_driver.yaml',
        start_unconfigured=True, control_cpu=control_cpu if control_cpu >= 0 else None,
        remote_env=remote_env,
    )
    limits = runpy.run_path(str(pkg / 'launch/effective_limits.py'))
    original_report = None

    def planning_gate(_context):
        nonlocal original_report
        try:
            if original_report is not None:
                if not report_path.exists() or report_path.read_bytes() != original_report:
                    raise RuntimeError('Hardware limits changed or were invalidated; restart the complete session')
                return [TimerAction(period=.5, actions=[OpaqueFunction(function=planning_gate)])]
            if not report_path.exists():
                return [TimerAction(period=.1, actions=[OpaqueFunction(function=planning_gate)])]
            report = limits['load_report'](report_path, config_file)
            planning_robot, applied = limits['apply_limits'](robot, report)
            nodes = stack['moveit_nodes'](planning_robot, remote_env=remote_env)
            original_report = report_path.read_bytes()
            ready_temp = ready_path.with_suffix('.tmp')
            ready_temp.write_text(json.dumps({'schema': 1, 'axes': applied, 'config_file': str(config_file)}))
            ready_temp.replace(ready_path)
            get_logger('coarm7_hardware').info('MoveIt arm limits synchronized with this configure session')
            return [*nodes, TimerAction(period=.5, actions=[OpaqueFunction(function=planning_gate)])]
        except (OSError, ValueError, RuntimeError, KeyError) as error:
            get_logger('coarm7_hardware').error('Planning limit synchronization failed: ' + str(error))
            return [EmitEvent(event=Shutdown(reason='Invalid effective planning limits'))]

    cleanup.append(RegisterEventHandler(OnProcessExit(target_action=controls[-1],
        on_exit=[EmitEvent(event=Shutdown(reason='Controller manager exited'))])))
    config = yaml.safe_load(config_file.read_text())
    tx_config = config.get('sdk_tx') or {}
    capabilities = yaml.safe_load((hardware / 'config/sdk_capabilities.yaml').read_text())
    if tx_config.get('timestamp_diagnostics', False) and not capabilities['extensions']:
        raise RuntimeError('Timestamp diagnostics require a development SDK-extension build; no helper was started')
    if tx_config.get('timestamp_diagnostics', False) and config['buses'] != ['can0', 'can1']:
        raise RuntimeError('The optional installed timestamp lease supports can0/can1 only; disable timestamp_diagnostics for other interfaces')
    # In staged hardware mode the controller manager is the last control node.
    lease = start_timestamp_lease(controls[-1]) if tx_config.get('timestamp_diagnostics', False) else []
    trace = []
    if IfCondition(LaunchConfiguration('gripper_trace')).evaluate(context):
        directory = LaunchConfiguration('diagnostics_dir').perform(context)
        trace = [stack['gripper_trace_node'](config_file, Path(directory) if directory else config_file.parent / 'diagnostics', control_cpu)]
    return [*cleanup, *lease, *controls, *trace, OpaqueFunction(function=planning_gate)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_config', description='Absolute path to a reviewed SDK configuration'),
        DeclareLaunchArgument('limits_session_dir', default_value='', description='Internal fresh session directory; normally created automatically'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gripper_trace', default_value='false', description='Enable passive detailed gripper recording'),
        DeclareLaunchArgument('diagnostics_dir', default_value='', description='Optional diagnostic output directory'),
        DeclareLaunchArgument('remote_rviz', default_value='false',
                              description='Enable loopback DDS listeners for an SSH-tunneled RViz'),
        DeclareLaunchArgument(
            'control_cpu', default_value='-1',
            description='CPU index for the controller_manager update thread, or -1 for no pinning',
        ),
        OpaqueFunction(function=setup),
    ])
