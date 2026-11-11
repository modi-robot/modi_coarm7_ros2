"""MoveIt Plan & Execute through CoArm7System with the fake transport only."""
from pathlib import Path
import runpy
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration


def setup(context):
    pkg = Path(get_package_share_directory('coarm7_bringup'))
    hardware = Path(get_package_share_directory('coarm7_hardware'))
    stack = runpy.run_path(str(pkg / 'launch/stack.py'))
    config_file = Path(LaunchConfiguration('config_file').perform(context)).expanduser().resolve(strict=True)
    robot = stack['description'](config_file)  # description rejects any non-fake transport
    staged = IfCondition(LaunchConfiguration('staged')).evaluate(context)
    control_cpu = int(LaunchConfiguration('control_cpu').perform(context))
    if control_cpu < -1 or (control_cpu >= 0 and not Path(f'/sys/devices/system/cpu/cpu{control_cpu}').is_dir()):
        raise RuntimeError('control_cpu must be -1 or an existing CPU')
    remote_env = None
    cleanup = []
    if IfCondition(LaunchConfiguration('remote_rviz')).evaluate(context):
        transport = runpy.run_path(str(pkg / 'launch/rviz_transport.py'))
        profiles = tempfile.TemporaryDirectory(prefix='coarm7-rviz-server-')
        remote_env = transport['server_environment'](Path(profiles.name))
        cleanup.append(RegisterEventHandler(OnShutdown(
            on_shutdown=lambda *_: profiles.cleanup())))
    trace = []
    if IfCondition(LaunchConfiguration('gripper_trace')).evaluate(context):
        directory = LaunchConfiguration('diagnostics_dir').perform(context)
        trace = [stack['gripper_trace_node'](config_file, Path(directory) if directory else Path.home() / '.ros/log', control_cpu)]
    return [
        *cleanup,
        *stack['control_nodes'](
            robot, pkg / 'config/controllers_driver.yaml', remote_env=remote_env,
            start_unconfigured=staged, control_cpu=control_cpu if control_cpu >= 0 else None,
            hardware_component='CoArm7HardwareOffline'),
        *stack['moveit_nodes'](robot, remote_env=remote_env),
        *trace,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=str(Path(get_package_share_directory('coarm7_hardware')) / 'config/offline.yaml')),
        DeclareLaunchArgument('staged', default_value='false', description='Let the project startup supervisor manage the fake lifecycle'),
        DeclareLaunchArgument('control_cpu', default_value='-1'),
        DeclareLaunchArgument('gripper_trace', default_value='false'),
        DeclareLaunchArgument('diagnostics_dir', default_value=''),
        DeclareLaunchArgument('remote_rviz', default_value='false',
                              description='Enable loopback DDS listeners for an SSH-tunneled RViz'),
        OpaqueFunction(function=setup),
    ])
