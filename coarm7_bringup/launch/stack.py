"""Shared CoArm7 ROS nodes for offline and hardware launch entry points."""
from pathlib import Path
import runpy

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_model = runpy.run_path(str(Path(__file__).with_name('model.py')))
description = _model['description']
HARDWARE_COMPONENT = _model['HARDWARE_COMPONENT']

CONTROLLERS = (
    'joint_state_broadcaster', 'left_arm_controller', 'right_arm_controller',
    'left_gripper_controller', 'right_gripper_controller',
)


def control_nodes(robot, controllers_file, *, start_unconfigured=False, control_cpu=None,
                  remote_env=None, hardware_component=HARDWARE_COMPONENT):
    remote_env = remote_env or {}
    parameters = [robot, str(controllers_file)]
    if start_unconfigured:
        parameters.append({
            'hardware_components_initial_state.unconfigured': [hardware_component],
        })
    if control_cpu is not None:
        parameters.append({'cpu_affinity': control_cpu})
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[robot],
             additional_env=remote_env.get('robot_state_publisher')),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=[
            '--frame-id', 'world', '--child-frame-id', 'coarm7_root'],
             additional_env=remote_env.get('static_transform_publisher')),
        Node(package='controller_manager', executable='ros2_control_node',
             output='both' if start_unconfigured else 'screen', parameters=parameters,
             additional_env=remote_env.get('ros2_control_node')),
    ]
    if not start_unconfigured:
        nodes.extend(Node(package='controller_manager', executable='spawner', arguments=[
            name, '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60'])
            for name in CONTROLLERS)
    return nodes


def moveit_parameters(robot):
    moveit = Path(get_package_share_directory('coarm7_moveit_config')) / 'config'

    def load(name):
        return yaml.safe_load((moveit / name).read_text())

    limits = load('joint_limits.yaml')
    effective = runpy.run_path(str(Path(__file__).with_name('effective_limits.py')))
    for name, bounds in effective['model_position_parameters'](robot).items():
        limits['joint_limits'].setdefault(name, {}).update(bounds)
    return [
        robot, {'robot_description_semantic': (moveit / 'coarm7.srdf').read_text()},
        {'robot_description_kinematics': load('kinematics.yaml')},
        {'robot_description_planning': limits},
        {'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
         'ompl': load('ompl_planning.yaml')},
    ]
def rviz_node(parameters, **kwargs):
    config = Path(get_package_share_directory('coarm7_bringup')) / 'rviz/coarm7.rviz'
    return Node(package='rviz2', executable='rviz2', arguments=['-d', str(config)],
                parameters=parameters, **kwargs)


def moveit_nodes(robot, remote_env=None):
    remote_env = remote_env or {}
    common = moveit_parameters(robot)
    config = Path(get_package_share_directory('coarm7_moveit_config')) / 'config'
    controllers = yaml.safe_load((config / 'moveit_controllers.yaml').read_text())
    return [
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             additional_env=remote_env.get('move_group'),
             parameters=common + [controllers, {
                 'allow_trajectory_execution': True, 'publish_robot_description_semantic': True,
                 'publish_planning_scene': True, 'publish_geometry_updates': True,
                 'publish_state_updates': True, 'publish_transforms_updates': True, 'use_sim_time': False}]),
        rviz_node(common, condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def gripper_trace_node(config_file, base_directory, control_cpu=None):
    """Passive observer; never publishes control requests or opens CAN."""
    import datetime
    import tempfile
    config_file = Path(config_file)
    config = yaml.safe_load(config_file.read_text())
    directory = Path(base_directory)
    directory.mkdir(parents=True, exist_ok=True)
    directory = tempfile.mkdtemp(
        prefix='coarm7-gripper-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S-'),
        dir=str(directory))
    avoid = {int((config.get('sdk_tx') or {}).get('cpu', -1)),
             int((config.get('sdk_rx') or {}).get('cpu', -1))}
    avoid.discard(-1)
    if control_cpu is not None and control_cpu >= 0:
        avoid.add(control_cpu)
    return Node(package='coarm7_diagnostics', executable='gripper_trace.py',
                arguments=['--directory', directory, '--config', str(config_file),
                           '--avoid-cpu', *[str(cpu) for cpu in sorted(avoid)]],
                output='log')
