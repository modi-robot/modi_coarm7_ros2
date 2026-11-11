"""SDK-free GenericSystem demo using the common project node assembly."""
from pathlib import Path
import runpy
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    pkg = Path(get_package_share_directory('coarm7_bringup'))
    stack = runpy.run_path(str(pkg / 'launch/stack.py'))
    source = Path(get_package_share_directory('coarm7_description')) / 'urdf/coarm7.urdf.xacro'
    robot = {'robot_description': xacro.process_file(str(source)).toxml()}
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        *stack['control_nodes'](robot, pkg / 'config/ros2_controllers.yaml'),
        *stack['moveit_nodes'](robot),
    ])
