"""200 Hz SDK-adapter offline stack without MoveIt or RViz."""
from pathlib import Path
import runpy

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription


def generate_launch_description():
    pkg = Path(get_package_share_directory('coarm7_bringup'))
    hardware = Path(get_package_share_directory('coarm7_hardware'))
    stack = runpy.run_path(str(pkg / 'launch/stack.py'))
    robot = stack['description'](hardware / 'config/offline.yaml')
    return LaunchDescription(stack['control_nodes'](robot, pkg / 'config/controllers_driver.yaml'))
