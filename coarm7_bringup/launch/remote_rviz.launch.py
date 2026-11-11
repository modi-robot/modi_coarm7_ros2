"""Start only RViz in the VM and keep its DDS links inside an SSH tunnel."""
from pathlib import Path
import re
import json
import os
import sys
import runpy
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def setup(context):
    target = LaunchConfiguration('controller').perform(context)
    domain = LaunchConfiguration('domain_id').perform(context)
    if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]*', target):
        raise RuntimeError('controller must be an SSH target such as user@controller')
    if not domain.isdecimal() or int(domain) > 232:
        raise RuntimeError('domain_id must be a ROS domain number between 0 and 232')

    pkg = Path(get_package_share_directory('coarm7_bringup'))
    stack = runpy.run_path(str(pkg / 'launch/stack.py'))
    transport = runpy.run_path(str(pkg / 'launch/rviz_transport.py'))
    directory = tempfile.TemporaryDirectory(prefix='coarm7-rviz-client-', dir='/tmp')
    socket = Path(directory.name) / 'ssh'
    ssh = ['ssh', '-S', str(socket)]
    forwards = []
    for port in transport['PORTS'].values():
        forwards.extend(['-L', f'127.0.0.1:{port}:127.0.0.1:{port}'])

    def cleanup(*_):
        if socket.exists():
            subprocess.run([*ssh, '-O', 'exit', '--', target], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
        directory.cleanup()

    try:
        subprocess.run([*ssh, '-M', '-fNT', '-o', 'Compression=yes',
                        '-o', 'ExitOnForwardFailure=yes',
                        '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=10',
                        '-o', 'ServerAliveCountMax=3', *forwards, '--', target], check=True)
        sockets = subprocess.run([*ssh, '-o', 'BatchMode=yes', '--', target, 'ss -H -ltn'],
                                 check=True, capture_output=True, text=True, timeout=8)
        listeners = {line.split()[3] for line in sockets.stdout.splitlines()
                     if len(line.split()) >= 5}
        missing = [port for port in transport['PORTS'].values()
                   if f'127.0.0.1:{port}' not in listeners]
        if missing:
            raise RuntimeError(f'Controller RViz DDS listeners are missing on {missing}. '
                               'Start CoArm7 with remote_rviz:=true and wait for configure / effective limit synchronization.')
        env = transport['profile_environment'](Path(directory.name) / 'client.xml')
        env['ROS_DOMAIN_ID'] = domain
        model_file = Path(directory.name) / 'live_model.json'
        helper = Path(get_package_prefix('coarm7_bringup')) / 'lib/coarm7_bringup/fetch_robot_model.py'
        subprocess.run([sys.executable, str(helper), '--output', str(model_file)],
                       env={**os.environ, **env}, check=True, timeout=30)
        live = json.loads(model_file.read_text())
        parameters = stack['moveit_parameters']({'robot_description': live['robot_description']})
        parameters[1]['robot_description_semantic'] = live['robot_description_semantic']
        rviz = stack['rviz_node'](
            parameters, additional_env=env, output='screen',
            on_exit=[EmitEvent(event=Shutdown(reason='RViz exited'))],
        )
        return [RegisterEventHandler(OnShutdown(on_shutdown=cleanup)), rviz]
    except BaseException:
        cleanup()
        raise


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', description='SSH target of the CoArm7 controller'),
        DeclareLaunchArgument('domain_id', default_value='0',
                              description='ROS domain used by the controller Run Service'),
        OpaqueFunction(function=setup),
    ])
