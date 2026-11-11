#!/usr/bin/env python3
"""Own one CoArm7 session: preflight, staged lifecycle, controllers and diagnostics.

Default mode is fake. Hardware mode requires an explicitly reviewed config.
This supervisor never publishes joint targets, resets drives or clears faults.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import runpy
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import yaml


def cpu_set(value):
    if not value:
        return set()
    cpus = set()
    for item in value.split(','):
        if not re.fullmatch(r'\d+(?:-\d+)?', item):
            raise ValueError('process_cpus must use CPU numbers/ranges, e.g. 4-7')
        pair = item.split('-'); first, last = int(pair[0]), int(pair[-1])
        if last < first or last > 4095:
            raise ValueError('Invalid process CPU range')
        cpus.update(range(first, last + 1))
    return cpus


def online_cpus():
    # Inherited affinity is not the permission ceiling: isolated CPUs may be
    # online and explicitly bindable even if the SSH parent does not use them.
    return cpu_set(Path('/sys/devices/system/cpu/online').read_text().strip())


def resolve_options(argv, share):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['fake', 'hardware'], default='fake')
    parser.add_argument('--hardware-config', '--config', dest='config', type=Path)
    parser.add_argument('--check-only', action='store_true', help='Validate config/commands only; no ROS nodes, CAN queries or sudo')
    for name in ['initialize-can', 'activate', 'start-controllers', 'rviz', 'remote-rviz',
                 'can-capture', 'usb-capture', 'gripper-trace', 'diagnostics-required']:
        parser.add_argument('--' + name, action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument('--control-cpu', type=int)
    parser.add_argument('--process-cpus')
    parser.add_argument('--domain-id', type=int)
    parser.add_argument('--diagnostics-dir', type=Path)
    parser.add_argument('--conflicting-process', action='append', default=None)
    args = parser.parse_args(argv)
    if args.config is None:
        if args.mode == 'hardware':
            raise ValueError('--hardware-config is required for hardware mode')
        args.config = share / 'config/offline.yaml'
    args.config = args.config.expanduser().resolve(strict=True)
    config = yaml.safe_load(args.config.read_text())
    if not isinstance(config, dict) or config.get('transport') != ('sdk' if args.mode == 'hardware' else 'fake'):
        raise ValueError('Requested mode does not match config transport')
    startup, diagnostics = config.get('startup') or {}, config.get('diagnostics') or {}
    if not isinstance(startup, dict) or not isinstance(diagnostics, dict):
        raise ValueError('startup and diagnostics must be mappings')
    for name, default in [('initialize_can', False), ('activate', True), ('start_controllers', True),
                          ('rviz', True), ('remote_rviz', False)]:
        value = getattr(args, name)
        if value is None: value = startup.get(name, default)
        if type(value) is not bool: raise ValueError(name + ' must be boolean')
        setattr(args, name, value)
    for name in ['can_capture', 'usb_capture', 'gripper_trace', 'diagnostics_required']:
        value = getattr(args, name)
        if value is None: value = diagnostics.get(name, False)
        if type(value) is not bool: raise ValueError(name + ' must be boolean')
        setattr(args, name, value)
    args.control_cpu = args.control_cpu if args.control_cpu is not None else startup.get('control_cpu', -1)
    if type(args.control_cpu) is not int or args.control_cpu < -1:
        raise ValueError('control_cpu must be -1 or a CPU number')
    online = online_cpus()
    if args.control_cpu >= 0 and args.control_cpu not in online:
        raise ValueError(f'control_cpu {args.control_cpu} is offline or absent; online CPUs: {sorted(online)}')
    args.process_cpus = args.process_cpus if args.process_cpus is not None else startup.get('process_cpus', '')
    if not isinstance(args.process_cpus, str): raise ValueError('process_cpus must be a string')
    unavailable = cpu_set(args.process_cpus) - online
    if unavailable:
        raise ValueError(f'process_cpus includes offline or absent CPUs: {sorted(unavailable)}; online CPUs: {sorted(online)}')
    args.sdk_cpus = []
    for name in ['sdk_rx', 'sdk_tx']:
        settings = config.get(name) or {}
        if not isinstance(settings, dict): raise ValueError(name + ' must be a mapping')
        cpu = settings.get('cpu', -1)
        priority = settings.get('priority', 90)
        if type(cpu) is not int or cpu < -1 or (args.mode == 'hardware' and cpu >= 0 and cpu not in online):
            raise ValueError(f'{name}.cpu {cpu!r} is invalid or offline; online CPUs: {sorted(online)}')
        if type(priority) is not int or not 1 <= priority <= 99: raise ValueError(name + '.priority must be in 1..99')
        if args.mode == 'hardware' and cpu >= 0: args.sdk_cpus.append(cpu)
    args.usb_cpu = diagnostics.get('usb_cpu', -1)
    if type(args.usb_cpu) is not int or args.usb_cpu < -1: raise ValueError('diagnostics.usb_cpu must be -1 or a CPU number')
    args.usb_channels = diagnostics.get('usb_channels', [0, 1])
    if (not isinstance(args.usb_channels, list) or len(args.usb_channels) != 2
            or any(type(c) is not int or not 0 <= c <= 15 for c in args.usb_channels)
            or len(set(args.usb_channels)) != 2):
        raise ValueError('diagnostics.usb_channels must map two distinct physical adapter channels to buses')
    args.domain_id = args.domain_id if args.domain_id is not None else int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if not 0 <= args.domain_id <= 232: raise ValueError('domain_id must be in 0..232')
    args.conflicting_process = args.conflicting_process if args.conflicting_process is not None else startup.get('conflicting_processes', [])
    if not isinstance(args.conflicting_process, list) or not all(isinstance(n, str) and re.fullmatch(r'[A-Za-z0-9_.-]+', n) for n in args.conflicting_process):
        raise ValueError('conflicting_processes must be a list of process names')
    base = args.diagnostics_dir or diagnostics.get('directory') or args.config.parent / 'diagnostics'
    args.diagnostics_dir = Path(base).expanduser()
    if not args.diagnostics_dir.is_absolute(): args.diagnostics_dir = args.config.parent / args.diagnostics_dir
    if not args.activate: args.start_controllers = False
    if args.mode == 'fake' and (args.initialize_can or args.can_capture or args.usb_capture):
        raise ValueError('Fake mode cannot initialize CAN or start CAN/USB capture')
    if args.mode == 'hardware':
        if config.get('commissioning_confirmed') is not True or config.get('deactivate_policy') != 'disable':
            raise ValueError('Hardware mode needs commissioning_confirmed: true and deactivate_policy: disable')
        if type(config.get('gripper_read_only')) is not bool:
            raise ValueError('Hardware config needs an explicit gripper_read_only boolean')
    if len(config.get('buses', [])) != 2 or len(set(config['buses'])) != 2 or not all(re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', n) for n in config['buses']):
        raise ValueError('Expected two distinct CAN interface names')
    if config.get('timing', {}).get('period_us') != 5000:
        raise ValueError('This integration requires the existing 5000 us period')
    settings = startup.get('can') or {}
    args.can_settings = {key: settings.get(key, default) for key, default in
                         [('bitrate', 1000000), ('data_bitrate', 5000000), ('restart_ms', 1000), ('tx_queue', 64)]}
    if not all(type(value) is int and value > 0 for value in args.can_settings.values()):
        raise ValueError('CAN timing/queue settings must be positive integers')
    return args, config


def controllers_for(config):
    names = ['left_arm_controller', 'right_arm_controller']
    if not config.get('gripper_read_only', False):
        names += ['left_gripper_controller', 'right_gripper_controller']
    return names


def can_state(link, name, settings, ready=False):
    if link.get('ifname') != name or link.get('linkinfo', {}).get('info_kind') != 'can':
        raise ValueError(name + ': expected one CAN interface')
    if 'UP' not in link.get('flags', []):
        if ready: raise ValueError(name + ': CAN interface is DOWN')
        return 'down'
    data = link.get('linkinfo', {}).get('info_data', {})
    expected = {'state': (data.get('state'), 'ERROR-ACTIVE'), 'MTU': (link.get('mtu'), 72),
                'bitrate': (data.get('bittiming', {}).get('bitrate'), settings['bitrate']),
                'data bitrate': (data.get('data_bittiming', {}).get('bitrate'), settings['data_bitrate']),
                'restart_ms': (data.get('restart_ms'), settings['restart_ms']),
                'tx_queue': (link.get('txqlen'), settings['tx_queue'])}
    bad = [f'{key}={actual!r}, expected {wanted!r}' for key, (actual, wanted) in expected.items() if actual != wanted]
    if bad: raise ValueError(name + ': ' + '; '.join(bad) + '. Refusing to reset an active interface.')
    return 'ready'


def capture_filters(config):
    result = []
    for index, bus in enumerate(config['buses']):
        tool_ids = sorted({g['tool_id'] for g in config['grippers'] if g['bus'] == index})
        result.append(bus + ',03F:C00007FF,080:C00007C0,' +
                      ','.join(f'{0x600+node:03X}:C00007FF' for node in tool_ids) + ',#1FFFFFFF')
    return result


def hardware_state(expected, component):
    import rclpy
    from controller_manager_msgs.srv import ListHardwareComponents
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('coarm7_startup_check', context=context)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        client = node.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
        if not client.wait_for_service(timeout_sec=4): raise RuntimeError('Hardware state service is unavailable')
        future = client.call_async(ListHardwareComponents.Request())
        executor.spin_until_future_complete(future, timeout_sec=4)
        if not future.done() or future.result() is None: raise RuntimeError('Hardware state service did not reply')
        actual = [(item.name, item.state.label) for item in future.result().component]
        if actual != [(component, expected)]:
            raise RuntimeError(f'Expected {component} {expected}; found {actual}')
    finally:
        executor.remove_node(node); executor.shutdown()
        node.destroy_node(); rclpy.shutdown(context=context)


class Session:
    def __init__(self, args, config, share):
        self.args, self.config, self.share = args, config, share
        self.component = 'CoArm7Hardware' if args.mode == 'hardware' else 'CoArm7HardwareOffline'
        self.launch = None
        self.capture = []
        self.streams = []
        self.lock = None
        self.usb_fd = None
        self.usb_adapter = None
        self.limits_session = None
        self.lib = Path(__file__).resolve().parent

    def log(self, message):
        print('CoArm7 startup: ' + message, flush=True)

    def command(self, command, *, capture=False, timeout=60):
        return subprocess.run(command, check=True, text=True, capture_output=capture, timeout=timeout)

    def handover(self):
        if self.launch is not None and self.launch.poll() is not None:
            raise RuntimeError('CoArm7 launch exited; inspect the first startup error')
        if self.args.conflicting_process:
            names = self.command(['ps', '-eo', 'comm='], capture=True).stdout.split()
            for name in self.args.conflicting_process:
                if name in names or name[:15] in names:
                    raise RuntimeError(name + ' is running; finish CAN handover first')

    def inspect_can(self, name, ready=False):
        data = json.loads(self.command(['ip', '-details', '-json', 'link', 'show', 'dev', name], capture=True).stdout)
        if len(data) != 1: raise ValueError(name + ': missing or ambiguous CAN interface')
        return can_state(data[0], name, self.args.can_settings, ready)

    def ready_can(self):
        if self.args.mode == 'hardware':
            for name in self.config['buses']: self.inspect_can(name, True)

    def initialize_can(self):
        # Inspect BOTH buses first. An already active/mismatched bus is never reset.
        states = {name: self.inspect_can(name) for name in self.config['buses']}
        if any(state == 'down' for state in states.values()) and not self.args.initialize_can:
            raise RuntimeError('CAN is DOWN; configure it externally or explicitly use --initialize-can')
        if any(state == 'down' for state in states.values()):
            self.command(['sudo', '-v'])
        self.handover()
        for name in self.config['buses']:
            if self.inspect_can(name) == 'down':
                c = self.args.can_settings
                self.command(['sudo', '-n', 'ip', 'link', 'set', 'dev', name, 'type', 'can',
                              'bitrate', str(c['bitrate']), 'dbitrate', str(c['data_bitrate']),
                              'restart-ms', str(c['restart_ms']), 'berr-reporting', 'on', 'fd', 'on'])
                self.command(['sudo', '-n', 'ip', 'link', 'set', 'dev', name, 'txqueuelen', str(c['tx_queue'])])
                self.command(['sudo', '-n', 'ip', 'link', 'set', 'dev', name, 'up'])
            self.inspect_can(name, True)

    def diagnostic_error(self, kind, error):
        if self.args.diagnostics_required: raise RuntimeError(kind + ': ' + str(error)) from error
        self.log(kind + ' disabled: ' + str(error))
        setattr(self.args, kind, False)

    def diagnostic_script(self, name):
        from ament_index_python.packages import get_package_prefix
        return Path(get_package_prefix('coarm7_diagnostics')) / 'lib/coarm7_diagnostics' / name

    def diagnostic_preflight(self):
        if self.args.gripper_trace:
            try:
                if not self.diagnostic_script('gripper_trace.py').is_file():
                    raise FileNotFoundError('Install coarm7_diagnostics for gripper tracing')
            except (OSError, LookupError) as error:
                self.diagnostic_error('gripper_trace', error)
        if self.args.can_capture and not shutil.which('candump'):
            self.diagnostic_error('can_capture', RuntimeError('candump is not installed'))
        if self.args.usb_capture:
            try:
                module = runpy.run_path(str(self.diagnostic_script('capture_usb.py')))
                self.usb_adapter = module['resolve_adapter'](interfaces=self.config['buses'])
                module['passive_cpu'](self.usb_avoid_cpus(), self.args.usb_cpu)
                access = runpy.run_path(str(self.diagnostic_script('usb_access.py')))
                self.usb_fd = access['open_usbmon'](self.usb_adapter[0])
                self.log('USB read access acquired; background recording needs no sudo')
            except (OSError, LookupError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
                self.diagnostic_error('usb_capture', error)
        if self.args.can_capture or self.args.usb_capture or self.args.gripper_trace:
            try:
                self.args.diagnostics_dir.mkdir(parents=True, exist_ok=True)
            except OSError as error:
                for kind in ['can_capture', 'usb_capture', 'gripper_trace']:
                    if getattr(self.args, kind): self.diagnostic_error(kind, error)

    def start_can_capture(self):
        if not self.args.can_capture: return
        path = self.args.diagnostics_dir / ('coarm7-can-' + time.strftime('%Y%m%d-%H%M%S') + '-' + str(os.getpid()) + '.log')
        stream = path.open('x'); self.streams.append(stream)
        process = subprocess.Popen(['stdbuf', '-oL', 'candump', '-L', '-x', '-d', '-D', '-r', '131072',
                                    *capture_filters(self.config)], stdin=subprocess.DEVNULL, stdout=stream,
                                   stderr=subprocess.STDOUT, close_fds=True, start_new_session=True)
        self.capture.append(process); time.sleep(.1)
        if process.poll() is not None: raise RuntimeError('candump exited; inspect ' + str(path))
        self.log('CAN capture: ' + str(path))

    def affinity_command(self):
        return [sys.executable, str(self.lib / 'affinity_guard.py'),
                '--cpus', self.args.process_cpus, '--control-cpu', str(self.args.control_cpu)] + [
                    part for cpu in sorted(set(self.args.sdk_cpus)) for part in ['--probe-cpu', str(cpu)]]

    def affinity_preflight(self):
        if not self.args.process_cpus and self.args.control_cpu < 0 and not self.args.sdk_cpus:
            return
        # A disposable child tests the kernel/cpuset decision before CAN setup.
        try:
            result = self.command(self.affinity_command() + ['--probe-only'], capture=True)
        except subprocess.CalledProcessError as error:
            detail = (error.stderr or error.stdout or str(error)).strip()
            raise RuntimeError('CPU binding preflight failed: ' + detail) from error
        self.log(result.stdout.strip())

    def launch_command(self):
        command = ['ros2', 'launch', 'coarm7_bringup',
                   'hardware.launch.py' if self.args.mode == 'hardware' else 'offline.launch.py',
                   f'control_cpu:={self.args.control_cpu}', f'rviz:={str(self.args.rviz).lower()}',
                   f'remote_rviz:={str(self.args.remote_rviz).lower()}',
                   f'gripper_trace:={str(self.args.gripper_trace).lower()}',
                   f'diagnostics_dir:={self.args.diagnostics_dir}']
        if self.args.mode == 'hardware':
            command.append('hardware_config:=' + str(self.args.config))
            if self.limits_session: command.append('limits_session_dir:=' + self.limits_session.name)
        else: command += ['staged:=true', 'config_file:=' + str(self.args.config)]
        return (self.affinity_command() + ['--'] if self.args.process_cpus or self.args.control_cpu >= 0 else []) + command

    def usb_avoid_cpus(self):
        return sorted({cpu for cpu in [self.args.control_cpu, *self.args.sdk_cpus] if cpu >= 0})

    def start_usb_capture(self):
        if not self.args.usb_capture: return
        directory = Path(tempfile.mkdtemp(prefix='coarm7-usbmon-' + time.strftime('%Y%m%d-%H%M%S-'), dir=self.args.diagnostics_dir))
        process = subprocess.Popen([sys.executable, str(self.diagnostic_script('capture_usb.py')), '--launch-pid', str(self.launch.pid),
                                    '--directory', str(directory), '--fd', str(self.usb_fd),
                                    '--interfaces', *self.config['buses'], '--adapter', *map(str, self.usb_adapter),
                                    '--channels', *map(str, self.args.usb_channels),
                                    '--cpu', str(self.args.usb_cpu), '--avoid-cpu', *map(str, self.usb_avoid_cpus())],
                                   close_fds=True, pass_fds=(self.usb_fd,), stdin=subprocess.DEVNULL,
                                   start_new_session=True)
        self.capture.append(process)
        os.close(self.usb_fd); self.usb_fd = None
        try:
            self.command([sys.executable, str(self.diagnostic_script('capture_usb.py')), '--wait-ready', str(directory),
                          '--supervisor-pid', str(process.pid), '--timeout', '25'], timeout=30)
        except BaseException:
            self.stop(process, signal.SIGTERM, 8)
            raise
        self.log('USB capture: ' + str(directory))

    def state(self, expected):
        self.handover(); hardware_state(expected, self.component)

    def wait_planning_limits(self):
        if self.args.mode != 'hardware': return
        if self.limits_session is None: raise RuntimeError('Missing planning limit session')
        ready = Path(self.limits_session.name) / 'planning_ready.json'
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            self.handover()
            if ready.is_file():
                data = json.loads(ready.read_text())
                if data.get('config_file') != str(self.args.config) or len(data.get('axes', {})) != 14:
                    raise RuntimeError('Invalid planning limit acknowledgement')
                self.log('MoveIt model uses this session effective arm limits')
                return
            time.sleep(.1)
        raise RuntimeError('MoveIt limit synchronization did not complete; hardware was not activated')

    def lifecycle(self):
        self.handover(); self.ready_can()
        self.command(['ros2', 'run', 'controller_manager', 'hardware_spawner', '--configure', self.component,
                      '--controller-manager-timeout', '25'])
        self.state('inactive')
        self.wait_planning_limits()
        self.command(['ros2', 'run', 'controller_manager', 'spawner', 'joint_state_broadcaster',
                      '--controller-manager-timeout', '25'])
        self.log('Feedback configured; ' + ('enabling hold' if self.args.activate else 'read-only state publishing'))
        if not self.args.activate: return
        self.state('inactive'); self.ready_can()
        self.command(['ros2', 'run', 'controller_manager', 'hardware_spawner', '--activate', self.component,
                      '--controller-manager-timeout', '25'])
        self.state('active')
        if self.args.start_controllers:
            self.command(['ros2', 'run', 'controller_manager', 'spawner', *controllers_for(self.config),
                          '--activate-as-group', '--controller-manager-timeout', '25', '--switch-timeout', '5',
                          '--service-call-timeout', '8'])
            self.state('active')
        self.log('Ready: ' + ('arms + ' + ('read-only grippers' if self.config.get('gripper_read_only', False) else 'grippers')
                             if self.args.start_controllers else 'hold only; no motion controllers started'))

    @staticmethod
    def stop(process, first_signal, timeout):
        if process is None or process.poll() is not None: return
        process.send_signal(first_signal)
        try: process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            # Only this session's process group; never global pkill/drive reset.
            try: os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError: pass
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try: os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError: pass
                process.wait(timeout=5)

    def cleanup(self):
        # Keep passive recordings alive until control nodes have shut down.
        self.stop(self.launch, signal.SIGINT, 30)
        for process in reversed(self.capture): self.stop(process, signal.SIGTERM, 8)
        for stream in self.streams: stream.close()
        if self.usb_fd is not None:
            os.close(self.usb_fd); self.usb_fd = None
        if self.limits_session is not None:
            self.limits_session.cleanup(); self.limits_session = None
        if self.lock is not None:
            os.close(self.lock); self.lock = None

    def run(self):
        os.environ['ROS_DOMAIN_ID'] = str(self.args.domain_id)
        if self.args.remote_rviz: os.environ['ROS_LOCALHOST_ONLY'] = '1'
        self.lock = os.open(self.args.config.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            try: fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error: raise RuntimeError('Another CoArm7 session holds this config-directory lock') from error
            self.handover()
            services = self.command(['ros2', 'service', 'list', '--no-daemon', '--spin-time', '1'], capture=True).stdout.splitlines()
            if '/controller_manager/list_hardware_components' in services:
                raise RuntimeError('A controller manager is already running in this ROS domain')
            self.affinity_preflight()
            self.diagnostic_preflight()
            if self.args.mode == 'hardware': self.initialize_can()
            try: self.start_can_capture()
            except (OSError, RuntimeError) as error: self.diagnostic_error('can_capture', error)
            self.handover()
            if self.args.mode == 'hardware':
                self.limits_session = tempfile.TemporaryDirectory(prefix='coarm7-limits-')
            self.launch = subprocess.Popen(self.launch_command(), close_fds=True, start_new_session=True)
            try: self.start_usb_capture()
            except (OSError, RuntimeError, subprocess.SubprocessError) as error: self.diagnostic_error('usb_capture', error)
            self.lifecycle()
            return self.launch.wait()
        finally:
            self.cleanup()


def main(argv=None):
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('coarm7_hardware'))
    args, config = resolve_options(argv, share)
    session = Session(args, config, share)
    if args.check_only:
        print(json.dumps({'mode': args.mode, 'config': str(args.config), 'domain_id': args.domain_id,
                          'launch_command': session.launch_command(), 'activate': args.activate,
                          'controllers': controllers_for(config) if args.start_controllers else [],
                          'diagnostics': {key: getattr(args, key) for key in ['can_capture', 'usb_capture', 'gripper_trace', 'diagnostics_required']},
                          'can_interfaces_modified': False, 'ros_nodes_started': False}, indent=2))
        return 0
    stopping = False
    def interrupt(signum, _):
        nonlocal stopping
        if stopping: return
        stopping = True
        raise KeyboardInterrupt
    for signum in [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]: signal.signal(signum, interrupt)
    return session.run()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print('CoArm7 startup failed: ' + str(error), file=sys.stderr)
        raise SystemExit(1)
