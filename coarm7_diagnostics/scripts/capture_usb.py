#!/usr/bin/env python3
"""Supervise the existing passive usbmon recorder alongside one CoArm7 launch.

No CAN socket, USB device command, interface setting or scheduling change to
another process is used. A read-only monitor FD is acquired before ROS starts;
all recording and file writing run as the deployment user without sudo.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import time



def write_json(path, value):
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def process_info(pid, proc=Path('/proc')):
    try:
        folder = proc / str(pid)
        fields = (folder / 'stat').read_text().rsplit(') ', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return {
            'pid': pid, 'parent': int(fields[1]), 'start': fields[19],
            'argv': [os.fsdecode(s) for s in (folder / 'cmdline').read_bytes().split(b'\0') if s],
        }
    except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError, ValueError):
        return None


def descendant(info, ancestor, proc=Path('/proc')):
    seen = set()
    while info and info['pid'] not in seen:
        if info['parent'] == ancestor:
            return True
        seen.add(info['pid'])
        info = process_info(info['parent'], proc)
    return False


def find_controller(launch, proc=Path('/proc')):
    matches = []
    for entry in proc.iterdir():
        if not entry.name.isdecimal():
            continue
        info = process_info(int(entry.name), proc)
        if (info and info['argv'] and Path(info['argv'][0]).name == 'ros2_control_node'
                and descendant(info, launch, proc)):
            matches.append(info)
    if len(matches) > 1:
        raise RuntimeError('More than one ros2_control_node belongs to this launch')
    return matches[0] if matches else None


def resolve_adapter(sys_root=Path('/sys'), devices=Path('/dev'), interfaces=('can0', 'can1')):
    adapters = []
    for name in interfaces:
        link = sys_root / 'class/net' / name / 'device'
        device = link.resolve(strict=True)
        if not device.is_relative_to(sys_root.resolve()):
            raise RuntimeError(f'{name} is outside the expected sysfs tree')
        for parent in (device, *device.parents):
            if (parent / 'busnum').is_file() and (parent / 'devnum').is_file():
                bus = int((parent / 'busnum').read_text())
                number = int((parent / 'devnum').read_text())
                if not (1 <= bus <= 999 and 1 <= number <= 127):
                    raise RuntimeError('Invalid USB bus/device identifier')
                adapters.append((bus, number))
                break
        else:
            raise RuntimeError(f'Cannot resolve the USB adapter for {name}')
    if len(set(adapters)) != 1:
        raise RuntimeError('This recorder requires the configured CAN interfaces on the same USB adapter')
    bus, device = adapters[0]
    monitor = devices / f'usbmon{bus}'
    if not monitor.exists():
        raise RuntimeError(f'{monitor} is absent; usbmon must be available before starting this test')
    return bus, device


def reader_command(bus, device, controller, launch, directory, reader, fd):
    if os.getuid() == 0 or os.getgid() == 0:
        raise RuntimeError('Run the USB capture supervisor as the normal deployment user')
    return [sys.executable, '-u', str(reader), '--fd', str(fd), '--directory', str(directory),
            '--bus', str(bus), '--device', str(device), '--controller-pid', str(controller),
            '--launch-pid', str(launch)]


def spawn_reader(bus, device, controller, launch, directory, reader, log, fd):
    # Only the explicitly authorized monitor FD crosses exec, never session locks.
    return subprocess.Popen(reader_command(bus, device, controller, launch, directory, reader, fd),
                            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                            close_fds=True, pass_fds=(fd,))


def passive_cpu(avoid_cpus=(), requested=-1):
    allowed = set(os.sched_getaffinity(0))
    passive = allowed - set(avoid_cpus)
    if requested >= 0:
        if requested not in passive:
            raise RuntimeError(f'USB capture CPU {requested} is unavailable or reserved; allowed={sorted(allowed)}, avoided={sorted(avoid_cpus)}')
        return requested
    if not passive: raise RuntimeError('No separate CPU is available for passive USB recording; review diagnostics.usb_cpu and thread bindings')
    return min(passive)


def stop_owned(info):
    if not info:
        return
    current = process_info(info['pid'])
    if current and current['start'] == info['start']:
        try:
            os.kill(info['pid'], signal.SIGTERM)
        except ProcessLookupError:
            pass


def wait_ready(directory, supervisor_pid, timeout):
    identity = process_info(supervisor_pid)
    if not identity:
        raise RuntimeError('USB capture supervisor already exited')
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        current = process_info(supervisor_pid)
        if not current or current['start'] != identity['start']:
            raise RuntimeError(f'USB capture startup failed; inspect {directory}/supervisor.json and reader.log')
        try:
            state = json.loads((directory / 'supervisor.json').read_text())
            if state.get('status') == 'error':
                raise RuntimeError(state['error'])
            if state.get('ready'):
                return state
        except (FileNotFoundError, json.JSONDecodeError):
            pass
        time.sleep(.1)
    raise RuntimeError(f'USB capture did not become ready; inspect {directory}/reader.log')


def supervise(launch_pid, directory, fd, interfaces=('can0', 'can1'), avoid_cpus=(), requested_cpu=-1, expected_adapter=None, channels=(0, 1)):
    launch = process_info(launch_pid)
    if not launch:
        raise RuntimeError('The selected CoArm7 launch is not alive')
    bus, device = resolve_adapter(interfaces=interfaces)
    if expected_adapter is not None and (bus, device) != tuple(expected_adapter):
        raise RuntimeError('USB adapter changed after authorization; restart the session')
    directory = directory.resolve(strict=True)
    scripts = Path(__file__).resolve().parent
    for name in ('usbmon_reader.py', 'usbmon_statistics.py'):
        if not (scripts / name).is_file():
            raise RuntimeError(f'Missing installed USB capture script: {name}')
    cpu = passive_cpu(avoid_cpus, requested_cpu)
    os.sched_setaffinity(0, {cpu})
    os.nice(10)
    state = {'pid': os.getpid(), 'launch_pid': launch_pid, 'bus': bus, 'device': device,
             'directory': str(directory), 'cpu': cpu, 'can_interfaces': list(interfaces), 'usb_channels': list(channels), 'nice': os.getpriority(os.PRIO_PROCESS, 0),
             'can_sockets_opened': False, 'usb_device_commands_sent': False,
             'ready': False, 'status': 'starting'}
    status = directory / 'supervisor.json'
    write_json(status, state)
    stop = False

    def request_stop(*_):
        nonlocal stop
        stop = True

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, request_stop)
    reader = observer = None
    reader_info = None
    log = (directory / 'reader.log').open('a')
    observer_log = (directory / 'statistics.log').open('a')
    try:
        end = time.monotonic() + 10
        control = None
        while time.monotonic() < end and not stop:
            alive = process_info(launch_pid)
            if not alive or alive['start'] != launch['start']:
                raise RuntimeError('CoArm7 launch exited before USB recording started')
            control = find_controller(launch_pid)
            if control:
                break
            time.sleep(.1)
        if not control:
            raise RuntimeError('Cannot find this launch\'s ros2_control_node')
        state['controller_pid'] = control['pid']
        write_json(status, state)
        reader = spawn_reader(bus, device, control['pid'], launch_pid, directory,
                              scripts / 'usbmon_reader.py', log, fd)
        end = time.monotonic() + 8
        while time.monotonic() < end and not stop:
            if reader.poll() is not None:
                raise RuntimeError('usbmon reader exited during startup: ' + (directory / 'reader.log').read_text()[-2000:])
            try:
                source = json.loads((directory / 'status.json').read_text())
                reader_info = process_info(int(source['pid']))
                if reader_info and source['status'] == 'capturing':
                    break
            except (FileNotFoundError, json.JSONDecodeError):
                pass
            time.sleep(.1)
        else:
            raise RuntimeError('usbmon reader did not report readiness')
        observer = subprocess.Popen([sys.executable, '-u', str(scripts / 'usbmon_statistics.py'), str(directory)],
                                    stdin=subprocess.DEVNULL, stdout=observer_log,
                                    stderr=subprocess.STDOUT, close_fds=True)
        end = time.monotonic() + 5
        while time.monotonic() < end and not stop:
            if observer.poll() is not None or reader.poll() is not None:
                raise RuntimeError('USB recorder/statistics process exited before readiness')
            if (directory / 'interval-statistics/statistics.json').is_file():
                break
            time.sleep(.1)
        else:
            raise RuntimeError('USB statistics did not become ready')
        state.update(ready=True, status='capturing', reader_pid=reader_info['pid'], observer_pid=observer.pid)
        write_json(status, state)
        print(f'CoArm7 USB passive capture ready: {directory}', flush=True)
        while not stop and reader.poll() is None:
            if observer.poll() is not None:
                raise RuntimeError('USB statistics process exited unexpectedly')
            time.sleep(.2)
        if stop:
            # Let the original reader finish its three-second post-exit tail.
            end = time.monotonic() + 4
            while reader.poll() is None and time.monotonic() < end:
                time.sleep(.1)
            stop_owned(reader_info)
        reader.wait(timeout=5)
        if reader.returncode:
            raise RuntimeError(f'USB reader exited with status {reader.returncode}')
        if observer:
            observer.wait(timeout=5)
            if observer.returncode:
                raise RuntimeError(f'USB statistics exited with status {observer.returncode}')
        state.update(ready=False, status='complete')
    except BaseException as error:
        state.update(ready=False, status='error', error=str(error))
        raise
    finally:
        if reader_info is None and reader is not None:
            try:
                candidate = process_info(int((directory / 'pid').read_text()))
                if (candidate and str(scripts / 'usbmon_reader.py') in candidate['argv']
                        and str(directory) in candidate['argv']):
                    reader_info = candidate
            except (OSError, ValueError):
                pass
        stop_owned(reader_info)
        if observer and observer.poll() is None:
            observer.terminate()
            try:
                observer.wait(timeout=3)
            except subprocess.TimeoutExpired:
                observer.kill(); observer.wait()
        if reader and reader.poll() is None:
            try:
                reader.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # The actual reader is unprivileged; never signal a root sudo monitor.
                current = process_info(reader_info['pid']) if reader_info else None
                if current and current['start'] == reader_info['start']:
                    os.kill(current['pid'], signal.SIGKILL)
                    reader.wait(timeout=3)
        log.close(); observer_log.close()
        write_json(status, state)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--launch-pid', type=int)
    parser.add_argument('--directory', type=Path)
    parser.add_argument('--wait-ready', type=Path)
    parser.add_argument('--supervisor-pid', type=int)
    parser.add_argument('--timeout', type=float, default=25)
    parser.add_argument('--fd', type=int)
    parser.add_argument('--interfaces', nargs=2, default=['can0', 'can1'])
    parser.add_argument('--avoid-cpu', type=int, nargs='*', default=[])
    parser.add_argument('--cpu', type=int, default=-1)
    parser.add_argument('--channels', type=int, nargs=2, default=[0, 1])
    parser.add_argument('--adapter', type=int, nargs=2)
    args = parser.parse_args()
    try:
        if args.check:
            bus, device = resolve_adapter(interfaces=args.interfaces)
            print(json.dumps({'usb_bus': bus, 'usb_device': device, 'can_interfaces': args.interfaces, 'capture_cpu': passive_cpu(args.avoid_cpu, args.cpu)}))
        elif args.wait_ready:
            if not args.supervisor_pid:
                parser.error('--supervisor-pid is required with --wait-ready')
            wait_ready(args.wait_ready, args.supervisor_pid, args.timeout)
        else:
            if not args.launch_pid or not args.directory:
                parser.error('--launch-pid and --directory are required')
            if args.fd is None: parser.error('--fd is required; use the CoArm7 startup entry to authorize USB access')
            import runpy
            access = runpy.run_path(str(Path(__file__).with_name('usb_access.py')))
            bus, _ = resolve_adapter(interfaces=args.interfaces)
            try:
                access['validate_fd'](args.fd, bus)
                supervise(args.launch_pid, args.directory, args.fd, args.interfaces,
                          args.avoid_cpu, args.cpu, args.adapter, args.channels)
            finally:
                os.close(args.fd)
    except Exception as error:
        print(f'CoArm7 USB capture: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
