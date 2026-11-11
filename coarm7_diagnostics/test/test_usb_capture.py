"""Replay usbmon events and process trees without USB/CAN hardware or sudo."""
import ctypes
import errno
import importlib.util
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
from types import SimpleNamespace
from unittest import mock

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'


def module(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / (name + '.py'))
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


capture = module('capture_usb')
reader = module('usbmon_reader')
statistics = module('usbmon_statistics')


def fake_process(root, pid, parent, executable, start=10):
    folder = root / str(pid); folder.mkdir()
    fields = ['S', str(parent)] + ['0'] * 17 + [str(start)]
    (folder / 'stat').write_text(f'{pid} (test process) ' + ' '.join(fields))
    (folder / 'cmdline').write_bytes(executable.encode() + b'\0')


def test_only_controller_descended_from_selected_launch_is_used(tmp_path):
    fake_process(tmp_path, 100, 1, '/usr/bin/python3')
    fake_process(tmp_path, 101, 100, '/usr/bin/python3')
    fake_process(tmp_path, 102, 101, '/opt/ros/humble/lib/controller_manager/ros2_control_node')
    fake_process(tmp_path, 200, 1, '/opt/ros/humble/lib/controller_manager/ros2_control_node')
    assert capture.find_controller(100, tmp_path)['pid'] == 102
    assert capture.find_controller(999, tmp_path) is None
    fake_process(tmp_path, 103, 100, '/opt/ros/humble/lib/controller_manager/ros2_control_node')
    with pytest.raises(RuntimeError, match='More than one'):
        capture.find_controller(100, tmp_path)


def make_adapter(tmp_path, separate=False):
    sysfs = tmp_path / 'sys'; devices = tmp_path / 'dev'; devices.mkdir()
    for i, name in enumerate(('can0', 'can1')):
        adapter = sysfs / 'devices' / ('usb-adapter-' + str(i if separate else 0))
        adapter.mkdir(parents=True, exist_ok=True)
        (adapter / 'busnum').write_text('1')
        (adapter / 'devnum').write_text(str(5 + i if separate else 5))
        interface = adapter / f'interface-{i}'; interface.mkdir()
        entry = sysfs / 'class/net' / name; entry.mkdir(parents=True)
        (entry / 'device').symlink_to(interface)
    (devices / 'usbmon1').touch()
    return sysfs, devices


def test_adapter_is_resolved_from_sysfs_without_opening_usb_or_can(tmp_path):
    sysfs, devices = make_adapter(tmp_path)
    with mock.patch.object(capture.os, 'open', side_effect=AssertionError('No device open allowed')):
        assert capture.resolve_adapter(sysfs, devices) == (1, 5)
    (devices / 'usbmon1').unlink()
    with pytest.raises(RuntimeError, match='absent'):
        capture.resolve_adapter(sysfs, devices)


def test_separate_adapters_fail_before_privileged_open(tmp_path):
    sysfs, devices = make_adapter(tmp_path, separate=True)
    with pytest.raises(RuntimeError, match='same USB adapter'):
        capture.resolve_adapter(sysfs, devices)


def sync_payload(channel, counter):
    data = bytearray(22)
    struct.pack_into('<HH', data, 0, len(data), 0x1000)
    data[12] = channel
    struct.pack_into('<I', data, 16, 0x03F)
    data[20:22] = bytes((0x3C, counter))
    return bytes(data)


def emcy_payload(channel, node, code):
    data = bytearray(30)
    struct.pack_into('<HH', data, 0, len(data), 1)
    struct.pack_into('<Q', data, 4, 123456)
    data[20] = channel
    struct.pack_into('<I', data, 24, 0x080 + node)
    struct.pack_into('<H', data, 28, code)
    return bytes(data)


def usb_event(urb, kind, endpoint, ns, payload=b''):
    values = [urb, ord(kind), 3, endpoint, 5, 1, 0, 0,
              ns // 1000000000, (ns % 1000000000) // 1000,
              0, len(payload), len(payload), bytes(8), 0, 0, 0, 0]
    return reader.HEADER.pack(*values), payload


def run_reader(tmp_path, events, rotate_only=False, mapping=None):
    directory = tmp_path / 'capture'; directory.mkdir()
    if mapping: (directory / 'supervisor.json').write_text(json.dumps(mapping))
    home = tmp_path / 'home'; (home / '.ros/log').mkdir(parents=True)
    (directory / 'unrelated.txt').write_text('preserve')
    read_fd, write_fd = os.pipe()
    clock = [0.0]; handlers = {}; selects = [0]
    queue = list(events)

    def monotonic():
        clock[0] += 6.0 if rotate_only else .2
        return clock[0]

    def select(*_):
        selects[0] += 1
        if rotate_only and selects[0] >= 20:
            handlers[signal.SIGTERM]()
        return ([read_fd], [], []) if queue else ([], [], [])

    def ioctl(fd, operation, arg=None, *args):
        assert fd == read_fd
        if operation == reader.RING_GET:
            return 1048576
        if operation == reader.RING_SET:
            return 0
        if operation == reader.STATS:
            arg[:] = struct.pack('=II', 0, 0); return 0
        assert operation == reader.GETX
        if not queue:
            raise OSError(errno.EAGAIN, 'Replay drained')
        header_ptr, data_ptr, capacity = struct.unpack('=QQQ', arg)
        header, payload = queue.pop(0)
        assert len(payload) <= capacity
        ctypes.memmove(header_ptr, header, len(header))
        if payload:
            ctypes.memmove(data_ptr, payload, len(payload))
        return 0

    argv = ['reader', '--fd', str(read_fd), '--directory', str(directory), '--bus', '1',
            '--device', '5', '--controller-pid', str(os.getpid()), '--launch-pid', str(os.getpid())]
    try:
        with mock.patch.object(sys, 'argv', argv), \
                mock.patch.object(reader.fcntl, 'ioctl', side_effect=ioctl), \
                mock.patch.object(reader.select, 'select', side_effect=select), \
                mock.patch.object(reader.time, 'monotonic', side_effect=monotonic), \
                mock.patch.object(reader.signal, 'signal', side_effect=lambda sig, fn: handlers.update({sig: fn})), \
                mock.patch.object(reader.pwd, 'getpwuid', return_value=SimpleNamespace(pw_dir=str(home))):
            reader.main()
    finally:
        os.close(write_fd)
        try: os.close(read_fd)
        except OSError: pass
    return directory


def replay_events():
    events = []
    start = 1000000000000
    # An actual USB submit/complete interval change, including counter wrap.
    for channel, submits, durations, first in [
        (0, [0, 5300000, 10000000], [100000, 200000, 250000], 255),
        (1, [20000, 5220000, 10020000], [120000, 120000, 120000], 42),
    ]:
        for i, (submitted, duration) in enumerate(zip(submits, durations)):
            urb = channel * 100 + i + 1
            events.append((start + submitted, usb_event(urb, 'S', 2 + 2 * channel,
                         start + submitted, sync_payload(channel, (first + i) % 256))))
            events.append((start + submitted + duration, usb_event(urb, 'C', 2 + 2 * channel,
                         start + submitted + duration)))
    events.append((start + 12000000, usb_event(999, 'C', 0x82, start + 12000000,
                                            emcy_payload(0, 5, 0xFF05))))
    return [entry for _, entry in sorted(events, key=lambda pair: pair[0])]


def test_reader_replay_preserves_first_ff05_and_statistics(tmp_path):
    directory = run_reader(tmp_path, replay_events())
    source = json.loads((directory / 'status.json').read_text())
    assert source['status'] == 'complete'
    assert source['reason'] == 'first_emcy_plus_3_seconds'
    assert source['sync_rows'] == 6
    assert source['first_emcy']['code'] == '0xFF05'
    assert source['first_emcy']['bus'] == 'can0'
    assert source['first_emcy']['node_id'] == 5
    assert source['can_sockets_opened'] is False
    assert (directory / 'unrelated.txt').read_text() == 'preserve'
    observer = statistics.Observer(directory)
    with mock.patch.object(statistics.signal, 'signal'):
        observer.run()
    summary = json.loads((directory / 'interval-statistics/statistics.json').read_text())
    left = summary['buses']['can0']
    assert left['counter_discontinuities'] == 0
    assert left['submit_interval']['min_us'] == 4700
    assert left['submit_interval']['max_us'] == 5300
    assert left['submit_interval']['max_negative_bias_us'] == -300
    assert left['submit_interval']['max_positive_bias_us'] == 300
    assert left['complete_interval']['min_us'] == 4750
    assert left['complete_interval']['max_us'] == 5400
    assert left['submit_interval']['thresholds_us']['150']['positive'] == 1
    assert left['submit_interval']['thresholds_us']['150']['negative'] == 1
    assert summary['buses']['can1']['submit_interval']['thresholds_us']['200']['absolute'] == 0
    assert summary['rows_retained_full_session'] == 6
    assert summary['pending_anomaly_windows'] == 0
    assert list((directory / 'interval-statistics/anomalies').glob('event-*/segment-*.usb.bin'))
    # Restart/replay reads saved offsets rather than double-counting the window.
    again = statistics.Observer(directory)
    with mock.patch.object(statistics.signal, 'signal'):
        again.run()
    repeated = json.loads((directory / 'interval-statistics/statistics.json').read_text())
    assert repeated['rows_retained_full_session'] == 6


def test_healthy_raw_window_rotates_without_deleting_saved_files(tmp_path):
    directory = run_reader(tmp_path, [], rotate_only=True)
    assert len(list(directory.glob('segment-*.usb.bin'))) == 14
    assert len(list(directory.glob('segment-*.sync.csv'))) == 14
    assert (directory / 'unrelated.txt').read_text() == 'preserve'
    assert json.loads((directory / 'status.json').read_text())['status'] == 'complete'


def test_snapshot_uses_actual_uid_home_despite_root_environment(tmp_path, monkeypatch):
    home = tmp_path / 'linux'; logs = home / '.ros/log'; logs.mkdir(parents=True)
    output = tmp_path / 'capture'; output.mkdir()
    (logs / 'ros2_control_node_42_1.log').write_text('feedback log')
    selected = logs / '2026-11-09-linux-100'; selected.mkdir()
    (selected / 'launch.log').write_text('selected launch')
    other = logs / '2026-11-09-linux-999'; other.mkdir()
    (other / 'launch.log').write_text('unrelated launch')
    monkeypatch.setenv('HOME', '/root')
    monkeypatch.setattr(reader.pwd, 'getpwuid', lambda uid: SimpleNamespace(pw_dir=str(home)))
    result = reader.snapshot_ros_logs(output, 42, 100)
    assert result['directory'] == str(logs)
    assert result['errors'] == []
    assert (output / 'snapshot-launch.log').read_text() == 'selected launch'
    assert (output / 'snapshot-ros2_control_node_42_1.log').read_text() == 'feedback log'


def test_snapshot_permission_error_does_not_fail_capture_shutdown(tmp_path, monkeypatch):
    original = Path.glob
    def denied(path, pattern):
        if path.name == 'log' and path.parent.name == '.ros':
            raise PermissionError('simulated inaccessible ROS logs')
        return original(path, pattern)
    monkeypatch.setattr(Path, 'glob', denied)
    directory = run_reader(tmp_path, replay_events())
    state = json.loads((directory / 'status.json').read_text())
    assert state['status'] == 'complete'
    assert state['first_emcy']['code'] == '0xFF05'
    assert len(state['ros_log_snapshot']['errors']) == 3
    assert list(directory.glob('*.usb.bin'))


def test_packet_decoder_rejects_truncated_records_without_guessing():
    full = sync_payload(0, 255)
    assert len(list(reader.records(full + b'\x00\x00\x00\x00'))) == 1
    assert list(reader.records(full[:-1])) == []
    assert reader.HEADER.size == 64


def test_reader_inherits_only_monitor_and_not_run_service_fd9(tmp_path):
    fake = tmp_path / 'reader.py'
    fake.write_text('''import argparse,json,os
p=argparse.ArgumentParser();p.add_argument("--fd",type=int);a,_=p.parse_known_args()
try:os.fstat(9);opened=True
except OSError:opened=False
print(json.dumps({"fd9_open":opened,"monitor":os.read(a.fd,4).decode(),"uid":os.getuid()}))
''')
    driver = r'''
import importlib.util,os,sys
from pathlib import Path
spec=importlib.util.spec_from_file_location("capture",sys.argv[1]);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
fd=os.open(sys.argv[2],os.O_RDONLY);os.dup2(fd,9);os.set_inheritable(9,True)
r,w=os.pipe();os.write(w,b"test");os.close(w)
with open(sys.argv[3],"w") as log:
 child=mod.spawn_reader(1,5,123,122,Path("/tmp"),Path(sys.argv[4]),log,r)
 raise SystemExit(child.wait(timeout=5))
'''
    lease = tmp_path / 'lease'; lease.touch()
    log = tmp_path / 'log'
    subprocess.run([sys.executable, '-c', driver, str(SCRIPTS / 'capture_usb.py'), str(lease), str(log), str(fake)],
                   check=True, timeout=8)
    assert json.loads(log.read_text()) == dict(fd9_open=False, monitor='test', uid=os.getuid())


def test_startup_failure_and_stale_pid_are_not_reported_as_ready(tmp_path):
    (tmp_path / 'supervisor.json').write_text(json.dumps({'ready': False, 'status': 'error', 'error': 'read denied'}))
    with pytest.raises(RuntimeError, match='read denied'):
        capture.wait_ready(tmp_path, os.getpid(), .2)
    with mock.patch.object(capture, 'process_info', return_value=None):
        with pytest.raises(RuntimeError, match='already exited'):
            capture.wait_ready(tmp_path, 999999, .2)


def test_metric_uses_signed_bias_and_strict_thresholds():
    metric = statistics.Metric(5000)
    for value in (5000, 5150, 4850, 5200, 4799):
        metric.add(value)
    value = metric.summary()
    assert value['max_negative_bias_us'] == -201
    assert value['max_positive_bias_us'] == 200
    assert value['thresholds_us']['150']['positive'] == 1
    assert value['thresholds_us']['150']['negative'] == 1
    assert value['thresholds_us']['200']['positive'] == 0
    assert value['thresholds_us']['200']['negative'] == 1


@pytest.mark.parametrize('interrupted', [False, True])
def test_supervisor_lifecycle_with_rootless_reader_and_real_observer(tmp_path, interrupted):
    # The dummy control child is /bin/sleep with a matching argv[0], not ROS.
    # The reader is a small status-file producer. No sudo or device is used.
    fake_reader = r'''
import json,os,signal,sys,time
from pathlib import Path
root=Path(sys.argv[1]);mode=sys.argv[3];stop=False
def finish(*_):
 global stop
 stop=True
signal.signal(signal.SIGTERM,finish)
try:os.fstat(9);raise RuntimeError("Run Service lock leaked")
except OSError:pass
state={"pid":os.getpid(),"controller_pid":int(sys.argv[2]),"status":"capturing",
 "started_at":"offline-test","sync":{},"dropped":0,"truncated":0,
 "completion_errors":0,"first_emcy":None,"current_segment":"segment-000001.usb.bin"}
def save():
 p=root/"status.json.tmp";p.write_text(json.dumps(state));p.replace(root/"status.json")
(root/"pid").write_text(str(os.getpid()))
(root/"segment-000001.usb.bin").write_bytes(b"")
(root/"segment-000001.sync.csv").write_text("bus,counter,urb_id,submit_ns,complete_ns,usb_duration_us,submit_interval_us,complete_interval_us,status\n")
save();start=time.monotonic()
while not stop and (mode=="interrupt" or time.monotonic()-start<1.5):time.sleep(.05)
state["status"]="complete";save()
'''
    driver = r'''
import importlib.util,os,signal,subprocess,sys
from pathlib import Path
spec=importlib.util.spec_from_file_location("cap",sys.argv[1]);cap=importlib.util.module_from_spec(spec);spec.loader.exec_module(cap)
root=Path(sys.argv[2]);reader_code=sys.argv[3];mode=sys.argv[4]
cap.resolve_adapter=lambda **kwargs:(1,5)
def spawn(bus,device,controller,launch,directory,reader,log,monitor_fd):
 return subprocess.Popen([sys.executable,"-c",reader_code,str(directory),str(controller),mode],
   stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,close_fds=True)
cap.spawn_reader=spawn
control=subprocess.Popen(["/opt/ros/humble/lib/controller_manager/ros2_control_node","40"],executable="/bin/sleep")
killer=None
fd=os.open(str(root/"lease"),os.O_CREAT|os.O_RDWR,0o600);os.dup2(fd,9);os.set_inheritable(9,True)
try:
 if mode=="interrupt":
  killer=subprocess.Popen([sys.executable,"-c","import os,signal,time;time.sleep(2);os.kill(%d,signal.SIGTERM)"%os.getpid()])
 cap.supervise(os.getpid(),root,fd)
finally:
 control.terminate();control.wait()
 if killer:killer.wait(timeout=3)
'''
    mode = 'interrupt' if interrupted else 'complete'
    subprocess.run([sys.executable, '-c', driver, str(SCRIPTS / 'capture_usb.py'), str(tmp_path), fake_reader, mode],
                   check=True, timeout=18)
    value = json.loads((tmp_path / 'supervisor.json').read_text())
    assert value['status'] == 'complete'
    assert value['ready'] is False
    assert capture.process_info(value['reader_pid']) is None
    assert capture.process_info(value['observer_pid']) is None
    stats = json.loads((tmp_path / 'interval-statistics/statistics.json').read_text())
    assert stats['status'] == 'complete'


def test_usb_fd_transfer_is_read_only_and_close_on_exec(tmp_path):
    import socket
    import array
    access = module('usb_access')
    fd = os.open('/dev/null', os.O_RDONLY | os.O_NONBLOCK)
    left, right = socket.socketpair()
    try:
        left.sendmsg([b'U'], [(socket.SOL_SOCKET, socket.SCM_RIGHTS, array.array('i', [fd]))])
        received = access.receive_fd(right, os.getuid())
        try:
            access.validate_fd(received, os.minor(os.fstat(fd).st_rdev))
            assert not os.get_inheritable(received)
            with pytest.raises(OSError): os.write(received, b'forbidden')
        finally: os.close(received)
    finally:
        left.close(); right.close(); os.close(fd)


def test_usb_fd_rejects_wrong_identity_and_writable_handle():
    import socket
    access = module('usb_access')
    left, right = socket.socketpair()
    try:
        with pytest.raises(RuntimeError, match='identity'): access.receive_fd(right, os.getuid() + 1)
    finally: left.close(); right.close()
    fd = os.open('/dev/null', os.O_RDWR | os.O_NONBLOCK)
    try:
        with pytest.raises(RuntimeError, match='read-only'): access.validate_fd(fd, os.minor(os.fstat(fd).st_rdev))
    finally: os.close(fd)


def test_usb_auth_failure_reports_before_background_launch(monkeypatch):
    access = module('usb_access')
    original_open = access.os.open
    def open_file(path, flags, *args, **kwargs):
        if str(path).startswith('/dev/usbmon'): raise PermissionError('offline denied')
        return original_open(path, flags, *args, **kwargs)
    monkeypatch.setattr(access.os, 'open', open_file)
    calls = []
    def popen(command, **kwargs):
        calls.append((command, kwargs))
        kwargs['stderr'].write('sudo: authorization denied'); kwargs['stderr'].flush()
        return SimpleNamespace(poll=lambda: 1)
    monkeypatch.setattr(access.subprocess, 'Popen', popen)
    with pytest.raises(RuntimeError, match='authorization denied'): access.open_usbmon(1, timeout=1)
    command, kwargs = calls[0]
    assert command[:2] == ['sudo', '--'] and '-n' not in command
    assert kwargs['close_fds'] and 'start_new_session' not in kwargs and 'stdin' not in kwargs


def test_capture_cpu_uses_configured_exclusions_and_validates_request(monkeypatch):
    monkeypatch.setattr(capture.os, 'sched_getaffinity', lambda pid: {0, 1, 2, 3})
    assert capture.passive_cpu({0, 2, 3}) == 1
    assert capture.passive_cpu({0, 2}, 3) == 3
    with pytest.raises(RuntimeError, match='unavailable or reserved'): capture.passive_cpu({2}, 2)
    with pytest.raises(RuntimeError, match='No separate CPU'): capture.passive_cpu({0, 1, 2, 3})


def test_named_interfaces_and_swapped_usb_channels_are_preserved(tmp_path):
    directory = run_reader(tmp_path, replay_events(), mapping={'can_interfaces': ['right_bus', 'left_bus'], 'usb_channels': [1, 0]})
    state = json.loads((directory / 'status.json').read_text())
    assert state['first_emcy']['bus'] == 'left_bus'
    observer = statistics.Observer(directory)
    with mock.patch.object(statistics.signal, 'signal'): observer.run()
    result = json.loads((directory / 'interval-statistics/statistics.json').read_text())
    assert result['buses']['left_bus']['submit_interval']['max_us'] == 5300
    assert result['buses']['right_bus']['submit_interval']['max_us'] == 5200


def test_foreground_opener_transfers_descriptor_without_sudo_cache(monkeypatch):
    # Execute the real opener protocol against /dev/null in an ordinary child.
    # Only device choice and privilege identity are simulated; no sudo is run.
    access = module('usb_access')
    original_open, original_popen = os.open, subprocess.Popen
    receive = access.receive_fd
    def open_file(path, flags, *args, **kwargs):
        if str(path).startswith('/dev/usbmon'): raise PermissionError('offline authorization required')
        return original_open(path, flags, *args, **kwargs)
    monkeypatch.setattr(access.os, 'open', open_file)
    monkeypatch.setattr(access, 'receive_fd', lambda peer: receive(peer, os.getuid()))
    prefix = "import os; original_open=os.open; os.geteuid=lambda:0; os.open=lambda path,flags:original_open('/dev/null',flags)\n"
    calls = []
    def opener(command, **kwargs):
        assert command[0] == 'sudo'
        calls.append(command)
        return original_popen([sys.executable, '-c', prefix + command[4], *command[5:]], **kwargs)
    monkeypatch.setattr(access.subprocess, 'Popen', opener)
    fd = access.open_usbmon(3, timeout=3)
    try:
        assert len(calls) == 1
        access.validate_fd(fd, 3)
        assert os.read(fd, 1) == b''
        assert not os.get_inheritable(fd)
    finally: os.close(fd)
