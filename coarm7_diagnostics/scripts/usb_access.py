#!/usr/bin/env python3
"""Acquire one read-only usbmon FD in the foreground; never open CAN/USB control.

Only the small opener runs under sudo. SCM_RIGHTS transfers the already-open
monitor to the normal user. No sudo timestamp is needed by background children.
"""
import array
import fcntl
import os
from pathlib import Path
import socket
import stat
import struct
import subprocess
import sys
import tempfile
import time

ROOT_OPEN = r'''
import array,os,socket,stat,sys
bus=int(sys.argv[1]); address=sys.argv[2]
if os.geteuid()!=0 or not 1<=bus<=999:
    raise SystemExit("Invalid usbmon request")
fd=os.open("/dev/usbmon%d"%bus,os.O_RDONLY|os.O_NONBLOCK|os.O_NOFOLLOW)
try:
    info=os.fstat(fd)
    if not stat.S_ISCHR(info.st_mode) or os.minor(info.st_rdev)!=bus:
        raise SystemExit("Expected selected usbmon character device")
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as peer:
        peer.connect(address)
        peer.sendmsg([b"U"],[(socket.SOL_SOCKET,socket.SCM_RIGHTS,array.array("i",[fd]))])
finally:
    os.close(fd)
'''


def validate_fd(fd, bus):
    info = os.fstat(fd)
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    if (not stat.S_ISCHR(info.st_mode) or os.minor(info.st_rdev) != bus
            or flags & os.O_ACCMODE != os.O_RDONLY or not flags & os.O_NONBLOCK):
        raise RuntimeError('Expected a read-only nonblocking usbmon descriptor for bus ' + str(bus))
    os.set_inheritable(fd, False)


def receive_fd(peer, expected_uid=0):
    received = []
    try:
        _, uid, _ = struct.unpack('3i', peer.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
        if uid != expected_uid:
            raise RuntimeError('Unexpected USB opener identity')
        data, ancillary, flags, _ = peer.recvmsg(1, socket.CMSG_SPACE(16), socket.MSG_CMSG_CLOEXEC)
        for level, kind, payload in ancillary:
            if level == socket.SOL_SOCKET and kind == socket.SCM_RIGHTS:
                values = array.array('i'); values.frombytes(payload[:len(payload) - len(payload) % values.itemsize])
                received.extend(values)
        if data != b'U' or flags & socket.MSG_CTRUNC or len(received) != 1:
            raise RuntimeError('USB opener did not return exactly one descriptor')
        return received.pop()
    finally:
        for fd in received: os.close(fd)


def open_usbmon(bus, timeout=120):
    if type(bus) is not int or not 1 <= bus <= 999: raise ValueError('Invalid USB bus')
    try:
        fd = os.open('/dev/usbmon%d' % bus, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
    except PermissionError:
        fd = None
    if fd is not None:
        try: validate_fd(fd, bus)
        except BaseException:
            os.close(fd); raise
        return fd
    print('CoArm7 USB: authorizing read-only usbmon before ROS startup (sudo may prompt).', flush=True)
    with tempfile.TemporaryDirectory(prefix='coarm7-usb-access-') as temporary, \
            socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server, \
            tempfile.TemporaryFile(mode='w+t') as errors:
        address = str(Path(temporary) / 'fd.sock')
        server.bind(address); server.listen(1); server.settimeout(.1)
        # Keep the caller's terminal/session for authentication. No password is
        # stored; the background recorder never invokes sudo or executes as root.
        process = subprocess.Popen(['sudo', '--', sys.executable, '-c', ROOT_OPEN, str(bus), address],
                                   stdout=subprocess.DEVNULL, stderr=errors, close_fds=True)
        result = None
        try:
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    peer, _ = server.accept()
                    with peer:
                        peer.settimeout(5)
                        result = receive_fd(peer)
                    validate_fd(result, bus)
                    if process.wait(timeout=5): raise RuntimeError('USB opener failed after descriptor transfer')
                    answer, result = result, None
                    return answer
                except socket.timeout:
                    if process.poll() is not None: break
            errors.seek(0)
            detail = errors.read().strip()
            raise RuntimeError('USB read authorization failed before ROS startup: ' +
                               (detail or 'authorization timed out; run from an interactive terminal'))
        finally:
            if result is not None: os.close(result)
            if process.poll() is None:
                process.terminate()
                try: process.wait(timeout=3)
                except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=3)
