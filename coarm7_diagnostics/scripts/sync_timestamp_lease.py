#!/usr/bin/python3 -I
"""Hold the CoArm7 CAN TX timestamp setting until the launcher closes stdin."""
import ctypes
import fcntl
import os
import signal
import socket
import sys

BUSES = (b"can0", b"can1")
SIOCSHWTSTAMP = 0x89B0
SIOCGHWTSTAMP = 0x89B1
HWTSTAMP_TX_ON = 1


class TimestampConfig(ctypes.Structure):
    _fields_ = [(name, ctypes.c_int) for name in ("flags", "tx_type", "rx_filter")]


class IfRequest(ctypes.Structure):
    _fields_ = [("name", ctypes.c_char * 16), ("data", ctypes.c_void_p), ("pad", ctypes.c_char * 16)]


libc = ctypes.CDLL(None, use_errno=True)


def ioctl(sock, name, op, config):
    request = IfRequest(name, ctypes.addressof(config))
    if libc.ioctl(sock.fileno(), op, ctypes.byref(request)) < 0:
        raise OSError(f"{name.decode()}: {os.strerror(ctypes.get_errno())}")


def read_config(sock, name):
    config = TimestampConfig()
    ioctl(sock, name, SIOCGHWTSTAMP, config)
    return config.flags, config.tx_type, config.rx_filter


def write_config(sock, name, values):
    ioctl(sock, name, SIOCSHWTSTAMP, TimestampConfig(*values))


def hold(sock):
    original = {name: read_config(sock, name) for name in BUSES}
    enabled = {name: (flags, HWTSTAMP_TX_ON, rx) for name, (flags, _, rx) in original.items()}
    result = 0
    try:
        for name, values in enabled.items():
            if values != original[name]:
                write_config(sock, name, values)
        for name, values in enabled.items():
            if read_config(sock, name) != values:
                raise OSError(f"{name.decode()}: unexpected timestamp configuration after enabling")
        sys.stdout.buffer.write(b"1")
        sys.stdout.buffer.flush()
        sys.stdin.buffer.read(1)
    except OSError as error:
        print(f"CoArm7 timestamp lease: {error}", file=sys.stderr)
        result = 1
    finally:
        for name, values in original.items():
            try:
                if read_config(sock, name) != values:
                    write_config(sock, name, values)
            except OSError as error:
                print(f"CoArm7 timestamp restore: {error}", file=sys.stderr)
        for name, values in original.items():
            try:
                if read_config(sock, name) != values:
                    raise OSError(f"{name.decode()}: original timestamp configuration did not restore")
            except OSError as error:
                print(f"CoArm7 timestamp restore: {error}", file=sys.stderr)
                result = 2
    return result


def main():
    if len(sys.argv) != 1 or os.geteuid() != 0:
        print("CoArm7 timestamp lease requires root and accepts no arguments.", file=sys.stderr)
        return 1
    # Stay alive for normal launch signals until the launcher closes the pipe.
    for signum in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, signal.SIG_IGN)
    try:
        fd = os.open("/run/coarm7-sync-timestamp.lock", os.O_CREAT | os.O_RDONLY | os.O_NOFOLLOW, 0o600)
        with os.fdopen(fd) as lock, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                print("CoArm7 timestamp lease already held.", file=sys.stderr)
                return 2
            return hold(sock)
    except OSError as error:
        print(f"CoArm7 timestamp lease: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
