#!/usr/bin/env python3
"""Read-only usbmon recorder. The supplied fd is opened before dropping root."""
import argparse, collections, ctypes, datetime, errno, fcntl, json, os, pwd
from pathlib import Path
import select, signal, struct, time

HEADER = struct.Struct("=QBBBBHBBqiiII8siiII")
GETX = (1 << 30) | (24 << 16) | (0x92 << 8) | 10
STATS = (2 << 30) | (8 << 16) | (0x92 << 8) | 3
RING_SET = (0x92 << 8) | 4
RING_GET = (0x92 << 8) | 5
CAPACITY = 65536
DLC = [0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64]

def records(payload):
    off = 0
    while off + 4 <= len(payload):
        size, kind = struct.unpack_from("<HH", payload, off)
        if size < 4 or off + size > len(payload):
            return
        yield kind, payload[off:off+size]
        off += size

def local_time(ns):
    return datetime.datetime.fromtimestamp(ns / 1e9).astimezone().isoformat(timespec="microseconds")

def snapshot_ros_logs(root, controller_pid, launch_pid):
    # sudo may leave HOME=/root after setuid. Resolve the actual reader user.
    logs = Path(pwd.getpwuid(os.getuid()).pw_dir) / ".ros/log"
    result = {"directory": str(logs), "files": [], "errors": []}

    def copy_latest(pattern, destination=None, launch=False):
        try:
            candidates = list(logs.glob(pattern))
            if launch:
                candidates = [p / "launch.log" for p in candidates if p.is_dir()]
            candidates = [p for p in candidates if p.is_file()]
            candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)
            if candidates:
                source = candidates[0]
                target = root / (destination or ("snapshot-" + source.name))
                target.write_bytes(source.read_bytes())
                result["files"].append(target.name)
        except OSError as error:
            # Log rotation, inaccessible files or a full disk must not mask
            # the capture result or turn a normal shutdown into an error.
            result["errors"].append({"pattern": pattern, "error": str(error)})

    copy_latest("ros2_control_node_%d_*.log" % controller_pid)
    copy_latest("move_group_*.log")
    if launch_pid:
        copy_latest("*-%d" % launch_pid, "snapshot-launch.log", launch=True)
    return result

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fd", type=int, required=True)
    ap.add_argument("--directory", required=True)
    ap.add_argument("--bus", type=int, required=True)
    ap.add_argument("--device", type=int, required=True)
    ap.add_argument("--controller-pid", type=int, required=True)
    ap.add_argument("--launch-pid", type=int)
    args = ap.parse_args()
    root = Path(args.directory).resolve()
    mapping_file = root / 'supervisor.json'
    mapping = json.loads(mapping_file.read_text()) if mapping_file.exists() else {}
    interfaces = mapping.get('can_interfaces', ['can0', 'can1'])
    channels = mapping.get('usb_channels', [0, 1])
    channel_index = {physical: index for index, physical in enumerate(channels)}
    fd = args.fd
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    controller = Path("/proc") / str(args.controller_pid)
    controller_stat = (controller / "stat").read_text().split(") ",1)[1].split()[19]
    state = {
        "pid": os.getpid(), "uid": os.getuid(), "gid": os.getgid(),
        "cpu_affinity": sorted(os.sched_getaffinity(0)),
        "scheduler": os.sched_getscheduler(0), "nice": os.getpriority(os.PRIO_PROCESS,0),
        "started_at": datetime.datetime.now().astimezone().isoformat(),
        "bus": args.bus, "device": args.device, "can_interfaces": interfaces, "usb_channels": channels, "controller_pid": args.controller_pid,
        "raw_header_bytes": HEADER.size, "raw_format": HEADER.format,
        "retention": "14 segments of 5 seconds; 65-70 seconds normally, then 3 seconds post EMCY",
        "other_device_payloads": "redacted; event headers only",
        "can_sockets_opened": False, "usb_device_commands_sent": False,
        "events": 0, "adapter_events": 0, "dropped": 0, "truncated": 0,
        "completion_errors": 0, "sync_rows": 0, "first_emcy": None,
        "usb_out": {}, "sync": {}, "status": "starting"
    }
    try:
        fcntl.ioctl(fd, RING_SET, 1048576)
        state["ring_request_bytes"] = 1048576
    except OSError as e:
        state["ring_request_error"] = str(e)
    state["ring_actual_bytes"] = fcntl.ioctl(fd, RING_GET)
    (root / "pid").write_text(str(os.getpid()) + "\n")
    raw = ctypes.create_string_buffer(CAPACITY)
    header = ctypes.create_string_buffer(64)
    request = bytearray(struct.pack("=QQQ", ctypes.addressof(header), ctypes.addressof(raw), CAPACITY))
    pending = {}
    previous_submit, previous_complete = {}, {}
    history = collections.deque()
    segment_id = 0
    segment_opened = None
    binary = csv = None
    fault_deadline = None
    next_status = 0.0
    last_controller_check = 0.0
    reason = "stopped"
    def rotate(now):
        nonlocal binary, csv, segment_id, segment_opened
        if binary:
            binary.close()
            csv.close()
        segment_id += 1
        base = root / ("segment-%06d" % segment_id)
        bp, cp = base.with_suffix(".usb.bin"), base.with_suffix(".sync.csv")
        binary = bp.open("wb", buffering=1024*1024)
        csv = cp.open("w", buffering=65536)
        csv.write("bus,counter,urb_id,submit_ns,complete_ns,usb_duration_us,submit_interval_us,complete_interval_us,status\n")
        history.append((bp,cp))
        segment_opened = now
        if not state["first_emcy"]:
            while len(history) > 14:
                for old in history.popleft():
                    assert old.parent == root
                    old.unlink()
        state["current_segment"] = bp.name

    def save_status(final=False):
        stats = bytearray(8)
        fcntl.ioctl(fd, STATS, stats)
        queued, dropped = struct.unpack("=II", stats)
        state["queued"] = queued
        state["dropped"] += dropped
        state["updated_at"] = datetime.datetime.now().astimezone().isoformat()
        state["pending_adapter_out"] = len(pending)
        if final:
            state["status"] = "complete"
            state["reason"] = reason
        else:
            state["status"] = "post_fault" if state["first_emcy"] else "capturing"
        binary.flush()
        csv.flush()
        temp = root / "status.json.tmp"
        temp.write_text(json.dumps(state, indent=2) + "\n")
        temp.replace(root / "status.json")

    rotate(time.monotonic())
    try:
        while running:
            now = time.monotonic()
            if fault_deadline is not None and now >= fault_deadline:
                reason = "first_emcy_plus_3_seconds" if state["first_emcy"] else "controller_exited_plus_3_seconds"
                break
            if now - last_controller_check >= 1:
                last_controller_check = now
                try:
                    current = (controller / "stat").read_text().split(") ",1)[1].split()[19]
                    alive = current == controller_stat
                except OSError:
                    alive = False
                if not alive and fault_deadline is None:
                    state["controller_exit_at"] = datetime.datetime.now().astimezone().isoformat()
                    fault_deadline = now + 3
                    reason = "controller_exited_plus_3_seconds"
            if now - segment_opened >= 5:
                rotate(now)
            if now >= next_status:
                save_status()
                next_status = now + 1
            ready, _, _ = select.select([fd], [], [], .1)
            if not ready:
                continue
            for _ in range(400):
                try:
                    fcntl.ioctl(fd, GETX, request, True)
                except OSError as e:
                    if e.errno in (errno.EAGAIN, errno.EINTR):
                        break
                    raise
                values = list(HEADER.unpack(header.raw))
                urb, event, transfer, ep, device, bus = values[:6]
                ns = values[8]*1000000000 + values[9]*1000
                status, cap = values[10], values[12]
                size = min(cap, CAPACITY)
                state["events"] += 1
                if cap > CAPACITY:
                    state["truncated"] += 1
                adapter = bus == args.bus and device == args.device and transfer == 3
                if not adapter:
                    values[7] = 1
                    values[12] = 0
                    values[13] = bytes(8)
                    binary.write(HEADER.pack(*values))
                    continue
                state["adapter_events"] += 1
                payload = ctypes.string_at(ctypes.addressof(raw), size)
                values[12] = size
                binary.write(HEADER.pack(*values))
                binary.write(payload)
                if event in (ord("C"),ord("E")) and status != 0:
                    state["completion_errors"] += 1
                out = not (ep & 0x80)
                if event == ord("S") and out:
                    syncs = []
                    for kind, rec in records(payload):
                        if kind == 0x1000 and len(rec) >= 22:
                            cid = struct.unpack_from("<I",rec,16)[0] & 0x1fffffff
                            if cid == 0x03f and rec[20] == 0x3c:
                                physical = rec[12] & 0xf
                                if physical in channel_index: syncs.append((channel_index[physical], rec[21]))
                    pending[(urb,ep)] = (ns,syncs)
                elif event in (ord("C"),ord("E")) and out:
                    item = pending.pop((urb,ep),None)
                    if item is not None:
                        submitted, syncs = item
                        duration = (ns-submitted)/1000
                        key = "0x%02x" % ep
                        ag = state["usb_out"].setdefault(key,{"count":0,"max_us":0,"ge_300_us":0,"ge_500_us":0})
                        ag["count"] += 1
                        ag["max_us"] = max(ag["max_us"],duration)
                        ag["ge_300_us"] += duration >= 300
                        ag["ge_500_us"] += duration >= 500
                        for channel,counter in syncs:
                            si = (submitted-previous_submit[channel])/1000 if channel in previous_submit else 0
                            ci = (ns-previous_complete[channel])/1000 if channel in previous_complete else 0
                            previous_submit[channel],previous_complete[channel]=submitted,ns
                            sg = state["sync"].setdefault(str(channel),{"count":0,"max_usb_us":0,"max_submit_interval_us":0,"max_complete_interval_us":0})
                            sg["count"] += 1
                            sg["max_usb_us"] = max(sg["max_usb_us"],duration)
                            sg["max_submit_interval_us"] = max(sg["max_submit_interval_us"],si)
                            sg["max_complete_interval_us"] = max(sg["max_complete_interval_us"],ci)
                            csv.write("%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%d\n"%(channel,counter,urb,submitted,ns,duration,si,ci,status))
                            state["sync_rows"] += 1
                elif event == ord("C") and not out and status == 0:
                    for kind, rec in records(payload):
                        if kind != 1 or len(rec) < 30:
                            continue
                        physical = rec[20] & 0xf
                        if physical not in channel_index: continue
                        channel = channel_index[physical]
                        cid = struct.unpack_from("<I",rec,24)[0] & 0x1fffffff
                        if not (0x81 <= cid <= 0x8e):
                            continue
                        fault = int.from_bytes(rec[28:30],"little")
                        if fault == 0:
                            continue
                        emcy = {"host_ns":ns,"host_time":local_time(ns),"hardware_rx_us":struct.unpack_from("<Q",rec,4)[0],
                                "bus":interfaces[channel],"node_id":cid-0x80,"can_id":"0x%03x"%cid,
                                "code":"0x%04X"%fault,"data_hex":rec[28:].hex(),"usb_urb_id":urb}
                        with (root/"emcy.jsonl").open("a") as f:
                            f.write(json.dumps(emcy)+"\n")
                        if state["first_emcy"] is None:
                            state["first_emcy"] = emcy
                            fault_deadline = time.monotonic()+3
                            (root/"FIRST_EMCY.json").write_text(json.dumps(emcy,indent=2)+"\n")
            # Bound stale pending entries without affecting recently submitted URBs.
            if len(pending) > 20000:
                cutoff = time.time_ns()-30000000000
                pending = {k:v for k,v in pending.items() if v[0] >= cutoff}
    except BaseException as e:
        reason = "capture_error"
        state["error"] = repr(e)
        raise
    finally:
        try:
            save_status(final=True)
        finally:
            binary.close()
            csv.close()
            os.close(fd)
            # Snapshot existing files only; never contact ROS or CAN here.
            state["ros_log_snapshot"] = snapshot_ros_logs(root, args.controller_pid, args.launch_pid)
            temp = root / "status.json.tmp"
            temp.write_text(json.dumps(state, indent=2) + "\n")
            temp.replace(root / "status.json")

if __name__ == "__main__":
    main()
