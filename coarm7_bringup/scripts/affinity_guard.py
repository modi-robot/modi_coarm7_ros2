#!/usr/bin/env python3
"""Verify explicit Linux affinity in a child before exec; never access ROS/CAN."""
import argparse
import json
import os
import sys


def parse_cpus(value):
    import re
    result = set()
    if not value: return result
    for part in value.split(','):
        if not re.fullmatch(r'\d+(?:-\d+)?', part): raise ValueError('Invalid CPU list')
        ends = [int(v) for v in part.split('-')]
        if ends[-1] < ends[0] or ends[-1] > 4095: raise ValueError('Invalid CPU range')
        result.update(range(ends[0], ends[-1] + 1))
    return result


def bind_exact(cpus, label):
    try:
        os.sched_setaffinity(0, cpus)
    except OSError as error:
        raise RuntimeError(f'{label}: kernel rejected CPUs {sorted(cpus)}: {error}; check online CPUs and cgroup/cpuset restrictions') from error
    actual = set(os.sched_getaffinity(0))
    if actual != cpus:
        raise RuntimeError(f'{label}: requested CPUs {sorted(cpus)}, actual {sorted(actual)}; kernel/cpuset filtered the request')
    return actual


def probe(cpus, control_cpu, sdk_cpus=()):
    inherited = set(os.sched_getaffinity(0))
    try:
        if cpus: bind_exact(cpus, 'process_cpus')
        # The update thread may explicitly bind outside the process default.
        if control_cpu >= 0: bind_exact({control_cpu}, 'control_cpu')
        for cpu in sdk_cpus: bind_exact({cpu}, 'SDK thread CPU')
    finally:
        bind_exact(inherited, 'restore probe affinity')
    return {'inherited': sorted(inherited), 'process_requested': sorted(cpus),
            'control_cpu': control_cpu, 'sdk_cpus': list(sdk_cpus), 'kernel_binding_verified': True}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cpus', default='')
    parser.add_argument('--control-cpu', type=int, default=-1)
    parser.add_argument('--probe-cpu', type=int, action='append', default=[])
    parser.add_argument('--probe-only', action='store_true')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    cpus = parse_cpus(args.cpus)
    if args.control_cpu < -1: raise ValueError('control_cpu must be -1 or non-negative')
    if any(cpu < 0 or cpu > 4095 for cpu in args.probe_cpu): raise ValueError('Invalid SDK thread CPU')
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if args.probe_only and command: raise ValueError('Probe mode cannot execute a command')
    if not args.probe_only and not command: raise ValueError('Command is required')
    result = probe(cpus, args.control_cpu, args.probe_cpu)
    if not args.probe_only:
        actual = bind_exact(cpus, 'process_cpus before exec') if cpus else set(os.sched_getaffinity(0))
        result['process_actual_before_exec'] = sorted(actual)
    print('CoArm7 CPU affinity: ' + json.dumps(result, sort_keys=True), flush=True)
    if not args.probe_only: os.execvp(command[0], command)
    return 0


if __name__ == '__main__':
    try: raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as error:
        print('CoArm7 CPU affinity failed: ' + str(error), file=sys.stderr)
        raise SystemExit(1)
