#!/usr/bin/env python3
"""Passive gripper reference/feedback windows. No commands or CAN access."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import shutil
import time

TERMINAL = {4: 'succeeded', 5: 'canceled', 6: 'aborted'}
CONTROLLERS = ('left_gripper_controller', 'right_gripper_controller')


def finite(values):
    return [float(x) if math.isfinite(x) else None for x in values]


def trajectory_point(message, *names):
    """Humble state aliases populate different old/new fields."""
    for name in names:
        point = getattr(message, name, None)
        if point is not None and len(point.positions):
            return point
    return None


class TraceWindows:
    """Bounded pre-event history plus independent post-event windows."""
    def __init__(self, before=2.0, after=3.0, capacity=2000, pending_limit=8):
        self.before, self.after = before, after
        self.history = deque(maxlen=capacity)
        self.pending, self.states = {}, {}
        self.pending_limit = pending_limit
        self.overflow = 0

    def sample(self, record):
        self.history.append(record)
        for window in self.pending.values():
            if record['received_monotonic_s'] <= window['deadline']:
                window['samples'].append(record)

    def status(self, controller, goal, status, now, wall_ns):
        key = (controller, goal)
        previous = self.states.get(key)
        if previous == status:
            return None
        self.states[key] = status
        if len(self.states) > 512:
            # Keep bounded history of old goal IDs. Active goals are retained.
            stale = next((k for k, v in self.states.items() if v in TERMINAL and k != key), None)
            if stale is not None:
                self.states.pop(stale)
        event = {'controller': controller, 'goal_id': goal, 'status': status,
                 'status_name': TERMINAL.get(status, str(status)),
                 'previous_status': previous, 'received_monotonic_s': now,
                 'received_wall_ns': wall_ns}
        if status in TERMINAL:
            if len(self.pending) >= self.pending_limit:
                self.overflow += 1
                event['window_skipped'] = 'pending limit reached'
            else:
                self.pending[key] = {
                    'event': event, 'deadline': now + self.after,
                    'samples': [r for r in self.history
                                if r['received_monotonic_s'] >= now - self.before],
                }
        return event

    def ready(self, now, shutdown=False):
        result = []
        for key, window in list(self.pending.items()):
            if shutdown or now >= window['deadline']:
                window['post_window_complete'] = now >= window['deadline']
                result.append(window)
                del self.pending[key]
        return result


class WindowWriter:
    def __init__(self, directory, maximum=128, free_bytes=256 * 1024 * 1024):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.maximum, self.free_bytes, self.saved = maximum, free_bytes, 0
        self.events = (self.directory / 'events.jsonl').open('a', buffering=1)

    def event(self, event):
        self.events.write(json.dumps(event, allow_nan=False) + '\n')

    def write(self, window):
        event = window['event']
        if self.saved >= self.maximum or shutil.disk_usage(self.directory).free < self.free_bytes:
            self.event({'window_not_saved': event, 'reason': 'window/free-space limit',
                        'existing_windows_preserved': True})
            return False
        self.saved += 1
        name = '%04d-%s-%s-%s.json' % (
            self.saved, event['controller'], event['status_name'], event['goal_id'][:16])
        target = self.directory / name
        temporary = target.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(window, allow_nan=False) + '\n')
        temporary.replace(target)
        return True

    def close(self):
        self.events.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--avoid-cpu', nargs='*', type=int, default=[])
    from rclpy.utilities import remove_ros_args
    args = parser.parse_args(remove_ros_args()[1:])
    # Ordinary, low-priority observer, outside the CAN and controller RT CPUs.
    allowed = os.sched_getaffinity(0)
    candidates = allowed - set(args.avoid_cpu)
    if candidates:
        os.sched_setaffinity(0, {min(candidates)})
    os.nice(10)
    import yaml
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from control_msgs.msg import JointTrajectoryControllerState
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.executors import ExternalShutdownException

    config = yaml.safe_load(Path(args.config).read_text())
    writer = WindowWriter(args.directory)
    metadata = {
        'schema': 1, 'started_wall_ns': time.time_ns(), 'started_monotonic_s': time.monotonic(),
        'control_commands_sent': False, 'can_access': False,
        'sample_semantics': 'JTC desired/reference and actual/feedback ROS angles; not wire targets',
        'time_semantics': 'same-host receipt monotonic/wall time; message ROS stamp recorded separately',
        'before_s': 2, 'after_s': 3, 'maximum_windows': 128,
        'retention': 'existing windows are never deleted by this observer',
        'grippers': config.get('grippers', []), 'cpu_affinity': sorted(os.sched_getaffinity(0)),
        'scheduler': os.sched_getscheduler(0), 'nice': os.getpriority(os.PRIO_PROCESS, 0),
    }
    (Path(args.directory) / 'session.json').write_text(json.dumps(metadata, indent=2) + '\n')
    trace = TraceWindows()
    rclpy.init()
    node = rclpy.create_node('coarm7_gripper_trace')
    node.get_logger().info('Passive gripper target/feedback trace: ' + args.directory)
    signatures = {}
    subscriptions = []

    def sample(controller, msg, topic):
        desired = trajectory_point(msg, 'reference', 'desired')
        actual = trajectory_point(msg, 'feedback', 'actual')
        if desired is None or actual is None:
            return
        stamp = [msg.header.stamp.sec, msg.header.stamp.nanosec]
        signature = (tuple(stamp), tuple(desired.positions), tuple(actual.positions))
        # Humble may offer both state and controller_state aliases.
        if signatures.get(controller) == signature:
            return
        signatures[controller] = signature
        trace.sample({
            'controller': controller, 'topic': topic,
            'received_monotonic_s': time.monotonic(), 'received_wall_ns': time.time_ns(),
            'message_ros_stamp': stamp, 'joint_names': list(msg.joint_names),
            'reference_position_rad': finite(desired.positions),
            'reference_velocity_rad_s': finite(desired.velocities),
            'reference_acceleration_rad_s2': finite(desired.accelerations),
            'feedback_position_rad': finite(actual.positions),
            'feedback_velocity_rad_s': finite(actual.velocities),
            'error_position_rad': finite(msg.error.positions),
        })

    def status(controller, msg):
        for item in msg.status_list:
            goal = bytes(item.goal_info.goal_id.uuid).hex()
            event = trace.status(controller, goal, item.status, time.monotonic(), time.time_ns())
            if event:
                writer.event(event)

    state_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
    status_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for controller in CONTROLLERS:
        for suffix in ('controller_state', 'state'):
            topic = '/' + controller + '/' + suffix
            subscriptions.append(node.create_subscription(
                JointTrajectoryControllerState, topic,
                lambda msg, c=controller, t=topic: sample(c, msg, t), state_qos))
        subscriptions.append(node.create_subscription(
            GoalStatusArray, '/' + controller + '/gripper_cmd/_action/status',
            lambda msg, c=controller: status(c, msg), status_qos))

    def flush():
        for window in trace.ready(time.monotonic()):
            writer.write(window)
    timer = node.create_timer(0.2, flush)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for window in trace.ready(time.monotonic(), shutdown=True):
            writer.write(window)
        writer.event({'observer_exit': True, 'saved_windows': writer.saved,
                      'pending_limit_hits': trace.overflow})
        writer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
