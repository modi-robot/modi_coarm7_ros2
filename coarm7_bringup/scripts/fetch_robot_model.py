#!/usr/bin/env python3
"""Read the running move_group model for remote RViz; no motion/lifecycle calls."""
import argparse
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET


def fetch(timeout=25):
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterType
    rclpy.init()
    node = rclpy.create_node('coarm7_remote_model_reader')
    try:
        client = node.create_client(GetParameters, '/move_group/get_parameters')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=min(1, max(.01, deadline-time.monotonic()))): continue
            request = GetParameters.Request(names=['robot_description', 'robot_description_semantic'])
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=max(.01, deadline-time.monotonic()))
            if not future.done() or future.result() is None: break
            values = future.result().values
            if len(values) == 2 and all(v.type == ParameterType.PARAMETER_STRING and v.string_value for v in values):
                result = dict(zip(request.names, [v.string_value for v in values]))
                for text in result.values():
                    if ET.fromstring(text).tag != 'robot': raise RuntimeError('Invalid live robot model')
                return result
            time.sleep(.1)
        raise RuntimeError('Live MoveIt model unavailable; wait for controller configure and limit synchronization')
    finally:
        node.destroy_node(); rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=25)
    args = parser.parse_args()
    result = fetch(args.timeout)
    args.output.write_text(json.dumps(result))


if __name__ == '__main__':
    try: main()
    except Exception as error:
        print('CoArm7 remote model: ' + str(error), file=sys.stderr); raise SystemExit(1)
