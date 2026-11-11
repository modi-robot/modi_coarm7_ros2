#!/usr/bin/env python3
"""Reuse the established MoveIt regression, then test trajectory cancellation."""
import argparse
import json
from pathlib import Path
import runpy
import time
import xml.etree.ElementTree as ET
import yaml
import rclpy
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from rcl_interfaces.srv import GetParameters
from controller_manager_msgs.srv import SwitchController
from action_msgs.msg import GoalStatus

# Executables are installed in lib/<package>, not the package share directory.
from ament_index_python.packages import get_package_prefix
base = runpy.run_path(str(Path(get_package_prefix('coarm7_bringup')) / 'lib/coarm7_bringup/validate_mock.py'))

class Validator(base['Validator']):
    def __init__(self):
        super().__init__('mock')
        self.expected_plugin = 'coarm7_hardware/CoArm7System'

    def parameter(self, node, name):
        client = self.create_client(GetParameters, node + '/get_parameters')
        if not client.wait_for_service(timeout_sec=30):
            raise RuntimeError('Missing parameter service: ' + node)
        req = GetParameters.Request(); req.names = [name]
        return self.wait_future(client.call_async(req)).values[0]

    def wait_ready(self):
        value = self.parameter('/robot_state_publisher', 'robot_description').string_value
        model = ET.fromstring(value)
        path = model.find("ros2_control/hardware/param[@name='config_file']").text
        if yaml.safe_load(Path(path).read_text())['transport'] != 'fake':
            raise RuntimeError('Offline validation refuses non-fake transport')
        rate = self.parameter('/controller_manager', 'update_rate').integer_value
        if rate != 200:
            raise RuntimeError(f'Expected 200 Hz, found {rate}')
        return super().wait_ready()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def cancel_test(self, side, gripper=False):
        if gripper: return self.gripper_cancel_test(side)
        names = [f'{side}_4C2_r_1_joint'] if gripper else [f'{side}_joint{i}' for i in range(1, 8)]
        controller = ('left' if side == 'L' else 'right') + ('_gripper_controller' if gripper else '_arm_controller')
        client = ActionClient(self, FollowJointTrajectory, f'/{controller}/follow_joint_trajectory')
        if not client.wait_for_server(timeout_sec=15):
            raise RuntimeError('Missing trajectory action')
        initial = self.snapshot()
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = names
        first = JointTrajectoryPoint(); first.positions = [initial[j] for j in names]
        first.time_from_start.nanosec = 100000000
        last = JointTrajectoryPoint(); last.positions = list(first.positions)
        last.positions[0] += (0.2 if initial[names[0]] < 0.4 else -0.2) if gripper else (0.08 if side == 'L' else -0.08)
        last.time_from_start.sec = 5
        goal.trajectory.points = [first, last]
        handle = self.wait_future(client.send_goal_async(goal))
        if not handle.accepted: raise RuntimeError('Cancellation test trajectory rejected')
        self.spin_for(0.5)
        cancel = self.wait_future(handle.cancel_goal_async())
        result = self.wait_future(handle.get_result_async())
        if not cancel.goals_canceling or result.status != GoalStatus.STATUS_CANCELED:
            raise RuntimeError('Trajectory did not enter CANCELED')
        self.spin_for(0.3); held = self.snapshot(); self.spin_for(0.5); after = self.snapshot()
        drift = max(abs(after[j]-held[j]) for j in names)
        if drift > 0.003: raise RuntimeError(f'Cancelled trajectory continued moving: {drift}')
        self.records.append(dict(name=f'{side}_{"gripper_" if gripper else ""}trajectory_cancel_hold', status='passed', drift_rad=drift))

    def gripper_cancel_test(self, side):
        controller = ('left' if side == 'L' else 'right') + '_gripper_controller'
        name = side + '_4C2_r_1_joint'
        client = ActionClient(self, GripperCommand, f'/{controller}/gripper_cmd')
        if not client.wait_for_server(timeout_sec=15): raise RuntimeError('Missing GripperCommand action')
        initial = self.snapshot()[name]
        target = 0.0 if initial > 0.4 else 0.8
        goal = GripperCommand.Goal(); goal.command.position = target
        handle = self.wait_future(client.send_goal_async(goal))
        if not handle.accepted: raise RuntimeError('Gripper endpoint rejected')
        self.spin_for(0.15)
        cancel = self.wait_future(handle.cancel_goal_async())
        result = self.wait_future(handle.get_result_async())
        if not cancel.goals_canceling or result.status != GoalStatus.STATUS_CANCELED:
            raise RuntimeError('Endpoint did not enter CANCELED')
        self.spin_for(0.2); held = self.snapshot()[name]
        self.spin_for(0.5); after = self.snapshot()[name]
        if abs(after-held)>0.003: raise RuntimeError('Canceled endpoint continued moving or rebounded')
        if abs(held-initial)<0.01 or abs(held-target)<0.01:
            raise RuntimeError('Cancellation was not exercised during motion')
        # A fresh goal after a stop must work, including the same requested endpoint.
        handle = self.wait_future(client.send_goal_async(goal))
        if not handle.accepted: raise RuntimeError('Endpoint did not resume after cancellation')
        result = self.wait_future(handle.get_result_async())
        if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.reached_goal:
            raise RuntimeError('Repeated endpoint after cancellation failed')
        self.records.append(dict(name=side+'_gripper_endpoint_cancel_stop_resume', status='passed', drift_rad=abs(after-held), stopped_position_rad=held))

    def controller_switch_test(self, controller='left_arm_controller'):
        client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        if not client.wait_for_service(timeout_sec=15): raise RuntimeError('Missing switch service')
        req = SwitchController.Request(); req.deactivate_controllers = [controller]; req.strictness = 2
        if not self.wait_future(client.call_async(req)).ok: raise RuntimeError('Controller stop failed')
        self.spin_for(0.2); before = self.snapshot(); self.spin_for(0.3); after = self.snapshot()
        drift = max(abs(after[j]-before[j]) for j in base['ACTIVE_JOINTS'])
        req = SwitchController.Request(); req.activate_controllers = [controller]; req.strictness = 2
        if not self.wait_future(client.call_async(req)).ok: raise RuntimeError('Controller restart failed')
        self.spin_for(0.2)
        if drift > 0.003: raise RuntimeError('Controller stop did not hold')
        self.spin_for(0.5); reactivated = self.snapshot()
        reactivation_drift = max(abs(reactivated[j]-after[j]) for j in base['ACTIVE_JOINTS'])
        if reactivation_drift > 0.003: raise RuntimeError('Controller reactivation replayed an old goal')
        self.records.append(dict(name=f'{controller}_deactivate_reactivate', status='passed',
                                 drift_rad=drift, reactivation_drift_rad=reactivation_drift))

    def run(self):
        report = super().run()
        self.cancel_test('L'); self.cancel_test('R'); self.controller_switch_test()
        self.cancel_test('L', gripper=True); self.cancel_test('R', gripper=True)
        self.controller_switch_test('left_gripper_controller')
        self.controller_switch_test('right_gripper_controller')
        report.update(transport='fake', controller_rate_hz=200, tests=self.records, final_positions=self.snapshot())
        return report

def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--report', required=True)
    args = parser.parse_args(); rclpy.init(); validator = Validator()
    try:
        report = validator.run()
    except Exception as error:
        report = dict(status='failed', error=str(error), tests=validator.records)
        raise
    finally:
        Path(args.report).parent.mkdir(parents=True, exist_ok=True)
        Path(args.report).write_text(json.dumps(report, indent=2)+'\n')
        validator.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
