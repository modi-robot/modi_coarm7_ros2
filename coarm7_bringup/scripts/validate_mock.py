#!/usr/bin/env python3
"""Exercise MoveGroup planning AND execution, then verify controller feedback."""
import argparse
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, MoveItErrorCodes, OrientationConstraint,
    PositionConstraint, RobotState,
)
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

JOINTS = [f"{side}_joint{i}" for side in ("L", "R") for i in range(1, 8)]
GRIPPERS = [f"{side}_4C2_r_1_joint" for side in ("L", "R")]
ACTIVE_JOINTS = JOINTS + GRIPPERS
MIMICS = {f"{side}_4C2_{part}_joint": f"{side}_4C2_r_1_joint"
          for side in ("L", "R") for part in ("l_1", "l_3", "r_3", "l_2", "r_2")}
FEEDBACK_JOINTS = ACTIVE_JOINTS + list(MIMICS)


class Validator(Node):
    def __init__(self, backend="mock"):
        super().__init__("coarm7_mock_validation")
        self.backend = backend
        if backend != 'mock': raise ValueError('Only mock validation is supported')
        self.expected_plugin = 'mock_components/GenericSystem'
        self.positions = {}
        self.last_feedback = 0.0
        self.feedback_names = set()
        self.feedback_stamp = 0.0
        self.create_subscription(JointState, "/joint_states", self.feedback,
                                 qos_profile_sensor_data)
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.controllers = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.hardware = self.create_client(ListHardwareComponents, "/controller_manager/list_hardware_components")
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.records = []
        defaults = yaml.safe_load((Path(get_package_share_directory("coarm7_moveit_config"))
                                   / "config/joint_limits.yaml").read_text())
        self.velocity_scaling = float(defaults["default_velocity_scaling_factor"])
        self.acceleration_scaling = float(defaults["default_acceleration_scaling_factor"])
        semantic = ET.parse(Path(get_package_share_directory("coarm7_moveit_config"))
                            / "config/coarm7.srdf").getroot()
        closed = semantic.find("group_state[@name='closed'][@group='both_grippers']")
        self.gripper_closed = {joint.attrib['name']: float(joint.attrib['value']) for joint in closed}

    def feedback(self, msg):
        self.feedback_names.update(msg.name)
        self.feedback_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for name, position in zip(msg.name, msg.position):
            self.positions[name] = position
        self.last_feedback = time.monotonic()

    def wait_future(self, future, timeout=30.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError("ROS operation timed out")
        if future.exception():
            raise future.exception()
        return future.result()

    def snapshot(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(j in self.positions for j in FEEDBACK_JOINTS) and time.monotonic() - self.last_feedback < 1.0:
                return {j: self.positions[j] for j in FEEDBACK_JOINTS}
        raise RuntimeError("No fresh arm, gripper and mimic joint feedback")

    def state(self, positions):
        result = RobotState()
        # Send only independent positions; MoveIt derives the mimic positions.
        result.joint_state.name = [j for j in positions if j in ACTIVE_JOINTS]
        result.joint_state.position = [positions[j] for j in result.joint_state.name]
        return result

    def check_valid(self, positions):
        request = GetStateValidity.Request()
        request.robot_state = self.state(positions)
        request.group_name = ""  # Check the entire robot, including both grippers.
        response = self.wait_future(self.validity.call_async(request))
        if not response.valid:
            contacts = [(c.contact_body_1, c.contact_body_2) for c in response.contacts]
            raise RuntimeError(f"Invalid robot configuration: {contacts}")

    def get_pose(self, positions, link):
        request = GetPositionFK.Request()
        request.header.frame_id = "world"
        request.fk_link_names = [link]
        request.robot_state = self.state(positions)
        response = self.wait_future(self.fk.call_async(request))
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"FK failed: {response.error_code.val}")
        return response.pose_stamped[0]

    def goal(self, group, constraints):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = group
        request.pipeline_id = "ompl"
        request.planner_id = "RRTConnectkConfigDefault"
        request.allowed_planning_time = 5.0
        request.num_planning_attempts = 3
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling
        request.start_state.is_diff = True
        request.goal_constraints = [constraints]
        request.workspace_parameters.header.frame_id = "world"
        request.workspace_parameters.min_corner.x = -2.0
        request.workspace_parameters.min_corner.y = -2.0
        request.workspace_parameters.min_corner.z = -0.2
        request.workspace_parameters.max_corner.x = 2.0
        request.workspace_parameters.max_corner.y = 2.0
        request.workspace_parameters.max_corner.z = 2.0
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def execute(self, name, group, constraints):
        started = time.monotonic()
        handle = self.wait_future(self.move.send_goal_async(self.goal(group, constraints)))
        if not handle.accepted:
            raise RuntimeError(f"{name}: MoveGroup goal rejected")
        wrapped = self.wait_future(handle.get_result_async(), 60.0)
        result = wrapped.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"{name}: MoveGroup error {result.error_code.val}")
        planned = result.planned_trajectory.joint_trajectory
        executed = result.executed_trajectory.joint_trajectory
        if not planned.points:
            raise RuntimeError(f"{name}: empty planned trajectory")
        record = {
            "name": name, "group": group, "plan_only": False,
            "moveit_error_code": result.error_code.val,
            "action_status": wrapped.status,
            "planned_points": len(planned.points),
            "executed_points": len(executed.points),
            "trajectory_joints": list(planned.joint_names),
            "elapsed_seconds": round(time.monotonic() - started, 3),
            "velocity_scaling": self.velocity_scaling,
            "acceleration_scaling": self.acceleration_scaling,
            "planned_duration_seconds": (planned.points[-1].time_from_start.sec
                                         + planned.points[-1].time_from_start.nanosec * 1e-9),
        }
        self.records.append(record)
        return record

    def joint_test(self, name, group, targets):
        before = self.snapshot()
        desired = dict(before, **targets)
        self.check_valid(desired)
        constraints = Constraints()
        constraints.name = name
        for joint, position in targets.items():
            constraint = JointConstraint()
            constraint.joint_name = joint
            constraint.position = float(position)
            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        record = self.execute(name, group, constraints)
        deadline = time.monotonic() + 3.0
        while True:
            after = self.snapshot()
            error = max(abs(after[j] - targets[j]) for j in targets)
            if error < 0.012 or time.monotonic() > deadline:
                break
        unchanged = max((abs(after[j] - before[j]) for j in ACTIVE_JOINTS if j not in targets), default=0.0)
        mimic_error = max(abs(after[j] - after[master]) for j, master in MIMICS.items())
        record.update(max_joint_error_rad=error, other_joint_motion_rad=unchanged,
                      max_mimic_error_rad=mimic_error)
        if error >= 0.012 or unchanged >= 0.003 or mimic_error >= 0.003:
            raise RuntimeError(f"{name}: feedback mismatch {record}")
        self.check_valid(after)
        print("PASS", json.dumps(record), flush=True)

    def pose_test(self, group, side):
        before = self.snapshot()
        desired = dict(before)
        desired[f"{side}_joint2"] += 0.035 if side == "L" else -0.035
        desired[f"{side}_joint4"] += -0.025 if side == "L" else 0.025
        self.check_valid(desired)
        link = f"{side}_4C2_ee_link_4"
        pose = self.get_pose(desired, link)
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = group
        ik_request.ik_request.ik_link_name = link
        ik_request.ik_request.robot_state = self.state(before)
        ik_request.ik_request.pose_stamped = pose
        ik_request.ik_request.avoid_collisions = True
        ik_request.ik_request.timeout.sec = 2
        ik_response = self.wait_future(self.ik.call_async(ik_request))
        if ik_response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"{group}: KDL IK failed {ik_response.error_code.val}")
        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = "world"
        position.link_name = link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]
        region_pose = Pose()
        region_pose.position = pose.pose.position
        region_pose.orientation.w = 1.0
        position.constraint_region.primitives = [sphere]
        position.constraint_region.primitive_poses = [region_pose]
        position.weight = 1.0
        orientation = OrientationConstraint()
        orientation.header.frame_id = "world"
        orientation.link_name = link
        orientation.orientation = pose.pose.orientation
        orientation.absolute_x_axis_tolerance = 0.01
        orientation.absolute_y_axis_tolerance = 0.01
        orientation.absolute_z_axis_tolerance = 0.01
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        record = self.execute(f"{group}_pose", group, constraints)
        after = self.snapshot()
        actual = self.get_pose(after, link).pose
        error = math.sqrt(sum((getattr(actual.position, x) - getattr(pose.pose.position, x)) ** 2 for x in "xyz"))
        dot = abs(sum(getattr(actual.orientation, x) * getattr(pose.pose.orientation, x) for x in "xyzw"))
        rotation_error = 2.0 * math.acos(min(1.0, dot))
        other = max(abs(after[j] - before[j]) for j in ACTIVE_JOINTS
                    if j in GRIPPERS or not j.startswith(side))
        record.update(ik_error_code=ik_response.error_code.val, position_error_m=error,
                      orientation_error_rad=rotation_error, other_arm_motion_rad=other,
                      tcp_link=link)
        if error > 0.004 or rotation_error > 0.03 or other > 0.003:
            raise RuntimeError(f"{group}: pose feedback mismatch {record}")
        self.check_valid(after)
        print("PASS", json.dumps(record), flush=True)

    def wait_ready(self):
        for client in (self.controllers, self.hardware, self.validity, self.fk, self.ik):
            if not client.wait_for_service(timeout_sec=60.0):
                raise RuntimeError(f"Missing service {client.srv_name}")
        if not self.move.wait_for_server(timeout_sec=60.0):
            raise RuntimeError("Missing MoveGroup action server")
        required = ("joint_state_broadcaster", "left_arm_controller", "right_arm_controller",
                    "left_gripper_controller", "right_gripper_controller")
        deadline = time.monotonic() + 60.0
        while True:
            controllers = self.wait_future(self.controllers.call_async(ListControllers.Request()))
            states = {item.name: item.state for item in controllers.controller}
            if all(states.get(name) == "active" for name in required):
                break
            if time.monotonic() > deadline:
                raise RuntimeError(f"Controllers not active: {states}")
            rclpy.spin_once(self, timeout_sec=0.1)
        hardware = self.wait_future(self.hardware.call_async(ListHardwareComponents.Request()))
        plugins = [component.class_type for component in hardware.component]
        if plugins != [self.expected_plugin]:
            raise RuntimeError(f"Expected only {self.expected_plugin}, got {plugins}")
        return states, plugins

    def run(self):
        states, plugins = self.wait_ready()
        initial = self.snapshot()
        self.check_valid(initial)
        left = {j: initial[j] for j in JOINTS[:7]}
        left["L_joint1"] += 0.06
        left["L_joint2"] -= 0.04
        left["L_joint4"] -= 0.05
        self.joint_test("left_joint_goal", "left_arm", left)
        current = self.snapshot()
        right = {j: current[j] for j in JOINTS[7:]}
        right["R_joint1"] -= 0.06
        right["R_joint2"] += 0.04
        right["R_joint4"] += 0.05
        self.joint_test("right_joint_goal", "right_arm", right)
        current = self.snapshot()
        both = {j: current[j] for j in JOINTS}
        both["L_joint3"] += 0.04
        both["R_joint3"] -= 0.04
        self.joint_test("both_arms_joint_goal", "both_arms", both)
        self.pose_test("left_arm", "L")
        self.pose_test("right_arm", "R")
        self.joint_test("return_to_initial", "both_arms", {j: initial[j] for j in JOINTS})
        # Use the same named targets as RViz, including the real opening-command margin.
        self.joint_test("left_gripper_closed", "left_gripper", {GRIPPERS[0]: self.gripper_closed[GRIPPERS[0]]})
        self.joint_test("right_gripper_closed", "right_gripper", {GRIPPERS[1]: self.gripper_closed[GRIPPERS[1]]})
        self.joint_test("both_grippers_open", "both_grippers", {j: 0.0 for j in GRIPPERS})
        self.joint_test("both_grippers_closed", "both_grippers", self.gripper_closed)
        # Verify arm planning/execution with the larger tool in its closed posture too.
        self.joint_test("arms_with_closed_grippers", "both_arms", both)
        self.joint_test("arms_home_with_closed_grippers", "both_arms", {j: initial[j] for j in JOINTS})
        self.joint_test("grippers_return_to_initial", "both_grippers", {j: initial[j] for j in GRIPPERS})
        return {"status": "passed", "hardware_plugins": plugins,
                "controllers": states, "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
                "tests": self.records, "final_positions": self.snapshot()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", required=True)
    args = parser.parse_args()
    rclpy.init()
    validator = Validator()
    try:
        report = validator.run()
    except Exception as error:
        report = {"status": "failed", "error": str(error), "tests": validator.records}
        raise
    finally:
        Path(args.report).parent.mkdir(parents=True, exist_ok=True)
        Path(args.report).write_text(json.dumps(report, indent=2) + "\n")
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
