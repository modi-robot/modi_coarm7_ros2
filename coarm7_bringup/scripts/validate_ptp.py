#!/usr/bin/env python3
"""Fake-only regression of automatic gripper PTP routing and rejection checks."""
import argparse
import json
from pathlib import Path
import runpy
import time
import xml.etree.ElementTree as ET
import rclpy
from ament_index_python.packages import get_package_prefix
from moveit_msgs.srv import GetMotionPlan, ApplyPlanningScene
from moveit_msgs.msg import Constraints, JointConstraint, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--report',required=True);args=parser.parse_args()
    base=runpy.run_path(str(Path(get_package_prefix('coarm7_bringup'))/'lib/coarm7_bringup/validate_offline.py'))
    rclpy.init();v=base['Validator']();rows=[];report={'status':'failed','tests':rows}
    try:
        # This verifies transport=fake before any scene modification or request.
        v.wait_ready()
        plugin=v.parameter('/move_group','ompl.planning_plugin').string_value
        if plugin!='coarm7_planning/GroupPlanner':raise RuntimeError('GroupPlanner is not active')
        client=v.create_client(GetMotionPlan,'/plan_kinematic_path')
        if not client.wait_for_service(timeout_sec=15):raise RuntimeError('Planning service missing')
        initial=v.snapshot()
        robot=ET.fromstring(v.parameter('/robot_state_publisher','robot_description').string_value)
        mimics={j.get('name'):(j.find('mimic').get('joint'),float(j.find('mimic').get('multiplier','1')),float(j.find('mimic').get('offset','0'))) for j in robot.findall('joint') if j.find('mimic') is not None}
        def rviz_goal(req):
            # MoveIt's constructGoalConstraints includes group mimic variables.
            constraints=req.motion_plan_request.goal_constraints[0].joint_constraints
            values={j.joint_name:j.position for j in constraints}
            for name,(master,factor,offset) in mimics.items():
                if master not in values:continue
                jc=JointConstraint();jc.joint_name=name;jc.position=values[master]*factor+offset
                jc.tolerance_above=0.001;jc.tolerance_below=0.001;jc.weight=1.0;constraints.append(jc)
            return req
        def request(group,targets,start=None):
            req=GetMotionPlan.Request();q=req.motion_plan_request
            q.group_name=group;q.pipeline_id='ompl';q.planner_id='RRTConnectkConfigDefault'
            q.num_planning_attempts=10;q.allowed_planning_time=5.0
            q.max_velocity_scaling_factor=1.0;q.max_acceleration_scaling_factor=0.5
            q.start_state=v.state(start or initial);goal=Constraints()
            for name,position in targets.items():
                jc=JointConstraint();jc.joint_name=name;jc.position=float(position)
                jc.tolerance_above=0.001;jc.tolerance_below=0.001;jc.weight=1.0;goal.joint_constraints.append(jc)
            q.goal_constraints=[goal]
            return req
        def plan(req):
            started=time.monotonic();res=v.wait_future(client.call_async(req),30).motion_plan_response
            return res,time.monotonic()-started
        def reject(name,req):
            res,elapsed=plan(req)
            if res.error_code.val==1:raise RuntimeError(name+' unexpectedly succeeded')
            rows.append(dict(name=name,status='passed',error_code=res.error_code.val,wall_s=elapsed))
        for group,sides in [('left_gripper',['L']),('right_gripper',['R']),('both_grippers',['L','R'])]:
            for close in [True,False]:
                targets={side+'_4C2_r_1_joint':0.85536 if close else 0.0 for side in sides}
                start=dict(initial,**{j:0.0 if close else 0.85536 for j in targets})
                res,elapsed=plan(request(group,targets,start))
                if res.error_code.val!=1:raise RuntimeError(group+' failed '+str(res.error_code.val))
                trajectory=res.trajectory.joint_trajectory;points=trajectory.points
                assert points and set(trajectory.joint_names)==set(targets)
                peak_v=max(abs(x) for point in points for x in point.velocities)
                peak_a=max(abs(x) for point in points for x in point.accelerations)
                assert peak_v<=0.12001 and peak_a<=0.06001,(peak_v,peak_a)
                sign=1 if close else -1
                assert all(sign*(b.positions[i]-a.positions[i])>=-1e-8 for a,b in zip(points,points[1:]) for i in range(len(a.positions)))
                error=max(abs(points[-1].positions[i]-targets[j]) for i,j in enumerate(trajectory.joint_names))
                assert error<0.0011,error
                for point in points:
                    state=dict(start);state.update(zip(trajectory.joint_names,point.positions));v.check_valid(state)
                duration=points[-1].time_from_start.sec+points[-1].time_from_start.nanosec*1e-9
                rows.append(dict(name=group+('_closed' if close else '_open'),status='passed',wall_s=elapsed,duration_s=duration,peak_velocity=peak_v,peak_acceleration=peak_a,final_error=error,full_robot_validity_checks=len(points)))
        for group,sides in [('left_gripper',['L']),('right_gripper',['R']),('both_grippers',['L','R'])]:
            for close in [True,False]:
                targets={side+'_4C2_r_1_joint':0.85536 if close else 0.0 for side in sides}
                start=dict(initial,**{j:0.0 if close else 0.85536 for j in targets})
                req=rviz_goal(request(group,targets,start))
                count=len(req.motion_plan_request.goal_constraints[0].joint_constraints)
                assert count==(12 if group=='both_grippers' else 6),count
                res,elapsed=plan(req);assert res.error_code.val==1,(group,res.error_code.val)
                names=res.trajectory.joint_trajectory.joint_names;assert set(names)==set(targets)
                rows.append(dict(name='rviz_mimic_'+group+('_closed' if close else '_open'),status='passed',input_goal_joints=count,trajectory_joints=list(names),wall_s=elapsed))
        req=rviz_goal(request('left_gripper',{'L_4C2_r_1_joint':0.2}))
        req.motion_plan_request.goal_constraints[0].joint_constraints[-1].position+=0.1
        reject('contradictory_mimic_target',req)
        # End-to-end fake execution through MoveGroup with the actual 12-variable goal shape.
        for close in [True,False]:
            req=rviz_goal(request('both_grippers',{'L_4C2_r_1_joint':0.85536 if close else 0.0,'R_4C2_r_1_joint':0.85536 if close else 0.0}))
            v.execute('rviz_mimic_both_'+('closed' if close else 'open'),'both_grippers',req.motion_plan_request.goal_constraints[0])
            target=0.85536 if close else 0.0;actual=v.snapshot()
            assert all(abs(actual[j]-target)<0.01 for j in ['L_4C2_r_1_joint','R_4C2_r_1_joint'])
            rows.append(dict(name='rviz_mimic_execute_'+('closed' if close else 'open'),status='passed'))
        # Different strokes must synchronize without reversing either finger.
        res,elapsed=plan(request('both_grippers',{'L_4C2_r_1_joint':0.2,'R_4C2_r_1_joint':0.6}))
        assert res.error_code.val==1
        rows.append(dict(name='asymmetric_both_grippers',status='passed',wall_s=elapsed))
        reject('out_of_bounds',request('left_gripper',{'L_4C2_r_1_joint':1.5}))
        reject('cross_group_joint',request('left_gripper',{'L_joint1':0.1}))
        reject('partial_both_grippers',request('both_grippers',{'L_4C2_r_1_joint':0.2}))
        req=request('left_gripper',{'L_4C2_r_1_joint':0.2});req.motion_plan_request.path_constraints.joint_constraints=req.motion_plan_request.goal_constraints[0].joint_constraints
        reject('unsupported_path_constraint',req)
        req=request('left_gripper',{'L_4C2_r_1_joint':0.2});req.motion_plan_request.goal_constraints.append(req.motion_plan_request.goal_constraints[0])
        reject('alternative_goals_not_silently_dropped',req)
        scene=v.create_client(ApplyPlanningScene,'/apply_planning_scene')
        assert scene.wait_for_service(timeout_sec=10)
        obstacle=CollisionObject();obstacle.id='coarm7_ptp_fake_test_blocker';obstacle.header.frame_id='world'
        solid=SolidPrimitive();solid.type=SolidPrimitive.BOX;solid.dimensions=[4.0,4.0,4.0]
        pose=Pose();pose.orientation.w=1.0;pose.position.z=0.5
        obstacle.primitives=[solid];obstacle.primitive_poses=[pose];obstacle.operation=CollisionObject.ADD
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.world.collision_objects=[obstacle]
        try:
            assert v.wait_future(scene.call_async(req)).success
            reject('colliding_scene_rejected',request('right_gripper',{'R_4C2_r_1_joint':0.2}))
        finally:
            obstacle.operation=CollisionObject.REMOVE;req.scene.world.collision_objects=[obstacle]
            assert v.wait_future(scene.call_async(req)).success
        report.update(status='passed',plugin=plugin,transport='fake',pipeline_id='ompl',can_access=False)
    except Exception as error:
        report['error']=str(error)
        raise
    finally:
        Path(args.report).parent.mkdir(parents=True,exist_ok=True)
        Path(args.report).write_text(json.dumps(report,indent=2)+'\n')
        v.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
