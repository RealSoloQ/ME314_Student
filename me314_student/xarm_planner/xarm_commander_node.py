#!/usr/bin/env python3

"""
Main node that listens for arm pose and gripper position messages and plans/executes the corresponding commands. 
Joint-space planning is used because it allows for velocity/acceleration limits to be set, whereas cartesian planning does not.
"""

import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from moveit_msgs.srv import (GetPositionIK, GetCartesianPath, GetMotionPlan, GetPositionFK)
from moveit_msgs.msg import (RobotTrajectory, MotionPlanRequest, Constraints, JointConstraint, RobotState)
from moveit_msgs.action import ExecuteTrajectory


class ME314_XArm_Commander(Node):
    def __init__(self):
        super().__init__('ME314_XArm_Commander_Node')

        ####################################################################
        # CLIENTS
        ####################################################################

        # Create clients for MoveIt services (asynchronously)
        self.cartesian_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.compute_ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.plan_path_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')

        # Create an action client for executing trajectories
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.wait_for_all_services_and_action()

        ####################################################################
        # CLASS ATTRIBUTES
        ####################################################################

        # Logger for if in sim or real mode, variable is set in launch file
        self.declare_parameter('use_sim', False) 
        self.use_sim = self.get_parameter('use_sim').value
        self.get_logger().info(f"Running with use_sim={self.use_sim}")

        self.current_gripper_position = 0.0
        self.home_joints_deg = [1.1, -48.5, 0.4, 32.8, 0.4, 81.9, 0.3]
        self.home_joints_rad = [math.radians(angle) for angle in self.home_joints_deg]
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        self.current_joint_positions = [None] * len(self.joint_names)

        self.gripper_group_name = "xarm_gripper"  
        self.gripper_joint_names = ["drive_joint"]

        ####################################################################
        # SUBSCRIBERS
        ####################################################################

        self.arm_pose_sub = self.create_subscription(Pose, '/me314_xarm_pose_cmd', self.arm_pose_cmd_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.gripper_cmd_sub = self.create_subscription(Float64, '/me314_xarm_gripper_cmd', self.gripper_cmd_callback, 10)

        ####################################################################
        # PUBLISHERS
        ####################################################################

        self.current_pose_pub = self.create_publisher(Pose, '/me314_xarm_current_pose', 10)
        self.curr_joint_position_deg_pub = self.create_publisher(JointState, '/me314_xarm_current_joint_positions_deg', 10)
        self.gripper_position_pub = self.create_publisher(Float64, '/me314_xarm_gripper_position', 10)

        # Timer to periodically publish the current end-effector pose (asynchronously)
        self.timer_period = 0.1  # seconds
        self.timer_pose = self.create_timer(self.timer_period, self.publish_current_pose)
        self.timer_gripper = self.create_timer(self.timer_period, self.publish_gripper_position)
        self.timer_joint_positions = self.create_timer(self.timer_period, self.publish_current_joint_positions)

        ####################################################################
        # INITIALIZATION
        ####################################################################

        # Move to home position (joint-based) to start
        self.get_logger().info("Moving to home position (joint-based).")
        self.plan_execute_joint_target_async(self.home_joints_rad, callback=self.home_move_done_callback)

        # Open gripper fully
        self.get_logger().info("Opening gripper fully.")
        self.plan_execute_gripper_async(0.0)

        self.get_logger().info("XArm commander node is ready.")

    ####################################################################
    # METHODS
    ####################################################################

    def wait_for_all_services_and_action(self):
        """Block until all required services and the action server are available."""
        while not self.cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for compute_cartesian_path service...')
        while not self.compute_ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for compute_ik service...')
        while not self.plan_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for plan_kinematic_path service...')
        while not self.execute_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for execute_trajectory action server...')
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for compute_fk service...')
        self.get_logger().info('All services and action servers are available!')

    def home_move_done_callback(self, success: bool):
        """Callback invoked once the home joint plan finishes or fails."""
        if not success:
            self.get_logger().warn("Failed to move to home position (joint-based).")
        else:
            self.get_logger().info("Home position move completed successfully (joint-based).")

    def joint_state_callback(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            self.get_logger().debug(f"Joint: {name} position: {position}")
            if name in self.joint_names:
                i = self.joint_names.index(name)
                self.current_joint_positions[i] = position
            if name == "xarm_gripper_drive_joint":  # update with the correct joint name
                self.current_gripper_position = position

    def publish_current_joint_positions(self):
        """Periodically publish the current joint positions in degrees."""
        if None in self.current_joint_positions:
            return
        msg = JointState()
        msg.name = self.joint_names
        msg.position = [math.degrees(pos) for pos in self.current_joint_positions]
        self.curr_joint_position_deg_pub.publish(msg)

    def publish_current_pose(self):
        """Asynchronously call the FK service for current joint positions; publish when complete."""
        if None in self.current_joint_positions:
            return  # Haven't received all joint states yet

        robot_state = RobotState()
        robot_state.joint_state.name = self.joint_names
        robot_state.joint_state.position = self.current_joint_positions
        
        req = GetPositionFK.Request()
        req.header.frame_id = "link_base"  # Base frame we define pose with respect to
        req.fk_link_names = ["link_tcp"]   # End-effector link frame pose we want
        req.robot_state = robot_state
        
        future = self.fk_client.call_async(req)
        future.add_done_callback(self.publish_current_pose_cb)

    def publish_current_pose_cb(self, future):
        """Callback that fires once the FK service returns."""
        try:
            res = future.result()
            if res is not None and len(res.pose_stamped) > 0:
                ee_pose = res.pose_stamped[0].pose
                self.current_pose_pub.publish(ee_pose)
        except Exception as e:
            self.get_logger().error(f"FK service call failed: {e}")

    def publish_gripper_position(self):
        """
        Periodically publish the current gripper drive_joint position.
        """
        if self.current_gripper_position is None:
            self.get_logger().warn("Gripper state not received yet.")
            return  # Gripper state not received yet
        msg = Float64()
        msg.data = self.current_gripper_position
        self.gripper_position_pub.publish(msg)

    ####################################################################
    # MOVEMENT EXECUTION
    ####################################################################
    def arm_pose_cmd_callback(self, msg: Pose):
        """
        Callback for receiving arm pose command, runs IK and executes in joint-space.
        """
        # If on real hardware, offset the z (assuming real hardware has F/T sensor mounted)
        if not self.use_sim:
            self.get_logger().info("Applying +0.058 m offset in Z because use_sim=False")
            msg.position.z += 0.058
        
        self.get_logger().info(f"Received pose command: [{msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}]")
        self.compute_ik_and_execute_joint_async(msg)

    def compute_ik_and_execute_joint_async(self, target_pose: Pose):
        """
        Asynchronously compute IK for target_pose,
        then if successful, plan and execute in joint space.
        """
        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = "xarm7"
        ik_req.ik_request.robot_state.is_diff = True

        # Fill in the PoseStamped
        ik_req.ik_request.pose_stamped.header.frame_id = "link_base"
        ik_req.ik_request.pose_stamped.pose = target_pose
        
        # Tolerances, constraints, etc., if desired
        ik_req.ik_request.timeout.sec = 2  # 2-second IK timeout

        future_ik = self.compute_ik_client.call_async(ik_req)
        future_ik.add_done_callback(self.compute_ik_done_cb)

    def compute_ik_done_cb(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"IK call failed: {e}")
            return
        
        if res.error_code.val != 1:
            self.get_logger().warn(f"IK did not succeed, error code: {res.error_code.val}")
            return
        
        # We have a valid joint solution:
        joint_solution = res.solution.joint_state

        # Build a list in the same order as self.joint_names
        desired_positions = [0.0] * len(self.joint_names)
        for name, pos in zip(joint_solution.name, joint_solution.position):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                desired_positions[idx] = pos
        
        self.get_logger().info("IK succeeded; now planning joint motion to that IK solution.")
        
        # Plan and execute the joint-based motion
        self.plan_execute_joint_target_async(desired_positions, callback=self.movement_done_cb)

    def movement_done_cb(self, success: bool):
        """Callback after the joint-based plan+execute for the IK solution finishes."""
        if success:
            self.get_logger().info("IK-based joint motion completed successfully.")
        else:
            self.get_logger().warn("IK-based joint motion failed to complete.")

    def plan_execute_joint_target_async(self, joint_positions, callback=None):
        req = GetMotionPlan.Request()
        motion_req = MotionPlanRequest()
        motion_req.workspace_parameters.header.frame_id = "link_base"
        motion_req.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        
        motion_req.start_state.is_diff = True
        
        # Set the goal state using joint constraints
        motion_req.goal_constraints.append(Constraints())
        for i, joint_name in enumerate(self.joint_names):
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = joint_positions[i]
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            motion_req.goal_constraints[0].joint_constraints.append(constraint)
        
        motion_req.group_name = "xarm7"
        motion_req.num_planning_attempts = 10
        motion_req.allowed_planning_time = 5.0
        
        # Slow down joint-space planning
        if self.use_sim:
            motion_req.max_velocity_scaling_factor = 0.2
            motion_req.max_acceleration_scaling_factor = 0.2
        else:
            motion_req.max_velocity_scaling_factor = 0.08
            motion_req.max_acceleration_scaling_factor = 0.08
        
        req.motion_plan_request = motion_req
        
        positions_deg = [math.degrees(angle) for angle in joint_positions]
        self.get_logger().info(f'Planning joint motion to positions (deg): {positions_deg}')
        
        future = self.plan_path_client.call_async(req)
        future.add_done_callback(lambda f: self.plan_path_done_cb(f, callback))

    def plan_path_done_cb(self, future, callback):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Joint path plan service call failed: {e}")
            if callback:
                callback(False)
            return
        
        if result.motion_plan_response.error_code.val != 1:
            self.get_logger().error(f"Planning failed, error code = {result.motion_plan_response.error_code.val}")
            if callback:
                callback(False)
            return
        
        self.get_logger().info("Joint motion plan succeeded, executing trajectory...")
        self.execute_trajectory_async(result.motion_plan_response.trajectory, callback)

    def execute_trajectory_async(self, trajectory: RobotTrajectory, callback=None):
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        self.get_logger().info("Sending trajectory for execution...")
        send_goal_future = self.execute_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda f: self.action_server_send_callback(f, callback))

    def action_server_send_callback(self, future, callback):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("ExecuteTrajectory goal was rejected by server.")
            if callback:
                callback(False)
            return
        
        self.get_logger().info("Goal accepted by server, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.action_server_execute_callback(f, callback))

    def action_server_execute_callback(self, future, callback):
        result = future.result().result
        if result.error_code.val != 1:
            self.get_logger().error(f"Trajectory execution failed with error code: {result.error_code.val}")
            if callback:
                callback(False)
            return

        self.get_logger().info("Trajectory execution succeeded.")
        if callback:
            callback(True)

    ## ------------------------------
    ##  Gripper planning
    ## ------------------------------
    def plan_execute_gripper_async(self, position: float, callback=None):
        req = GetMotionPlan.Request()
        motion_req = MotionPlanRequest()
        motion_req.workspace_parameters.header.frame_id = "link_base"
        motion_req.workspace_parameters.header.stamp = self.get_clock().now().to_msg()

        motion_req.start_state.is_diff = True

        # Build goal constraints for the gripper joint(s)
        motion_req.goal_constraints.append(Constraints())
        for jn in self.gripper_joint_names:
            constraint = JointConstraint()
            constraint.joint_name = jn
            constraint.position = position
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            motion_req.goal_constraints[0].joint_constraints.append(constraint)

        motion_req.group_name = self.gripper_group_name
        motion_req.num_planning_attempts = 10
        motion_req.allowed_planning_time = 5.0
        motion_req.max_velocity_scaling_factor = 0.1
        motion_req.max_acceleration_scaling_factor = 0.1

        req.motion_plan_request = motion_req

        self.get_logger().info(f"Planning gripper motion to {math.degrees(position):.2f} deg")

        future = self.plan_path_client.call_async(req)
        future.add_done_callback(lambda f: self.plan_path_done_cb(f, callback))

    def gripper_cmd_callback(self, msg: Float64):
        # Capture the commanded value locally
        commanded_value = msg.data
        # Use a callback that updates the local gripper state upon success
        self.plan_execute_gripper_async(commanded_value,
            callback=lambda success: self.gripper_move_done_cb(success, commanded_value)
        )

    def gripper_move_done_cb(self, success: bool, commanded_value: float):
        if success:
            self.get_logger().info("Gripper motion succeeded; updating local gripper state.")
            self.current_gripper_position = commanded_value
        else:
            self.get_logger().warn("Gripper motion failed.")

def main(args=None):
    rclpy.init(args=args)
    commander = ME314_XArm_Commander()
    
    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass
    
    commander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
