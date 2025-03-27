#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import RobotTrajectory, MotionPlanRequest, Constraints, JointConstraint
import time
import math
import sys

class PoseCommander(Node):
    def __init__(self, use_sim=False):
        super().__init__('pose_commander')
        
        # Store the simulation flag
        self.use_sim = use_sim
        self.get_logger().info(f"Running with use_sim={self.use_sim}")
        
        # Create clients for the available MoveIt services
        self.cartesian_path_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path')
        while not self.cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for compute_cartesian_path service...')
        
        self.compute_ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.compute_ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for compute_ik service...')
        
        # Create client for motion planning
        self.plan_path_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        while not self.plan_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for plan_kinematic_path service...')
        
        # Create action client for executing trajectories
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        while not self.execute_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for execute_trajectory action server...')
            
        self.get_logger().info('All services and action servers are available!')
        
        # Define the home position using joint angles (in degrees)
        self.home_joints_deg = [0, -48.6, 0, 33, 0, 81.1, 0]
        # Convert to radians for ROS
        self.home_joints_rad = [math.radians(angle) for angle in self.home_joints_deg]
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'
        ]  # Update these with your actual joint names
        
        # Immediately move to the home joint position
        self.get_logger().info("Initializing robot to home joint position...")
        success = self.plan_and_execute_joint_target(self.home_joints_rad)
        if success:
            self.get_logger().info("Successfully initialized to home position")
        else:
            self.get_logger().error("Failed to initialize to home position")
        
        # Get the Cartesian pose at home position for the position loop
        self.home_pose = self.get_current_pose()
    
    def plan_and_execute_joint_target(self, joint_positions):
        """Plan and execute a motion to target joint positions using available services"""
        # Create the motion planning request
        req = GetMotionPlan.Request()
        motion_req = MotionPlanRequest()
        motion_req.workspace_parameters.header.frame_id = "link_base"
        motion_req.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        
        # Set the start state to the current state
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
        
        # Set the planning group
        motion_req.group_name = "xarm7"
        motion_req.num_planning_attempts = 10
        motion_req.allowed_planning_time = 5.0
        
        # MODIFIED: Reduced velocity and acceleration scaling for slower movement
        motion_req.max_velocity_scaling_factor = 0.1  # Reduced from 0.5
        motion_req.max_acceleration_scaling_factor = 0.1  # Reduced from 0.5
        
        req.motion_plan_request = motion_req
        
        # Log joint targets
        positions_deg = [math.degrees(angle) for angle in joint_positions]
        self.get_logger().info(f'Planning joint motion to positions (deg): {positions_deg}')
        
        # Send the request and wait for response
        future = self.plan_path_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            result = future.result()
            if result.motion_plan_response.error_code.val != 1:
                self.get_logger().error(f'Motion planning failed with error code: {result.motion_plan_response.error_code.val}')
                return False
                
            self.get_logger().info('Joint motion plan succeeded, executing trajectory...')
            # Execute the planned trajectory
            return self.execute_trajectory(result.motion_plan_response.trajectory)
        else:
            self.get_logger().error(f'Service call failed: {future.exception()}')
            return False
    
    def send_pose_for_ik(self, target_pose: Pose):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "xarm7"
        req.ik_request.pose_stamped.header.frame_id = "link_base"
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.avoid_collisions = True
        
        self.get_logger().info(f'Sending IK request for pose: position=[{target_pose.position.x}, {target_pose.position.y}, {target_pose.position.z}]')
        future = self.compute_ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f'IK solution found: {result.error_code.val == 1}')
            return result.error_code.val == 1
        else:
            self.get_logger().error(f'Service call failed: {future.exception()}')
            return False
    
    def compute_and_execute_cartesian_path(self, target_pose: Pose):
        
        req = GetCartesianPath.Request()
        req.header.frame_id = "link_base"
        req.start_state.is_diff = True
        req.group_name = "xarm7"
        req.waypoints = [target_pose]
        
        # MODIFIED: Reduced step size for finer, slower path
        req.max_step = 0.005  # Reduced from 0.01 for finer-grained path
        
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        
        # NOTE: These scaling factors aren't available in GetCartesianPath request
        # Velocity and acceleration will be controlled when we execute the trajectory
        
        self.get_logger().info(f'Computing Cartesian path for pose: position=[{target_pose.position.x}, {target_pose.position.y}, {target_pose.position.z}]')
        future = self.cartesian_path_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f'Cartesian path computed, fraction: {result.fraction}')
            
            if result.fraction < 0.9:
                self.get_logger().warn('Less than 90% of path was computed successfully')
                return False
                
            return self.execute_trajectory(result.solution)
        else:
            self.get_logger().error(f'Service call failed: {future.exception()}')
            return False
    
    def execute_trajectory(self, trajectory: RobotTrajectory):
        # MODIFIED: Scale down the trajectory time to slow execution
        for point in trajectory.joint_trajectory.points:
            # Double the time from start to slow down execution
            point.time_from_start.sec *= 3
            point.time_from_start.nanosec *= 3
            
            # Scale down velocities and accelerations
            for i in range(len(point.velocities)):
                point.velocities[i] *= 0.3  # Reduce velocity to 30%
                point.accelerations[i] *= 0.3  # Reduce acceleration to 30%
        
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        self.get_logger().info('Sending trajectory for execution...')
        send_goal_future = self.execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return False
            
        self.get_logger().info('Goal accepted by server, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
            
        result = get_result_future.result().result
        self.get_logger().info(f'Trajectory execution completed with error code: {result.error_code.val}')
        
        return result.error_code.val == 1
    
    def get_current_pose(self):
        # This is a placeholder - we would ideally get the current pose from TF or a topic
        # For this example, we'll use a default pose
        curr_pose = Pose()
        curr_pose.position.x = 0.241
        curr_pose.position.y = 0.0047
        curr_pose.position.z = 0.3551
        curr_pose.orientation.x = 1.0
        curr_pose.orientation.y = 0.0
        curr_pose.orientation.z = 0.0
        curr_pose.orientation.w = 0.0
        return curr_pose
        
    def run_position_loop(self):
        """
        Loop indefinitely:
        1. Move to home position
        2. Move to position A (50mm up in Z)
        3. Move to position B (50mm forward in X)
        4. Repeat
        
        If use_sim is False, add 50mm to Z for all goal positions
        """
        self.get_logger().info("Starting position loop...")
        
        try:
            while rclpy.ok():
                # Get home position
                home_pose = Pose()
                home_pose.position.x = self.home_pose.position.x
                home_pose.position.y = self.home_pose.position.y
                home_pose.position.z = self.home_pose.position.z
                home_pose.orientation = self.home_pose.orientation
                
                # If not in simulation, add 0.05m to Z
                if not self.use_sim:
                    home_pose.position.z += 0.05
                    self.get_logger().info(f"Adding 0.05m to Z (use_sim={self.use_sim})")
                
                # Move to home position
                self.get_logger().info(f"Moving to home position: [{home_pose.position.x}, {home_pose.position.y}, {home_pose.position.z}]")
                if not self.compute_and_execute_cartesian_path(home_pose):
                    self.get_logger().warn("Failed to move to home position, retrying...")
                    time.sleep(2.0)
                    continue
                
                # MODIFIED: Increased pause between movements
                time.sleep(3.0)  # Increased from 1.0
                
                # Create position A: 50mm up in Z from home
                pose_a = Pose()
                pose_a.position.x = self.home_pose.position.x
                pose_a.position.y = self.home_pose.position.y
                pose_a.position.z = self.home_pose.position.z + 0.05  # 50mm up
                
                # If not in simulation, add additional 0.05m to Z
                if not self.use_sim:
                    pose_a.position.z += 0.05
                    
                pose_a.orientation = self.home_pose.orientation
                
                # Move to position A
                self.get_logger().info(f"Moving to position A (up in Z): [{pose_a.position.x}, {pose_a.position.y}, {pose_a.position.z}]")
                if not self.compute_and_execute_cartesian_path(pose_a):
                    self.get_logger().warn("Failed to move to position A, returning to home...")
                    continue
                
                # MODIFIED: Increased pause between movements
                time.sleep(3.0)  # Increased from 1.0
                
                # Create position B: 50mm forward in X from home
                pose_b = Pose()
                pose_b.position.x = self.home_pose.position.x + 0.05  # 50mm forward
                pose_b.position.y = self.home_pose.position.y
                pose_b.position.z = self.home_pose.position.z
                
                # If not in simulation, add 0.05m to Z
                if not self.use_sim:
                    pose_b.position.z += 0.05
                    
                pose_b.orientation = self.home_pose.orientation
                
                # Move to position B
                self.get_logger().info(f"Moving to position B (forward in X): [{pose_b.position.x}, {pose_b.position.y}, {pose_b.position.z}]")
                if not self.compute_and_execute_cartesian_path(pose_b):
                    self.get_logger().warn("Failed to move to position B, returning to home...")
                    continue
                
                # MODIFIED: Increased pause between movements
                time.sleep(3.0)  # Increased from 1.0
                
        except KeyboardInterrupt:
            self.get_logger().info("Loop interrupted by user")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    # Check for use_sim flag in command line arguments
    use_sim = False
    if len(sys.argv) > 1 and "--use_sim" in sys.argv:
        use_sim = True
    
    commander = PoseCommander(use_sim=use_sim)
    
    # Run the position loop
    commander.run_position_loop()
    
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()