#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time
from std_msgs.msg import Float64


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(Pose, '/me314_xarm_pose_cmd', 10)
        self.gripper_pub_ = self.create_publisher(Float64, '/me314_xarm_gripper_cmd', 10)


    def publish_pose(self, pose: Pose):
        self.publisher_.publish(pose)
        self.get_logger().info(f"Published Pose:\n"
                               f"  position=({pose.position.x}, {pose.position.y}, {pose.position.z})\n"
                               f"  orientation=({pose.orientation.x}, {pose.orientation.y}, "
                               f"{pose.orientation.z}, {pose.orientation.w})")

    def publish_gripper_position(self, gripper_pos: float):
        """
        Publishes a Float64 message indicating the desired gripper position.
        For example:
          0.0 is "fully open"
          1.0 is "closed"
        """
        msg = Float64()
        msg.data = gripper_pos
        self.gripper_pub_.publish(msg)
        self.get_logger().info(f"Published gripper command: {gripper_pos:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()

    # Define a few target poses here
    p0 = Pose()
    p0.position.x = 0.2408 + 0.1
    p0.position.y = 0.0021
    p0.position.z = 0.3029
    p0.orientation.x = 1.0
    p0.orientation.y = 0.0
    p0.orientation.z = 0.0
    p0.orientation.w = 0.0 

    p1 = Pose()
    p1.position.x = p0.position.x
    p1.position.y = p0.position.y 
    p1.position.z = p0.position.z + 0.2
    p1.orientation.x = 1.0
    p1.orientation.y = 0.0
    p1.orientation.z = 0.0
    p1.orientation.w = 0.0

    poses = [p0, p1]

    # Let's first open the gripper (0.0 to 1.0, where 0.0 is fully open and 1.0 is fully closed)
    node.get_logger().info("Opening gripper...")
    node.publish_gripper_position(0.0)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(5.0)

    # Move the arm to each pose
    for i, pose in enumerate(poses):
        node.get_logger().info(f"Publishing Pose {i+1}...")
        node.publish_pose(pose)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(8.0)  # Wait a few seconds to let the robot move

    # Now close the gripper.
    node.get_logger().info("Closing gripper...")
    node.publish_gripper_position(0.5)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(5.0)

    node.get_logger().info("All actions done. Shutting down.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
