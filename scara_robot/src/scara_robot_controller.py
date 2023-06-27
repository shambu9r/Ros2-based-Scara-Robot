#!/usr/bin/env python3

import os
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped


class SCARARobotController(Node):
    def __init__(self):
        super().__init__('scara_robot_controller')

        self.joint_names = ['joint1', 'joint2', 'joint3']
        self.num_joints = len(self.joint_names)

        self.subscription = self.create_subscription(PoseStamped, 'desired_pose', self.desired_pose_callback, 10)

        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def desired_pose_callback(self, msg):
        # Convert the desired pose to joint angles using inverse kinematics
        desired_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        joint_angles = self.calculate_inverse_kinematics(desired_position)

        # Publish the joint angles
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = joint_angles
        self.publisher.publish(joint_state)

    def calculate_inverse_kinematics(self, desired_position):
        x, y, z = desired_position

        # Solve for joint angles using inverse kinematics equations
        link_lengths = [0.5, 0.3]
        a1, a2 = link_lengths
        theta2 = np.arccos((x**2 + y**2 - a1**2 - a2**2) / (2 * a1 * a2))
        theta1 = np.arctan2(y, x) - np.arctan2((a2 * np.sin(theta2)), (a1 + a2 * np.cos(theta2)))
        d3 = z

        return [theta1, theta2, d3]

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = [0.1, 0.2, 0.3]  # Placeholder values
        self.publisher.publish(joint_state)

        # Broadcast the transforms for the links
        tf_broadcaster = TransformBroadcaster(self)
        for joint_name in self.joint_names:
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = self.get_clock().now().to_msg()
            tf_stamped.header.frame_id = 'base_link'
            tf_stamped.child_frame_id = joint_name

            tf_stamped.transform.translation.x = 0.0
            tf_stamped.transform.translation.y = 0.0
            tf_stamped.transform.translation.z = 0.0
            tf_stamped.transform.rotation.x = 0.0
            tf_stamped.transform.rotation.y = 0.0
            tf_stamped.transform.rotation.z = 0.0
            tf_stamped.transform.rotation.w = 1.0

            tf_broadcaster.sendTransform(tf_stamped)


def main(args=None):
    rclpy.init(args=args)

    # Create the SCARA robot controller node
    scara_robot_controller = SCARARobotController()
    scara_robot_controller.get_logger().info('SCARA Robot Controller node has been started.')

    rclpy.spin(scara_robot_controller)

    scara_robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

