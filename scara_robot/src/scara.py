import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from scipy.linalg import pinv


class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')
        self.publisher = self.create_publisher(TransformStamped, 'end_effector_transform', 10)

    def calculate_forward_kinematics(self, joint1, joint2, joint3):
        # Robot parameters 
        link1_length = 0.5
        link2_length = 0.3

        # Calculate the forward kinematics
        x = link1_length * np.cos(joint1) + link2_length * np.cos(joint1 + joint2)
        y = link1_length * np.sin(joint1) + link2_length * np.sin(joint1 + joint2)
        z = 0.0  # Assuming a planar SCARA robot, no vertical movement

        # Return the end effector position and orientation
        return np.array([x, y, z]), np.array([0.0, 0.0, joint1 + joint2, 0.0])

    def publish_end_effector_transform(self, end_effector_transform):
        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        transform_stamped_msg.header.frame_id = 'base_link'
        transform_stamped_msg.child_frame_id = 'end_effector_link'
        transform_stamped_msg.transform.translation.x = end_effector_transform[0]
        transform_stamped_msg.transform.translation.y = end_effector_transform[1]
        transform_stamped_msg.transform.translation.z = end_effector_transform[2]
        transform_stamped_msg.transform.rotation.x = end_effector_transform[3]
        transform_stamped_msg.transform.rotation.y = end_effector_transform[4]
        transform_stamped_msg.transform.rotation.z = end_effector_transform[5]
        transform_stamped_msg.transform.rotation.w = end_effector_transform[6]
        self.publisher.publish(transform_stamped_msg)


class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        self.subscription = self.create_subscription(
            TransformStamped,
            'end_effector_transform',
            self.end_effector_transform_callback,
            10
        )
        self.publisher = self.create_publisher(JointState, 'desired_joint_states', 10)
        self.jacobian = None  # Jacobian matrix

    def end_effector_transform_callback(self, msg):
        end_effector_transform = msg.transform
        desired_joint_states = self.calculate_inverse_kinematics(end_effector_transform)
        self.publish_desired_joint_states(desired_joint_states)

    def calculate_inverse_kinematics(self, end_effector_transform):
        # Extract the translation and rotation components from the end effector transform
        desired_position = np.array([end_effector_transform.translation.x, end_effector_transform.translation.y, end_effector_transform.translation.z])
        desired_orientation = np.array([end_effector_transform.rotation.x, end_effector_transform.rotation.y, end_effector_transform.rotation.z, end_effector_transform.rotation.w])

        # Initialize the desired joint angles
        desired_joint_states = [0.0, 0.0, 0.0]

        # Set the convergence threshold and maximum iterations for the inverse kinematics solver
        convergence_threshold = 0.001
        max_iterations = 100

        # Perform inverse kinematics using the Jacobian and resolved rate motion control
        for iteration in range(max_iterations):
            current_position, current_orientation = self.calculate_forward_kinematics(desired_joint_states[0], desired_joint_states[1], desired_joint_states[2])

            # Calculate the position error
            position_error = desired_position - current_position

            # Calculate the orientation error
            orientation_error = self.calculate_orientation_error(desired_orientation, current_orientation)

            # Combine the position and orientation errors into a single error vector
            error_vector = np.concatenate((position_error, orientation_error))

            # Calculate the Jacobian matrix
            self.calculate_jacobian(desired_joint_states[0], desired_joint_states[1], desired_joint_states[2])

            # Calculate the pseudo-inverse of the Jacobian matrix
            jacobian_inverse = pinv(self.jacobian)

            # Calculate the joint velocities using the resolved rate motion control equation
            joint_velocities = np.dot(jacobian_inverse, error_vector)

            # Update the joint angles using the joint velocities
            desired_joint_states += joint_velocities

            # Check if the convergence threshold has been reached
            if np.linalg.norm(joint_velocities) < convergence_threshold:
                break

        return desired_joint_states

    def calculate_forward_kinematics(self, joint1, joint2, joint3):
        # Robot parameters (replace with your robot's parameters)
        link1_length = 0.5
        link2_length = 0.3

        # Calculate the forward kinematics
        x = link1_length * np.cos(joint1) + link2_length * np.cos(joint1 + joint2)
        y = link1_length * np.sin(joint1) + link2_length * np.sin(joint1 + joint2)
        z = 0.0  # Assuming a planar SCARA robot, no vertical movement

        # Return the end effector position and orientation
        return np.array([x, y, z]), np.array([0.0, 0.0, joint1 + joint2, 0.0])

    def calculate_orientation_error(self, desired_orientation, current_orientation):
        # Calculate the orientation error as the difference between desired and current orientations
        error = desired_orientation - current_orientation
        return error

    def calculate_jacobian(self, joint1, joint2, joint3):
        # Robot parameters (replace with your robot's parameters)
        link1_length = 0.5
        link2_length = 0.3

        # Calculate the Jacobian matrix
        jacobian = np.array([
            [-link1_length * np.sin(joint1) - link2_length * np.sin(joint1 + joint2), -link2_length * np.sin(joint1 + joint2), 0.0],
            [link1_length * np.cos(joint1) + link2_length * np.cos(joint1 + joint2), link2_length * np.cos(joint1 + joint2), 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0]
        ])

        self.jacobian = jacobian

    def publish_desired_joint_states(self, desired_joint_states):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint1', 'joint2', 'joint3']
        joint_state_msg.position = desired_joint_states
        self.publisher.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    forward_kinematics_node = ForwardKinematicsNode()
    inverse_kinematics_node = InverseKinematicsNode()
    rclpy.spin(forward_kinematics_node)
    rclpy.spin(inverse_kinematics_node)
    forward_kinematics_node.destroy_node()
    inverse_kinematics_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

