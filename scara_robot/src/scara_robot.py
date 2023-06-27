#!/usr/bin/env python3

import numpy as np


class SCARARobot:
    def __init__(self, link_lengths):
        self.link_lengths = link_lengths

    def calculate_inverse_kinematics(self, desired_position):
        x, y, z = desired_position

        # Solve for joint angles using inverse kinematics equations
        a1, a2 = self.link_lengths
        theta2 = np.arccos((x**2 + y**2 - a1**2 - a2**2) / (2 * a1 * a2))
        theta1 = np.arctan2(y, x) - np.arctan2((a2 * np.sin(theta2)), (a1 + a2 * np.cos(theta2)))
        d3 = z

        return [theta1, theta2, d3]

    def calculate_jacobian(self, joint_angles):
        theta1, theta2, _ = joint_angles
        a1, a2 = self.link_lengths

        # Compute the Jacobian matrix
        jacobian = np.array([[-a1 * np.sin(theta1) - a2 * np.sin(theta1 + theta2), -a2 * np.sin(theta1 + theta2), 0],
                             [a1 * np.cos(theta1) + a2 * np.cos(theta1 + theta2), a2 * np.cos(theta1 + theta2), 0],
                             [0, 0, 1]])

        return jacobian

    def resolved_rate_motion_control(self, current_joint_angles, desired_position, gain=0.1):
        # Calculate the position error
        current_position = self.calculate_forward_kinematics(current_joint_angles)[:3]
        position_error = desired_position - current_position

        # Calculate the Jacobian matrix
        jacobian = self.calculate_jacobian(current_joint_angles)

        # Calculate the joint velocities using the resolved rate motion control equation
        joint_velocities = np.dot(np.linalg.pinv(jacobian), gain * position_error)

        # Update the joint angles using the joint velocities
        new_joint_angles = current_joint_angles + joint_velocities

        return new_joint_angles

    def calculate_forward_kinematics(self, joint_angles):
        theta1, theta2, d3 = joint_angles
        a1, a2 = self.link_lengths

        # Calculate the forward kinematics
        x = a1 * np.cos(theta1) + a2 * np.cos(theta1 + theta2)
        y = a1 * np.sin(theta1) + a2 * np.sin(theta1 + theta2)
        z = d3

        return np.array([x, y, z])

def main():
    # Link lengths (in meters)
    link_lengths = [0.5, 0.3]

    # Create a SCARA robot object
    robot = SCARARobot(link_lengths)

    # Joint angles (in radians)
    initial_joint_angles = [0.1, 0.2, 0.3]

    # Desired end effector position
    desired_position = np.array([0.2, 0.3, 0.1])

    # Calculate inverse kinematics
    desired_joint_angles = robot.calculate_inverse_kinematics(desired_position)

    # Print the initial and desired joint angles
    print("Initial Joint Angles:", initial_joint_angles)
    print("Desired Joint Angles:", desired_joint_angles)

    # Perform resolved rate motion control
    new_joint_angles = robot.resolved_rate_motion_control(initial_joint_angles, desired_position)

    # Print the new joint angles
    print("New Joint Angles:", new_joint_angles)

if __name__ == '__main__':
    main()

