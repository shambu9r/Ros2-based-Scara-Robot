import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'scara_robot'
    package_share_directory = get_package_share_directory(package_name)

    # Launch the Python script
    scara_robot_node = Node(
        package=package_name,
        executable='scara_robot.py',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(scara_robot_node)

    return ld

