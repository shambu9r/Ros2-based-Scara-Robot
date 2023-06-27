import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from scara_robot.Scripts import GazeboRosPaths  

def generate_launch_description():
    package_name = 'scara_robot'
    package_share_directory = get_package_share_directory(package_name)
    urdf_file = os.path.join(package_share_directory, "urdf", "scara1.urdf")

    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()
    env = {
        "GAZEBO_MODEL_PATH": model_path,
        "GAZEBO_PLUGIN_PATH": plugin_path,
        "GAZEBO_RESOURCE_PATH": media_path,
    }

    gazebo_process = ExecuteProcess(
        cmd=["gazebo", "-s", "libgazebo_ros_factory.so"],
        output="screen",
        additional_env=env,
    )

    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "scara_robot", "-b", "-file", urdf_file],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        arguments=[urdf_file],
    )

    return LaunchDescription([
        gazebo_process,
        spawn_entity_node,
        robot_state_publisher_node
    ])

