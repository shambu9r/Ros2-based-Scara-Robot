import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the package share directory
    package_share_directory = get_package_share_directory('scara_robot')

    # Xacro macro file paths
    xacro_macros = os.path.join(package_share_directory, 'urdf', 'macros.xacro')
    scara_xacro = os.path.join(package_share_directory, 'urdf', 'scara.xacro')

    # Create the xacro macro substitution
    xacro_macro = {'xacro_macros': xacro_macros}

    # Create the URDF file path after xacro substitution
    urdf_file_substituted = os.path.join(package_share_directory, 'urdf', 'robot.urdf')

    # Create the Gazebo simulation launch description
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share_directory, 'launch', 'gazebo.launch.py'))
    )

    # Create the robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_file_substituted}]
    )

    # Create the joint state publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Create the Gazebo spawn entity node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', 'scara_robot', '-file', urdf_file_substituted]
    )

    # Create the launch description with all the nodes and actions
    launch_description = LaunchDescription()

    # Add the nodes and actions to the launch description
    launch_description.add_action(gazebo_launch)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher)
    launch_description.add_action(spawn_entity)

    return launch_description

