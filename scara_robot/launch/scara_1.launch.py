import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to the SCARA robot xacro file
    scara_xacro_file = os.path.join(get_package_share_directory('scara_robot'), 'urdf', 'scara.xacro')

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    xacro_file = DeclareLaunchArgument('xacro_file', default_value=scara_xacro_file)
    rviz_config = DeclareLaunchArgument('rviz_config', default_value=os.path.join(get_package_share_directory('scara_robot'), 'rviz', 'scara_robot.rviz'))

    # Define the Gazebo launch description
    gazebo_launch = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_control.so',
            '-s', 'libgazebo_ros_diff_drive.so',
            '-s', 'libgazebo_ros_joint_state_publisher.so',
            '-s', 'libgazebo_ros_joint_trajectory_publisher.so',
            LaunchConfiguration('xacro_file'),
        ],
        output='screen'
    )

    # Define the robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[LaunchConfiguration('xacro_file')]
    )

    # Define the joint state publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[LaunchConfiguration('xacro_file')]
    )

    # Define the RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the launch actions to the launch description
    ld.add_action(use_sim_time)
    ld.add_action(xacro_file)
    ld.add_action(rviz_config)
    ld.add_action(gazebo_launch)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz)

    return ld

