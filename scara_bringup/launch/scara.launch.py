
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('scara_description'), 'config', 'scara.config.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('scara_description'),
            'config',
            'scara_controllers.yaml',
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('scara_description'), 'rviz', 'scara.rviz']
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
    )
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_joint_velocity_controller'],
    )

    slider_config = PathJoinSubstitution(
        [
            FindPackageShare('scara_bringup'),
            'config',
            'scara_vel_sp_config.yaml',
        ]
    )

    slider_node = Node(
        package='slider_publisher', 
        executable='slider_publisher', 
        name='slider_publisher',
        parameters=[{'rate': 10.0}],
        arguments = [slider_config])

    nodes = [
        rviz_node,
        control_node,
        robot_state_pub_node,
        # joint_state_broadcaster_spawner,
        # robot_controller_spawner,
        # slider_node,
    ]

    return LaunchDescription(nodes)
