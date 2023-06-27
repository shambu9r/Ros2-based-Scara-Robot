from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_file = 'ros2_ws/src/scara_robot/urdf/scara_robot.urdf'
    robot_name = 'scara_robot'
    rviz_config_file = '/rviz/scara_robot.rviz'

    robot_description = DeclareLaunchArgument(
        'robot_description',
        default_value=urdf_file,
        description='Path to the URDF file'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': LaunchConfiguration('robot_description')}],
        output='screen'
    )

    

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-file', LaunchConfiguration('urdf_file'),
            '-x', '0',
            '-y', '0',
            '-z', '0',
            '-Y', '0'
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('urdf_file', default_value=urdf_file, description='Path to URDF file'),
        DeclareLaunchArgument('robot_description', default_value=urdf_file, description='Robot description parameter'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config_file, description='Path to RViz configuration file'),
        robot_description,
              
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])),
                    
                    
        spawn_entity,
        rviz,
        robot_state_publisher
    ])
if __name__ == '__main__':
    generate_launch_description()
