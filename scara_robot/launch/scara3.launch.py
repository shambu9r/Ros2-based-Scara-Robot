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
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set the path to the Gazebo world file
    world_file = os.path.join(get_package_share_directory('scara_robot'), 'worlds', 'scara.world')

    # Set the path to the URDF file
    urdf_file = os.path.join(get_package_share_directory('scara_robot'), 'urdf', 'scara1.urdf')

    # Create the launch description
    ld = LaunchDescription()

    gazebo_node = Node(
    package='gazebo_ros',
    executable='/usr/bin/gazebo',
    name='gazebo',
    arguments=['-s', 'libgazebo_ros_factory.so', world_file, '--verbose'],
    output='screen'
   )

    ld.add_action(gazebo_node)



    # Launch the SCARA robot controller node
    controller_node = Node(
        package='scara_robot',
        executable='scara_robot_controller.py',
        name='scara_robot_controller',
        output='screen'
    )
    ld.add_action(controller_node)

    return ld
   
                    

