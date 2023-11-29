import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Method to launch the nodes in the package with bag record flag"""
    record = LaunchConfiguration('record')

    return LaunchDescription([

        DeclareLaunchArgument(
            'record',
            default_value='False'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(
                    'turtlebot3_gazebo'), 'launch'), '/turtlebot3_world.launch.py'
            ])
        ),

        Node(
            package='walker',
            executable='walker',
        ),

        ExecuteProcess(
            condition=IfCondition(record),
            cmd=['ros2', 'bag', 'record', '-a', '-x /camera.+'],
            shell=True
        )

    ])