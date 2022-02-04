import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    """
    teleop_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('teleop_twist_joy'), 'launch'),
                '/teleop-launch.py']),
        launch_arguments={'joy_config': 'xbox', 'joy_dev': ''}.items(),
        )
    """

    config_filepath = os.path.join(
            get_package_share_directory('teleop_twist_joy'), 'config', 'xbox.config.yaml')

    joy_node = Node(
        package='joy', 
        executable='joy_node', 
        name='joy_node',
        parameters=[{
            'device_name': '',
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }])

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy', 
        executable='teleop_node',
        name='teleop_twist_joy_node', 
        parameters=[config_filepath]
        )

    return LaunchDescription([
        joy_node,
        teleop_twist_joy_node
    ])
