"""
Launch perception + controller + RViz for line following.

Usage:
    ros2 launch qcar2_line_tracker line_tracker.launch.py
    ros2 launch qcar2_line_tracker line_tracker.launch.py robot:=qcar2

To support a new robot, add config/perception_<name>.yaml and
config/controller_<name>.yaml, then pass robot:=<name>.

Assumes simulation.launch.py is already running.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('qcar2_line_tracker')
    rviz_config = os.path.join(pkg_share, 'rviz', 'qcar_sensors.rviz')

    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='qcar2',
        description='Robot name; selects config/{perception,controller}_<robot>.yaml',
    )
    robot = LaunchConfiguration('robot')

    perception_cfg = PathJoinSubstitution([
        FindPackageShare('qcar2_line_tracker'),
        'config', ['perception_', robot, '.yaml'],
    ])
    controller_cfg = PathJoinSubstitution([
        FindPackageShare('qcar2_line_tracker'),
        'config', ['controller_', robot, '.yaml'],
    ])

    return LaunchDescription([
        robot_arg,

        Node(
            package='qcar2_line_tracker',
            executable='line_perception_node',
            name='line_perception',
            parameters=[perception_cfg],
            output='screen',
        ),

        Node(
            package='qcar2_line_tracker',
            executable='line_controller_node',
            name='line_controller',
            parameters=[controller_cfg],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),

    ])
