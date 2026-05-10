"""
Launch QCar2 with camera + LiDAR bridged to ROS 2.

Usage:
    ros2 launch qcar2 simulation.launch.py
    ros2 launch qcar2 simulation.launch.py world:=qcar_straight
    ros2 launch qcar2 simulation.launch.py world:=qcar_track

Available worlds (in husarion_gz_worlds/worlds/):
    qcar_track    — circular lane-following track (default, production)
    qcar_straight — straight 30 m calibration track (same lane width).
                    Use this to tune lane_width_bev_px and the source
                    trapezoid corners; the values transfer directly to
                    qcar_track because the lane width is identical.

Spawn pose is (x=2.0, y=0, yaw=pi/2). Both worlds have their lane
centreline aligned so the car spawns at the start of the lane.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _world_arg(default='qcar_track'):
    """Parse `world:=<name>` from sys.argv (no LaunchConfiguration needed)."""
    for arg in sys.argv:
        if arg.startswith('world:='):
            return arg.split(':=', 1)[1]
    return default


def generate_launch_description():
    qcar2_share = get_package_share_directory('qcar2')
    worlds_share = get_package_share_directory('husarion_gz_worlds')

    urdf_file = os.path.join(qcar2_share, 'urdf', 'QCar2.urdf')
    world_name = _world_arg()
    world_file = os.path.join(worlds_share, 'worlds', f'{world_name}.sdf')
    if not os.path.isfile(world_file):
        raise RuntimeError(
            f'World file not found: {world_file}. '
            f'Available worlds: '
            + ', '.join(sorted(
                f[:-4] for f in os.listdir(os.path.join(worlds_share, "worlds"))
                if f.endswith('.sdf')
            ))
        )
    print(f"[simulation.launch.py] Loading world: {world_name} -> {world_file}")

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Set Gazebo resource path so it can find QCar2 meshes
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.pathsep.join([
            qcar2_share,
            worlds_share,
            os.path.join(worlds_share, 'models'),
        ])
    )

    return LaunchDescription([
        gz_resource_path,

        # ── Gazebo Sim ────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py',
                ])
            ]),
            launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        ),

        # ── Spawn QCar2 on round track at start/finish line ──────────────────
        # Centerline radius=2.0m, spawn at (2.0, 0) facing +Y (counter-clockwise)
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'qcar2',
                '-file', urdf_file,
                '-x',    '2.0',
                '-y',    '0.0',
                '-z',    '0.05',
                '-Y',    '1.5708',
            ],
            output='screen',
        ),

        # ── Robot state publisher ─────────────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # ── ROS ↔ Gz topic bridge ─────────────────────────────────────────────
        # [  = Gz → ROS   (sensor output)
        # ]  = ROS → Gz   (command input)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/qcar2/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/qcar2/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/qcar2/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/model/qcar2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/qcar2/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            output='screen',
        ),

    ])
