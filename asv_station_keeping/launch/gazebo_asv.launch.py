import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'asv_station_keeping'
    pkg_share = get_package_share_directory(package_name)

    world_file = os.path.join(
        pkg_share,
        'worlds',
        'asv_test.sdf'
    )

    urdf_file = os.path.join(
        pkg_share,
        'urdf',
        'asv_station_keeping.urdf'
    )

    gazebo_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([

        # Start Gazebo Fortress
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'gz_args': f'-r "{world_file}"',
                'on_exit_shutdown': 'true',
            }.items()
        ),

        # Publish the robot description and TF tree
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # Spawn the ASV into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', 'asv_test',
                '-name', 'asv',
                '-file', urdf_file,
                '-x', '0',
                '-y', '0',
                '-z', '0.3'
            ],
            output='screen'
        ),

    ])
