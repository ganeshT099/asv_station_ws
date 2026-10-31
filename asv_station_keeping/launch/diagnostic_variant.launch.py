import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'asv_station_keeping'
    package_share = get_package_share_directory(package_name)
    variant = LaunchConfiguration('variant')
    world_file = os.path.join(package_share, 'worlds', 'asv_test.sdf')
    variant_dir = os.path.join(package_share, 'diagnostics', 'variants')
    gazebo_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py',
    )

    # LaunchConfiguration cannot be joined with os.path.join at Python
    # evaluation time, so the path is assembled as a launch substitution.
    variant_file = PathJoinSubstitution([variant_dir, variant])

    return LaunchDescription([
        DeclareLaunchArgument(
            'variant',
            default_value='00_original.urdf',
            description='Diagnostic URDF filename from diagnostics/variants',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'gz_args': f'-r "{world_file}"',
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        TimerAction(
            period=2.0,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-world', 'asv_test',
                    '-name', 'asv_diag',
                    '-file', variant_file,
                    '-x', '0', '-y', '0', '-z', '0.3',
                ],
                output='screen',
            )],
        ),
    ])
