import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Gazebo, the mixer, and the repeatable motion validator."""
    package_name = 'asv_station_keeping'
    package_share = get_package_share_directory(package_name)
    research_launch = os.path.join(
        package_share,
        'launch',
        'research_catamaran.launch.py',
    )
    mixer = Node(
        package=package_name,
        executable='research_twist_to_thruster',
        name='research_twist_to_thruster',
        output='screen',
        parameters=[{
            'command_scale': LaunchConfiguration('command_scale'),
            'max_thrust': LaunchConfiguration('max_thrust'),
            'slew_rate': LaunchConfiguration('slew_rate'),
            'enable_tilt_guard': LaunchConfiguration('enable_tilt_guard'),
            'tilt_soft_limit': LaunchConfiguration('tilt_soft_limit'),
            'tilt_hard_limit': LaunchConfiguration('tilt_hard_limit'),
        }],
    )
    validator = Node(
        package=package_name,
        executable='verify_motion_stability',
        name='research_motion_validation',
        output='screen',
        parameters=[{
            'command': LaunchConfiguration('command'),
            'maneuver_time': LaunchConfiguration('maneuver_time'),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file',
            default_value='research_pool_dynamics.sdf',
        ),
        DeclareLaunchArgument('spawn_z', default_value='-0.10'),
        DeclareLaunchArgument('command', default_value='0.15'),
        DeclareLaunchArgument('maneuver_time', default_value='3.0'),
        DeclareLaunchArgument('command_scale', default_value='1.5'),
        DeclareLaunchArgument('max_thrust', default_value='6.0'),
        DeclareLaunchArgument('slew_rate', default_value='0.35'),
        DeclareLaunchArgument('enable_tilt_guard', default_value='true'),
        DeclareLaunchArgument('tilt_soft_limit', default_value='0.10'),
        DeclareLaunchArgument('tilt_hard_limit', default_value='0.35'),
        DeclareLaunchArgument('software_rendering', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(research_launch),
            launch_arguments={
                'world_file': LaunchConfiguration('world_file'),
                'spawn_z': LaunchConfiguration('spawn_z'),
                'software_rendering': LaunchConfiguration(
                    'software_rendering'),
            }.items(),
        ),
        TimerAction(period=4.0, actions=[mixer]),
        TimerAction(period=8.0, actions=[validator]),
    ])
