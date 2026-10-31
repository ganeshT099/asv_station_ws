"""One-command guide demonstration: side-by-side controlled vs uncontrolled.

Starts Gazebo (with wind/current arrows, trail and target marker), RViz, the
dashboard and the narrated fault-injection timeline, and spawns an orange
uncontrolled twin boat in the same sea so the effect of station keeping is
visible at a glance.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('asv_station_keeping')
    visual = os.path.join(share, 'launch', 'visual_station_keeping.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('open_dashboard', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(visual),
            launch_arguments={
                'ghost': 'true',
                'scenario': 'narrated',
                'demo_mode': 'false',
                'target_x': '0.0', 'target_y': '0.0', 'target_yaw': '0.0',
                'current_x': '0.10', 'current_y': '0.0',
                'wind_speed': '3.0', 'wind_direction_deg': '135.0',
                'start_rviz': LaunchConfiguration('start_rviz'),
                'open_dashboard': LaunchConfiguration('open_dashboard'),
            }.items(),
        ),
    ])
