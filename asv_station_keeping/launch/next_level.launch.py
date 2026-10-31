"""Next-level development launch: everything built so far, plus new additions.

Layer 1 (BASELINE) includes the existing, staff-demo-ready stack unchanged
(Gazebo, controller, estimator, RViz, dashboard, ghost twin, scenarios).
Layer 2 (NEW_FEATURES) is where new nodes/launch files are added, so the
finished demo launches (showcase / visual_station_keeping) are never touched.

Run:
    ros2 launch asv_station_keeping next_level.launch.py
    ros2 launch asv_station_keeping next_level.launch.py ghost:=false scenario:=none
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Compose the baseline demo stack with the new-feature layer."""
    package_name = 'asv_station_keeping'
    share = get_package_share_directory(package_name)
    visual = os.path.join(share, 'launch', 'visual_station_keeping.launch.py')

    # ------------------------------------------------------------------
    # Layer 1: BASELINE (previous work). Arguments are forwarded so any
    # of them can be overridden from the command line.
    # ------------------------------------------------------------------
    baseline_args = {
        'ghost': 'true',
        'scenario': 'narrated',
        'demo_mode': 'false',
        'target_x': '0.0',
        'target_y': '0.0',
        'target_yaw': '0.0',
        'current_x': '0.10',
        'current_y': '0.0',
    }
    forwarded = {
        key: LaunchConfiguration(key)
        for key in (
            'ghost', 'scenario', 'mission', 'use_dob', 'headless',
            'controller_type', 'allocation_mode',
            'start_rviz', 'open_dashboard', 'wind_speed',
            'wind_direction_deg', 'seed',
        )
    }
    forwarded.update({
        key: value for key, value in baseline_args.items()
        if key not in forwarded
    })

    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(visual),
        launch_arguments=forwarded.items(),
    )

    # ------------------------------------------------------------------
    # Layer 2: NEW_FEATURES. Add new Node(...) / IncludeLaunchDescription
    # entries to this list as the next level is built.
    # Example:
    #   Node(package=package_name, executable='my_new_node',
    #        parameters=[{'use_sim_time': True}], output='screen'),
    # ------------------------------------------------------------------
    new_features = []

    return LaunchDescription([
        DeclareLaunchArgument('ghost', default_value='true'),
        DeclareLaunchArgument('scenario', default_value='narrated'),
        DeclareLaunchArgument('mission', default_value='none'),
        DeclareLaunchArgument('use_dob', default_value='false'),
        DeclareLaunchArgument(
            'controller_type', default_value='pid',
            description='Control law: pid | lqr | smc | mpc.'),
        DeclareLaunchArgument(
            'allocation_mode', default_value='lstsq',
            description='Thrust allocation: lstsq | optimal.'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('open_dashboard', default_value='true'),
        DeclareLaunchArgument('wind_speed', default_value='3.0'),
        DeclareLaunchArgument('wind_direction_deg', default_value='135.0'),
        DeclareLaunchArgument('seed', default_value='1'),
        baseline,
        *new_features,
    ])
