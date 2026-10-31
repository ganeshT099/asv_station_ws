"""Launch the complete live station-keeping demonstration stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start Gazebo, controller, RViz, visualization and browser dashboard."""
    package_name = 'asv_station_keeping'
    package_share = get_package_share_directory(package_name)
    dynamic_launch = os.path.join(
        package_share,
        'launch',
        'dynamic_station_keeping.launch.py',
    )
    rviz_config = os.path.join(
        package_share,
        'rviz',
        'station_keeping.rviz',
    )

    visualization = Node(
        package=package_name,
        executable='research_station_keeping_visualization',
        name='research_station_keeping_visualization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'visualization_frame': 'world',
            'fallback_frame': 'world',
            'broadcast_tf': True,
            'tf_parent_frame': 'world',
            'tf_child_frame': 'base_footprint',
            'publish_rate': 10.0,
            'stale_timeout': 1.5,
        }],
    )
    dashboard = Node(
        package=package_name,
        executable='research_station_keeping_dashboard',
        name='research_station_keeping_dashboard',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'host': LaunchConfiguration('dashboard_host'),
            'advertise_host': LaunchConfiguration('dashboard_url_host'),
            'port': ParameterValue(
                LaunchConfiguration('dashboard_port'), value_type=int),
            'open_browser': ParameterValue(
                LaunchConfiguration('open_dashboard'), value_type=bool),
            'history_size': ParameterValue(
                LaunchConfiguration('history_size'), value_type=int),
        }],
    )
    demo = Node(
        package=package_name,
        executable='research_station_keeping_demo',
        name='research_station_keeping_demo',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'auto_start': ParameterValue(
                LaunchConfiguration('auto_start_demo'), value_type=bool),
            'phase_duration': ParameterValue(
                LaunchConfiguration('demo_phase_duration'), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration('demo_mode')),
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='research_station_keeping_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_x', default_value='0.40',
            description='Desired station x coordinate in metres.'),
        DeclareLaunchArgument(
            'target_y', default_value='0.20',
            description='Desired station y coordinate in metres.'),
        DeclareLaunchArgument(
            'target_yaw', default_value='0.20',
            description='Desired heading in radians.'),
        DeclareLaunchArgument(
            'current_x', default_value='0.10',
            description='World-frame current x component in m/s.'),
        DeclareLaunchArgument(
            'current_y', default_value='0.04',
            description='World-frame current y component in m/s.'),
        DeclareLaunchArgument(
            'current_z', default_value='0.00',
            description='World-frame current z component in m/s.'),
        DeclareLaunchArgument('enable_yaw_control', default_value='true'),
        DeclareLaunchArgument('ghost', default_value='false'),
        DeclareLaunchArgument('scenario', default_value='none'),
        DeclareLaunchArgument('mission', default_value='none'),
        DeclareLaunchArgument('use_dob', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('wind_speed', default_value='3.0'),
        DeclareLaunchArgument('wind_direction_deg', default_value='135.0'),
        DeclareLaunchArgument('wind_gust_std', default_value='0.30'),
        DeclareLaunchArgument('wind_direction_wander_deg', default_value='12.0'),
        DeclareLaunchArgument('current_turbulence_std', default_value='0.02'),
        DeclareLaunchArgument('wave_height', default_value='0.10'),
        DeclareLaunchArgument('wave_period', default_value='2.5'),
        DeclareLaunchArgument('wave_direction_deg', default_value='135.0'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('use_estimator', default_value='true'),
        DeclareLaunchArgument(
            'log_file',
            default_value='/tmp/research_dynamic_station_keeping.csv'),
        DeclareLaunchArgument(
            'world_file', default_value='research_pool_dynamics.sdf'),
        DeclareLaunchArgument(
            'model_file', default_value='research_catamaran.urdf'),
        DeclareLaunchArgument('spawn_z', default_value='-0.10'),
        DeclareLaunchArgument('software_rendering', default_value='false'),
        DeclareLaunchArgument('demo_mode', default_value='true'),
        DeclareLaunchArgument('auto_start_demo', default_value='true'),
        DeclareLaunchArgument('demo_phase_duration', default_value='12.0'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('open_dashboard', default_value='true'),
        DeclareLaunchArgument(
            'dashboard_host',
            default_value='0.0.0.0',
            description='Dashboard bind address; 0.0.0.0 listens locally and on LAN.'),
        DeclareLaunchArgument(
            'dashboard_url_host',
            default_value='127.0.0.1',
            description='Host printed/opened for the dashboard browser.'),
        DeclareLaunchArgument('dashboard_port', default_value='8765'),
        DeclareLaunchArgument('history_size', default_value='1200'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dynamic_launch),
            launch_arguments={
                'target_x': LaunchConfiguration('target_x'),
                'target_y': LaunchConfiguration('target_y'),
                'target_yaw': LaunchConfiguration('target_yaw'),
                'current_x': LaunchConfiguration('current_x'),
                'current_y': LaunchConfiguration('current_y'),
                'current_z': LaunchConfiguration('current_z'),
                'enable_yaw_control': LaunchConfiguration(
                    'enable_yaw_control'),
                'wind_speed': LaunchConfiguration('wind_speed'),
                'wind_direction_deg': LaunchConfiguration('wind_direction_deg'),
                'wind_gust_std': LaunchConfiguration('wind_gust_std'),
                'wind_direction_wander_deg': LaunchConfiguration('wind_direction_wander_deg'),
                'current_turbulence_std': LaunchConfiguration('current_turbulence_std'),
                'wave_height': LaunchConfiguration('wave_height'),
                'wave_period': LaunchConfiguration('wave_period'),
                'wave_direction_deg': LaunchConfiguration('wave_direction_deg'),
                'seed': LaunchConfiguration('seed'),
                'use_estimator': LaunchConfiguration('use_estimator'),
                'ghost': LaunchConfiguration('ghost'),
                'scenario': LaunchConfiguration('scenario'),
                'mission': LaunchConfiguration('mission'),
                'use_dob': LaunchConfiguration('use_dob'),
                'headless': LaunchConfiguration('headless'),
                'log_file': LaunchConfiguration('log_file'),
                'world_file': LaunchConfiguration('world_file'),
                'model_file': LaunchConfiguration('model_file'),
                'spawn_z': LaunchConfiguration('spawn_z'),
                'software_rendering': LaunchConfiguration(
                    'software_rendering'),
            }.items(),
        ),
        TimerAction(
            period=4.0,
            actions=[visualization, dashboard, demo, rviz],
        ),
    ])
