import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'asv_station_keeping'
    package_share = get_package_share_directory(package_name)
    research_launch = os.path.join(
        package_share,
        'launch',
        'research_catamaran.launch.py',
    )
    controller_config = os.path.join(
        package_share,
        'config',
        'research_controller.yaml',
    )

    controller = Node(
        package=package_name,
        executable='research_station_keeping',
        name='research_station_keeping_pid',
        output='screen',
        parameters=[
            controller_config,
            {
                'target_x': LaunchConfiguration('target_x'),
                'target_y': LaunchConfiguration('target_y'),
                'target_yaw': LaunchConfiguration('target_yaw'),
                'current_x': LaunchConfiguration('current_x'),
                'current_y': LaunchConfiguration('current_y'),
                'current_z': LaunchConfiguration('current_z'),
                'enable_yaw_control': LaunchConfiguration(
                    'enable_yaw_control'),
            },
        ],
    )
    current_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='research_current_bridge',
        arguments=[
            '/ocean_current@geometry_msgs/msg/Vector3]ignition.msgs.Vector3d',
        ],
        output='screen',
    )
    current = Node(
        package=package_name,
        executable='research_current_disturbance',
        name='research_current_disturbance',
        output='screen',
        parameters=[{
            'current_x': LaunchConfiguration('current_x'),
            'current_y': LaunchConfiguration('current_y'),
            'current_z': LaunchConfiguration('current_z'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('target_x', default_value='0.0'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('target_yaw', default_value='0.0'),
        DeclareLaunchArgument('current_x', default_value='0.0'),
        DeclareLaunchArgument('current_y', default_value='0.0'),
        DeclareLaunchArgument('current_z', default_value='0.0'),
        DeclareLaunchArgument(
            'enable_yaw_control',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='research_pool_dynamics.sdf',
        ),
        DeclareLaunchArgument(
            'model_file',
            default_value='research_catamaran.urdf',
        ),
        DeclareLaunchArgument('spawn_z', default_value='-0.10'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(research_launch),
            launch_arguments={
                'world_file': LaunchConfiguration('world_file'),
                'model_file': LaunchConfiguration('model_file'),
                'spawn_z': LaunchConfiguration('spawn_z'),
            }.items(),
        ),
        TimerAction(period=4.0, actions=[current_bridge, current]),
        TimerAction(period=7.0, actions=[controller]),
    ])
