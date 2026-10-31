import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch the closed-loop station-keeping research experiment."""
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
    current_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='research_current_bridge',
        arguments=[
            '/ocean_current@geometry_msgs/msg/Vector3]ignition.msgs.Vector3d',
        ],
        output='screen',
    )

    def number(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    current = Node(
        package=package_name,
        executable='research_current_disturbance',
        name='research_current_disturbance',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'ghost': ParameterValue(
                LaunchConfiguration('ghost'), value_type=bool),
            'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
            'current_x': number('current_x'),
            'current_y': number('current_y'),
            'current_z': number('current_z'),
            'current_turbulence_std': number('current_turbulence_std'),
            'wind_speed': number('wind_speed'),
            'wind_direction_deg': number('wind_direction_deg'),
            'wind_gust_std': number('wind_gust_std'),
            'wind_direction_wander_deg': number('wind_direction_wander_deg'),
            'wave_height': number('wave_height'),
            'wave_heave_scale': number('wave_heave_scale'),
            'wave_moment_scale': number('wave_moment_scale'),
            'wave_period': number('wave_period'),
            'wave_direction_deg': number('wave_direction_deg'),
        }],
    )
    estimator = Node(
        package=package_name,
        executable='research_state_estimator',
        name='research_state_estimator',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_estimator')),
    )
    marker = Node(
        package=package_name,
        executable='research_target_marker',
        name='research_target_marker',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    scenario = Node(
        package=package_name,
        executable='research_scenario_runner',
        name='research_scenario_runner',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scenario': LaunchConfiguration('scenario'),
            'events_json': ParameterValue(
                LaunchConfiguration('scenario_events'), value_type=str),
        }],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('scenario'), "' != 'none'"])),
    )
    mapping = Node(
        package=package_name,
        executable='research_map_quality',
        name='research_map_quality',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'log_file': LaunchConfiguration('map_log'),
        }],
    )
    mission = Node(
        package=package_name,
        executable='research_mission_runner',
        name='research_mission_runner',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mode': LaunchConfiguration('mission'),
        }],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('mission'), "' != 'none'"])),
    )
    feedback_topic = PythonExpression([
        "'/research_catamaran/estimated_odometry' if '",
        LaunchConfiguration('use_estimator'),
        "' == 'true' else '/model/research_catamaran/odometry'",
    ])
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
                'log_file': LaunchConfiguration('log_file'),
                'odom_topic': feedback_topic,
                'use_dob': ParameterValue(
                    LaunchConfiguration('use_dob'), value_type=bool),
                'hold_enabled': ParameterValue(
                    LaunchConfiguration('hold_enabled'), value_type=bool),
                'actuator_model': ParameterValue(
                    LaunchConfiguration('actuator_model'), value_type=bool),
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('target_x', default_value='0.0'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('target_yaw', default_value='0.0'),
        DeclareLaunchArgument('current_x', default_value='0.10'),
        DeclareLaunchArgument('current_y', default_value='0.00'),
        DeclareLaunchArgument('current_z', default_value='0.00'),
        DeclareLaunchArgument('enable_yaw_control', default_value='true'),
        # Sea state.  Wind direction is the direction the wind blows TOWARD
        # in the world frame (0 = +X, 90 = +Y).  Defaults are a moderate,
        # repeatable breeze with gusts, chop and a turbulent current.
        DeclareLaunchArgument('wind_speed', default_value='3.0'),
        DeclareLaunchArgument('wind_direction_deg', default_value='135.0'),
        DeclareLaunchArgument('wind_gust_std', default_value='0.30'),
        DeclareLaunchArgument('wind_direction_wander_deg', default_value='12.0'),
        DeclareLaunchArgument('current_turbulence_std', default_value='0.02'),
        DeclareLaunchArgument('wave_height', default_value='0.10'),
        DeclareLaunchArgument('wave_period', default_value='2.5'),
        DeclareLaunchArgument('wave_direction_deg', default_value='135.0'),
        DeclareLaunchArgument('seed', default_value='1'),
        # Visual smoothness of wave heave / tilt (1.0 = full coupling).
        DeclareLaunchArgument('wave_heave_scale', default_value='0.1'),
        DeclareLaunchArgument('wave_moment_scale', default_value='0.04'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'use_dob', default_value='false',
            description='Feed the disturbance-observer estimate forward.'),
        DeclareLaunchArgument('scenario_events', default_value=''),
        DeclareLaunchArgument(
            'mission', default_value='none',
            description='none | weathervane (bow into wind) | waypoints'),
        DeclareLaunchArgument('map_log', default_value='/tmp/asv_map_quality.csv'),
        DeclareLaunchArgument(
            'scenario', default_value='none',
            description='"narrated" runs the scripted fault-injection demo.'),
        DeclareLaunchArgument(
            'ghost', default_value='false',
            description='Also spawn a passive uncontrolled twin boat.'),
        DeclareLaunchArgument(
            'ghost_x', default_value=PythonExpression([
                "round(1.8*__import__('math').cos(__import__('math')."
                "radians(", LaunchConfiguration('wind_direction_deg'),
                "-90)) - 1.0*__import__('math').cos(__import__('math')."
                "radians(", LaunchConfiguration('wind_direction_deg'), ")), 2)"
            ])),
        DeclareLaunchArgument(
            'ghost_y', default_value=PythonExpression([
                "round(1.8*__import__('math').sin(__import__('math')."
                "radians(", LaunchConfiguration('wind_direction_deg'),
                "-90)) - 1.0*__import__('math').sin(__import__('math')."
                "radians(", LaunchConfiguration('wind_direction_deg'), ")), 2)"
            ])),
        DeclareLaunchArgument(
            'hold_enabled', default_value='true',
            description='false = thrusters off (uncontrolled baseline).'),
        DeclareLaunchArgument(
            'actuator_model', default_value='true',
            description='Thruster deadband, lag and weaker reverse thrust.'),
        DeclareLaunchArgument(
            'use_estimator', default_value='true',
            description='Close the loop on noisy GPS/IMU instead of truth.'),
        DeclareLaunchArgument('log_file', default_value=(
            '/tmp/research_dynamic_station_keeping.csv')),
        DeclareLaunchArgument('world_file', default_value=(
            'research_pool_dynamics.sdf')),
        DeclareLaunchArgument('model_file', default_value=(
            'research_catamaran.urdf')),
        DeclareLaunchArgument('spawn_z', default_value='-0.10'),
        DeclareLaunchArgument('software_rendering', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(research_launch),
            launch_arguments={
                'world_file': LaunchConfiguration('world_file'),
                'model_file': LaunchConfiguration('model_file'),
                'spawn_z': LaunchConfiguration('spawn_z'),
                'software_rendering': LaunchConfiguration(
                    'software_rendering'),
                'headless': LaunchConfiguration('headless'),
                'ghost': LaunchConfiguration('ghost'),
                'ghost_x': LaunchConfiguration('ghost_x'),
                'ghost_y': LaunchConfiguration('ghost_y'),
            }.items(),
        ),
        TimerAction(
            period=4.0, actions=[current_bridge, current, estimator]),
        TimerAction(
            period=7.0, actions=[controller, marker, scenario, mapping, mission]),
    ])
