"""Launch the standalone geometric path-tracking demonstration."""

from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _prepare_marker_world(source: Path) -> str:
    """Create a temporary Fortress world with Gazebo's MarkerManager enabled."""
    target = Path('/tmp/asv_path_tracking_pool.sdf')
    text = source.read_text(encoding='utf-8')
    if 'name="Marker Manager"' not in text:
        insertion = (
            '      <plugin filename="MarkerManager" name="Marker Manager"/>\n'
        )
        text = text.replace(
            '      <plugin filename="EntityTree" name="Entity Tree">',
            insertion + '      <plugin filename="EntityTree" name="Entity Tree">',
            1,
        )
    target.write_text(text, encoding='utf-8')
    return str(target)


def generate_launch_description():
    """Start Gazebo, path reference, controller, RViz and dashboard."""
    package_name = 'asv_path_tracking'
    package_share = Path(get_package_share_directory(package_name))
    vehicle_share = Path(
        get_package_share_directory('asv_station_keeping'))
    world_path = _prepare_marker_world(
        vehicle_share / 'worlds' / 'research_pool_dynamics.sdf')
    gazebo_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py',
    )
    config_path = package_share / 'config' / 'path_tracking.yaml'
    rviz_path = package_share / 'rviz' / 'path_tracking.rviz'
    urdf_path = vehicle_share / 'urdf' / 'research_catamaran.urdf'
    robot_description = urdf_path.read_text(encoding='utf-8')

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='asv_path_tracking_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/research_catamaran/joint_states@sensor_msgs/msg/JointState['
            'ignition.msgs.Model',
            '/research_catamaran/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/research_catamaran/gps@sensor_msgs/msg/NavSatFix['
            'ignition.msgs.NavSat',
            '/model/research_catamaran/odometry@nav_msgs/msg/Odometry['
            'ignition.msgs.Odometry',
            '/model/research_catamaran/joint/'
            'port_front_propeller_joint/cmd_thrust@'
            'std_msgs/msg/Float64]ignition.msgs.Double',
            '/model/research_catamaran/joint/'
            'starboard_front_propeller_joint/cmd_thrust@'
            'std_msgs/msg/Float64]ignition.msgs.Double',
            '/model/research_catamaran/joint/'
            'port_rear_propeller_joint/cmd_thrust@'
            'std_msgs/msg/Float64]ignition.msgs.Double',
            '/model/research_catamaran/joint/'
            'starboard_rear_propeller_joint/cmd_thrust@'
            'std_msgs/msg/Float64]ignition.msgs.Double',
        ],
    )
    current_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='asv_path_tracking_current_bridge',
        output='screen',
        arguments=[
            '/ocean_current@geometry_msgs/msg/Vector3]'
            'ignition.msgs.Vector3d',
        ],
    )
    current = Node(
        package='asv_station_keeping',
        executable='research_current_disturbance',
        name='asv_path_tracking_current_disturbance',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'current_x': LaunchConfiguration('current_x'),
            'current_y': LaunchConfiguration('current_y'),
            'current_z': LaunchConfiguration('current_z'),
        }],
    )
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='asv_path_tracking_state_publisher',
        output='screen',
        remappings=[
            ('/joint_states', '/research_catamaran/joint_states'),
        ],
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }],
    )
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'asv_research_pool',
            '-name', 'research_catamaran',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', LaunchConfiguration('spawn_z'),
        ],
    )
    generator = Node(
        package=package_name,
        executable='path_generator',
        name='asv_path_generator',
        output='screen',
        parameters=[
            str(config_path),
            {
                'use_sim_time': True,
                'path_type': LaunchConfiguration('path_type'),
                'path_width': ParameterValue(
                    LaunchConfiguration('path_width'), value_type=float),
                'path_height': ParameterValue(
                    LaunchConfiguration('path_height'), value_type=float),
                'speed': ParameterValue(
                    LaunchConfiguration('speed'), value_type=float),
            },
        ],
    )
    controller = Node(
        package=package_name,
        executable='path_tracking_controller',
        name='asv_path_tracking_controller',
        output='screen',
        parameters=[
            str(config_path),
            {
                'use_sim_time': True,
                'log_file': LaunchConfiguration('log_file'),
            },
        ],
    )
    visualization = Node(
        package=package_name,
        executable='path_tracking_visualization',
        name='asv_path_tracking_visualization',
        output='screen',
        parameters=[str(config_path)],
    )
    gazebo_markers = Node(
        package='asv_path_tracking_gazebo',
        executable='gazebo_path_marker_bridge',
        name='asv_path_tracking_gazebo_marker_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('show_gazebo_path')),
        parameters=[str(config_path)],
    )
    dashboard = Node(
        package=package_name,
        executable='path_tracking_dashboard',
        name='asv_path_tracking_dashboard',
        output='screen',
        parameters=[
            str(config_path),
            {
                'use_sim_time': True,
                'host': LaunchConfiguration('dashboard_host'),
                'port': ParameterValue(
                    LaunchConfiguration('dashboard_port'), value_type=int),
                'open_browser': ParameterValue(
                    LaunchConfiguration('open_dashboard'), value_type=bool),
            },
        ],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='asv_path_tracking_rviz',
        output='screen',
        arguments=['-d', str(rviz_path)],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('path_type', default_value='circle'),
        DeclareLaunchArgument('path_width', default_value='1.60'),
        DeclareLaunchArgument('path_height', default_value='1.20'),
        DeclareLaunchArgument('speed', default_value='0.10'),
        DeclareLaunchArgument('current_x', default_value='0.10'),
        DeclareLaunchArgument('current_y', default_value='0.00'),
        DeclareLaunchArgument('current_z', default_value='0.00'),
        DeclareLaunchArgument('spawn_z', default_value='-0.10'),
        DeclareLaunchArgument('log_file', default_value=(
            '/tmp/asv_path_tracking.csv')),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('open_dashboard', default_value='true'),
        DeclareLaunchArgument('show_gazebo_path', default_value='true'),
        DeclareLaunchArgument('software_rendering', default_value='false'),
        DeclareLaunchArgument('dashboard_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('dashboard_port', default_value='8780'),
        SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb'),
        SetEnvironmentVariable(name='QT_X11_NO_MITSHM', value='1'),
        SetEnvironmentVariable(
            name='LIBGL_ALWAYS_SOFTWARE',
            value='1',
            condition=IfCondition(LaunchConfiguration('software_rendering')),
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                str(vehicle_share),
                os.pathsep,
                os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'gz_args': TextSubstitution(
                    text=f'-r "{world_path}"'),
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        gz_bridge,
        state_publisher,
        TimerAction(period=3.0, actions=[spawn]),
        TimerAction(period=4.0, actions=[current_bridge, current]),
        TimerAction(
            period=6.0,
            actions=[
                generator,
                controller,
                visualization,
                gazebo_markers,
                dashboard,
                rviz,
            ],
        ),
    ])
