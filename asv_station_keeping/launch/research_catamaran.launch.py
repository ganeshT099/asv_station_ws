import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)


def generate_launch_description():
    package_name = 'asv_station_keeping'
    pkg_share = get_package_share_directory(package_name)

    world_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        LaunchConfiguration('world_file'),
    ])
    gazebo_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_file',
            default_value='research_catamaran.urdf',
            description='URDF file under the package urdf directory.',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='research_pool_dynamics.sdf',
            description=(
                'World file under the package worlds directory. The '
                'dynamics world is the default for motion and algorithms; '
                'use research_pool.sdf only for visual display tests.'),
        ),
        DeclareLaunchArgument(
            'spawn_z',
            default_value='-0.10',
            description=(
                'Initial vehicle height. This starts the simplified hulls '
                'near their buoyant equilibrium in the dynamics world.'),
        ),
        DeclareLaunchArgument(
            'ghost', default_value='false',
            description='Spawn a passive, uncontrolled twin for comparison.'),
        DeclareLaunchArgument('ghost_x', default_value='1.5'),
        DeclareLaunchArgument('ghost_y', default_value='-2.5'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true runs the Gazebo server only (no GUI window).'),
        DeclareLaunchArgument(
            'software_rendering',
            default_value='false',
            description='Use Mesa software rendering for Gazebo if needed.',
        ),

        # Prefer the X11 Qt backend on Ubuntu when a Wayland session is active;
        # this avoids the blank-viewer path seen on some Fortress/Ogre2 setups.
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
                pkg_share,
                os.pathsep,
                EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'gz_args': [
                    PythonExpression([
                        "'-r -s \"' if '",
                        LaunchConfiguration('headless'),
                        "' == 'true' else '-r \"'",
                    ]),
                    world_file,
                    TextSubstitution(text='"'),
                ],
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        # Convert Fortress transport topics into ordinary ROS 2 topics. The
        # thrust coefficients are intentionally marked as calibration values
        # in the URDF; replace them with measured data for experiments.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='research_catamaran_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/research_catamaran/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                '/research_catamaran/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                '/research_catamaran/gps@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
                '/research_catamaran/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/model/research_catamaran/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                (
                    '/model/research_catamaran/joint/'
                    'port_front_propeller_joint/cmd_thrust@'
                    'std_msgs/msg/Float64]ignition.msgs.Double'
                ),
                (
                    '/model/research_catamaran/joint/'
                    'starboard_front_propeller_joint/cmd_thrust@'
                    'std_msgs/msg/Float64]ignition.msgs.Double'
                ),
                (
                    '/model/research_catamaran/joint/'
                    'port_rear_propeller_joint/cmd_thrust@'
                    'std_msgs/msg/Float64]ignition.msgs.Double'
                ),
                (
                    '/model/research_catamaran/joint/'
                    'starboard_rear_propeller_joint/cmd_thrust@'
                    'std_msgs/msg/Float64]ignition.msgs.Double'
                ),
            ],
            output='screen',
        ),

        Node(
            package='asv_station_gz_bridge',
            executable='sea_state_gz_bridge',
            name='sea_state_gz_bridge',
            output='screen',
            parameters=[{
                'world': 'asv_research_pool',
                'link': 'base_footprint',
                'ghost': ParameterValue(
                    LaunchConfiguration('ghost'), value_type=bool),
            }],
        ),

        OpaqueFunction(function=_configure_model, args=[pkg_share]),
    ])


def _configure_model(context, pkg_share):
    """Load the selected model at launch time and spawn it after Gazebo."""
    model_file = LaunchConfiguration('model_file').perform(context)
    urdf_file = os.path.join(pkg_share, 'urdf', model_file)
    with open(urdf_file, 'r', encoding='utf-8') as file:
        robot_description = file.read()

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='research_catamaran_state_publisher',
        output='screen',
        # Fortress publishes the continuous propeller joint states on the
        # namespaced topic below.  robot_state_publisher listens to
        # /joint_states by default; without this remap the fixed links appear
        # in RViz but all four propeller links report missing transforms.
        remappings=[
            (
                '/joint_states',
                '/research_catamaran/joint_states',
            ),
        ],
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'asv_research_pool',
            '-name', 'research_catamaran',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', LaunchConfiguration('spawn_z'),
        ],
        output='screen',
    )
    actions = [state_publisher, TimerAction(period=3.0, actions=[spawn])]
    if LaunchConfiguration('ghost').perform(context) == 'true':
        actions += _ghost_actions(context, robot_description)
    return actions


def _ghost_actions(context, robot_description):
    """Spawn a passive copy (no controller) in a contrasting colour."""
    import re
    text = robot_description.replace('research_catamaran', 'ghost_catamaran')
    text = text.replace('0.08 0.22 0.42', '0.85 0.30 0.08')
    # The twin needs no sensors; dropping them saves simulator time.
    text = re.sub(
        r'<gazebo reference="sensor_head">.*?</gazebo>', '', text,
        flags=re.S)
    path = '/tmp/ghost_catamaran.urdf'
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(text)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'asv_research_pool', '-name', 'ghost_catamaran',
            '-file', path,
            '-x', LaunchConfiguration('ghost_x').perform(context),
            '-y', LaunchConfiguration('ghost_y').perform(context),
            '-z', LaunchConfiguration('spawn_z').perform(context),
        ],
        output='screen',
    )
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ghost_bridge',
        arguments=[
            '/model/ghost_catamaran/odometry@nav_msgs/msg/Odometry'
            '[ignition.msgs.Odometry'],
        output='screen',
    )
    return [odom_bridge, TimerAction(period=3.5, actions=[spawn])]
