import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    package_name = 'asv_assem_sim'
    package_share = get_package_share_directory(package_name)
    world_file = os.path.join(package_share, 'worlds', 'asv_test.sdf')
    urdf_file = os.path.join(package_share, 'urdf', 'asv_assem_sim.urdf')
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py',
    )

    with open(urdf_file, 'r', encoding='utf-8') as model_file:
        robot_description = model_file.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'software_rendering',
            default_value='false',
            description=(
                'Use Mesa software rendering if Gazebo opens with a black '
                '3D viewport.'),
        ),
        # Gazebo Fortress GUI can open successfully but render a black scene
        # under GNOME Wayland. XCB uses the XWayland path recommended by the
        # Gazebo warning shown on affected Ubuntu systems.
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
                package_share,
                os.pathsep,
                EnvironmentVariable(
                    'GZ_SIM_RESOURCE_PATH', default_value=''),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={
                # Ogre 1 avoids the black viewport produced by Ogre 2 on
                # several Ubuntu Wayland / hybrid-GPU combinations.
                'gz_args': f'--render-engine ogre -r "{world_file}"',
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),
        # Make Gazebo time and the model sensors available as normal ROS 2
        # topics. Fortress uses the ignition.msgs type names.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='asv_gazebo_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/asv/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                '/asv/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                '/asv/gps@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
                '/asv/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/model/asv/joint/left_front_steering_joint/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
                '/model/asv/joint/left_rear_steering_joint/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
                '/model/asv/joint/right_front_steering_joint/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
                '/model/asv/joint/right_rear_steering_joint/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
                '/model/asv/joint/left_front_propeller_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double',
                '/model/asv/joint/left_rear_propeller_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double',
                '/model/asv/joint/right_front_propeller_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double',
                '/model/asv/joint/right_rear_propeller_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double',
            ],
            output='screen',
        ),
        TimerAction(
            period=2.0,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-world', 'asv_test',
                    '-name', 'asv',
                    '-topic', 'robot_description',
                    '-x', '0', '-y', '0', '-z', '0.35',
                ],
                output='screen',
            )],
        ),
    ])
