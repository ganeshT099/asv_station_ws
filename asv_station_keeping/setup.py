from setuptools import find_packages, setup
from glob import glob

package_name = 'asv_station_keeping'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/urdf',
            glob('urdf/*.urdf')
        ),
        (
            'share/' + package_name + '/worlds',
            glob('worlds/*.sdf')
        ),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml')
        ),
        (
            'share/' + package_name + '/rviz',
            glob('rviz/*.rviz')
        ),
        (
            'share/' + package_name + '/dashboard',
            glob('dashboard/*.html')
            + glob('dashboard/*.js')
            + glob('dashboard/*.css')
        ),
        (
            'share/' + package_name + '/meshes',
            glob('meshes/*.stl')
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
        (
            'share/' + package_name + '/diagnostics/variants',
            glob('diagnostics/variants/*.urdf')
            + glob('diagnostics/variants/*.txt')
        ),
        (
            'share/' + package_name + '/diagnostics',
            glob('diagnostics/*.md')
        ),
    ],
    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='gt0908',
    maintainer_email='gt0908@srmist.edu.in',

    description=(
        'ROS 2 Humble / Gazebo Fortress two-hull four-thruster ASV '
        'research simulation.'
    ),

    license='Apache-2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'research_station_keeping = '
            'asv_station_keeping.station_keeping_pid:main',
            'research_current_disturbance = '
            'asv_station_keeping.current_disturbance:main',
            'research_mission_runner = '
            'asv_station_keeping.mission_runner:main',
            'research_map_quality = '
            'asv_station_keeping.map_quality:main',
            'research_scenario_runner = '
            'asv_station_keeping.scenario_runner:main',
            'research_target_marker = '
            'asv_station_keeping.target_marker:main',
            'research_state_estimator = '
            'asv_station_keeping.state_estimator:main',
            'research_twist_to_thruster = '
            'asv_station_keeping.twist_to_thruster:main',
            'research_maneuver_demo = '
            'asv_station_keeping.maneuver_demo:main',
            'verify_motion_stability = '
            'asv_station_keeping.motion_validation:main',
            'evaluate_research_run = '
            'asv_station_keeping.evaluate_run:main',
            'research_station_keeping_visualization = '
            'asv_station_keeping.station_keeping_visualization:main',
            'research_station_keeping_dashboard = '
            'asv_station_keeping.station_keeping_dashboard:main',
            'research_station_keeping_demo = '
            'asv_station_keeping.station_keeping_demo:main',
            'research_study = '
            'asv_station_keeping.research_study:main',
        ],
    },
)
