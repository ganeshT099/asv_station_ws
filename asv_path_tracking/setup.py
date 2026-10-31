from glob import glob

from setuptools import find_packages, setup


package_name = 'asv_path_tracking'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/dashboard',
         glob('dashboard/*.html') + glob('dashboard/*.js')
         + glob('dashboard/*.css')),
        ('share/' + package_name + '/diagnostics', glob('diagnostics/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gt0908',
    maintainer_email='gt0908@srmist.edu.in',
    description='Standalone geometric ASV path-tracking demonstration.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'path_generator = asv_path_tracking.path_generator:main',
            'path_tracking_controller = '
            'asv_path_tracking.path_tracking_controller:main',
            'path_tracking_visualization = '
            'asv_path_tracking.path_tracking_visualization:main',
            'path_tracking_dashboard = '
            'asv_path_tracking.path_tracking_dashboard:main',
        ],
    },
)
