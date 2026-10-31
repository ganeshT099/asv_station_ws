from glob import glob
from setuptools import find_packages, setup


package_name = 'asv_assem_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/meshes', glob('meshes/*.stl')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gt0908',
    maintainer_email='gt0908@srmist.edu.in',
    description='ASV Dynamic Station Keeping simulation',
    license='Apache-2.0',
)
