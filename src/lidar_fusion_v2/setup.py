from glob import glob
from setuptools import find_packages, setup

package_name = 'lidar_fusion_v2'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/tools', glob('tools/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='FMA',
    maintainer_email='blueheart0815@gmail.com',
    description='Independent four-LiDAR unifier',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'fusion_node = lidar_fusion_v2.fusion_node:main',
        'wall_calibrator = lidar_fusion_v2.wall_calibrator:main',
    ]},
)
