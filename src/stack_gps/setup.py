from setuptools import find_packages, setup

package_name = 'stack_gps'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='김윤기 (팀장)',
    maintainer_email='kyg100800@gmail.com',
    description='GPS·IMU 융합, RTK, waypoint ref',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stack_gps_node = stack_gps.node:main',
        ],
    },
)
