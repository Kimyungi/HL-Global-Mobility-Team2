from setuptools import find_packages, setup

package_name = 'stack_parking'

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
    maintainer='손상민',
    maintainer_email='kyg100800@gmail.com',
    description='주차공간 인식·로컬맵·주차 경로 + MPC/Vehicle MGM(dSPACE)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stack_parking_node = stack_parking.node:main',
        ],
    },
)
