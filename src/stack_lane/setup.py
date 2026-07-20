from setuptools import find_packages, setup

package_name = 'stack_lane'

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
    maintainer='이현준',
    maintainer_email='kyg100800@gmail.com',
    description='차선 검출(YOLO) → 차선 ref (camera 100ms)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stack_lane_node = stack_lane.node:main',
        ],
    },
)
