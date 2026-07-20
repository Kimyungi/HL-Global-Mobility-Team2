from setuptools import find_packages, setup

package_name = 'stack_traffic'

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
    maintainer='김재민',
    maintainer_email='kyg100800@gmail.com',
    description='신호등·정지선 인식 → 정지 요구',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stack_traffic_node = stack_traffic.node:main',
        ],
    },
)
