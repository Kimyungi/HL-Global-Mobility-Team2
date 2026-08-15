from glob import glob
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
        (
            'share/' + package_name + '/models',
            ['models/yolov8n.pt', 'models/README.md'],
        ),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='김재민',
    maintainer_email='kyg100800@gmail.com',
    description='YOLO 신호등·OAK 정지선 거리 판정 → 정지 요구',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stack_traffic_node = stack_traffic.node:main',
            'stack_traffic_ml_preflight = stack_traffic.ml_preflight:main',
        ],
    },
)
