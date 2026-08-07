import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'stack_avoid'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='이기돈',
    maintainer_email='kyg100800@gmail.com',
    description='장애물 인지, 회피 가능 판정 재료(TTC·측방), 회피 경로',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stack_avoid_node = stack_avoid.node:main',
            'fake_scan = stack_avoid.fake_scan:main',        # 테스트: 합성 스캔
            'avoid_viz = stack_avoid.avoid_viz:main',        # 테스트: 회피 출력 RViz 마커
            'angle_labels = stack_avoid.angle_labels:main',  # 테스트: 각도 눈금(방향 확인)
            'avoid_drive_sim = stack_avoid.avoid_drive_sim:main',  # 테스트: 폐루프 회피주행 sim
            'avoid_to_ref = stack_avoid.avoid_to_ref:main',        # 테스트: 회피점→dSPACE(실차 조향)
            'step_injector = stack_avoid.step_injector:main',      # 실측 ①②: 측방 스텝 주입
            'mark = stack_avoid.mark:main',                        # 실측: 구간 라벨(/test/event)
        ],
    },
)
