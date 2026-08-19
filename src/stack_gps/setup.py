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
        ('share/' + package_name + '/launch', ['launch/walk_test.launch.py']),
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
            # 현장에서 "지금 이 자리"를 정지 지점·회피 구간으로 찍는 도구.
            # 실차 launch 가 도는 중에 새 터미널로 실행한다 (구독만 — 포트 안 건드림).
            'mark_zone = stack_gps.mark_zone:main',
        ],
    },
)
