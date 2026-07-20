from setuptools import find_packages, setup

package_name = 'stack_estop'

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
    maintainer='박찬미',
    maintainer_email='kyg100800@gmail.com',
    description='돌발 장애물 감지 → 긴급 정지 요구',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stack_estop_node = stack_estop.node:main',
        ],
    },
)
