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
        ],
    },
)
