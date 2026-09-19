from setuptools import find_packages, setup

setup(
    name='stack_exit_decision', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/stack_exit_decision']),
                ('share/stack_exit_decision', ['package.xml']),
                ('share/stack_exit_decision/models', ['models/exit_decision_yolo26n.pt', 'models/README.md'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='손상민', maintainer_email='shonmaker@users.noreply.github.com',
    description='YOLO exit-signal observations for MGM Last_mission_state', license='MIT',
    entry_points={'console_scripts': ['exit_detector = stack_exit_decision.node:main']},
)
