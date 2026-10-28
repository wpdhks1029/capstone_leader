from setuptools import setup

setup(
    name='convoy_robot', version='0.1.0', packages=['convoy_robot'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/convoy_robot']),
                ('share/convoy_robot', ['package.xml']),
                ('share/convoy_robot/config', ['config/real_leader_bridge.yaml'])],
    install_requires=['setuptools'],
    description='Leader-only convoy interface snapshot; follower nodes intentionally excluded',
    entry_points={'console_scripts': [
        'real_leader_bridge_node.py = convoy_robot.real_leader_bridge_node:main']},
)
