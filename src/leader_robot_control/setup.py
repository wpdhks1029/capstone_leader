from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'leader_robot_control'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'urdf'
            ),
            glob('urdf/*')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*')
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='sunuk',
    maintainer_email='sunuk@todo.todo',

    description='Leader robot control package',
    license='TODO',

    tests_require=[
        'pytest'
    ],

    entry_points={
        'console_scripts': [

            'md200t_controller = '
            'leader_robot_control.md200t_controller:main',

            'simple_odom = '
            'leader_robot_control.simple_odom:main',

        ],
    },
)
