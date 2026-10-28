from setuptools import find_packages, setup

package_name = 'iahrs_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunuk',
    maintainer_email='sunuk@todo.todo',
    description='ROS2 driver for IntelliThings iAHRS RB-SDA-v1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'iahrs_node = iahrs_ros2.iahrs_node:main',
        ],
    },
)
