from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'swarm_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.sdf')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 Jazzy 4-robot swarm navigation with Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_tf = swarm_bringup.odom_to_tf:main',
            'odom_tf_broadcaster = swarm_bringup.odom_tf_broadcaster:main',
            'obstacle_broadcaster = swarm_bringup.obstacle_broadcaster:main',
            'follower_controller = swarm_bringup.follower_controller:main',
            'formation_controller = swarm_bringup.formation_controller:main',
        ],
    },
)
