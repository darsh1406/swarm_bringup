import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    swarm_pkg  = get_package_share_directory('swarm_bringup')

    robots = [
        ('robot1', '-0.5', '-0.5'),
        ('robot2', '-1.0',  '0.5'),
        ('robot3', '-1.0', '-1.7'),
        ('robot4', '-1.2', '-0.5'),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'x_pose': '-99.0',
            'y_pose': '-99.0'
        }.items()
    )

    actions = [gazebo]

    for i, (robot_name, x, y) in enumerate(robots):
        delay = float(8 + i * 3)

        sdf_file = os.path.join(
            swarm_pkg, 'config', f'waffle_{robot_name}.sdf'
        )

        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot_name,
                '-file', sdf_file,
                '-x', x,
                '-y', y,
                '-z', '0.01',
            ],
            output='screen'
        )

        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'ros_gz_bridge_{robot_name}',
            arguments=[
                f'{robot_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'{robot_name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'{robot_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            output='screen'
        )

        odom_tf = Node(
            package='swarm_bringup',
            executable='odom_tf_broadcaster',
            name=f'odom_tf_broadcaster_{robot_name}',
            parameters=[{'robot_name': robot_name}],
            output='screen'
        )

        actions.append(
            TimerAction(
                period=delay,
                actions=[spawn, bridge, odom_tf]
            )
        )

    return LaunchDescription(actions)