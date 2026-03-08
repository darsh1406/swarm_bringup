import os
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    swarm_pkg = get_package_share_directory('swarm_bringup')

    # Follower robot configs: (name, map_x, map_y)
    followers = [
        ('robot2', 1.0,  1.0),
        ('robot3', 1.0, -1.2),
        ('robot4', 0.8,  0.0),
    ]

    all_actions = []

    for robot_name, map_x, map_y in followers:
        params_file = os.path.join(
            swarm_pkg, 'config', f'nav2_{robot_name}_params.yaml'
        )

        group = GroupAction([
            PushRosNamespace(robot_name),

            # Static map -> robotX/odom TF
            Node(
                package='swarm_bringup',
                executable='odom_to_tf',
                name=f'odom_to_tf_{robot_name}',
                parameters=[{
                    'robot_name': robot_name,
                    'spawn_x': map_x,
                    'spawn_y': map_y,
                }],
                output='screen'
            ),

            # Obstacle broadcaster (higher-priority robots as obstacles)
            Node(
                package='swarm_bringup',
                executable='obstacle_broadcaster',
                name=f'obstacle_broadcaster_{robot_name}',
                parameters=[{'robot_name': robot_name}],
                output='screen'
            ),

            # Map server
            Node(
                package='nav2_map_server',
                executable='map_server',
                name=f'{robot_name}/map_server',
                parameters=[params_file],
                output='screen'
            ),

            # Planner server
            Node(
                package='nav2_planner',
                executable='planner_server',
                name=f'{robot_name}/planner_server',
                parameters=[params_file],
                output='screen'
            ),

            # Controller server
            Node(
                package='nav2_controller',
                executable='controller_server',
                name=f'{robot_name}/controller_server',
                parameters=[params_file],
                remappings=[('cmd_vel', f'/{robot_name}/cmd_vel')],
                output='screen'
            ),

            # Smoother server
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name=f'{robot_name}/smoother_server',
                parameters=[params_file],
                output='screen'
            ),

            # Behavior server
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name=f'{robot_name}/behavior_server',
                parameters=[params_file],
                remappings=[('cmd_vel', f'/{robot_name}/cmd_vel')],
                output='screen'
            ),

            # BT navigator
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name=f'{robot_name}/bt_navigator',
                parameters=[params_file],
                output='screen'
            ),

            # Waypoint follower
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name=f'{robot_name}/waypoint_follower',
                parameters=[params_file],
                output='screen'
            ),

            # Velocity smoother
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name=f'{robot_name}/velocity_smoother',
                parameters=[params_file],
                remappings=[
                    ('cmd_vel', f'/{robot_name}/cmd_vel_smoothed'),
                    ('cmd_vel_smoothed', f'/{robot_name}/cmd_vel'),
                ],
                output='screen'
            ),

            # Lifecycle manager
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name=f'{robot_name}/lifecycle_manager_navigation',
                parameters=[params_file],
                output='screen'
            ),
        ])

        all_actions.append(group)

    return LaunchDescription(all_actions)
