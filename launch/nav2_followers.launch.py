import os
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    swarm_pkg = get_package_share_directory('swarm_bringup')
    map_file = '/home/darshptl/swarm_fresh_ws/src/swarm_bringup/maps/my_map.yaml'

    robots = [
        {'name': 'robot2', 'spawn_x': 1.0, 'spawn_y': 1.0},
        {'name': 'robot3', 'spawn_x': 1.0, 'spawn_y': -1.2},
        {'name': 'robot4', 'spawn_x': 0.8, 'spawn_y': 0.0},
    ]

    actions = []

    for robot in robots:
        name = robot['name']
        params_file = os.path.join(swarm_pkg, 'config', f'nav2_{name}_params.yaml')
        base_frame = f'{name}/base_footprint'
        odom_frame = f'{name}/odom'

        group = GroupAction([
            PushRosNamespace(name),

            # Static map -> robotX/odom TF
            Node(
                package='swarm_bringup',
                executable='odom_to_tf',
                name='odom_to_tf',
                parameters=[{
                    'robot_name': name,
                    'spawn_x': robot['spawn_x'],
                    'spawn_y': robot['spawn_y'],
                }],
                output='screen'
            ),

            # Static TF base_footprint -> base_link
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_footprint_to_base_link',
                arguments=['0', '0', '0.010', '0', '0', '0',
                           f'{name}/base_footprint', f'{name}/base_link'],
                output='screen'
            ),

            # Static TF base_link -> base_scan
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_base_scan',
                arguments=['-0.064', '0', '0.121', '0', '0', '0',
                           f'{name}/base_link', f'{name}/base_scan'],
                output='screen'
            ),

            # Map server
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[
                    params_file,
                    {'yaml_filename': map_file}
                ],
                output='screen'
            ),

            # Planner server
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                parameters=[
                    params_file,
                    {
                        'robot_base_frame': base_frame,
                        'global_costmap.robot_base_frame': base_frame,
                        'global_costmap.global_costmap.robot_base_frame': base_frame,
                        'global_costmap.global_frame': 'map',
                    }
                ],
                output='screen'
            ),

            # Controller server
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                parameters=[
                    params_file,
                    {
                        'robot_base_frame': base_frame,
                        'local_costmap.robot_base_frame': base_frame,
                        'local_costmap.local_costmap.robot_base_frame': base_frame,
                        'local_costmap.global_frame': odom_frame,
                        'controller_plugins': ['FollowPath'],
                        'FollowPath.plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                        'FollowPath.desired_linear_vel': 0.2,
                        'FollowPath.lookahead_dist': 0.6,
                        'FollowPath.min_lookahead_dist': 0.3,
                        'FollowPath.max_lookahead_dist': 0.9,
                        'FollowPath.rotate_to_heading_angular_vel': 1.8,
                        'FollowPath.transform_tolerance': 0.1,
                        'FollowPath.use_rotate_to_heading': True,
                        'FollowPath.allow_reversing': False,
                        'FollowPath.max_angular_accel': 3.2,
                        'FollowPath.enable_stamped_cmd_vel': False,
                    }
                ],
                remappings=[('cmd_vel', f'/{name}/cmd_vel')],
                output='screen'
            ),

            # Smoother server
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                parameters=[params_file],
                output='screen'
            ),

            # Behavior server
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                parameters=[
                    params_file,
                    {
                        'robot_base_frame': base_frame,
                        'global_frame': odom_frame,
                    }
                ],
                remappings=[('cmd_vel', f'/{name}/cmd_vel')],
                output='screen'
            ),

            # BT navigator
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                parameters=[
                    params_file,
                    {
                        'robot_base_frame': base_frame,
                        'global_frame': 'map',
                        'odom_topic': f'/{name}/odom',
                    }
                ],
                output='screen'
            ),

            # Waypoint follower
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                parameters=[params_file],
                output='screen'
            ),

            # Velocity smoother
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                parameters=[params_file],
                remappings=[
                    ('cmd_vel', f'/{name}/cmd_vel_smoothed'),
                    ('cmd_vel_smoothed', f'/{name}/cmd_vel'),
                ],
                output='screen'
            ),

            # Lifecycle manager
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                parameters=[
                    params_file,
                    {
                        'node_names': [
                            'map_server',
                            'planner_server',
                            'controller_server',
                            'smoother_server',
                            'behavior_server',
                            'bt_navigator',
                            'waypoint_follower',
                            'velocity_smoother',
                        ],
                        'autostart': True,
                        'use_sim_time': True,
                    }
                ],
                output='screen'
            ),
        ])

        actions.append(group)

    return LaunchDescription(actions)
