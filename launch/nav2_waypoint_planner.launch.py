import os
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    swarm_pkg = get_package_share_directory('swarm_bringup')
    params_file = os.path.join(swarm_pkg, 'config', 'nav2_waypoint_planner_params.yaml')
    map_file = '/home/darshptl/swarm_fresh_ws/src/swarm_bringup/maps/my_map.yaml'

    return LaunchDescription([
        # Static TF map -> base_link for waypoint planner costmap
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='waypoint_map_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        GroupAction([
            PushRosNamespace('waypoint_planner'),

            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[params_file, {'yaml_filename': map_file}],
                output='screen'
            ),

            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                parameters=[params_file, {
                    'global_costmap.global_frame': 'map',
                    'global_costmap.robot_base_frame': 'base_link',
                    'use_sim_time': True,
                }],
                output='screen'
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_waypoint',
                parameters=[{
                    'node_names': ['map_server', 'planner_server'],
                    'autostart': True,
                    'use_sim_time': True,
                }],
                output='screen'
            ),
        ])
    ])
