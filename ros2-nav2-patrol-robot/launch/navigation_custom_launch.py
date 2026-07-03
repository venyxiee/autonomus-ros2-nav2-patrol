from launch import LaunchDescription
from launch_ros.actions import Node

PARAMS = '/home/faldo-ivan/skripsi/src/my_robot/config/nav2fixed_params.yaml'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                PARAMS,
                {'local_costmap.local_costmap.width': 1.0,
                 'local_costmap.local_costmap.height': 1.0}
            ],
            remappings=[('cmd_vel', 'cmd_vel_nav')]
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[PARAMS]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[PARAMS]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[PARAMS]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[PARAMS]
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[PARAMS]
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[PARAMS],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel')
            ]
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[PARAMS],
            remappings=[
                ('cmd_vel_smoothed', 'cmd_vel_nav'),
                ('cmd_vel', 'cmd_vel')
            ]
        ),
        # Filter mask server
#        Node(
#            package='nav2_map_server',
#            executable='map_server',
#            name='filter_mask_server',
#            output='screen',
#            parameters=[{
#                'use_sim_time': False,
#                'yaml_filename': '/home/faldo-ivan/skripsi/keepout_mask.yaml',
#                'topic_qos_durability': 'transient_local'
#            }],
#            remappings=[('map', '/global_costmap/filter_mask')]
#        ),

        # Costmap filter info server
#        Node(
#            package='nav2_map_server',
#            executable='costmap_filter_info_server',
#            name='costmap_filter_info_server',
#            output='screen',
#            parameters=[{
#                'use_sim_time': False,
#                'filter_info_topic': '/costmap_filter_info',
#                'type': 0,
#                'filter_mask_topic': '/global_costmap/filter_mask',
#                'base': 0.0,
#                'multiplier': 1.0
#            }]
#        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                    'collision_monitor'
 #                   'filter_mask_server',        # ← tambahkan
 #                   'costmap_filter_info_server' # ← tambahkan
                ]
            }]
        ),
    ])
