import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    demo_dir = get_package_share_directory('teb_nav2_demo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(demo_dir, 'maps', 'liosam_map_full.yaml')
    default_params = os.path.join(demo_dir, 'params', 'teb_burger.yaml')
    rviz_config = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_yaw = LaunchConfiguration('start_yaw')
    use_rviz = LaunchConfiguration('rviz')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {
                'yaml_filename': map_file,
                'use_sim_time': use_sim_time,
            },
        ],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Planning-only mode still needs a stable robot frame for the global costmap.
    # Put base_link at the selected start pose; the action goal can override the
    # path start with use_start=true.
    map_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='planning_map_to_base_link',
        output='screen',
        arguments=[
            '--x',
            start_x,
            '--y',
            start_y,
            '--z',
            '0.0',
            '--roll',
            '0.0',
            '--pitch',
            '0.0',
            '--yaw',
            start_yaw,
            '--frame-id',
            'map',
            '--child-frame-id',
            'base_link',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'planner_server'],
            }
        ],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the Nav2 map yaml file.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the Nav2 parameters file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'start_x',
            default_value='0.0',
            description='Planning-only base_link x position in map frame.',
        ),
        DeclareLaunchArgument(
            'start_y',
            default_value='0.0',
            description='Planning-only base_link y position in map frame.',
        ),
        DeclareLaunchArgument(
            'start_yaw',
            default_value='0.0',
            description='Planning-only base_link yaw in map frame.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz with the default Nav2 configuration.',
        ),
        map_to_base_link,
        map_server,
        planner_server,
        lifecycle_manager,
        rviz_node,
    ])
