import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    demo_dir = get_package_share_directory('teb_nav2_demo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    validation_params = os.path.join(demo_dir, 'params', 'teb_burger_nav2_world.yaml')
    validation_map = os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml')
    validation_world = os.path.join(nav2_bringup_dir, 'worlds', 'world_only.model')

    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Whether to run Gazebo without gzclient.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(demo_dir, 'launch', 'teb_tb3_sim.launch.py')
            ),
            launch_arguments={
                'params_file': validation_params,
                'map': validation_map,
                'world': validation_world,
                'x_pose': '-2.0',
                'y_pose': '-0.5',
                'yaw': '0.0',
                'headless': headless,
                'use_sim_time': use_sim_time,
                'robot_name': 'mini_mec_nav2_validation',
                'map_offset_x': '0.0',
                'map_offset_y': '0.0',
                'map_offset_yaw': '0.0',
            }.items(),
        ),
    ])
