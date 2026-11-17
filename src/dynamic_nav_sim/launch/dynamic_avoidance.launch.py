"""generic demo chassis + physical crossing obstacle + Nav2/TEB, isolated from real hardware."""
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, EmitEvent, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def launch_setup(context):
    pkg = Path(get_package_share_directory('dynamic_nav_sim'))
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    description = ET.tostring(ET.parse(pkg/'urdf/generic_robot.urdf').getroot(), encoding='unicode')
    # Absolute file URIs resolve in both Gazebo and RViz, including symlink installs.
    description = description.replace('package://dynamic_nav_sim/', pkg.as_uri()+'/')
    sim = {'use_sim_time': True}
    scene = LaunchConfiguration('scene').perform(context)
    profile = LaunchConfiguration('profile').perform(context)
    layout = json.loads((pkg/'config'/f'{scene}_scene.json').read_text())
    suffix='_doors' if scene=='long_corridor' else ''
    config = str(pkg/'config'/f'nav2_{profile}{suffix}.yaml')
    window_size = int(LaunchConfiguration('local_map_size').perform(context))
    window_resolution = float(LaunchConfiguration('local_map_resolution').perform(context))
    if not 3 <= window_size <= 10 or not .01 <= window_resolution <= .1:
        raise ValueError('local_map_size must be 3..10 m; local_map_resolution must be 0.01..0.1 m')
    config = RewrittenYaml(source_file=config, param_rewrites={
        'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': str(pkg/'config/navigate_observed.xml'),
        'local_costmap.local_costmap.ros__parameters.width': str(window_size),
        'local_costmap.local_costmap.ros__parameters.height': str(window_size),
        'local_costmap.local_costmap.ros__parameters.resolution': str(window_resolution),
    }, convert_types=True)
    world = str(pkg/'worlds'/f'{scene}.world')
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-topic','robot_description','-entity','demo_cart','-x',str(layout['start'][0]),'-y',str(layout['start'][1]),'-z','.02'], output='screen')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2/'launch/navigation_launch.py')),
        launch_arguments={'use_sim_time':'true','params_file':config,'autostart':'true','use_composition':'False'}.items())
    def after_spawn(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Robot spawn failed"))]
        return [navigation]

    return [
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI',''),
        ExecuteProcess(cmd=['gzserver','--verbose','-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so',world,'--ros-args','--params-file',str(pkg/'config/gazebo.yaml'),'--'],output='screen'),
        ExecuteProcess(cmd=['gzclient'],condition=IfCondition(LaunchConfiguration('gui')),output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[sim,{'robot_description':description}],output='screen'),
        Node(package='dynamic_nav_sim',executable='filter_self_scan.py',parameters=[sim],output='screen'),
        Node(package='dynamic_nav_sim',executable='scan_obstacle_tracker.py',parameters=[sim],condition=UnlessCondition(LaunchConfiguration('publish_ground_truth')),output='screen'),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', parameters=[str(pkg/'config/slam_online.yaml'),sim],output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=after_spawn)),
        spawn,
        Node(package='dynamic_nav_sim',executable='moving_obstacle.py',parameters=[sim,{'publish_ground_truth':LaunchConfiguration('publish_ground_truth'),'scene':scene,'profile':LaunchConfiguration('profile')}],output='screen'),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(pkg/'config/long_corridor.rviz') if scene=='long_corridor' else str(nav2/'rviz/nav2_default_view.rviz')],parameters=[sim],condition=IfCondition(LaunchConfiguration('rviz')),output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene',default_value='long_corridor',choices=['dynamic_crossing','narrow_static','narrow_dynamic','long_corridor']),
        DeclareLaunchArgument('profile',default_value='responsive',choices=['baseline','responsive']),
        DeclareLaunchArgument('gui',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('local_map_size',default_value='5',description='Rolling local costmap side length in metres.'),
        DeclareLaunchArgument('local_map_resolution',default_value='0.02',description='Local costmap metres per cell.'),
        DeclareLaunchArgument('publish_ground_truth',default_value='false',description='Debug only: send obstacle truth to TEB; false uses laser tracking and live costmaps.'),
        OpaqueFunction(function=launch_setup),
    ])
