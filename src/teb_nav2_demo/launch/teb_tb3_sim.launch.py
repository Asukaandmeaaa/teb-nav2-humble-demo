import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def load_initial_pose(pose_file):
    pose = {
        'initial_pose_x': '0.0',
        'initial_pose_y': '0.0',
        'initial_pose_a': '0.0',
    }
    with open(pose_file, 'r', encoding='utf-8') as file:
        for raw_line in file:
            line = raw_line.strip()
            if not line or line.startswith('#') or ':' not in line:
                continue
            key, value = line.split(':', 1)
            key = key.strip()
            if key in pose:
                pose[key] = value.strip()
    return pose


def generate_launch_description():
    demo_dir = get_package_share_directory('teb_nav2_demo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    workspace_dir = os.path.abspath(
        os.path.join(demo_dir, '..', '..', '..', '..')
    )
    debug_dir = os.path.join(workspace_dir, 'debug', 'teb_tb3_sim_latest')
    ros_log_dir = os.path.join(debug_dir, 'ros_logs')
    path_debug_dir = os.path.join(debug_dir, 'path_debug')

    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    robot_name = LaunchConfiguration('robot_name')
    map_offset_x = LaunchConfiguration('map_offset_x')
    map_offset_y = LaunchConfiguration('map_offset_y')
    map_offset_yaw = LaunchConfiguration('map_offset_yaw')
    rviz_config = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    default_params = os.path.join(demo_dir, 'params', 'teb_burger.yaml')
    default_map = os.path.join(demo_dir, 'maps', 'WHEELTEC.yaml')
    default_world = os.path.join(demo_dir, 'worlds', 'WHEELTEC.world')
    default_robot_urdf = os.path.join(demo_dir, 'urdf', 'mini_mec_gazebo.urdf')
    default_pose = load_initial_pose(os.path.join(demo_dir, 'maps', 'WHEELTEC_end.yaml'))
    gazebo_model_root = os.path.dirname(demo_dir)

    with open(default_robot_urdf, 'r', encoding='utf-8') as file:
        robot_description = file.read()

    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            gazebo_model_root,
            ':',
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
        ],
    )

    ros_log_dir_env = SetEnvironmentVariable(
        name='ROS_LOG_DIR',
        value=ros_log_dir,
    )

    prepare_debug_dir = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            (
                f'rm -rf "{debug_dir}" && mkdir -p "{ros_log_dir}" && '
                f'cat > "{os.path.join(debug_dir, "run_context.txt")}" <<EOF\n'
                f'workspace_dir: {workspace_dir}\n'
                f'demo_dir: {demo_dir}\n'
                f'default_params_file: {default_params}\n'
                f'default_map_file: {default_map}\n'
                f'default_world_file: {default_world}\n'
                f'default_robot_urdf: {default_robot_urdf}\n'
                f'default_initial_pose_x: {default_pose["initial_pose_x"]}\n'
                f'default_initial_pose_y: {default_pose["initial_pose_y"]}\n'
                f'default_initial_pose_a: {default_pose["initial_pose_a"]}\n'
                'default_map_offset_x: 0.0\n'
                'default_map_offset_y: -0.15\n'
                'default_map_offset_yaw: 0.0\n'
                'EOF'
            ),
        ],
        output='screen',
    )

    collect_debug_snapshot = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            (
                f'sleep 12; '
                f'source /opt/ros/humble/setup.bash; '
                f'source "{os.path.join(workspace_dir, "install", "setup.bash")}"; '
                f'ros2 node list > "{os.path.join(debug_dir, "node_list.txt")}" 2>&1 || true; '
                f'ros2 topic list > "{os.path.join(debug_dir, "topic_list.txt")}" 2>&1 || true; '
                f'ros2 topic info /cmd_vel > "{os.path.join(debug_dir, "cmd_vel_topic_info.txt")}" 2>&1 || true; '
                f'ros2 topic info /cmd_vel_nav > "{os.path.join(debug_dir, "cmd_vel_nav_topic_info.txt")}" 2>&1 || true; '
                f'ros2 topic info /odom > "{os.path.join(debug_dir, "odom_topic_info.txt")}" 2>&1 || true; '
                f'ros2 topic info /plan > "{os.path.join(debug_dir, "plan_topic_info.txt")}" 2>&1 || true; '
                f'ros2 topic info /local_plan > "{os.path.join(debug_dir, "local_plan_topic_info.txt")}" 2>&1 || true; '
                f'ros2 topic info /teb_feedback > "{os.path.join(debug_dir, "teb_feedback_topic_info.txt")}" 2>&1 || true; '
                f'ros2 topic info /teb_debug/actual_path > "{os.path.join(debug_dir, "actual_path_topic_info.txt")}" 2>&1 || true; '
                f'timeout 5s ros2 topic echo /odom --once > "{os.path.join(debug_dir, "odom_sample.txt")}" 2>&1 || true; '
                f'timeout 5s ros2 topic echo /plan --once > "{os.path.join(debug_dir, "plan_sample.txt")}" 2>&1 || true; '
                f'timeout 5s ros2 topic echo /local_plan --once > "{os.path.join(debug_dir, "local_plan_sample.txt")}" 2>&1 || true; '
                f'timeout 5s ros2 topic echo /scan --once > "{os.path.join(debug_dir, "scan_sample.txt")}" 2>&1 || true; '
                f'timeout 5s ros2 topic echo /tf --once > "{os.path.join(debug_dir, "tf_sample.txt")}" 2>&1 || true; '
                f'ros2 run tf2_tools view_frames > "{os.path.join(debug_dir, "view_frames_stdout.txt")}" 2>&1 || true; '
                f'if [ -f frames.pdf ]; then mv -f frames.pdf "{os.path.join(debug_dir, "frames.pdf")}"; fi; '
                f'if [ -f frames.yaml ]; then mv -f frames.yaml "{os.path.join(debug_dir, "frames.yaml")}"; fi; '
                f'ros2 run tf2_ros tf2_echo map base_link > "{os.path.join(debug_dir, "tf_map_to_base_link.txt")}" 2>&1 & '
                f'pid1=$!; '
                f'sleep 3; kill $pid1 >/dev/null 2>&1 || true; wait $pid1 >/dev/null 2>&1 || true'
            ),
        ],
        output='screen',
    )

    collect_launch_log = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            (
                f'sleep 3; '
                f'latest_dir=$(ls -td /home/lslubuntu/.ros/log/* 2>/dev/null | head -n 1); '
                f'if [ -n "$latest_dir" ] && [ -f "$latest_dir/launch.log" ]; then '
                f'cp "$latest_dir/launch.log" "{os.path.join(debug_dir, "launch.log")}"; '
                f'printf "%s\\n" "$latest_dir" > "{os.path.join(debug_dir, "ros_log_source.txt")}"; '
                'fi'
            ),
        ],
        output='screen',
    )

    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
            world_file,
        ],
        output='both',
    )

    gazebo_client = ExecuteProcess(
        condition=UnlessCondition(headless),
        cmd=['gzclient'],
        output='both',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }
        ],
    )

    path_recorder = Node(
        package='teb_nav2_demo',
        executable='path_recorder.py',
        name='path_recorder',
        output='both',
        parameters=[
            {
                'output_dir': path_debug_dir,
                'plan_topic': '/plan',
                'local_plan_topic': '/local_plan',
                'odom_topic': '/odom',
                'cmd_vel_topic': '/cmd_vel',
                'cmd_vel_nav_topic': '/cmd_vel_nav',
                'actual_path_topic': '/teb_debug/actual_path',
                'min_record_distance': 0.02,
                'map_offset_x': map_offset_x,
                'map_offset_y': map_offset_y,
                'map_offset_yaw': map_offset_yaw,
            }
        ],
    )

    # For the planar_move validation setup, keep map and odom aligned so Nav2,
    # Gazebo motion, and the static map share one consistent frame.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        output='both',
        arguments=[map_offset_x, map_offset_y, '0', '0', '0', map_offset_yaw, 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='both',
        arguments=[
            '-file',
            default_robot_urdf,
            '-entity',
            robot_name,
            '-package_to_model',
            '-x',
            LaunchConfiguration('x_pose'),
            '-y',
            LaunchConfiguration('y_pose'),
            '-z',
            LaunchConfiguration('z_pose'),
            '-Y',
            LaunchConfiguration('yaw'),
        ],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'False',
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
            'namespace': '',
            'use_namespace': 'False',
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='both',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the Nav2 parameters file.',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the map yaml file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Full path to the Gazebo world file.',
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value=default_pose['initial_pose_x'],
            description='Initial robot x position in the map/world frame.',
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value=default_pose['initial_pose_y'],
            description='Initial robot y position in the map/world frame.',
        ),
        DeclareLaunchArgument(
            'z_pose',
            default_value='0.01',
            description='Initial robot z position in the map/world frame.',
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value=default_pose['initial_pose_a'],
            description='Initial robot yaw in the map/world frame.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Whether to run Gazebo without gzclient.',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='mini_mec',
            description='Name of the robot entity in Gazebo.',
        ),
        DeclareLaunchArgument(
            'map_offset_x',
            default_value='0.0',
            description='Static x offset from map to odom.',
        ),
        DeclareLaunchArgument(
            'map_offset_y',
            default_value='-0.15',
            description='Static y offset from map to odom.',
        ),
        DeclareLaunchArgument(
            'map_offset_yaw',
            default_value='0.0',
            description='Static yaw offset from map to odom in radians.',
        ),
        ros_log_dir_env,
        prepare_debug_dir,
        gazebo_model_path,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        path_recorder,
        map_to_odom,
        spawn_robot,
        nav2_bringup,
        collect_launch_log,
        collect_debug_snapshot,
        rviz_node,
        rviz_exit_handler,
    ])
