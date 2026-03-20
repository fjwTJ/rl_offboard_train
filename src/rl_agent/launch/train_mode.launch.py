from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # 基础运行参数
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir',
        default_value=[EnvironmentVariable('HOME'), '/PX4-Autopilot'],
        description='PX4-Autopilot 目录路径',
    )
    xrce_port_arg = DeclareLaunchArgument(
        'xrce_port',
        default_value='8888',
        description='MicroXRCEAgent UDP 端口',
    )
    use_sim_tf_arg = DeclareLaunchArgument(
        'use_sim_tf',
        default_value='true',
        description='camera_tf_publisher_node 使用仿真相机外参',
    )
    cmd_mode_arg = DeclareLaunchArgument(
        'cmd_mode',
        default_value='rl',
        description='cmd_vel_mux 输出模式: pid 或 rl（训练默认 rl）',
    )

    # 可选组件
    run_heuristic_policy_arg = DeclareLaunchArgument(
        'run_heuristic_policy',
        default_value='false',
        description='启动占位 heuristic_policy_node。外部训练器发布 /uav/cmd_vel_rl 时建议关闭',
    )
    run_tracker_arg = DeclareLaunchArgument(
        'run_tracker',
        default_value='false',
        description='启动 PID tracker_node（发布 /uav/cmd_vel_pid）',
    )
    run_episode_manager_arg = DeclareLaunchArgument(
        'run_episode_manager',
        default_value='true',
        description='启动 episode_manager_node（收到 /rl/done 后自动复位）',
    )
    run_px4_sitl_direct_arg = DeclareLaunchArgument(
        'run_px4_sitl_direct',
        default_value='false',
        description='直接由 launch 常驻启动 PX4 SITL；训练复位模式下建议关闭，由 episode_manager 管理',
    )
    lost_done_timeout_sec_arg = DeclareLaunchArgument(
        'lost_done_timeout_sec',
        default_value='3.0',
        description='目标连续丢失多少秒后置 /rl/done=true',
    )
    offboard_auto_start_mission_arg = DeclareLaunchArgument(
        'offboard_auto_start_mission',
        default_value='true',
        description='offboard_control_srv 到达起飞高度后是否自动进入 mission',
    )
    offboard_auto_start_require_target_arg = DeclareLaunchArgument(
        'offboard_auto_start_require_target',
        default_value='false',
        description='offboard_control_srv 自动进入 mission 是否要求目标未丢失',
    )
    offboard_auto_start_delay_sec_arg = DeclareLaunchArgument(
        'offboard_auto_start_delay_sec',
        default_value='0.5',
        description='offboard_control_srv 自动进入 mission 的延时（秒）',
    )

    # PX4 + DDS
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl_default', 'gz_x500_depth'],
        cwd=LaunchConfiguration('px4_dir'),
        condition=IfCondition(LaunchConfiguration('run_px4_sitl_direct')),
        output='screen',
        emulate_tty=True,
    )
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', LaunchConfiguration('xrce_port')],
        output='screen',
        emulate_tty=True,
    )

    # Gazebo 桥接
    bridge_depth = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/realsense/rgbd/depth_image@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen',
    )
    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/realsense/rgbd/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen',
    )
    bridge_rgb = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/realsense/rgbd/image@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen',
    )
    # 感知链路
    depth_convert = Node(package='depth_convert', executable='depth_convert_node', output='screen')
    yolo_node = Node(
        package='yolo_detector',
        executable='yolo_node',
        condition=UnlessCondition(LaunchConfiguration('run_episode_manager')),
        output='screen',
    )
    target_depth_node = Node(
        package='yolo_detector',
        executable='target_depth_node',
        condition=UnlessCondition(LaunchConfiguration('run_episode_manager')),
        output='screen',
    )
    camera_tf_publisher_node = Node(
        package='yolo_detector',
        executable='camera_tf_publisher_node',
        parameters=[{'use_sim_tf': LaunchConfiguration('use_sim_tf')}],
        output='screen',
    )

    # 决策与训练桥接
    tracker_node = Node(
        package='yolo_detector',
        executable='tracker_node',
        condition=IfCondition(LaunchConfiguration('run_tracker')),
        output='screen',
    )
    heuristic_policy_node = Node(
        package='rl_agent',
        executable='heuristic_policy_node',
        condition=IfCondition(LaunchConfiguration('run_heuristic_policy')),
        output='screen',
    )
    cmd_vel_mux_node = Node(
        package='rl_agent',
        executable='cmd_vel_mux_node',
        parameters=[{'mode': LaunchConfiguration('cmd_mode')}],
        output='screen',
    )
    rl_env_bridge_node = Node(
        package='rl_agent',
        executable='rl_env_bridge_node',
        parameters=[{'lost_done_timeout_sec': LaunchConfiguration('lost_done_timeout_sec')}],
        output='screen',
    )
    episode_manager_node = Node(
        package='rl_agent',
        executable='episode_manager_node',
        condition=IfCondition(LaunchConfiguration('run_episode_manager')),
        parameters=[{
            'px4_dir': LaunchConfiguration('px4_dir'),
            'px4_run_cmd': 'make px4_sitl_default gz_x500_depth',
            'px4_startup_wait_sec': 8.0,
            'yolo_run_cmd': 'ros2 run yolo_detector yolo_node',
            'target_depth_run_cmd': 'ros2 run yolo_detector target_depth_node',
            'target_lost_monitor_run_cmd': 'ros2 run yolo_detector target_lost_monitor_node',
            'perception_startup_grace_sec': 1.0,
            'offboard_run_cmd': PythonExpression([
                "'ros2 run px4_ros_com offboard_control_srv --ros-args "
                "-p auto_start_mission:=' + '", LaunchConfiguration('offboard_auto_start_mission'), "' + "
                "' -p auto_start_require_target:=' + '", LaunchConfiguration('offboard_auto_start_require_target'), "' + "
                "' -p auto_start_delay_sec:=' + '", LaunchConfiguration('offboard_auto_start_delay_sec'), "'"
            ]),
        }],
        output='screen',
    )
    target_lost_monitor_node = Node(
        package='yolo_detector',
        executable='target_lost_monitor_node',
        condition=UnlessCondition(LaunchConfiguration('run_episode_manager')),
        output='screen',
    )

    return LaunchDescription([
        px4_dir_arg,
        xrce_port_arg,
        use_sim_tf_arg,
        cmd_mode_arg,
        run_heuristic_policy_arg,
        run_tracker_arg,
        run_episode_manager_arg,
        run_px4_sitl_direct_arg,
        lost_done_timeout_sec_arg,
        offboard_auto_start_mission_arg,
        offboard_auto_start_require_target_arg,
        offboard_auto_start_delay_sec_arg,
        px4_sitl,
        micro_xrce_agent,
        bridge_depth,
        bridge_camera_info,
        bridge_rgb,
        depth_convert,
        yolo_node,
        target_depth_node,
        camera_tf_publisher_node,
        tracker_node,
        target_lost_monitor_node,
        heuristic_policy_node,
        cmd_vel_mux_node,
        rl_env_bridge_node,
        episode_manager_node,
    ])
