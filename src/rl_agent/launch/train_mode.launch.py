from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
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
    run_rl_policy_arg = DeclareLaunchArgument(
        'run_rl_policy',
        default_value='false',
        description='启动占位 rl_policy_node。外部训练器发布 /uav/cmd_vel_rl 时建议关闭',
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
    run_world_service_bridge_arg = DeclareLaunchArgument(
        'run_world_service_bridge',
        default_value='true',
        description='桥接 Gazebo world reset 相关服务到 ROS2（episode_manager 需要）',
    )
    lost_done_timeout_sec_arg = DeclareLaunchArgument(
        'lost_done_timeout_sec',
        default_value='3.0',
        description='目标连续丢失多少秒后置 /rl/done=true',
    )
    episode_enable_world_reset_arg = DeclareLaunchArgument(
        'episode_enable_world_reset',
        default_value='true',
        description='episode_manager 是否调用 Gazebo world reset',
    )
    episode_world_reset_mode_arg = DeclareLaunchArgument(
        'episode_world_reset_mode',
        default_value='model_only',
        description='world reset 模式: model_only | all | time_only',
    )
    episode_enable_set_pose_arg = DeclareLaunchArgument(
        'episode_enable_set_pose',
        default_value='true',
        description='episode_manager 是否调用 Gazebo set_pose',
    )

    # PX4 + DDS
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl_default', 'gz_x500_depth'],
        cwd=LaunchConfiguration('px4_dir'),
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
    bridge_world_control = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/default/control@ros_gz_interfaces/srv/ControlWorld'],
        condition=IfCondition(LaunchConfiguration('run_world_service_bridge')),
        output='screen',
    )
    bridge_world_set_pose = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        condition=IfCondition(LaunchConfiguration('run_world_service_bridge')),
        output='screen',
    )

    # 感知链路
    depth_convert = Node(package='depth_convert', executable='depth_convert_node', output='screen')
    yolo_node = Node(package='yolo_detector', executable='yolo_node', output='screen')
    target_depth_node = Node(package='yolo_detector', executable='target_depth_node', output='screen')
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
    rl_policy_node = Node(
        package='rl_agent',
        executable='rl_policy_node',
        condition=IfCondition(LaunchConfiguration('run_rl_policy')),
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
            'enable_world_reset': LaunchConfiguration('episode_enable_world_reset'),
            'world_reset_mode': LaunchConfiguration('episode_world_reset_mode'),
            'enable_set_pose': LaunchConfiguration('episode_enable_set_pose'),
            'model_name': 'x500_depth_0',
            'init_x': 0.0,
            'init_y': 0.0,
            'init_z': 0.3,
        }],
        output='screen',
    )
    target_lost_monitor_node = Node(
        package='yolo_detector',
        executable='target_lost_monitor_node',
        output='screen',
    )

    return LaunchDescription([
        px4_dir_arg,
        xrce_port_arg,
        use_sim_tf_arg,
        cmd_mode_arg,
        run_rl_policy_arg,
        run_tracker_arg,
        run_episode_manager_arg,
        run_world_service_bridge_arg,
        lost_done_timeout_sec_arg,
        episode_enable_world_reset_arg,
        episode_world_reset_mode_arg,
        episode_enable_set_pose_arg,
        px4_sitl,
        micro_xrce_agent,
        bridge_depth,
        bridge_camera_info,
        bridge_rgb,
        bridge_world_control,
        bridge_world_set_pose,
        depth_convert,
        yolo_node,
        target_depth_node,
        camera_tf_publisher_node,
        tracker_node,
        target_lost_monitor_node,
        rl_policy_node,
        cmd_vel_mux_node,
        rl_env_bridge_node,
        episode_manager_node,
    ])
