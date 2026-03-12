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
        default_value='pid',
        description='cmd_vel_mux 输出模式: pid 或 rl',
    )

    # 可选组件
    run_tracker_arg = DeclareLaunchArgument(
        'run_tracker',
        default_value='true',
        description='启动 PID tracker_node（发布 /uav/cmd_vel_pid）',
    )
    run_rl_policy_arg = DeclareLaunchArgument(
        'run_rl_policy',
        default_value='true',
        description='启动占位 rl_policy_node（发布 /uav/cmd_vel_rl）',
    )
    run_world_service_bridge_arg = DeclareLaunchArgument(
        'run_world_service_bridge',
        default_value='true',
        description='桥接 Gazebo world reset 相关服务到 ROS2',
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
    target_lost_monitor_node = Node(
        package='yolo_detector',
        executable='target_lost_monitor_node',
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
        output='screen',
    )

    return LaunchDescription([
        px4_dir_arg,
        xrce_port_arg,
        use_sim_tf_arg,
        cmd_mode_arg,
        run_tracker_arg,
        run_rl_policy_arg,
        run_world_service_bridge_arg,
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
        target_lost_monitor_node,
        tracker_node,
        rl_policy_node,
        cmd_vel_mux_node,
        rl_env_bridge_node,
    ])
