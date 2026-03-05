from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Core runtime args
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir',
        default_value=[EnvironmentVariable('HOME'), '/PX4-Autopilot'],
        description='Path to PX4-Autopilot directory',
    )
    xrce_port_arg = DeclareLaunchArgument(
        'xrce_port',
        default_value='8888',
        description='MicroXRCEAgent UDP port',
    )
    use_sim_tf_arg = DeclareLaunchArgument(
        'use_sim_tf',
        default_value='true',
        description='Use simulator camera extrinsic in camera_tf_publisher_node',
    )
    cmd_mode_arg = DeclareLaunchArgument(
        'cmd_mode',
        default_value='rl',
        description='cmd_vel_mux mode: pid or rl (default rl for training)',
    )

    # Optional components
    run_rl_policy_arg = DeclareLaunchArgument(
        'run_rl_policy',
        default_value='false',
        description='Run placeholder rl_policy_node. Keep false when external trainer publishes /uav/cmd_vel_rl',
    )
    run_tracker_arg = DeclareLaunchArgument(
        'run_tracker',
        default_value='false',
        description='Run PID tracker_node publisher (/uav/cmd_vel_pid)',
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

    # Gazebo bridge
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

    # Perception
    depth_convert = Node(package='depth_convert', executable='depth_convert_node', output='screen')
    yolo_node = Node(package='yolo_detector', executable='yolo_node', output='screen')
    target_depth_node = Node(package='yolo_detector', executable='target_depth_node', output='screen')
    camera_tf_publisher_node = Node(
        package='yolo_detector',
        executable='camera_tf_publisher_node',
        parameters=[{'use_sim_tf': LaunchConfiguration('use_sim_tf')}],
        output='screen',
    )

    # Decision + bridge
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
        rl_policy_node,
        cmd_vel_mux_node,
        rl_env_bridge_node,
    ])
