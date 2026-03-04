from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir',
        default_value=[EnvironmentVariable('HOME'), '/PX4-Autopilot'],
        description='Path to PX4-Autopilot directory',
    )
    use_sim_tf_arg = DeclareLaunchArgument(
        'use_sim_tf',
        default_value='true',
        description='Use simulator camera extrinsic TF in camera_tf_publisher_node',
    )
    cmd_mode_arg = DeclareLaunchArgument(
        'cmd_mode',
        default_value='pid',
        description='cmd_vel_mux mode: pid or rl',
    )
    xrce_port_arg = DeclareLaunchArgument(
        'xrce_port',
        default_value='8888',
        description='MicroXRCEAgent UDP port',
    )

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

    depth_convert = Node(
        package='depth_convert',
        executable='depth_convert_node',
        output='screen',
    )

    yolo_node = Node(
        package='yolo_detector',
        executable='yolo_node',
        output='screen',
    )
    target_depth_node = Node(
        package='yolo_detector',
        executable='target_depth_node',
        output='screen',
    )
    camera_tf_publisher_node = Node(
        package='yolo_detector',
        executable='camera_tf_publisher_node',
        parameters=[{'use_sim_tf': LaunchConfiguration('use_sim_tf')}],
        output='screen',
    )

    rl_policy_node = Node(
        package='rl_agent',
        executable='rl_policy_node',
        output='screen',
    )
    tracker_node = Node(
        package='yolo_detector',
        executable='tracker_node',
        output='screen',
    )
    cmd_vel_mux_node = Node(
        package='rl_agent',
        executable='cmd_vel_mux_node',
        parameters=[{'mode': LaunchConfiguration('cmd_mode')}],
        output='screen',
    )

    return LaunchDescription([
        px4_dir_arg,
        use_sim_tf_arg,
        cmd_mode_arg,
        xrce_port_arg,
        px4_sitl,
        micro_xrce_agent,
        bridge_depth,
        bridge_camera_info,
        bridge_rgb,
        depth_convert,
        yolo_node,
        target_depth_node,
        camera_tf_publisher_node,
        rl_policy_node,
        tracker_node,
        cmd_vel_mux_node,
    ])
