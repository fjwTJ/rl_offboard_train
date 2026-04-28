from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir',
        default_value=[EnvironmentVariable('HOME'), '/PX4-Autopilot'],
        description='PX4-Autopilot directory.',
    )
    xrce_port_arg = DeclareLaunchArgument(
        'xrce_port',
        default_value='8888',
        description='MicroXRCEAgent UDP port.',
    )
    run_px4_sitl_direct_arg = DeclareLaunchArgument(
        'run_px4_sitl_direct',
        default_value='true',
        description='Launch the main PX4 SITL instance directly.',
    )
    run_target_px4_arg = DeclareLaunchArgument(
        'run_target_px4',
        default_value='true',
        description='Launch the target UAV PX4 instance.',
    )
    target_px4_instance_arg = DeclareLaunchArgument(
        'target_px4_instance',
        default_value='2',
        description='Target UAV PX4 instance id.',
    )
    target_px4_sys_autostart_arg = DeclareLaunchArgument(
        'target_px4_sys_autostart',
        default_value='4001',
        description='Target UAV PX4_SYS_AUTOSTART.',
    )
    target_px4_model_pose_arg = DeclareLaunchArgument(
        'target_px4_model_pose',
        default_value='5,0',
        description='Target UAV spawn pose, such as x,y or x,y,z,...',
    )
    target_px4_sim_model_arg = DeclareLaunchArgument(
        'target_px4_sim_model',
        default_value='gz_x500',
        description='Target UAV PX4_SIM_MODEL.',
    )
    target_px4_spawn_delay_sec_arg = DeclareLaunchArgument(
        'target_px4_spawn_delay_sec',
        default_value='8.0',
        description='Delay before launching the target PX4 instance.',
    )

    use_sim_tf_arg = DeclareLaunchArgument(
        'use_sim_tf',
        default_value='true',
        description='Use simulation camera extrinsics in camera_tf_publisher_node.',
    )
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/realsense/rgbd/image',
        description='RGB image topic.',
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/realsense/rgbd/depth_image',
        description='Raw depth image topic.',
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/realsense/rgbd/camera_info',
        description='Camera info topic.',
    )
    depth_16uc1_topic_arg = DeclareLaunchArgument(
        'depth_16uc1_topic',
        default_value='/realsense/rgbd/depth_image_16uc1',
        description='Converted 16UC1 depth image topic.',
    )
    run_yolo_arg = DeclareLaunchArgument(
        'run_yolo',
        default_value='true',
        description='Run YOLO detector.',
    )
    run_target_depth_arg = DeclareLaunchArgument(
        'run_target_depth',
        default_value='true',
        description='Run target depth projection node.',
    )
    run_target_lost_monitor_arg = DeclareLaunchArgument(
        'run_target_lost_monitor',
        default_value='true',
        description='Run target lost monitor.',
    )
    target_lost_timeout_sec_arg = DeclareLaunchArgument(
        'target_lost_timeout_sec',
        default_value='1.0',
        description='Target lost timeout in seconds.',
    )
    yolo_enable_visualization_arg = DeclareLaunchArgument(
        'yolo_enable_visualization',
        default_value='true',
        description='Enable YOLO OpenCV visualization window.',
    )

    offboard_rl_train_mode_arg = DeclareLaunchArgument(
        'offboard_rl_train_mode',
        default_value='true',
        description='Let the main offboard controller enter Mission automatically after takeoff.',
    )
    offboard_takeoff_height_arg = DeclareLaunchArgument(
        'offboard_takeoff_height',
        default_value='5.0',
        description='Main UAV takeoff height in meters.',
    )
    offboard_takeoff_yaw_arg = DeclareLaunchArgument(
        'offboard_takeoff_yaw',
        default_value='1.57',
        description='Main UAV takeoff yaw in radians.',
    )
    offboard_state_active_topic_arg = DeclareLaunchArgument(
        'offboard_state_active_topic',
        default_value='/uav/state_active',
        description='Main UAV state topic.',
    )
    offboard_cmd_vel_topic_arg = DeclareLaunchArgument(
        'offboard_cmd_vel_topic',
        default_value='/uav/cmd_vel_body',
        description='Main UAV body-frame velocity command topic.',
    )
    mission_control_topic_arg = DeclareLaunchArgument(
        'mission_control_topic',
        default_value='/mission_control',
        description='Mission control command topic.',
    )
    target_lost_topic_arg = DeclareLaunchArgument(
        'target_lost_topic',
        default_value='/perception/target_lost',
        description='Target lost state topic.',
    )
    reset_topic_arg = DeclareLaunchArgument(
        'reset_topic',
        default_value='/rl/reset',
        description='RL soft reset pulse topic.',
    )

    target_px4_namespace_arg = DeclareLaunchArgument(
        'target_px4_namespace',
        default_value='/px4_2/fmu/',
        description='Target UAV PX4 topic namespace.',
    )
    target_cmd_vel_topic_arg = DeclareLaunchArgument(
        'target_cmd_vel_topic',
        default_value='/target_uav/cmd_vel_body',
        description='Target UAV body-frame velocity command topic.',
    )
    target_state_active_topic_arg = DeclareLaunchArgument(
        'target_state_active_topic',
        default_value='/target_uav/state_active',
        description='Target UAV state topic.',
    )
    target_takeoff_height_arg = DeclareLaunchArgument(
        'target_takeoff_height',
        default_value='5.0',
        description='Target UAV takeoff height in meters.',
    )
    target_takeoff_yaw_arg = DeclareLaunchArgument(
        'target_takeoff_yaw',
        default_value='1.57',
        description='Target UAV takeoff yaw in radians.',
    )
    target_control_rate_hz_arg = DeclareLaunchArgument(
        'target_control_rate_hz',
        default_value='10.0',
        description='Target offboard control rate.',
    )
    target_cmd_timeout_sec_arg = DeclareLaunchArgument(
        'target_cmd_timeout_sec',
        default_value='0.5',
        description='Target command timeout before hovering.',
    )
    run_target_random_motion_arg = DeclareLaunchArgument(
        'run_target_random_motion',
        default_value='true',
        description='Run target UAV random motion generator.',
    )
    target_random_publish_rate_hz_arg = DeclareLaunchArgument(
        'target_random_publish_rate_hz',
        default_value='20.0',
        description='Target random motion publish rate.',
    )
    target_mode_duration_min_sec_arg = DeclareLaunchArgument(
        'target_mode_duration_min_sec',
        default_value='1.0',
        description='Minimum target random motion mode duration.',
    )
    target_mode_duration_max_sec_arg = DeclareLaunchArgument(
        'target_mode_duration_max_sec',
        default_value='3.0',
        description='Maximum target random motion mode duration.',
    )
    target_max_forward_speed_arg = DeclareLaunchArgument(
        'target_max_forward_speed',
        default_value='1.2',
        description='Target random motion maximum forward speed.',
    )
    target_max_lateral_speed_arg = DeclareLaunchArgument(
        'target_max_lateral_speed',
        default_value='0.6',
        description='Target random motion maximum lateral speed.',
    )
    target_max_vertical_speed_arg = DeclareLaunchArgument(
        'target_max_vertical_speed',
        default_value='0.15',
        description='Target random motion maximum vertical speed.',
    )
    target_max_yaw_rate_arg = DeclareLaunchArgument(
        'target_max_yaw_rate',
        default_value='0.35',
        description='Target random motion maximum yaw rate.',
    )
    target_safe_radius_m_arg = DeclareLaunchArgument(
        'target_safe_radius_m',
        default_value='8.0',
        description='Target random motion safe XY radius.',
    )
    target_enable_vertical_motion_arg = DeclareLaunchArgument(
        'target_enable_vertical_motion',
        default_value='false',
        description='Allow target random vertical motion.',
    )

    px4_sitl = ExecuteProcess(
        cmd=['env', 'make', 'px4_sitl_default', 'gz_x500_depth'],
        cwd=LaunchConfiguration('px4_dir'),
        condition=IfCondition(LaunchConfiguration('run_px4_sitl_direct')),
        output='screen',
        emulate_tty=True,
    )
    target_px4 = ExecuteProcess(
        cmd=[
            'env',
            'PX4_GZ_STANDALONE=1',
            PythonExpression(["'PX4_SYS_AUTOSTART=' + '", LaunchConfiguration('target_px4_sys_autostart'), "'"]),
            PythonExpression(["'PX4_GZ_MODEL_POSE=' + '", LaunchConfiguration('target_px4_model_pose'), "'"]),
            PythonExpression(["'PX4_SIM_MODEL=' + '", LaunchConfiguration('target_px4_sim_model'), "'"]),
            './build/px4_sitl_default/bin/px4',
            '-i',
            LaunchConfiguration('target_px4_instance'),
        ],
        cwd=LaunchConfiguration('px4_dir'),
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration('run_px4_sitl_direct'),
                "' == 'true' and '",
                LaunchConfiguration('run_target_px4'),
                "' == 'true'",
            ])
        ),
        output='screen',
        emulate_tty=True,
    )
    delayed_target_px4 = TimerAction(
        period=LaunchConfiguration('target_px4_spawn_delay_sec'),
        actions=[target_px4],
    )
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', LaunchConfiguration('xrce_port')],
        output='screen',
        emulate_tty=True,
    )

    bridge_depth = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[PythonExpression(["'", LaunchConfiguration('depth_topic'), "@sensor_msgs/msg/Image@gz.msgs.Image'"])],
        output='screen',
    )
    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            PythonExpression([
                "'",
                LaunchConfiguration('camera_info_topic'),
                "@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'",
            ])
        ],
        output='screen',
    )
    bridge_rgb = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[PythonExpression(["'", LaunchConfiguration('rgb_topic'), "@sensor_msgs/msg/Image@gz.msgs.Image'"])],
        output='screen',
    )

    depth_convert = Node(
        package='depth_convert',
        executable='depth_convert_node',
        parameters=[{
            'input_topic': LaunchConfiguration('depth_topic'),
            'output_topic': LaunchConfiguration('depth_16uc1_topic'),
        }],
        output='screen',
    )
    yolo_node = Node(
        package='yolo_detector',
        executable='yolo_node',
        condition=IfCondition(LaunchConfiguration('run_yolo')),
        parameters=[{
            'image_topic': LaunchConfiguration('rgb_topic'),
            'enable_visualization': LaunchConfiguration('yolo_enable_visualization'),
        }],
        output='screen',
    )
    target_depth_node = Node(
        package='yolo_detector',
        executable='target_depth_node',
        condition=IfCondition(LaunchConfiguration('run_target_depth')),
        parameters=[{
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'depth_topic': LaunchConfiguration('depth_16uc1_topic'),
        }],
        output='screen',
    )
    camera_tf_publisher_node = Node(
        package='yolo_detector',
        executable='camera_tf_publisher_node',
        parameters=[{'use_sim_tf': LaunchConfiguration('use_sim_tf')}],
        output='screen',
    )
    target_lost_monitor_node = Node(
        package='yolo_detector',
        executable='target_lost_monitor_node',
        condition=IfCondition(LaunchConfiguration('run_target_lost_monitor')),
        parameters=[{'target_timeout_sec': LaunchConfiguration('target_lost_timeout_sec')}],
        output='screen',
    )

    offboard_control_srv = Node(
        package='px4_ros_com',
        executable='offboard_control_srv',
        output='screen',
        parameters=[{
            'rl_train_mode': LaunchConfiguration('offboard_rl_train_mode'),
            'takeoff_height': LaunchConfiguration('offboard_takeoff_height'),
            'takeoff_yaw': LaunchConfiguration('offboard_takeoff_yaw'),
            'state_active_topic': LaunchConfiguration('offboard_state_active_topic'),
            'mission_control_topic': LaunchConfiguration('mission_control_topic'),
            'cmd_vel_topic': LaunchConfiguration('offboard_cmd_vel_topic'),
            'target_lost_topic': LaunchConfiguration('target_lost_topic'),
            'reset_topic': LaunchConfiguration('reset_topic'),
            'peer_state_active_topic': LaunchConfiguration('target_state_active_topic'),
            'rl_training_mode': True,
        }],
    )
    target_offboard_control = Node(
        package='target_uav_offboard',
        executable='target_offboard_control',
        output='screen',
        parameters=[{
            'px4_namespace': LaunchConfiguration('target_px4_namespace'),
            'cmd_vel_topic': LaunchConfiguration('target_cmd_vel_topic'),
            'state_active_topic': LaunchConfiguration('target_state_active_topic'),
            'takeoff_height': LaunchConfiguration('target_takeoff_height'),
            'takeoff_yaw': LaunchConfiguration('target_takeoff_yaw'),
            'control_rate_hz': LaunchConfiguration('target_control_rate_hz'),
            'cmd_timeout_sec': LaunchConfiguration('target_cmd_timeout_sec'),
            'rl_training_mode': True,
            'peer_state_active_topic': LaunchConfiguration('offboard_state_active_topic'),
            'reset_topic': LaunchConfiguration('reset_topic'),
        }],
    )
    target_random_motion_node = Node(
        package='target_uav_offboard',
        executable='target_random_motion_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_target_random_motion')),
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('target_cmd_vel_topic'),
            'reset_topic': LaunchConfiguration('reset_topic'),
            'state_active_topic': LaunchConfiguration('target_state_active_topic'),
            'odometry_topic': [LaunchConfiguration('target_px4_namespace'), 'out/vehicle_odometry'],
            'motion_mode_topic': '/target_uav/motion_mode',
            'publish_rate_hz': LaunchConfiguration('target_random_publish_rate_hz'),
            'mode_duration_min_sec': LaunchConfiguration('target_mode_duration_min_sec'),
            'mode_duration_max_sec': LaunchConfiguration('target_mode_duration_max_sec'),
            'max_forward_speed': LaunchConfiguration('target_max_forward_speed'),
            'max_lateral_speed': LaunchConfiguration('target_max_lateral_speed'),
            'max_vertical_speed': LaunchConfiguration('target_max_vertical_speed'),
            'max_yaw_rate': LaunchConfiguration('target_max_yaw_rate'),
            'safe_radius_m': LaunchConfiguration('target_safe_radius_m'),
            'enable_vertical_motion': LaunchConfiguration('target_enable_vertical_motion'),
        }],
    )

    return LaunchDescription([
        px4_dir_arg,
        xrce_port_arg,
        run_px4_sitl_direct_arg,
        run_target_px4_arg,
        target_px4_instance_arg,
        target_px4_sys_autostart_arg,
        target_px4_model_pose_arg,
        target_px4_sim_model_arg,
        target_px4_spawn_delay_sec_arg,
        use_sim_tf_arg,
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        depth_16uc1_topic_arg,
        run_yolo_arg,
        run_target_depth_arg,
        run_target_lost_monitor_arg,
        target_lost_timeout_sec_arg,
        yolo_enable_visualization_arg,
        offboard_rl_train_mode_arg,
        offboard_takeoff_height_arg,
        offboard_takeoff_yaw_arg,
        offboard_state_active_topic_arg,
        offboard_cmd_vel_topic_arg,
        mission_control_topic_arg,
        target_lost_topic_arg,
        reset_topic_arg,
        target_px4_namespace_arg,
        target_cmd_vel_topic_arg,
        target_state_active_topic_arg,
        target_takeoff_height_arg,
        target_takeoff_yaw_arg,
        target_control_rate_hz_arg,
        target_cmd_timeout_sec_arg,
        run_target_random_motion_arg,
        target_random_publish_rate_hz_arg,
        target_mode_duration_min_sec_arg,
        target_mode_duration_max_sec_arg,
        target_max_forward_speed_arg,
        target_max_lateral_speed_arg,
        target_max_vertical_speed_arg,
        target_max_yaw_rate_arg,
        target_safe_radius_m_arg,
        target_enable_vertical_motion_arg,
        px4_sitl,
        delayed_target_px4,
        micro_xrce_agent,
        bridge_depth,
        bridge_camera_info,
        bridge_rgb,
        depth_convert,
        yolo_node,
        target_depth_node,
        camera_tf_publisher_node,
        target_lost_monitor_node,
        offboard_control_srv,
        target_offboard_control,
        target_random_motion_node,
    ])
