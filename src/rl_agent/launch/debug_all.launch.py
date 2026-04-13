from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
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
        default_value='pid',
        description='cmd_vel_mux 输出模式: pid 或 rl',
    )
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/realsense/rgbd/image',
        description='RGB 图像 topic',
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/realsense/rgbd/depth_image',
        description='原始深度图 topic',
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/realsense/rgbd/camera_info',
        description='相机内参 topic',
    )
    depth_16uc1_topic_arg = DeclareLaunchArgument(
        'depth_16uc1_topic',
        default_value='/realsense/rgbd/depth_image_16uc1',
        description='16UC1 深度图 topic',
    )

    # 可选组件
    run_tracker_arg = DeclareLaunchArgument(
        'run_tracker',
        default_value='true',
        description='启动 PID tracker_node（发布 /uav/cmd_vel_pid）',
    )
    run_heuristic_policy_arg = DeclareLaunchArgument(
        'run_heuristic_policy',
        default_value='true',
        description='启动占位 heuristic_policy_node（发布 /uav/cmd_vel_rl）',
    )
    run_world_service_bridge_arg = DeclareLaunchArgument(
        'run_world_service_bridge',
        default_value='true',
        description='桥接 Gazebo world reset 相关服务到 ROS2',
    )
    run_target_px4_arg = DeclareLaunchArgument(
        'run_target_px4',
        default_value='true',
        description='启动目标无人机 PX4 实例',
    )
    target_px4_instance_arg = DeclareLaunchArgument(
        'target_px4_instance',
        default_value='2',
        description='目标无人机 PX4 实例编号',
    )
    target_px4_sys_autostart_arg = DeclareLaunchArgument(
        'target_px4_sys_autostart',
        default_value='4001',
        description='目标无人机 PX4_SYS_AUTOSTART',
    )
    target_px4_model_pose_arg = DeclareLaunchArgument(
        'target_px4_model_pose',
        default_value='5,2',
        description='目标无人机生成位置，格式为 x,y 或 x,y,z,...',
    )
    target_px4_sim_model_arg = DeclareLaunchArgument(
        'target_px4_sim_model',
        default_value='gz_x500',
        description='目标无人机 PX4_SIM_MODEL',
    )
    target_px4_spawn_delay_sec_arg = DeclareLaunchArgument(
        'target_px4_spawn_delay_sec',
        default_value='8.0',
        description='主 PX4/Gazebo 启动后，延时多少秒再加入目标无人机',
    )

    # PX4 + DDS
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl_default', 'gz_x500_depth'],
        cwd=LaunchConfiguration('px4_dir'),
        output='screen',
        emulate_tty=True,
    )
    target_px4 = ExecuteProcess(
        cmd=[
            'env',
            'PX4_GZ_STANDALONE=1',
            PythonExpression([
                "'PX4_SYS_AUTOSTART=' + '", LaunchConfiguration('target_px4_sys_autostart'), "'"
            ]),
            PythonExpression([
                "'PX4_GZ_MODEL_POSE=' + '", LaunchConfiguration('target_px4_model_pose'), "'"
            ]),
            PythonExpression([
                "'PX4_SIM_MODEL=' + '", LaunchConfiguration('target_px4_sim_model'), "'"
            ]),
            './build/px4_sitl_default/bin/px4',
            '-i',
            LaunchConfiguration('target_px4_instance'),
        ],
        cwd=LaunchConfiguration('px4_dir'),
        condition=IfCondition(LaunchConfiguration('run_target_px4')),
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

    # Gazebo 桥接
    bridge_depth = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[PythonExpression([
            "'", LaunchConfiguration('depth_topic'), "@sensor_msgs/msg/Image@gz.msgs.Image'"
        ])],
        output='screen',
    )
    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[PythonExpression([
            "'", LaunchConfiguration('camera_info_topic'),
            "@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'"
        ])],
        output='screen',
    )
    bridge_rgb = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[PythonExpression([
            "'", LaunchConfiguration('rgb_topic'), "@sensor_msgs/msg/Image@gz.msgs.Image'"
        ])],
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
        parameters=[{'image_topic': LaunchConfiguration('rgb_topic')}],
        output='screen',
    )
    target_depth_node = Node(
        package='yolo_detector',
        executable='target_depth_node',
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
        output='screen',
    )

    return LaunchDescription([
        px4_dir_arg,
        xrce_port_arg,
        use_sim_tf_arg,
        cmd_mode_arg,
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        depth_16uc1_topic_arg,
        run_tracker_arg,
        run_heuristic_policy_arg,
        run_world_service_bridge_arg,
        run_target_px4_arg,
        target_px4_instance_arg,
        target_px4_sys_autostart_arg,
        target_px4_model_pose_arg,
        target_px4_sim_model_arg,
        target_px4_spawn_delay_sec_arg,
        px4_sitl,
        delayed_target_px4,
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
        heuristic_policy_node,
        cmd_vel_mux_node,
        #rl_env_bridge_node,
    ])
