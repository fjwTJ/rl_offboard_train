from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # PX4_2 的 ROS 2 话题前缀。
    px4_namespace_arg = DeclareLaunchArgument(
        'px4_namespace',
        default_value='/px4_2/fmu/',
        description='Target UAV PX4 topic namespace.',
    )
    # 低层 offboard 执行器订阅的速度指令话题。
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/target_uav/cmd_vel_body',
        description='Body-frame velocity command topic for the target UAV.',
    )
    # 目标机 mission 激活状态话题；供随机运动节点做门控。
    mission_active_topic_arg = DeclareLaunchArgument(
        'mission_active_topic',
        default_value='/target_uav/mission_active',
        description='Publishes whether the target UAV is in mission state.',
    )
    # 目标机起飞高度。
    takeoff_height_arg = DeclareLaunchArgument(
        'takeoff_height',
        default_value='3.0',
        description='Takeoff altitude in meters.',
    )
    # 目标机起飞朝向。
    takeoff_yaw_arg = DeclareLaunchArgument(
        'takeoff_yaw',
        default_value='1.57',
        description='Takeoff yaw in radians.',
    )
    # 低层 offboard 控制循环频率。
    control_rate_hz_arg = DeclareLaunchArgument(
        'control_rate_hz',
        default_value='10.0',
        description='Control loop frequency.',
    )
    # 随机动作节点停止发布后，低层执行器保留最后动作的时间。
    cmd_timeout_sec_arg = DeclareLaunchArgument(
        'cmd_timeout_sec',
        default_value='0.5',
        description='How long to keep the last velocity command before hovering.',
    )
    # 是否启动目标机随机运动节点。
    run_random_motion_arg = DeclareLaunchArgument(
        'run_random_motion',
        default_value='true',
        description='Run the target random motion generator for RL target behavior.',
    )
    # 随机运动节点发布当前模式名的调试话题。
    motion_mode_topic_arg = DeclareLaunchArgument(
        'motion_mode_topic',
        default_value='/target_uav/motion_mode',
        description='Debug topic for the current target motion mode.',
    )
    # 随机运动节点输出频率。
    random_publish_rate_hz_arg = DeclareLaunchArgument(
        'random_publish_rate_hz',
        default_value='20.0',
        description='Publish rate for the target random motion node.',
    )
    # 单次随机模式最短持续时间。
    mode_duration_min_sec_arg = DeclareLaunchArgument(
        'mode_duration_min_sec',
        default_value='1.0',
        description='Minimum duration for a sampled motion mode.',
    )
    # 单次随机模式最长持续时间。
    mode_duration_max_sec_arg = DeclareLaunchArgument(
        'mode_duration_max_sec',
        default_value='3.0',
        description='Maximum duration for a sampled motion mode.',
    )
    # 随机目标前向速度上限。
    max_forward_speed_arg = DeclareLaunchArgument(
        'max_forward_speed',
        default_value='1.2',
        description='Maximum forward speed for target random motion.',
    )
    # 随机目标横向速度上限。
    max_lateral_speed_arg = DeclareLaunchArgument(
        'max_lateral_speed',
        default_value='0.6',
        description='Maximum lateral speed for target random motion.',
    )
    # 随机目标垂向速度上限。
    max_vertical_speed_arg = DeclareLaunchArgument(
        'max_vertical_speed',
        default_value='0.15',
        description='Maximum vertical speed for target random motion.',
    )
    # 随机目标偏航角速度上限。
    max_yaw_rate_arg = DeclareLaunchArgument(
        'max_yaw_rate',
        default_value='0.35',
        description='Maximum yaw rate for target random motion.',
    )
    # 目标机允许活动的安全半径。
    safe_radius_m_arg = DeclareLaunchArgument(
        'safe_radius_m',
        default_value='8.0',
        description='XY radius limit before the target motion biases back toward home.',
    )
    # 是否启用垂向随机动作。
    enable_vertical_motion_arg = DeclareLaunchArgument(
        'enable_vertical_motion',
        default_value='false',
        description='Allow vertical random motion for the target UAV.',
    )

    node = Node(
        package='target_uav_offboard',
        executable='target_offboard_control',
        output='screen',
        parameters=[{
            'px4_namespace': LaunchConfiguration('px4_namespace'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'mission_active_topic': LaunchConfiguration('mission_active_topic'),
            'takeoff_height': LaunchConfiguration('takeoff_height'),
            'takeoff_yaw': LaunchConfiguration('takeoff_yaw'),
            'control_rate_hz': LaunchConfiguration('control_rate_hz'),
            'cmd_timeout_sec': LaunchConfiguration('cmd_timeout_sec'),
        }],
    )
    random_motion_node = Node(
        package='target_uav_offboard',
        executable='target_random_motion_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_random_motion')),
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'mission_active_topic': LaunchConfiguration('mission_active_topic'),
            'odometry_topic': [LaunchConfiguration('px4_namespace'), 'out/vehicle_odometry'],
            'motion_mode_topic': LaunchConfiguration('motion_mode_topic'),
            'publish_rate_hz': LaunchConfiguration('random_publish_rate_hz'),
            'mode_duration_min_sec': LaunchConfiguration('mode_duration_min_sec'),
            'mode_duration_max_sec': LaunchConfiguration('mode_duration_max_sec'),
            'max_forward_speed': LaunchConfiguration('max_forward_speed'),
            'max_lateral_speed': LaunchConfiguration('max_lateral_speed'),
            'max_vertical_speed': LaunchConfiguration('max_vertical_speed'),
            'max_yaw_rate': LaunchConfiguration('max_yaw_rate'),
            'safe_radius_m': LaunchConfiguration('safe_radius_m'),
            'enable_vertical_motion': LaunchConfiguration('enable_vertical_motion'),
        }],
    )

    return LaunchDescription([
        px4_namespace_arg,
        cmd_vel_topic_arg,
        mission_active_topic_arg,
        takeoff_height_arg,
        takeoff_yaw_arg,
        control_rate_hz_arg,
        cmd_timeout_sec_arg,
        run_random_motion_arg,
        motion_mode_topic_arg,
        random_publish_rate_hz_arg,
        mode_duration_min_sec_arg,
        mode_duration_max_sec_arg,
        max_forward_speed_arg,
        max_lateral_speed_arg,
        max_vertical_speed_arg,
        max_yaw_rate_arg,
        safe_radius_m_arg,
        enable_vertical_motion_arg,
        node,
        random_motion_node,
    ])
