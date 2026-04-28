from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix


def _make_train_node(context):
    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    train_arguments = [
        PathJoinSubstitution([FindPackagePrefix('rl_agent'), 'lib', 'rl_agent', 'train_rl']),
        '--exp-dir',
        cfg('exp_dir'),
        '--total-timesteps',
        cfg('total_timesteps'),
        '--save-freq',
        cfg('save_freq'),
        '--seed',
        cfg('seed'),
        '--device',
        cfg('device'),
        '--reset-topic',
        cfg('reset_topic'),
        '--target-state-active-topic',
        cfg('target_state_active_topic'),
        '--max-vx',
        cfg('max_vx'),
        '--max-vy',
        cfg('max_vy'),
        '--max-vz',
        cfg('max_vz'),
        '--max-yaw-rate',
        cfg('max_yaw_rate'),
        '--reset-timeout-sec',
        cfg('reset_timeout_sec'),
    ]

    resume_from = cfg('resume_from')
    if resume_from:
        train_arguments.extend(['--resume-from', resume_from])

    tensorboard_log = cfg('tensorboard_log')
    if tensorboard_log:
        train_arguments.extend(['--tensorboard-log', tensorboard_log])

    if cfg('progress_bar').lower() == 'true':
        train_arguments.append('--progress-bar')

    return [
        ExecuteProcess(
            cmd=train_arguments,
            output='screen',
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'exp_dir',
            default_value='runs/ppo_target_tracking',
            description='Training experiment output directory.',
        ),
        DeclareLaunchArgument(
            'resume_from',
            default_value='',
            description='Existing model zip path to resume from.',
        ),
        DeclareLaunchArgument(
            'total_timesteps',
            default_value='200000',
            description='Additional PPO timesteps to train.',
        ),
        DeclareLaunchArgument(
            'save_freq',
            default_value='10000',
            description='Checkpoint interval in timesteps.',
        ),
        DeclareLaunchArgument(
            'seed',
            default_value='42',
            description='Random seed.',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cpu',
            description='Stable-Baselines3 device string.',
        ),
        DeclareLaunchArgument(
            'tensorboard_log',
            default_value='',
            description='TensorBoard log root; empty disables it.',
        ),
        DeclareLaunchArgument(
            'progress_bar',
            default_value='true',
            description='Enable Stable-Baselines3 progress bar.',
        ),
        DeclareLaunchArgument(
            'max_vx',
            default_value='1.0',
            description='RL action maximum forward velocity.',
        ),
        DeclareLaunchArgument(
            'max_vy',
            default_value='0.4',
            description='RL action maximum lateral velocity.',
        ),
        DeclareLaunchArgument(
            'max_vz',
            default_value='1.0',
            description='RL action maximum vertical velocity.',
        ),
        DeclareLaunchArgument(
            'max_yaw_rate',
            default_value='1.6',
            description='RL action maximum yaw rate.',
        ),
        DeclareLaunchArgument(
            'reset_timeout_sec',
            default_value='60.0',
            description='Training environment reset timeout.',
        ),
        DeclareLaunchArgument(
            'reset_topic',
            default_value='/rl/reset',
            description='RL soft reset pulse topic.',
        ),
        DeclareLaunchArgument(
            'target_state_active_topic',
            default_value='/target_uav/state_active',
            description='Target UAV state topic observed by the RL environment.',
        ),
        OpaqueFunction(function=_make_train_node),
    ])
