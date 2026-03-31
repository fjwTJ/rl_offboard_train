from setuptools import find_packages, setup
from glob import glob

package_name = 'rl_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fjwjetson',
    maintainer_email='fjwjetson@todo.todo',
    description='RL policy and command mux nodes for PX4 target tracking.',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'heuristic_policy_node = rl_agent.inference.heuristic_policy_node:main',
            'cmd_vel_mux_node = rl_agent.bridges.cmd_vel_mux_node:main',
            'rl_env_bridge_node = rl_agent.bridges.rl_env_bridge_node:main',
            'rl_metrics_node = rl_agent.evaluation.rl_metrics_node:main',
            'episode_manager_node = rl_agent.orchestration.episode_manager_node:main',
            'policy_inference_node = rl_agent.inference.policy_inference_node:main',
            'train_rl = rl_agent.training.train_ppo:main',
        ],
    },
)
