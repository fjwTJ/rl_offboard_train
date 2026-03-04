from setuptools import find_packages, setup

package_name = 'rl_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'rl_policy_node = rl_agent.rl_policy_node:main',
            'cmd_vel_mux_node = rl_agent.cmd_vel_mux_node:main',
        ],
    },
)
