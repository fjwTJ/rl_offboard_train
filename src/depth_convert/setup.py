from setuptools import find_packages, setup

package_name = 'depth_convert'

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
    maintainer='fjw',
    maintainer_email='fjw514@gmail.com',
    description='Convert float32 depth images into 16UC1 depth images.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_convert_node = depth_convert.depth_convert_node:main',
        ],
    },
)
