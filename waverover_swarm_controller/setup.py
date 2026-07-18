from glob import glob
from setuptools import find_packages, setup


package_name = 'waverover_swarm_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='waverover',
    maintainer_email='waverover@todo.todo',
    description='PC-only WaveRover swarm coordinator.',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'coordinator = '
            'waverover_swarm_controller.coordinator_node:main',
            'synthetic_mcs = '
            'waverover_swarm_controller.synthetic_mcs:main',
            'visualize_targets = '
            'waverover_swarm_controller.target_visualizer:main',
        ],
    },
)
