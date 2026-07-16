from glob import glob

from setuptools import find_packages, setup

package_name = 'waverover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        (
            'share/' + package_name + '/config',
            ['config/robot_defaults.yaml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='waverover',
    maintainer_email='waverover@todo.todo',
    description='Unified namespaced WaveRover SLAM/MCS onboard stack.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waverover_manual_lr = waverover.namespaced_manual_lr:main',
            'waverover_teleop = waverover.namespaced_teleop:main',
        ],
    },
)
