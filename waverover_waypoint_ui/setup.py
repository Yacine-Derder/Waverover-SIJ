from glob import glob
from setuptools import find_packages, setup


package_name = 'waverover_waypoint_ui'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='waverover',
    maintainer_email='waverover@todo.todo',
    description='Terminal sender for namespaced WaveRover waypoints.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_ui = waverover_waypoint_ui.waypoint_ui:main',
        ],
    },
)
