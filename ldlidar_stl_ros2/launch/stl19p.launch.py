#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

'''
Parameter Description:
---
- Set laser scan direction:
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop function:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degrees.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degrees.
'''

def generate_launch_description():
  # LDROBOT LiDAR publisher node for STL-19P/LD19 family
  ldlidar_node = Node(
      package='ldlidar_stl_ros2',
      executable='ldlidar_stl_ros2_node',
      name='STL19P',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'topic_name': 'scan'},
        {'frame_id': 'laser'},
        {'port_name': '/dev/ttyUSB0'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 0.0},
        {'angle_crop_max': 0.0},
        {'scan_points': 501}
      ]
  )

  # base_footprint to laser tf node, matching the existing WAVEROVER SLAM frame conventions.
  base_footprint_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_footprint_to_laser_stl19p',
    arguments=['0','0','0.10','0','0','0','base_footprint','laser']
  )

  ld = LaunchDescription()
  ld.add_action(ldlidar_node)
  ld.add_action(base_footprint_to_laser_tf_node)

  return ld
