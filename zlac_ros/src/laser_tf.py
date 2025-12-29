# launch/static_tf_laser_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.045', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            name='base_to_laser_tf'
        ),
    ])
