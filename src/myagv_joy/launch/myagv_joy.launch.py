
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='myagv_joy',
            executable='teleop_twist_joy_node',
            name='teleop_twist_joy_node',
            output='screen'
        ),
    ])
