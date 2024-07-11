from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='myagv_communication',
            executable='serial',
            name='serial_node',
            output='screen'
        ),
    ])