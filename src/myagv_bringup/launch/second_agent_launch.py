from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='myagv_communication',
            executable='pd_control_node',  # ここに実行ファイル名（.pyを除く）を入力してください
            name='pd_control_node',
            output='screen'
        ),
        Node(
            package='myagv_communication',
            executable='move_control_node',  # ここに実行ファイル名（.pyを除く）を入力してください
            name='move_control_node',
            output='screen'
        ),
        Node(
            package='myagv_communication',
            executable='second_move_forward_node',
            name='move_forward_node',
            output='screen',
        ),
        Node(
            package='myagv_communication',
            executable='fcurve_node',
            name='fcurve_node',
            output='screen',
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
