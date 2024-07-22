from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # tf_launch.py があるディレクトリを基準に相対パスを決定
    script_dir = os.path.dirname(os.path.realpath(__file__))
    urdf_file = os.path.join(script_dir, '../urdf/agv.urdf')

    # urdf_file = '/home/gari/myagv_ws/src/myagv_description/urdf/agv.urdf'

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    namespace = 'agv2'

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=namespace,
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
