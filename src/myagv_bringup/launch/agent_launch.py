from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # launchファイルのディレクトリを基準に相対パスを決定
    script_dir = os.path.dirname(os.path.realpath(__file__))

    # yamlファイルの相対パスを設定
    yaml_file = os.path.join(script_dir, '../../my_param/usb_cam_params.yaml')
    yaml_file = os.path.abspath(yaml_file)
    if not os.path.exists(yaml_file):
        raise FileNotFoundError(f"YAML file not found: {yaml_file}")
    
    # camera_infoファイルの相対パスを設定
    camera_info_file = os.path.join(script_dir, '../../my_param/default_cam.yaml')
    camera_info_file = os.path.abspath(camera_info_file)
    if not os.path.exists(camera_info_file):
        raise FileNotFoundError(f"Camera info file not found: {camera_info_file}")
    
    namespace = LaunchConfiguration('namespace', default='')
    device = LaunchConfiguration('device', default='/dev/video6')  # Updated device to /dev/video4

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the nodes'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='/dev/video6',  # Updated device to /dev/video4
            description='Device ID for USB camera'
        ),

        ComposableNodeContainer(
            name='apriltag_container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='usb_cam',
                    plugin='usb_cam::UsbCamNode',
                    name='camera',
                    namespace='usb_cam',
                    parameters=[{
                        'video_device': LaunchConfiguration('device', default='/dev/video6'),  # Updated device to /dev/video4
                        'camera_info_url': f'file://{camera_info_file}'
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify',
                    namespace=namespace,
                    remappings=[
                        ('image', '/image_raw'),
                        ('camera_info', '/camera_info')
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='apriltag_ros',
                    plugin='AprilTagNode',
                    name='apriltag',
                    namespace=namespace,
                    remappings=[
                        ('/apriltag/image_rect', '/image_raw'),
                        ('/apriltag/camera_info', '/camera_info')
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output='screen',
        ),
        Node(
            package='myagv_communication',
            executable='pd_control_node',  # ここに実行ファイル名（.pyを除く）を入力してください
            name='pd_control_node',
            namespace=namespace,
            output='screen'
        ),
        Node(
            package='myagv_communication',
            executable='move_control_node',  # ここに実行ファイル名（.pyを除く）を入力してください
            name='move_control_node',
            namespace=namespace,
            output='screen'
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace=namespace,
            parameters=[{'video_device': '/dev/video6'}, yaml_file],  # Updated device to /dev/video4
            output='screen',
        ),
        Node(
            package='myagv_communication',
            executable='move_forward_node',
            name='move_forward_node',
            namespace=namespace,  # このnamespaceがノードに渡されます
            output='screen',
            parameters=[{'namespace': LaunchConfiguration('namespace')}]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
