from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    device = LaunchConfiguration('device', default='0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='0',
            description='Device ID for USB camera'
        ),

        ComposableNodeContainer(
            name='apriltag_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='usb_cam',
                    plugin='usb_cam::UsbCamNode',
                    name='camera',
                    namespace='usb_cam',
                    parameters=[{
                        'video_device': LaunchConfiguration('device', default='/dev/video0'),
                        'camera_info_url': 'file:///home/agv2/.ros/camera_info/default_cam.yaml'
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify',
                    namespace='usb_cam',
                    remappings=[
                        ('image', 'image_raw'),
                        ('camera_info', 'camera_info')
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='apriltag_ros',
                    plugin='AprilTagNode',
                    name='apriltag',
                    namespace='apriltag',
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
            output='screen'
        ),
        Node(
            package='myagv_communication',
            executable='move_control_node',  # ここに実行ファイル名（.pyを除く）を入力してください
            name='move_control_node',
            output='screen'
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=['/home/agv1/myagv_ws/src/my_param/usb_cam_params.yaml'],
            output='screen',
        ),
    ])


'''
ここに入っているファイルはagentだから、以下の3点のファイルを起動する。
・apriltag
・Navigation
・move_controller
・usb_cam_node
・pd_control_node






'''
