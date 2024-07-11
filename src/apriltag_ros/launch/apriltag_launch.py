from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration

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
                        'camera_info_url': 'file:///home/agv1/.ros/camera_info/default_cam.yaml'
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
                    namespace='apriltag',\
                    remappings=[
                        ('/apriltag/image_rect', '/image_raw'),
                        ('/apriltag/camera_info', '/camera_info')
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output='screen',
        ),
    ])
