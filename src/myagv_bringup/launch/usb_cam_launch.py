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
            default_value='/dev/video2', ################ls /dev/video* これでデバイス番号を確認し、変更する
            description='Device ID for USB camera'
        ),

        ComposableNodeContainer(
            name='usb_cam_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='usb_cam',
                    plugin='usb_cam::UsbCamNode',
                    name='camera',
                    namespace='',
                    parameters=[{
                        'video_device': LaunchConfiguration('device'),
                        'camera_info_url': 'file:///home/gari/.ros/camera_info/default_cam.yaml' ################
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output='screen',
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
