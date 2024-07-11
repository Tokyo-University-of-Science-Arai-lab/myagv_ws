from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    map_yaml_file = LaunchConfiguration('map', default='/home/agv2/myagv_ws_proto2/src/myagv_navigation/map/map_1718364393.yaml')
    params_file = LaunchConfiguration('params_file', default='/home/agv2/myagv_ws_proto2/src/my_param/agv1_param.yaml')

    # Nav2 bringup launch
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'map': map_yaml_file,
                          'params_file': params_file}.items(),
    )

    rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='/home/agv2/myagv_ws_proto2/src/myagv_navigation/map/map_1718364393.yaml',
            description='Full path to map yaml file to load'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='/home/agv2/myagv_ws_proto2/src/my_param/agv1_param.yaml',
            description='Full path to the ROS2 parameters file to use for all launched nodes'
        ),
        nav2_bringup_launch,
        rviz_node
    ])

# /home/agv1/agv1_ws/map_1718364393.yaml'
# from launch import LaunchDescription
# from ament_index_python.packages import get_package_share_directory
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# import os

# def generate_launch_description():
#     use_sim_time = LaunchConfiguration('use_sim_time', default='False')
#     map_yaml_file = LaunchConfiguration('map', default='/home/agv1/agv1_ws/map_1718364393.yaml')
#     params_file = LaunchConfiguration('params_file', default='/home/agv1/myagv_ws/src/myagv_navigation/config/my_param/agv1_param.yaml')

#     # Nav2 bringup launch
#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')

#     nav2_bringup_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
#         launch_arguments={'use_sim_time': use_sim_time,
#                           'map': map_yaml_file,
#                           'params_file': params_file}.items(),
#     )

#     rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

#     # RViz node
#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='screen',
#         arguments=['-d', rviz_config_file],
#         parameters=[{'use_sim_time': use_sim_time}]
#     )

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='False',
#             description='Use simulation (Gazebo) clock if true'
#         ),
#         DeclareLaunchArgument(
#             'map',
#             default_value='/home/agv1/agv1_ws/map_1718364393.yaml',
#             description='Full path to map yaml file to load'
#         ),
#         DeclareLaunchArgument(
#             'params_file',
#             default_value='/home/agv1/myagv_ws/src/myagv_navigation/config/agv1_param.yaml',
#             description='Full path to the ROS2 parameters file to use for all launched nodes'
#         ),
#         nav2_bringup_launch,
#         rviz_node
#     ])

# from launch import LaunchDescription
# from ament_index_python.packages import get_package_share_directory
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# import os

# def generate_launch_description():
#     use_sim_time = LaunchConfiguration('use_sim_time', default='False')
#     map_yaml_file = LaunchConfiguration('map', default='/home/agv1/agv1_ws/map_1718364393.yaml')
#     params_file = LaunchConfiguration('params_file', default='/home/agv1/myagv_ws/src/myagv_navigation/config/agv1_param.yaml')

#     # Nav2 bringup launch
#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')

#     nav2_bringup_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
#         launch_arguments={'use_sim_time': use_sim_time,
#                           'map': map_yaml_file,
#                           'params_file': params_file}.items(),
#     )

#     rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

#     # RViz node
#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='screen',
#         arguments=['-d', rviz_config_file],
#         parameters=[{'use_sim_time': use_sim_time}]
#     )

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='False',
#             description='Use simulation (Gazebo) clock if true'
#         ),
#         DeclareLaunchArgument(
#             'map',
#             default_value='/home/agv1/agv1_ws/map_1718364393.yaml',
#             description='Full path to map yaml file to load'
#         ),
#         DeclareLaunchArgument(
#             'params_file',
#             default_value='/home/agv1/myagv_ws/src/myagv_navigation/config/agv1_param.yaml',
#             description='Full path to the ROS2 parameters file to use for all launched nodes'
#         ),
#         nav2_bringup_launch,
#         rviz_node
#     ])
