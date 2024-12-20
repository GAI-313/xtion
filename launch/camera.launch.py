import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    namespace = LaunchConfiguration('xtion_namespace')

    container = launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver
                launch_ros.descriptions.ComposableNode(
                    package='openni2_camera',
                    plugin='openni2_wrapper::OpenNI2Driver',
                    name='driver',
                    namespace=namespace,
                    parameters=[
                        {'depth_registration': True},
                        {'use_device_time': True},
                        {'rgb_camera_info_url': 'file://' + os.path.join(get_package_share_directory('xtion'),'config','rgb_PS1080_PrimeSense.yaml')},
                        {'depth_camera_info_url': 'file://' + os.path.join(get_package_share_directory('xtion'),'config','rgb_PS1080_PrimeSense.yaml')}
                    ],
                    remappings=[('depth/image', 'depth_registered/image_raw')],
                ),
                # Create XYZRGB point cloud
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='points_xyzrgb',
                    namespace=namespace,
                    parameters=[{'queue_size': 10}],
                    remappings=[('rgb/image_rect_color', 'rgb/image_raw'),
                                ('rgb/camera_info', 'rgb/camera_info'),
                                ('depth_registered/image_rect', 'depth_registered/image_raw'),
                                ('points', 'depth_registered/points'), ],
                ),
            ],
            output='screen',
    )

    # Include robot description launch
    description = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            ThisLaunchFileDir(), '/xtion_descroption.launch'
        ]),
        condition=IfCondition(LaunchConfiguration('sample_view'))
    )


    return launch.LaunchDescription([
            DeclareLaunchArgument(
                'sample_view',
                default_value='false',
                description='Sample preview'
            ),
            DeclareLaunchArgument(
                'xtion_namespace',
                default_value='camera',
                description='Xtion camera nodes namespace'
            ),
            container,
            launch_ros.actions.Node(
                package='rviz2',
                executable='rviz2',
                condition=IfCondition(LaunchConfiguration('sample_view')),
                arguments=[
                    '-d', os.path.join(
                    get_package_share_directory('xtion'),
                    'config', 'xtion_viewer.rviz'
                )]
            ),
            description
        ])
