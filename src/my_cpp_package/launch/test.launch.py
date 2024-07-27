# load_components.launch.py
from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        LoadComposableNodes(
            target_container='component_manager',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_cpp_package',
                    plugin='my_cpp_package::PointCloudPublisher',
                    name='pointcloud_publisher',
                    # output='screen',
                ),
                ComposableNode(
                    package='my_cpp_package',
                    plugin='my_cpp_package::PointCloud2Subscriber',
                    name='pointcloud2_subscriber',
                    # output='screen',
                )
            ]
        )
    ])