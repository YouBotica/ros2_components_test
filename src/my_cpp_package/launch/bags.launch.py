import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def get_share_file(package_name: str, *args: str) -> str:
    """Convert package-relative path to absolute path. Any additional args
    will be appended to the package_name, separated by '/'.

    Args:
        package_name (str): Package name.

    Returns:
        os.path: Absolute path.
    """
    return os.path.join(get_package_share_directory(package_name), *args)


def generate_launch_description():


    iris_pkg_share = get_package_share_directory('ros2_iris_driver')

    # Add the path to the parameters file
    params_file = os.path.join(iris_pkg_share, 'param', 'car_2.yaml')


    # Create a launch description
    ld = LaunchDescription()

    # Add nodes:
    iris_ros_driver= Node(
        package='ros2_iris_driver',
        executable='iris_ros_driver',
        name='iris_ros_driver',
        namespace='/perception/',
        output='screen',
        parameters=[{"yaml_fp": params_file}],
        # extra_arguments=[{'use_intra_process_comms': True}]
    )

    minimal_pointcloud_pub = Node(
        package='my_cpp_package',
        executable='minimal_publisher_icp',
        output='screen',
        # extra_arguments=[{'use_intra_process_comms': True}]
    )

    minimal_subscriber_pub = Node(
        package='my_cpp_package',
        executable='minimal_subscriber_icp',
        output='screen',
        # extra_arguments=[{'use_intra_process_comms': True}]
    )



    # robot_description_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         get_share_file("av24_description", "launch", "av24_description.launch.py")
    #     ),
    # ) # NOTE: Launched in sensors.launch.py in on-vehicle

    ld.add_action(iris_ros_driver)
    ld.add_action(minimal_pointcloud_pub)
    ld.add_action(minimal_subscriber_pub)

    # ld.add_action(robot_description_launch)

    return ld
    

# import launch
# import launch_ros
# import yaml
# from launch import LaunchDescription
# from launch_ros.actions import ComposableNodeContainer
# from launch_ros.descriptions import ComposableNode
# from ament_index_python import get_package_share_directory
# import os

# '''
# Used to load parameters for composable nodes from a standard param file
# '''

# def get_share_file(package_name, file_name):
#     return os.path.join(get_package_share_directory(package_name), file_name)

# def dump_params(param_file_path, node_name):
#     with open(param_file_path, 'r') as file:
#         return [yaml.safe_load(file)[node_name]['ros__parameters']]


# def generate_launch_description():

#     bags_pkg_path = get_package_share_directory('my_cpp_package') # os.path.join(get_package_share_directory(os.path.join('my_cpp_package')), 'param', )


#     return LaunchDescription([
#         ComposableNodeContainer(
#             name='composable_container',
#             namespace='',
#             package='rclcpp_components',
#             executable='component_container',
#             composable_node_descriptions=[
#                 ComposableNode(
#                     package='rosbag2_transport',
#                     plugin='rosbag2_transport::Recorder',
#                     name='recorder',
#                     parameters=dump_params(os.path.join(bags_pkg_path, 'param', 'bags.param.yaml'), "recorder"),
#                     # extra_arguments=[{'use_intra_process_comms': True}]
#                 ),
#                 # Add your other components here
#             ],
#             output='screen',
#         )
#     ])
