cmake_minimum_required(VERSION 3.8)
project(my_cpp_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp_components REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Create shared libraries for components
# add_library(minimal_publisher_component SHARED src/minimal_publisher.cpp)
# ament_target_dependencies(minimal_publisher_component rclcpp std_msgs sensor_msgs PCL rclcpp_components)
# rclcpp_components_register_nodes(minimal_publisher_component "PointCloud2Subscriber")

# add_library(minimal_subscriber_component SHARED src/minimal_subscriber.cpp)
# ament_target_dependencies(minimal_subscriber_component rclcpp std_msgs sensor_msgs PCL rclcpp_components)
# rclcpp_components_register_nodes(minimal_subscriber_component "PointCloudPublisher")



# add_executable(minimal_publisher_icp src/minimal_publisher.cpp)
# ament_target_dependencies(minimal_publisher_icp rclcpp std_msgs sensor_msgs PCL rclcpp_components)
# rclcpp_components_register_nodes(minimal_publisher_component "PointCloudPublisher")


# add_executable(minimal_subscriber_icp src/minimal_subscriber.cpp)
# ament_target_dependencies(minimal_subscriber_icp rclcpp std_msgs sensor_msgs PCL rclcpp_components)
# rclcpp_components_register_nodes(minimal_subscriber_component "MinimalSubscriber")

# target_link_libraries(minimal_publisher_component minimal_subscriber_component)


# add_executable(integrated_pub_sub src/integrated_pub_sub.cpp)
# ament_target_dependencies(integrated_pub_sub rclcpp std_msgs sensor_msgs PCL rclcpp_components)


# install(DIRECTORY
#   launch
#   param
#   DESTINATION share/${PROJECT_NAME}
# )

# install(
#   TARGETS minimal_publisher_icp
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(
#   TARGETS minimal_subscriber_icp
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(
#   TARGETS
#   integrated_pub_sub
#   DESTINATION lib/${PROJECT_NAME}
# )




# Create shared libraries for components
add_library(pointcloud_publisher_component SHARED src/minimal_publisher.cpp)
ament_target_dependencies(pointcloud_publisher_component rclcpp std_msgs sensor_msgs PCL rclcpp_components)
rclcpp_components_register_nodes(pointcloud_publisher_component "my_cpp_package::PointCloudPublisher")

add_library(pointcloud_subscriber_component SHARED src/minimal_subscriber.cpp)
ament_target_dependencies(pointcloud_subscriber_component rclcpp std_msgs sensor_msgs PCL rclcpp_components)
rclcpp_components_register_nodes(pointcloud_subscriber_component "my_cpp_package::PointCloud2Subscriber")

# Create executable for component manager
add_executable(component_manager src/integrated_pub_sub.cpp)
ament_target_dependencies(component_manager rclcpp rclcpp_components)
target_link_libraries(component_manager pointcloud_publisher_component pointcloud_subscriber_component)

install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    pointcloud_publisher_component
    pointcloud_subscriber_component
    component_manager
  DESTINATION ${PROJECT_NAME}/${PROJECT_NAME}/lib/
)



ament_package()



