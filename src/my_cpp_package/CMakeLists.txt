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

include_directories(include)

add_library(composition_nodes SHARED
            src/minimal_publisher.cpp
            src/minimal_subscriber.cpp)

target_compile_definitions(composition_nodes
          PRIVATE "MINIMAL_COMPOSITION_DLL")
ament_target_dependencies(composition_nodes rclcpp rclcpp_components std_msgs sensor_msgs PCL)


# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

# add_executable(pointcloud_publisher_component src/minimal_publisher.cpp)
# target_link_libraries(pointcloud_publisher_component composition_nodes)
# ament_target_dependencies(pointcloud_publisher_component
#   rclcpp)


# add_executable(pointcloud_subscriber_component src/minimal_subscriber.cpp)
# target_link_libraries(pointcloud_subscriber_component composition_nodes)
# ament_target_dependencies(pointcloud_subscriber_component
#   rclcpp)

add_executable(composition_composed src/composed.cpp)
target_link_libraries(composition_composed composition_nodes)
ament_target_dependencies(composition_composed rclcpp class_loader)

install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  composition_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  composition_composed
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# _______________________________________________________________________

# Create shared libraries for components
# add_library(pointcloud_publisher_component SHARED src/minimal_publisher.cpp)
# ament_target_dependencies(pointcloud_publisher_component rclcpp std_msgs sensor_msgs PCL rclcpp_components)
# rclcpp_components_register_nodes(pointcloud_publisher_component "my_cpp_package::PointCloudPublisher")


# add_library(pointcloud_subscriber_component SHARED src/minimal_subscriber.cpp)
# ament_target_dependencies(pointcloud_subscriber_component rclcpp std_msgs sensor_msgs PCL rclcpp_components)
# rclcpp_components_register_nodes(pointcloud_subscriber_component "my_cpp_package::PointCloud2Subscriber")


# # Create executable for component manager
# add_executable(component_manager src/integrated_pub_sub.cpp)
# ament_target_dependencies(component_manager rclcpp rclcpp_components)
# target_link_libraries(component_manager pointcloud_publisher_component pointcloud_subscriber_component)


# install(DIRECTORY
#   launch
#   param
#   DESTINATION share/${PROJECT_NAME}
# )

# install(
#   TARGETS
#     pointcloud_publisher_component
#     pointcloud_subscriber_component
#     component_manager
#   DESTINATION lib/${PROJECT_NAME}
# )



# ament_package()



