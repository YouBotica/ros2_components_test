cmake_minimum_required(VERSION 3.8)
project(icp_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage_default_plugins REQUIRED)

# add include directories
include_directories(include)

# Add ipc_publisher library
add_library(ipc_publisher src/ipc_publisher.cpp)
ament_target_dependencies(ipc_publisher rclcpp std_msgs)
target_include_directories(ipc_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add ipc_subscriber library
add_library(ipc_subscriber src/ipc_subscriber.cpp)
ament_target_dependencies(ipc_subscriber rclcpp std_msgs)
target_include_directories(ipc_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(ipc_recorder src/ipc_recorder.cpp)
ament_target_dependencies(ipc_recorder rclcpp std_msgs rosbag2_cpp rosbag2_storage_default_plugins)
target_include_directories(ipc_recorder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Create executable for component manager
add_executable(ipc_main src/ipc_main.cpp)
ament_target_dependencies(ipc_main rclcpp std_msgs)
target_include_directories(ipc_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ipc_main ipc_publisher ipc_subscriber ipc_recorder)

install(
  TARGETS
    ipc_publisher
    ipc_subscriber
    ipc_recorder
    ipc_main
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()



# cmake_minimum_required(VERSION 3.8)
# project(icp_test)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)

# # add include directories
# include_directories(include)

# # Add ipc_publisher executable
# add_library(ipc_publisher src/ipc_publisher.cpp)
# ament_target_dependencies(ipc_publisher rclcpp std_msgs)
# # rclcpp_components_register_nodes(pointcloud_publisher_component "my_cpp_package::PointCloudPublisher")

# # Add ipc_subscriber executable
# add_library(ipc_subscriber src/ipc_subscriber.cpp)
# # target_include_directories(ipc_subscriber PUBLIC include)
# ament_target_dependencies(ipc_subscriber rclcpp std_msgs)
# # rclcpp_components_register_nodes(pointcloud_subscriber_component "my_cpp_package::PointCloudSubscriber")

# # Create executable for component manager
# add_executable(ipc_main src/ipc_main.cpp)
# ament_target_dependencies(ipc_main ipc_publisher ipc_subscriber rclcpp std_msgs)
# # target_link_libraries(ipc_main ipc_publisher ipc_subscriber)

# install(
#   TARGETS
#     ipc_publisher
#     ipc_subscriber
#     ipc_main
#   DESTINATION lib/${PROJECT_NAME}
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()


# ament_package()

