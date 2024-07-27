cmake_minimum_required(VERSION 3.5)
project(ros2_iris_driver)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")

# set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(ament_cmake REQUIRED)
find_package(lum_drivers_data_client REQUIRED)
find_package(lum_drivers_lidar_iris_data_preprocessor_a REQUIRED)
find_package(lum_platform_interface REQUIRED)
find_package(lum_common_ros_2_com_pointcloud_codecs REQUIRED)
find_package(lum_drivers_lidar_iris_codecs REQUIRED)
find_package(lum_drivers_lidar_iris_control_interface REQUIRED)
find_package(lum_drivers_lidar_iris_vsomeip REQUIRED)
find_package(lum_platform_signal REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(luminar_iris_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters commmon)
find_package(pcl_conversions REQUIRED)
find_package(backward_ros REQUIRED)
include_directories(
  include/
  ${YAML_CPP_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(iris_data_interface src/interface/iris_data_interface.cpp)
target_link_libraries(iris_data_interface
                      lum_common_ros_2_com_pointcloud_codecs::lum_common_ros_2_com_pointcloud_codecs
                      lum_drivers_data_client::lum_drivers_data_client
                      lum_drivers_lidar_iris_codecs::lum_drivers_lidar_iris_codecs
                      lum_drivers_lidar_iris_data_preprocessor_a::lum_drivers_lidar_iris_data_preprocessor_a
                      lum_platform_interface::lum_platform_interface
)
ament_target_dependencies(iris_data_interface yaml-cpp sensor_msgs pcl_conversions PCL)
add_library(iris_control_interface src/interface/iris_control_interface.cpp)
target_link_libraries(iris_control_interface
                      lum_drivers_lidar_iris_control_interface::lum_drivers_lidar_iris_control_interface
                      lum_drivers_lidar_iris_vsomeip::lum_drivers_lidar_iris_vsomeip
                      lum_platform_signal::lum_platform_signal
                      lum_platform_interface::lum_platform_interface
)
ament_target_dependencies(iris_control_interface yaml-cpp luminar_iris_msgs)
add_executable(iris_ros_driver src/iris_driver.cpp)
target_link_libraries(iris_ros_driver
                      iris_data_interface
                      iris_control_interface
                      lum_platform_interface::lum_platform_interface
                      yaml-cpp
)
ament_target_dependencies(iris_ros_driver yaml-cpp rclcpp sensor_msgs luminar_iris_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS iris_data_interface
  EXPORT export_iris_data_interface
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  TARGETS iris_control_interface
  EXPORT export_iris_control_interface
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_include_directories(include)
ament_export_libraries(iris_data_interface iris_control_interface)

set_target_properties(
  iris_data_interface
  iris_control_interface
  iris_ros_driver PROPERTIES
   INSTALL_RPATH_USE_LINK_PATH TRUE
   INSTALL_RPATH "\$ORIGIN"
)

install(TARGETS
  iris_ros_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  param
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  lib/lum_drivers_lidar_iris_data_preprocessor_a/lib/liblum_drivers_lidar_iris_data_preprocessor_a.so
  lib/lum_drivers_lidar_iris_control_interface/lib/liblum_drivers_lidar_iris_control_interface.so
  lib/vsomeip/lib/libvsomeip.so
  lib/vsomeip/lib/libvsomeip.so.2
  lib/vsomeip/lib/libvsomeip.so.2.99.99
  lib/vsomeip/lib/libvsomeip3-cfg.so
  lib/vsomeip/lib/libvsomeip3-cfg.so.3
  lib/vsomeip/lib/libvsomeip3-cfg.so.3.1.20
  lib/vsomeip/lib/libvsomeip3-e2e.so
  lib/vsomeip/lib/libvsomeip3-e2e.so.3
  lib/vsomeip/lib/libvsomeip3-e2e.so.3.1.20
  lib/vsomeip/lib/libvsomeip3-sd.so
  lib/vsomeip/lib/libvsomeip3-sd.so.3
  lib/vsomeip/lib/libvsomeip3-sd.so.3.1.20
  lib/vsomeip/lib/libvsomeip3.so
  lib/vsomeip/lib/libvsomeip3.so.3
  lib/vsomeip/lib/libvsomeip3.so.3.1.20
  DESTINATION lib/ #${PROJECT_NAME}
)
ament_package()
