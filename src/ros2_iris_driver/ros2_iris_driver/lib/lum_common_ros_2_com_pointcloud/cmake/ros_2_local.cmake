find_package(sensor_msgs REQUIRED)

target_include_directories(lum_common_ros_2_com_pointcloud::lum_common_ros_2_com_pointcloud
                           INTERFACE ${sensor_msgs_INCLUDE_DIRS}
)

target_link_directories(lum_common_ros_2_com_pointcloud::lum_common_ros_2_com_pointcloud INTERFACE
                        /opt/ros/galactic/lib
)
