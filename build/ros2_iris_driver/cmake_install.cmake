# Install script for directory: /home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/artificial/bags_ws/install/ros2_iris_driver")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/artificial/bags_ws/build/ros2_iris_driver/libiris_data_interface.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/artificial/bags_ws/build/ros2_iris_driver/libiris_control_interface.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_iris_driver/environment" TYPE FILE FILES "/opt/ros/humble/lib/python3.10/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_iris_driver/environment" TYPE FILE FILES "/home/artificial/bags_ws/build/ros2_iris_driver/ament_cmake_environment_hooks/library_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_iris_driver/iris_ros_driver" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_iris_driver/iris_ros_driver")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_iris_driver/iris_ros_driver"
         RPATH "$ORIGIN:/opt/ros/galactic/lib:/opt/ros/humble/lib:/home/artificial/bags_ws/install/luminar_iris_msgs/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ros2_iris_driver" TYPE EXECUTABLE FILES "/home/artificial/bags_ws/build/ros2_iris_driver/iris_ros_driver")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_iris_driver/iris_ros_driver" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_iris_driver/iris_ros_driver")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_iris_driver/iris_ros_driver"
         OLD_RPATH "/opt/ros/galactic/lib:/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/boost/lib:/opt/ros/humble/lib:/home/artificial/bags_ws/install/luminar_iris_msgs/lib:/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/lum_drivers_lidar_iris_data_preprocessor_a/lib:/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/lum_drivers_lidar_iris_control_interface/lib:/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/vsomeip/lib:"
         NEW_RPATH "$ORIGIN:/opt/ros/galactic/lib:/opt/ros/humble/lib:/home/artificial/bags_ws/install/luminar_iris_msgs/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_iris_driver/iris_ros_driver")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_iris_driver" TYPE DIRECTORY FILES "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/param")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE FILE FILES
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/lum_drivers_lidar_iris_data_preprocessor_a/lib/liblum_drivers_lidar_iris_data_preprocessor_a.so"
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/lum_drivers_lidar_iris_control_interface/lib/liblum_drivers_lidar_iris_control_interface.so"
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/vsomeip/lib/libvsomeip.so"
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/vsomeip/lib/libvsomeip.so.2"
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/vsomeip/lib/libvsomeip.so.2.99.99"
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/vsomeip/lib/libvsomeip3-cfg.so"
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/vsomeip/lib/libvsomeip3-cfg.so.3"
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/vsomeip/lib/libvsomeip3-cfg.so.3.1.20"
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/vsomeip/lib/libvsomeip3-e2e.so"
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/vsomeip/lib/libvsomeip3-e2e.so.3"
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/vsomeip/lib/libvsomeip3-e2e.so.3.1.20"
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/vsomeip/lib/libvsomeip3-sd.so"
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/vsomeip/lib/libvsomeip3-sd.so.3"
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/vsomeip/lib/libvsomeip3-sd.so.3.1.20"
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/vsomeip/lib/libvsomeip3.so"
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/vsomeip/lib/libvsomeip3.so.3"
    "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/lib/vsomeip/lib/libvsomeip3.so.3.1.20"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/artificial/bags_ws/build/ros2_iris_driver/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/ros2_iris_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/artificial/bags_ws/build/ros2_iris_driver/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/ros2_iris_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_iris_driver/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_iris_driver/environment" TYPE FILE FILES "/home/artificial/bags_ws/build/ros2_iris_driver/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_iris_driver/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_iris_driver/environment" TYPE FILE FILES "/home/artificial/bags_ws/build/ros2_iris_driver/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_iris_driver" TYPE FILE FILES "/home/artificial/bags_ws/build/ros2_iris_driver/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_iris_driver" TYPE FILE FILES "/home/artificial/bags_ws/build/ros2_iris_driver/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_iris_driver" TYPE FILE FILES "/home/artificial/bags_ws/build/ros2_iris_driver/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_iris_driver" TYPE FILE FILES "/home/artificial/bags_ws/build/ros2_iris_driver/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_iris_driver" TYPE FILE FILES "/home/artificial/bags_ws/build/ros2_iris_driver/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/artificial/bags_ws/build/ros2_iris_driver/ament_cmake_index/share/ament_index/resource_index/packages/ros2_iris_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_iris_driver/cmake" TYPE FILE FILES "/home/artificial/bags_ws/build/ros2_iris_driver/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_iris_driver/cmake" TYPE FILE FILES "/home/artificial/bags_ws/build/ros2_iris_driver/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_iris_driver/cmake" TYPE FILE FILES
    "/home/artificial/bags_ws/build/ros2_iris_driver/ament_cmake_core/ros2_iris_driverConfig.cmake"
    "/home/artificial/bags_ws/build/ros2_iris_driver/ament_cmake_core/ros2_iris_driverConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_iris_driver" TYPE FILE FILES "/home/artificial/bags_ws/src/ros2_iris_driver/ros2_iris_driver/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/artificial/bags_ws/build/ros2_iris_driver/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
