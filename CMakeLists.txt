cmake_minimum_required(VERSION 3.8)
project(gstreamer_ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)


add_library(gstreamer_ros_bridge SHARED
  src/gstreamer_bridge.cpp
  #src/gstreamer_publisher.cpp
  src/gstreamer_cam.cpp
)

# link dependencies
ament_target_dependencies(gstreamer_ros_bridge
  PUBLIC
    rclcpp
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
    rclcpp_components
    pluginlib
    std_srvs
)

#find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0>=1.14)
pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.0>=1.14)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0>=1.14)

target_include_directories(gstreamer_ros_bridge
  PUBLIC
    include
    include/gstreamer-ros-bridge
    ${OpenCV_INCLUDE_DIRS}
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_APP_INCLUDE_DIRS}
  PRIVATE
    ${GST_INCLUDE_DIRS}
)

target_compile_features(gstreamer_ros_bridge
  PUBLIC
    cxx_std_17
)

target_link_libraries(gstreamer_ros_bridge
  PUBLIC
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_APP_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

rclcpp_components_register_nodes(gstreamer_ros_bridge 
  "gstreamer_ros_bridge::GStreamerCam"
  "gstreamer_ros_bridge::GStreamerBridge"
)

# Install library
install(TARGETS gstreamer_ros_bridge
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

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

ament_package()
