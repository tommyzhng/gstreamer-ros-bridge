cmake_minimum_required(VERSION 3.0.2)
project(gstreamer_ros_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  pluginlib
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

#find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0>=1.14)
pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.0>=1.14)


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs pluginlib nodelet sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/gstreamer_ros_bridge
  include/gstreamer_ros_bridge
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTAPP_INCLUDE_DIRS}   # include headers from gstapp
)

link_directories(
  ${GSTREAMER_LIBRARY_DIRS}
  ${GSTAPP_LIBRARY_DIRS}   # link gstapp library
  ${OpenCV_LIBRARY_DIRS}   
)

add_library(gstreamer_ros_bridge src/gstreamer_bridge.cpp src/gstreamer_publisher.cpp src/gstreamer_cam.cpp)
target_include_directories(gstreamer_ros_bridge PUBLIC include)
target_compile_features(gstreamer_ros_bridge PUBLIC cxx_std_17)

target_include_directories(gstreamer_ros_bridge PRIVATE ${GST_INCLUDE_DIRS})
target_link_libraries(gstreamer_ros_bridge ${GST_LIBRARIES} ${OpenCV_LIBRARIES} ${cv_bridge_LIBRARIES} ${catkin_LIBRARIES})

# node executables
# add_executable(gstreamer_cam_node src/gstreamer_cam_node.cpp)
# target_link_libraries(gstreamer_cam_node PUBLIC 
#   ${catkin_LIBRARIES} 
#   ${GSTREAMER_LIBRARIES} 
#   ${GSTAPP_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   gstreamer_ros_bridge)
# add_dependencies(gstreamer_cam_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(gstreamer_bridge_node src/gstreamer_bridge_node.cpp)
# target_link_libraries(gstreamer_bridge_node PUBLIC ${catkin_LIBRARIES} gstreamer_ros_bridge)
# add_dependencies(gstreamer_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(gstreamer_publisher_node src/gstreamer_publisher_node.cpp)
target_link_libraries(gstreamer_publisher_node PUBLIC ${catkin_LIBRARIES} gstreamer_ros_bridge)
add_dependencies(gstreamer_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
