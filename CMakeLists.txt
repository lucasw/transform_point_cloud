cmake_minimum_required(VERSION 2.8.3)
project(transform_point_cloud)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslint
  tf2_py
  tf2_ros
  tf2_sensor_msgs
  sensor_msgs
)

roslint_cpp()

catkin_package(
  CATKIN_DEPENDS roscpp tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(transform_point_cloud
  src/transform_point_cloud)

target_link_libraries(transform_point_cloud ${catkin_LIBRARIES})
