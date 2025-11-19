cmake_minimum_required(VERSION 3.0.2)
project(hsr_plane_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    tf2_ros
    tf2_geometry_msgs
    cv_bridge
    pcl_ros
    pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/plane_detection_node.py
  scripts/depth_processor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
