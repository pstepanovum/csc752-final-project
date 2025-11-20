cmake_minimum_required(VERSION 3.0.2)
project(csc752-final-project)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/plane_detection_node.py
  scripts/depth_processor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES plane_detection.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
