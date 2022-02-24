cmake_minimum_required(VERSION 2.8.3)

project(tensorflow_classifier)

include_directories(${catkin_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  rospy
  apriltags_ros
  cv_bridge
  std_msgs
  message_filters
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS src/cnn_classifier.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
