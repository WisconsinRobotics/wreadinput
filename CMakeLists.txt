cmake_minimum_required(VERSION 3.0.2)
project(wreadinput)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    wready
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS message_runtime
)
