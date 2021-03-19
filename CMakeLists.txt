cmake_minimum_required(VERSION 3.0.2)
project(wreadinput)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
    wready
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

# Generate messages in the 'msg' folder
#add_message_files(
  #FILES
  #Int16Pair.msg
#)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
