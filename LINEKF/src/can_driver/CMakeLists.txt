cmake_minimum_required(VERSION 2.8.3)
project(can_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    
    std_msgs
    roscpp
    rospy
    message_generation
)




add_message_files(
  FILES
  steerWheel_angle.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES can_driver
    CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
${catkin_INCLUDE_DIRS}
)
