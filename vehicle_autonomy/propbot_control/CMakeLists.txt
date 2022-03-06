cmake_minimum_required(VERSION 2.8.3)
project(propbot_control)

find_package(catkin REQUIRED COMPONENTS
controller_manager
roscpp
diff_drive_controller
hardware_interface
roslib
urdf
roslaunch)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

roslaunch_add_file_check(launch)

add_executable(propbot_base 
  src/propbot_base.cpp
)

target_link_libraries(propbot_base PUBLIC ${catkin_LIBRARIES})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
