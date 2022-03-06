cmake_minimum_required(VERSION 2.8.3)
project(propbot_control)

find_package(catkin REQUIRED COMPONENTS
controller_manager
roscpp
diff_drive_controller
hardware_interface
roslaunch)

catkin_package()

roslaunch_add_file_check(launch)

add_executable(${PROJECT_NAME}_node 
  src/propbot_base.cpp
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
