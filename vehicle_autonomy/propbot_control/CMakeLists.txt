cmake_minimum_required(VERSION 2.8.3)
project(propbot_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
  roslaunch
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

roslaunch_add_file_check(launch)

add_executable(skid_steer_adjust 
  src/skid_steer_adjust.cc
)

target_link_libraries(skid_steer_adjust PUBLIC ${catkin_LIBRARIES})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
