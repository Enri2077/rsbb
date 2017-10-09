cmake_minimum_required(VERSION 2.8.3)
project(rockin_mocap)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

roslaunch_add_file_check(launch)
