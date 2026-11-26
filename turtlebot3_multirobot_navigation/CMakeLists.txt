cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_multirobot_navigation)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch maps multi_param rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
