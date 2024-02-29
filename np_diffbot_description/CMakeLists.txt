cmake_minimum_required(VERSION 3.5)
project(np_diffbot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY diffbot/urdf diffbot/rviz
  DESTINATION share/${PROJECT_NAME}/np_diffbot_description
)

ament_package()
