cmake_minimum_required(VERSION 3.8)
project(whycode_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  worlds
  models
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


