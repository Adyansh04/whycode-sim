cmake_minimum_required(VERSION 3.16)
project(whycode_baselines LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)

add_executable(aruco_node src/aruco_node.cpp)
target_compile_options(aruco_node PRIVATE -O3 -march=native -Wall -Wextra)
target_link_libraries(aruco_node ${OpenCV_LIBS})
target_include_directories(aruco_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(aruco_node rclcpp sensor_msgs geometry_msgs tf2_msgs)

install(TARGETS aruco_node DESTINATION lib/${PROJECT_NAME})

ament_package()
