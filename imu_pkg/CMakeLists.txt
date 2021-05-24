cmake_minimum_required(VERSION 3.0.2)
project(imu_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imu_pkg
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs 
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   include/imu.cpp
   include/imu.h
)

add_executable(imu_node src/imu_main.cpp)
add_dependencies(imu_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_node
   ${catkin_LIBRARIES} ${PROJECT_NAME}
)
