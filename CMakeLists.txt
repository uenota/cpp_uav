cmake_minimum_required(VERSION 2.8.3)
project(cpp_uav)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(torres_etal_2016 src/torres_etal_2016.cpp)

target_link_libraries(torres_etal_2016 ${catkin_LIBRARIES})