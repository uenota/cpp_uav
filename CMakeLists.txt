cmake_minimum_required(VERSION 2.8.3)
project(cpp_uav)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  Torres16.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Executables
add_executable(torres_etal_2016 src/torres_etal_2016.cpp)

add_dependencies(torres_etal_2016 ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(torres_etal_2016 ${catkin_LIBRARIES})

# Tests
catkin_add_gtest(test_torres_etal_2016 test/test_torres_etal_2016.cpp)
target_link_libraries(test_torres_etal_2016 ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  #add_rostest_gtest(tests_mynode test/mynode.test src/test/test_mynode.cpp [more cpp files])
  #target_link_libraries(tests_mynode ${catkin_LIBRARIES})
endif()