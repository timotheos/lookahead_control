cmake_minimum_required(VERSION 2.8.3)
project(lookahead_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  nav_msgs
  geometry_msgs
  cmake_modules
)
find_package(Eigen REQUIRED)

catkin_package(
  DEPENDS Eigen
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include
)
add_executable(lookahead_control src/lookahead_control.cpp)
add_dependencies(lookahead_control nav_msgs_generate_messages_cpp)

target_link_libraries(lookahead_control 
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lookahead_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
