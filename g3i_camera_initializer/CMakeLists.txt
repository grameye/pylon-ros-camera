cmake_minimum_required(VERSION 3.0.2)
project(g3i_camera_initializer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  camera_control_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES g3i_camera_initializer
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/g3i_camera_initializer_node.cpp
)

add_executable(${PROJECT_NAME}_node src/g3i_camera_initializer_node.cpp src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

target_include_directories(
  ${PROJECT_NAME}_node
  PUBLIC
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest_gtest(
#     test_g3i_camera_initializer
#     test/g3i_camera_initializer.test
#     test/src/test_g3i_camera_initializer.cpp
#   )
#   target_link_libraries(test_g3i_camera_initializer ${catkin_LIBRARIES})
# endif()