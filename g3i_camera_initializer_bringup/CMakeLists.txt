cmake_minimum_required(VERSION 3.0.2)
project(g3i_camera_initializer_bringup)

find_package(catkin REQUIRED)

find_package(
  catkin
)

catkin_package(
 LIBRARIES ${PROJECT_NAME}
)

# Add .launch
install(
    DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
