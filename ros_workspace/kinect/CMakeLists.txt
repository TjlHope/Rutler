cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
rosbuild_genmsg()
rosbuild_add_library(${PROJECT_NAME} src/process_users.cpp)
rosbuild_add_executable(user_tracker src/user_tracker.cpp)
target_link_libraries(user_tracker ${PROJECT_NAME})
