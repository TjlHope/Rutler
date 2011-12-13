cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# Add system openni include path
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src /usr/include/ni /usr/include/nite /usr/include/openni)
#find_package(GLUT)
#find_package(OpenGL)
#find_package(GL)
#find_package(NI)
find_package(OpenNI)
set(CMAKE_CXX_FLAGS "-I${PROJECT_SOURCE_DIR}/src -I/usr/include/ni -I/usr/include/GL")
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
rosbuild_add_library(${PROJECT_NAME} src/SceneDrawer.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(kinect_tracker src/main.cpp)
target_link_libraries(kinect_tracker ${PROJECT_NAME})
