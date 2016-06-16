cmake_minimum_required(VERSION 2.8.3)
project(lsd_slam_viewer)

# Set default build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF(NOT CMAKE_BUILD_TYPE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  dynamic_reconfigure
  rosbag
  roslib
  sensor_msgs
  message_generation
  image_transport
  lsd_slam_msgs
)

## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)
include(cmake/FindQGLViewer.cmake)

# SSE flags
set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTXML TRUE)
set(CMAKE_CXX_FLAGS "${SSE_FLAGS} -std=c++11")

###################################
## catkin specific configuration ##
###################################

## Generate code for the dynamic reconfigure files
generate_dynamic_reconfigure_options(cfg/LsdViewerParams.cfg)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure rosbag roslib sensor_msgs image_transport lsd_slam_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDES} 
  ${EIGEN3_INCLUDE_DIR} 
  ${QGLVIEWER_INCLUDE_DIR}
  ${OpenGL_INCLUDE_DIRS}
  thirdparty/Sophus
)

set(SOURCES
  src/main_viewer.cpp
  src/PointCloudViewer.cpp
  src/KeyFrameDisplay.cpp
  src/KeyFrameGraphDisplay.cpp
  src/settings.cpp
)

## Declare a cpp executable
add_executable(viewer ${SOURCES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(viewer ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(viewer
  ${QT_LIBRARIES} 
  ${QGLVIEWER_LIBRARY}
  ${OPENGL_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS viewer
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lsd_slam_viewer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
