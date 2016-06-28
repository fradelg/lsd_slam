cmake_minimum_required(VERSION 3.1)
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

# Find catkin macros and libraries
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

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)

# Find the required Qt modules
find_package(Qt5Widgets)
find_package(Qt5OpenGL)
find_package(Qt5Xml)

include(cmake/FindQGLViewer.cmake)
message(STATUS "Found QGLViewer library at ${QGLVIEWER_LIBRARY}")

# Check and set SSE flags
include(cmake/FindSSE.cmake)
FindSSE()
if(SSE3_FOUND)
  if(SSSE3_FOUND)
    set(CXX_DFLAGS -msse3 -mssse3)
  endif(SSSE3_FOUND)
endif(SSE3_FOUND)

if(SSE4_2_FOUND)
  set(CXX_DFLAGS -msse4.2 -mpopcnt)
endif(SSE4_2_FOUND)
add_definitions(${CXX_DFLAGS})
message(STATUS "Found SSE flags: ${CXX_DFLAGS}")

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

###################################
## catkin specific configuration ##
###################################

# Generate code for the dynamic reconfigure files
generate_dynamic_reconfigure_options(cfg/LsdViewerParams.cfg)

# The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure rosbag roslib sensor_msgs image_transport lsd_slam_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${QGLVIEWER_INCLUDE_DIR}
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
  ${catkin_LIBRARIES}
  ${OPENGL_gl_LIBRARY}
  ${QGLVIEWER_LIBRARY}
  Qt5::Widgets
  Qt5::OpenGL
  Qt5::Xml
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables and/or libraries for installation
install(TARGETS viewer
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
