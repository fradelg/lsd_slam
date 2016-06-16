cmake_minimum_required(VERSION 3.0)
project(lsd_slam_core)

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
  sensor_msgs
  image_transport
  roscpp
)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(X11 REQUIRED)
include(cmake/FindG2O.cmake)
include(cmake/FindSuiteParse.cmake)

message("-- CHOLMOD_INCLUDE_DIR : " ${CHOLMOD_INCLUDE_DIR})
message("-- CSPARSE_INCLUDE_DIR : " ${CSPARSE_INCLUDE_DIR})
message("-- G2O_INCLUDE_DIR : " ${G2O_INCLUDE_DIR})

# FabMap
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
#include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
#add_definitions("-DHAVE_FABMAP")
#set(FABMAP_LIB openFABMAP )

## Generate code for the dynamic reconfigure files
generate_dynamic_reconfigure_options(
  cfg/LSDDebugParams.cfg
  cfg/LSDParams.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES lsdslam
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure lsd_slam_viewer roslib sensor_msgs
  DEPENDS system_lib Eigen3 SuiteSparse g2o
)

###########
## Build ##
###########

# SSE flags
add_definitions("-DUSE_ROS")
add_definitions("-DENABLE_SSE")

# Also add some useful compiler flag
set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} ${SSE_FLAGS} -march=native  -std=c++11"
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
  ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
  ${PROJECT_SOURCE_DIR}/src/
)

# Set source files
set(SOURCE_FILES
  src/DataStructures/Frame.cpp
  src/DataStructures/FramePoseStruct.cpp
  src/DataStructures/FrameMemory.cpp
  src/SlamSystem.cpp
  src/LiveSLAMWrapper.cpp
  src/DepthEstimation/DepthMap.cpp
  src/DepthEstimation/DepthMapPixelHypothesis.cpp
  src/util/globalFuncs.cpp
  src/util/SophusUtil.cpp
  src/util/settings.cpp
  src/util/Undistorter.cpp
  src/Tracking/Sim3Tracker.cpp
  src/Tracking/Relocalizer.cpp
  src/Tracking/SE3Tracker.cpp
  src/Tracking/TrackingReference.cpp
  src/GlobalMapping/FabMap.cpp
  src/GlobalMapping/KeyFrameGraph.cpp
  src/GlobalMapping/g2oTypeSim3Sophus.cpp
  src/GlobalMapping/TrackableKeyFrameSearch.cpp
  src/IOWrapper/Timestamp.cpp
  src/IOWrapper/ROS/ROSImageStreamThread.cpp
  src/IOWrapper/ROS/ROSOutput3DWrapper.cpp
  src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
)

## Build the shared library
add_library(lsd_slam SHARED ${SOURCE_FILES})

## Declare cpp executables
add_executable(live_slam src/main_live_odometry.cpp)
add_executable(dataset src/main_on_images.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(lsd_slam ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(live_slam ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(dataset ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(lsd_slam
  ${FABMAP_LIB}
  ${G2O_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CSPARSE_LIBRARY}
)

target_link_libraries(live_slam lsd_slam
  ${catkin_LIBRARIES}
  ${G2O_LIBRARIES}
  ${X11_LIBRARIES}
)

target_link_libraries(dataset lsd_slam
  ${catkin_LIBRARIES}
  ${G2O_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS lsd_slam live_slam dataset
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  calib/OpenCV_example_calib.cfg
  calib/FOV_example_calib.cfg
  launch/lsd_slam.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
