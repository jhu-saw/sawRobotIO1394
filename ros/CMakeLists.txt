#
# (C) Copyright 2017-2022 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.1)
project (robot_io VERSION 2.1.0)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

## find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin REQUIRED
              COMPONENTS
              cisst_ros_crtk
              roscpp
              roslib
              )

# find cisst and make sure the required libraries have been compiled
set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonQt
     cisstVector
     cisstVectorQt
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstParameterTypesQt
     cisstQt)

find_package (cisst 1.1.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  include (${CISST_USE_FILE})

  # include_directories(include)
  include_directories (${catkin_INCLUDE_DIRS})

  file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")

  catkin_package (INCLUDE_DIRS "${CATKIN_DEVEL_PREFIX}/include"
                  CATKIN_DEPENDS cisst_ros_crtk roscpp roslib)

  # sawRobotIO1394 has been compiled within cisst, we should find it automatically
  find_package (sawRobotIO1394 2.1.0)

  if (sawRobotIO1394_FOUND)

    include_directories (${CATKIN_DEVEL_PREFIX}/include
                         ${sawRobotIO1394_INCLUDE_DIR})

    link_directories (${sawRobotIO1394_LIBRARY_DIR})

    add_executable (robot_io_console
      src/mts_ros_crtk_robot_io_bridge.h
      src/mts_ros_crtk_robot_io_bridge.cpp
      src/robot_io_console.cpp)
    target_link_libraries (robot_io_console
                           ${sawRobotIO1394_LIBRARIES}
                           ${catkin_LIBRARIES})
    cisst_target_link_libraries (robot_io_console ${REQUIRED_CISST_LIBRARIES})

  else (sawRobotIO1394_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires sawRobotIO1394")
  endif (sawRobotIO1394_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
