#
# (C) Copyright 2013-2026 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.16)
project (sawRobotIO1394ExamplesDataPlot VERSION 2.5.0)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstVectorQt
  cisstNumerical
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED COMPONENTS ${REQUIRED_CISST_LIBRARIES})

# catkin/ROS paths
cisst_set_output_path ()

# sawRobotIO1394 has been compiled within cisst, we should find it automatically
find_package (sawRobotIO1394 REQUIRED)

if (sawRobotIO1394_FOUND)

  # sawRobotIO1394 configuration
  include_directories (${sawRobotIO1394_INCLUDE_DIR})
  link_directories (${sawRobotIO1394_LIBRARY_DIR})

  include_directories (${CMAKE_CURRENT_BINARY_DIR})
  set (CMAKE_AUTOMOC ON)

  add_executable (sawRobotIO1394QtDataPlot
                  main.cpp
                  plotObject.h
                  plotObject.cpp)
  set_property (TARGET sawRobotIO1394QtDataPlot PROPERTY FOLDER "sawRobotIO1394")

  # link against non cisst libraries and cisst components
  target_link_libraries (sawRobotIO1394QtDataPlot PRIVATE
                         ${sawRobotIO1394_LIBRARIES}
                         ${cisst_LIBRARIES})

endif (sawRobotIO1394_FOUND)
