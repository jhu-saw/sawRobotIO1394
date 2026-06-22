#
# (C) Copyright 2009-2026 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.16)
project (sawRobotIO1394QtConsole VERSION 2.4.0)

# create a list of required cisst libraries
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
)

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

  add_executable (sawRobotIO1394QtConsole mainQt.cpp)
  set_target_properties (sawRobotIO1394QtConsole PROPERTIES
                         COMPONENT sawRobotIO1394-Applications
                         FOLDER "sawRobotIO1394")

  # link against non cisst libraries and cisst components
  target_link_libraries (sawRobotIO1394QtConsole PRIVATE
                         ${sawRobotIO1394_LIBRARIES}
                         ${cisst_LIBRARIES})

  install (TARGETS sawRobotIO1394QtConsole
    COMPONENT sawRobotIO1394-Applications
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

endif (sawRobotIO1394_FOUND)
