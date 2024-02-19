#
# (C) Copyright 2013-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawRobotIO1394ExamplesXMLtoJSON VERSION 2.2.1)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstCommonXML
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  # sawRobotIO1394 has been compiled within cisst, we should find it automatically
  find_package (sawRobotIO1394 REQUIRED)

  if (sawRobotIO1394_FOUND)

    # sawRobotIO1394 configuration
    include_directories (${sawRobotIO1394_INCLUDE_DIR})
    link_directories (${sawRobotIO1394_LIBRARY_DIR})

    add_executable (sawRobotIO1394XMLtoJSON main.cpp)
    set_target_properties (sawRobotIO1394XMLtoJSON PROPERTIES
                           COMPONENT sawRobotIO1394-Applications
                           FOLDER "sawRobotIO1394")

    # link against non cisst libraries and cisst components
    target_link_libraries (sawRobotIO1394XMLtoJSON
                           ${sawRobotIO1394_LIBRARIES})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawRobotIO1394XMLtoJSON ${REQUIRED_CISST_LIBRARIES})

    install (TARGETS sawRobotIO1394XMLtoJSON
      COMPONENT sawRobotIO1394-Applications
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib)

  endif (sawRobotIO1394_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
