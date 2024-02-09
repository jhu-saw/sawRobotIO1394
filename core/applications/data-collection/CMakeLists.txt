#
# (C) Copyright 2013-2016 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonXML
                              cisstVector
                              cisstNumerical
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawRobotIO1394ExamplesDataCollection_IS_CATKIN_BUILT)
  if (sawRobotIO1394ExamplesDataCollection_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  # sawRobotIO1394 has been compiled within cisst, we should find it automatically
  find_package (sawRobotIO1394 REQUIRED)

  if (sawRobotIO1394_FOUND)

    # sawRobotIO1394 configuration
    include_directories (${sawRobotIO1394_INCLUDE_DIR})
    link_directories (${sawRobotIO1394_LIBRARY_DIR})

    include_directories (${CMAKE_CURRENT_BINARY_DIR})

    add_executable (sawRobotIO1394DataCollection
                    main.cpp)
    set_property (TARGET sawRobotIO1394DataCollection PROPERTY FOLDER "sawRobotIO1394")

    # link against non cisst libraries and cisst components
    target_link_libraries (sawRobotIO1394DataCollection
                           ${sawRobotIO1394_LIBRARIES})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawRobotIO1394DataCollection ${REQUIRED_CISST_LIBRARIES})

  endif (sawRobotIO1394_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
