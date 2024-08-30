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
project (sawRobotIO1394ExamplesDataPlot VERSION 2.2.1)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstCommonXML
  cisstVector
  cisstVectorQt
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
  cisst_set_output_path ()

  # sawRobotIO1394 has been compiled within cisst, we should find it automatically
  find_package (sawRobotIO1394 REQUIRED)

  if (sawRobotIO1394_FOUND)

    # sawRobotIO1394 configuration
    include_directories (${sawRobotIO1394_INCLUDE_DIR})
    link_directories (${sawRobotIO1394_LIBRARY_DIR})

    include_directories (${CMAKE_CURRENT_BINARY_DIR})
    if (CISST_HAS_QT4)
      qt4_wrap_cpp (QT_WRAPPED_CPP
                    plotObject.h)
    else (CISST_HAS_QT4)
      set (CMAKE_AUTOMOC ON)
    endif (CISST_HAS_QT4)

    add_executable (sawRobotIO1394QtDataPlot
                    main.cpp
                    plotObject.h
                    plotObject.cpp
                    ${QT_WRAPPED_CPP})
    set_property (TARGET sawRobotIO1394QtDataPlot PROPERTY FOLDER "sawRobotIO1394")

    # link against non cisst libraries and cisst components
    target_link_libraries (sawRobotIO1394QtDataPlot
                           ${sawRobotIO1394_LIBRARIES})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawRobotIO1394QtDataPlot ${REQUIRED_CISST_LIBRARIES})

  endif (sawRobotIO1394_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
