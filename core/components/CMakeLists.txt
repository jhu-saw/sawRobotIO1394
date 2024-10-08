#
# (C) Copyright 2012-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawRobotIO1394 VERSION 2.3.0)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonXML
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes)

find_package (cisst 1.3.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # Modify CMake configuration to use cisst
  include (${CISST_USE_FILE})

  # make sure CISST_USE_SI_UNITS is defined and set to 1
  if (NOT DEFINED CISST_USE_SI_UNITS)
    message (FATAL_ERROR "You need to upgrade to a version of cisst that defines CISST_USE_SI_UNITS (and set CISST_USE_SI_UNITS to ON)")
  endif ()
  if (NOT CISST_USE_SI_UNITS)
    message (FATAL_ERROR "You need to make sure CISST_USE_SI_UNITS to ON")
  endif ()

  # catkin/ROS paths
  cisst_set_output_path ()

  # Compile lib Amp1394 and examples within this project
  set (LIBRARY_OUTPUT_PATH "${CISST_BINARY_DIR}/lib") # for Python wrappers
  set (Amp1394_BUILD_SWIG ${CISST_HAS_SWIG_PYTHON} CACHE BOOL "" FORCE)
  add_subdirectory (code/Amp1394)
  set (CMAKE_PREFIX_PATH "${sawRobotIO1394_BINARY_DIR}/code/Amp1394" ${CMAKE_PREFIX_PATH})
  find_package (Amp1394 REQUIRED)

  # reset catkin/ROS paths
  cisst_set_output_path ()

  # Allow c++ code to find local header files
  include_directories (BEFORE ${sawRobotIO1394_SOURCE_DIR})

  # create/configure file for find_package (sawRobotIO1394)
  set (sawRobotIO1394_INCLUDE_DIR "${sawRobotIO1394_SOURCE_DIR}/include"
                                  "${sawRobotIO1394_BINARY_DIR}/include"
                                  ${Amp1394_INCLUDE_DIR})
  set (sawRobotIO1394_HEADER_DIR "${sawRobotIO1394_SOURCE_DIR}/include/sawRobotIO1394")
  set (sawRobotIO1394_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}" ${Amp1394_LIBRARY_DIR})
  set (sawRobotIO1394_LIBRARIES sawRobotIO1394 ${Amp1394_LIBRARIES})

  include_directories (BEFORE ${sawRobotIO1394_INCLUDE_DIR})
  link_directories (${Amp1394_LIBRARY_DIR})

  if (CISST_HAS_QT)
    add_subdirectory (code/Qt)
    set (sawRobotIO1394_LIBRARIES ${sawRobotIO1394_LIBRARIES} sawRobotIO1394Qt)
  endif ()

  # add all config files for this component
  cisst_add_config_files (sawRobotIO1394)

  # create data type using the data generator
  cisst_data_generator (sawRobotIO1394
                        "${sawRobotIO1394_BINARY_DIR}/include" # where to save the file
                        "sawRobotIO1394/"    # sub directory for include
                        code/osaConfiguration1394.cdg)

  # create the library
  add_library (sawRobotIO1394
               ${sawRobotIO1394_HEADER_DIR}/osaXML1394.h
               ${sawRobotIO1394_HEADER_DIR}/sawRobotIO1394Export.h
               ${sawRobotIO1394_HEADER_DIR}/mtsRobot1394.h
               ${sawRobotIO1394_HEADER_DIR}/mtsDigitalInput1394.h
               ${sawRobotIO1394_HEADER_DIR}/mtsDigitalOutput1394.h
               ${sawRobotIO1394_HEADER_DIR}/mtsDallasChip1394.h
               ${sawRobotIO1394_HEADER_DIR}/mtsRobotIO1394.h
               code/osaXML1394.cpp
               code/mtsRobot1394.cpp
               code/mtsDigitalInput1394.cpp
               code/mtsDigitalOutput1394.cpp
               code/mtsDallasChip1394.cpp
               code/mtsRobotIO1394.cpp
               ${sawRobotIO1394_CISST_DG_SRCS}
               ${sawRobotIO1394_CISST_DG_HDRS})

  set_target_properties (sawRobotIO1394 PROPERTIES
                         VERSION ${sawRobotIO1394_VERSION}
                         FOLDER "sawRobotIO1394")

  target_link_libraries (sawRobotIO1394 Amp1394)

  # link rtai lib (may need to add Xenomai support)
  if (CISST_HAS_LINUX_RTAI)
    target_link_libraries (sawRobotIO1394 ${RTAI_LIBRARIES})
  endif (CISST_HAS_LINUX_RTAI)

  # link cisst lib
  cisst_target_link_libraries (sawRobotIO1394 ${REQUIRED_CISST_LIBRARIES})

  # Install targets for headers and library
  install (DIRECTORY "${sawRobotIO1394_SOURCE_DIR}/include/sawRobotIO1394"
                     "${sawRobotIO1394_BINARY_DIR}/include/sawRobotIO1394"
           DESTINATION include
           COMPONENT sawRobotIO1394-dev
           PATTERN .svn EXCLUDE)

  install (TARGETS sawRobotIO1394 COMPONENT sawRobotIO1394
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

endif(cisst_FOUND_AS_REQUIRED)
