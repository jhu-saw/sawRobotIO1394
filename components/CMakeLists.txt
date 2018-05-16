#
# (C) Copyright 2012-2018 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8)

project (sawRobotIO1394)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonXML
     cisstVector
     cisstNumerical
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes)

find_package (cisst 1.0.10 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # Modify CMake configuration to use cisst
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawRobotIO1394_IS_CATKIN_BUILT)
  if (sawRobotIO1394_IS_CATKIN_BUILT)
    set (sawRobotIO1394_CONFIG_FILE_DIR
         "${CATKIN_DEVEL_PREFIX}/share/sawRobotIO1394/cmake")
    set (LIBRARY_OUTPUT_PATH "${sawRobotIO1394_BINARY_DIR}/lib")
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  else ()
    set (sawRobotIO1394_CONFIG_FILE_DIR "${sawRobotIO1394_BINARY_DIR}")
  endif ()

  # Compile lib Amp1394 and examples within this project
  add_subdirectory (code/Amp1394)
  set (CMAKE_PREFIX_PATH "${sawRobotIO1394_BINARY_DIR}/code/Amp1394" ${CMAKE_PREFIX_PATH})
  find_package(Amp1394 REQUIRED)

  # Allow c++ code to find local header files
  include_directories (${sawRobotIO1394_SOURCE_DIR})

  # create/configure file for find_package (sawRobotIO1394)
  set (sawRobotIO1394_INCLUDE_DIR "${sawRobotIO1394_SOURCE_DIR}/include"
                                  "${sawRobotIO1394_BINARY_DIR}/include"
                                  ${Amp1394_INCLUDE_DIR})
  set (sawRobotIO1394_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}" ${Amp1394_LIBRARY_DIR})
  set (sawRobotIO1394_LIBRARIES sawRobotIO1394 ${Amp1394_LIBRARIES})

  # Set the version number
  set (sawRobotIO1394_VERSION_MAJOR "1")
  set (sawRobotIO1394_VERSION_MINOR "6")
  set (sawRobotIO1394_VERSION_PATCH "0")
  set (sawRobotIO1394_VERSION "${sawRobotIO1394_VERSION_MAJOR}.${sawRobotIO1394_VERSION_MINOR}.${sawRobotIO1394_VERSION_PATCH}")

  # Generate sawRobotIO1394Revision.h
  configure_file ("${sawRobotIO1394_SOURCE_DIR}/code/sawRobotIO1394Revision.h.in"
                  "${sawRobotIO1394_BINARY_DIR}/include/sawRobotIO1394/sawRobotIO1394Revision.h")

  include_directories (${sawRobotIO1394_INCLUDE_DIR})
  set (sawRobotIO1394_HEADER_DIR "${sawRobotIO1394_SOURCE_DIR}/include/sawRobotIO1394")
  link_directories (${Amp1394_LIBRARY_DIR})

  # create the library
  add_library (sawRobotIO1394
               ${sawRobotIO1394_HEADER_DIR}/osaConfiguration1394.h
               ${sawRobotIO1394_HEADER_DIR}/osaXML1394.h
               code/osaXML1394.cpp
               ${sawRobotIO1394_HEADER_DIR}/sawRobotIO1394Export.h
               ${sawRobotIO1394_HEADER_DIR}/mtsRobot1394.h
               ${sawRobotIO1394_HEADER_DIR}/mtsDigitalInput1394.h
               ${sawRobotIO1394_HEADER_DIR}/mtsDigitalOutput1394.h
               ${sawRobotIO1394_HEADER_DIR}/mtsRobotIO1394.h
               code/mtsRobot1394.cpp
               code/mtsDigitalInput1394.cpp
               code/mtsDigitalOutput1394.cpp
               code/mtsRobotIO1394.cpp)

  set_property (TARGET sawRobotIO1394 PROPERTY FOLDER "sawRobotIO1394")
  target_link_libraries (sawRobotIO1394 Amp1394)

  # link rtai lib (may need to add Xenomai support)
  if (CISST_HAS_LINUX_RTAI)
    target_link_libraries (sawRobotIO1394 ${RTAI_LIBRARIES})
  endif (CISST_HAS_LINUX_RTAI)

  # link cisst lib
  cisst_target_link_libraries (sawRobotIO1394 ${REQUIRED_CISST_LIBRARIES})

  if (CISST_HAS_QT)
    # add Qt code
    add_subdirectory (code/Qt)
    set (sawRobotIO1394_LIBRARIES ${sawRobotIO1394_LIBRARIES} ${sawRobotIO1394Qt_LIBRARIES})
  endif (CISST_HAS_QT)

  set (sawRobotIO1394_CMAKE_CONFIG_FILE
       "${sawRobotIO1394_CONFIG_FILE_DIR}/sawRobotIO1394Config.cmake")

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawRobotIO1394Config.cmake.in"
                  "${sawRobotIO1394_CMAKE_CONFIG_FILE}"
                   @ONLY)

  cisst_add_config_version ("${sawRobotIO1394_CONFIG_FILE_DIR}/sawRobotIO1394ConfigVersion.cmake"
                            VERSION ${sawRobotIO1394_VERSION}
                            DESTINATION ${CISST_CMAKE_INSTALL_SUFFIX}/saw
                            COMPONENT sawRobotIO1394)

  # Install targets for headers and library
  install (DIRECTORY
           ${sawRobotIO1394_SOURCE_DIR}/include/sawRobotIO1394
           ${sawRobotIO1394_BINARY_DIR}/include/sawRobotIO1394
           DESTINATION include
           PATTERN .svn EXCLUDE)

  install (TARGETS sawRobotIO1394
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

  install (FILES "${sawRobotIO1394_CMAKE_CONFIG_FILE}"
           DESTINATION "share/sawRobotIO1394")

endif(cisst_FOUND_AS_REQUIRED)
