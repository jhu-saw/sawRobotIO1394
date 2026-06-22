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

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstVectorQt
     cisstNumerical
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstParameterTypesQt
)

# cisst
find_package (cisst REQUIRED COMPONENTS ${REQUIRED_CISST_LIBRARIES})

# for Qt generated header files
include_directories (${CMAKE_CURRENT_BINARY_DIR})
set (CMAKE_AUTOMOC ON)

add_library (sawRobotIO1394Qt
             ${sawRobotIO1394_HEADER_DIR}/sawRobotIO1394ExportQt.h
             ${sawRobotIO1394_HEADER_DIR}/mtsRobot1394QtWidget.h
             mtsRobot1394QtWidget.cpp
             ${sawRobotIO1394_HEADER_DIR}/mtsRobotIO1394QtWidgetFactory.h
             mtsRobotIO1394QtWidgetFactory.cpp)
set_target_properties (sawRobotIO1394Qt PROPERTIES
                       VERSION ${sawRobotIO1394_VERSION}
                       FOLDER "sawRobotIO1394")
target_link_libraries (sawRobotIO1394Qt PUBLIC
                       ${cisst_LIBRARIES}
                       ${sawRobotIO1394_LIBRARIES})

install (TARGETS sawRobotIO1394Qt COMPONENT sawRobotIO1394-Qt
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)
