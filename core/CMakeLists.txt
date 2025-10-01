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

cmake_minimum_required(VERSION 3.16)
project (sawRobotIO1394Core VERSION 2.3.0)

find_package (cisst REQUIRED)
include (${CISST_USE_FILE})
cisst_cpack_settings (
  VENDOR "JHU"
  MAINTAINER "anton.deguet@jhu.edu")

add_subdirectory (components)

# Examples of config files
add_subdirectory (share)

# Applications
set (sawRobotIO1394_DIR "${sawRobotIO1394Core_BINARY_DIR}/components")
add_subdirectory (applications)

# Tests programs, will only get compiled if cisstTestsDriver has been compiled
# add_subdirectory (tests)

include (CPack)
cpack_add_component (sawRobotIO1394)
cpack_add_component (sawRobotIO1394-Qt
                     DEPENDS sawRobotIO1394)
cpack_add_component (sawRobotIO1394-Applications
                     DEPENDS sawRobotIO1394 sawRobotIO1394-Qt)
