#
# (C) Copyright 2012-2021 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required(VERSION 3.10)
project (sawRobotIO1394All VERSION 2.1.0)

if (UNIX)
  set (CPACK_PACKAGE_VENDOR "JHU")
  set (CPACK_DEBIAN_PACKAGE_MAINTAINER "anton.deguet@jhu.edu")
  set (CPACK_GENERATOR "DEB")
  set (CPACK_DEB_PACKAGE_COMPONENT ON)
  set (CPACK_DEB_COMPONENT_INSTALL ON)
  set (CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
  set (CPACK_DEBIAN_PACKAGE_GENERATE_SHLIBS ON)
endif (UNIX)

add_subdirectory (components)

# Examples of config files
add_subdirectory (share)

# Applications
set (sawRobotIO1394_DIR "${sawRobotIO1394All_BINARY_DIR}/components")
add_subdirectory (applications)

# Tests programs, will only get compiled if cisstTestsDriver has been compiled
add_subdirectory (tests)

include (CPack)
cpack_add_component (sawRobotIO1394)
cpack_add_component (sawRobotIO1394Applications
                     DEPENDS sawRobotIO1394)
