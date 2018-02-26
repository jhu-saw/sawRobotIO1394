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

# Qt based example, CMake will look for cisstQt
add_subdirectory (Qt)

# utility to calibrate current 0 based on current feedback
add_subdirectory (current-calibration)

# utility to plot velocities
add_subdirectory (data-plot)

# utility to collect data
add_subdirectory (data-collection)
