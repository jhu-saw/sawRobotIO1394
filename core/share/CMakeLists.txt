#
# (C) Copyright 2013-2026 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.5 REQUIRED COMPONENTS cisstCommon)

set (FILES_TO_COPY saw-robot-io.schema.json saw-robot-io-suj-si.schema.json)

install (FILES ${FILES_TO_COPY}
         DESTINATION share/sawRobotIO1394)
