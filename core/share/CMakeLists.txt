#
# (C) Copyright 2013 Johns Hopkins University (JHU), All Rights
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
find_package (cisst REQUIRED)

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  set (FILES_TO_COPY sawRobotIO1394Example.xml sawRobotIO1394TestBoard.xml)

  # set (DESTINATION_DIRECTORY "${CISST_BINARY_DIR}/${CISST_SHARE_INSTALL_SUFFIX}/sawRobotIO1394")
  # file (MAKE_DIRECTORY "${DESTINATION_DIRECTORY}")
  # foreach (_file ${FILES_TO_COPY})
  #   add_custom_target (sawRobotIO1394_copy_${_file} ALL
  #                      COMMAND ${CMAKE_COMMAND} -E copy_if_different
  #                              "${CMAKE_CURRENT_SOURCE_DIR}/${_file}"
  #                              "${DESTINATION_DIRECTORY}/${_file}"
  #                      COMMENT "Copying data files ${_file} to ${DESTINATION_DIRECTORY}")
  # endforeach (_file)

endif (cisst_FOUND)
