#
# (C) Copyright 2017-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (robot_io VERSION 2.2.1)

# first test for ROS1
set (ROS1_depends cisst_ros_crtk roscpp roslib)
find_package (catkin QUIET
  COMPONENTS ${ROS1_depends})

# ROS 1
if (catkin_FOUND)
  set (ROS_VERSION 1)
  catkin_package (CATKIN_DEPENDS cisst_ros_crtk roscpp roslib)
  include_directories (${catkin_INCLUDE_DIRS})

else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    find_package (ament_cmake_python REQUIRED)
    set (ROS_VERSION 2)
    set (ROS2_depends
      rclcpp std_msgs std_srvs geometry_msgs sensor_msgs diagnostic_msgs
      tf2_ros tf2_msgs cisst_msgs crtk_msgs cisst_ros_bridge cisst_ros_crtk)
    foreach (_dep ${ROS2_depends})
      find_package (${_dep} REQUIRED)
    endforeach ()

  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

# find cisst and make sure the required libraries have been compiled
set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonQt
     cisstVector
     cisstVectorQt
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstParameterTypesQt
     cisstQt)

find_package (cisst 1.2.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # set paths
  cisst_set_output_path ()

  # sawRobotIO1394 has been compiled within cisst, we should find it automatically
  find_package (sawRobotIO1394 2.2.0)

  if (sawRobotIO1394_FOUND)

    # sawRobotIO1394 configuration
    include_directories (${sawRobotIO1394_INCLUDE_DIR})
    link_directories (${sawRobotIO1394_LIBRARY_DIR})

    add_executable (robot_io_console
      src/mts_ros_crtk_robot_io_bridge.h
      src/mts_ros_crtk_robot_io_bridge.cpp
      src/robot_io_console.cpp)
    target_link_libraries (robot_io_console
                           ${sawRobotIO1394_LIBRARIES}
                           ${catkin_LIBRARIES})
    cisst_target_link_libraries (robot_io_console ${REQUIRED_CISST_LIBRARIES})

    install (TARGETS robot_io_console
      LIBRARY DESTINATION lib/${PROJECT_NAME}
      RUNTIME DESTINATION lib/${PROJECT_NAME}
      LIBRARY DESTINATION lib
      INCLUDES DESTINATION include)

    if (ROS_VERSION EQUAL 2)
      ament_target_dependencies (
        robot_io_console
        ${ROS2_depends}
        )
      ament_package ()
    endif ()

  else (sawRobotIO1394_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires sawRobotIO1394")
  endif (sawRobotIO1394_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
