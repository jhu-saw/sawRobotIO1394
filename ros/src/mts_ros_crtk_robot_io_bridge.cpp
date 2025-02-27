/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2022-10-12

  (C) Copyright 2022-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// project include
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsManagerLocal.h>

#include <cisst_ros_bridge/mtsROSBridge.h>
#include <saw_robot_io_1394_ros/mts_ros_crtk_robot_io_bridge.h>

CMN_IMPLEMENT_SERVICES_DERIVED(mts_ros_crtk_robot_io_bridge, mtsComponent);

mts_ros_crtk_robot_io_bridge::mts_ros_crtk_robot_io_bridge(const std::string & name,
                                                           cisst_ral::node_ptr_t node_handle,
                                                           const std::string & ros_prefix,
                                                           const double & ros_period,
                                                           const double & tf_period,
                                                           const bool read_write,
                                                           const bool perform_spin):
    mtsComponent(name),
    m_ros_prefix(ros_prefix),
    m_ros_period(ros_period),
    m_tf_period(tf_period),
    m_read_write(read_write)
{
    m_bridge = new mts_ros_crtk_bridge_provided(name + "_actual_bridge", node_handle,
                                                5.0 * cmn_ms, perform_spin);

    // This function will make the required interface to be connected with
    // the provided interface of mtsRobotIO1394 named Configure with predefined function names.
    m_configuration_interface = AddInterfaceRequired("RobotConfiguration");
    if (m_configuration_interface) {
        m_configuration_interface->AddFunction("GetRobotNames", m_configuration.GetRobotNames);
        m_configuration_interface->AddFunction("GetDigitalInputNames", m_configuration.GetDigitalInputNames);
        m_configuration_interface->AddFunction("GetName", m_configuration.GetName);
    }
}

void mts_ros_crtk_robot_io_bridge::Configure(const std::string &) {
    this->bridge_all();
}

void mts_ros_crtk_robot_io_bridge::Startup(void) {
    this->bridge_all();
}

void mts_ros_crtk_robot_io_bridge::bridge_all(void)
{
    if (already_bridged) {
        return;
    }
    already_bridged = true;

    if (!m_configuration_interface->GetConnectedInterface()) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: unable to connect to configuration interface"
                                 << std::endl;
        return;
    }

    mtsManagerLocal * _component_manager = mtsComponentManager::GetInstance();
    m_pub_bridge_extra = new mtsROSBridge("io-bridge-extra",
                                          m_ros_period, m_bridge->node_handle_ptr());
    
    // robots, one component per robot with 2 interfaces to be connected
    m_configuration.GetName(m_io_component_name);

    std::vector<std::string> robot_names;
    m_configuration.GetRobotNames(robot_names);
    for (const auto & robot : robot_names) {
        m_bridge->bridge_interface_provided(m_io_component_name, robot,
                                            m_ros_prefix + robot,
                                            m_ros_period, m_tf_period, m_read_write);

        std::string _ros_namespace = m_ros_prefix + robot;
        cisst_ral::clean_namespace(_ros_namespace);

        m_pub_bridge_extra->AddPublisherFromCommandRead<vctDoubleVec, CISST_RAL_MSG(sensor_msgs, JointState)>
            ("io-" + robot, "GetActuatorFeedbackCurrent",
             _ros_namespace + "/measured_current");
        m_pub_bridge_extra->AddPublisherFromCommandRead<vctDoubleVec, CISST_RAL_MSG(sensor_msgs, JointState)>
            ("io-" + robot, "GetActuatorRequestedCurrent",
             _ros_namespace + "/servo_current");
        m_pub_bridge_extra->AddPublisherFromCommandRead<vctDoubleVec, CISST_RAL_MSG(sensor_msgs, JointState)>
            ("io-" + robot, "GetActuatorTimestamp",
             _ros_namespace + "/timestamp");

        // add
        m_connections.Add("io-bridge-extra", "io-" + robot,
                          m_io_component_name, robot);
    }

    std::vector<std::string> input_names;
    m_configuration.GetDigitalInputNames(input_names);
    for (const auto & input : input_names) {
        m_bridge->bridge_interface_provided(m_io_component_name, input,
                                            m_ros_prefix + input,
                                            m_ros_period, m_tf_period, m_read_write);
    }

    m_bridge->Connect();
    _component_manager->AddComponent(m_pub_bridge_extra);
    m_connections.Connect();
}
