/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-02-07

  (C) Copyright 2013-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system
#include <iostream>
// cisst/saw
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>
#include <cisstMultiTask/mtsQtWidgetComponent.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobotIO1394QtWidgetFactory.h>

#include <saw_robot_io_1394_ros/mts_ros_crtk_robot_io_bridge.h>

#include <QApplication>

int main(int argc, char ** argv)
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // create ROS node handle
    cisst_ral::ral ral(argc, argv, "robot_io_console");
    auto rosNode = ral.node();

    // parse options
    cmnCommandLineOptions options;
    std::string port = mtsRobotIO1394::DefaultPort();
    std::string protocol;
    std::list<std::string> configFiles;
    std::string robotName = "Robot";
    double periodInSeconds = 1.0 * cmn_ms;
    double rosPeriod = 2.0 * cmn_ms;
    double tfPeriod = 20.0 * cmn_ms;
    std::list<std::string> managerConfig;

    options.AddOptionMultipleValues("c", "config",
                                    "configuration file",
                                    cmnCommandLineOptions::REQUIRED_OPTION, &configFiles);
    options.AddOptionOneValue("p", "port",
                              "port used to communicate with the dVRK controllers (fw, udp)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &port);
    options.AddOptionOneValue("f", "firewire-protocol",
                              "FireWire protocol",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &protocol);
    options.AddOptionOneValue("n", "name",
                              "robot name",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &robotName);
    options.AddOptionOneValue("i", "io-period",
                              "IO read/write period interval in seconds (default is 1 ms, 0.001)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &periodInSeconds);
    options.AddOptionNoValue("C", "calibration-mode",
                             "run in calibration mode, doesn't require lookup table for pots/encoder on Si arms",
                             cmnCommandLineOptions::OPTIONAL_OPTION);
    options.AddOptionOneValue("P", "ros-period",
                              "period in seconds to read all tool positions (default 0.002, 2 ms, 500Hz).  There is no point to have a period higher than the device",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);
    options.AddOptionOneValue("T", "tf-ros-period",
                              "period in seconds to read all components and broadcast tf2 (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the arm component's period",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &tfPeriod);
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON files to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);
    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");

    if (!options.Parse(argc, argv, std::cerr)) {
        return -1;
    }

    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments;

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // RobotIO
    mtsRobotIO1394 * robotIO = new mtsRobotIO1394("robotIO", periodInSeconds, port);
    if (options.IsSet("firewire-protocol")) {
        robotIO->SetProtocol(protocol);
    }
    robotIO->SetCalibrationMode(options.IsSet("calibration-mode"));
    componentManager->AddComponent(robotIO);

    // create a Qt application
    QApplication application(argc, argv);
    cmnQt::QApplicationExitsOnCtrlC();
    if (options.IsSet("dark-mode")) {
        cmnQt::SetDarkMode();
    }

    mtsRobotIO1394QtWidgetFactory * robotWidgetFactory = new mtsRobotIO1394QtWidgetFactory("robotWidgetFactory");
    componentManager->AddComponent(robotWidgetFactory);

    for (const auto & configFile : configFiles) {
        if (!cmnPath::Exists(configFile)) {
            std::cerr << "Can't find file \"" << configFile << "\"" << std::endl;
            return -1;
        }
        robotIO->Configure(configFile);
    }

    componentManager->Connect("robotWidgetFactory", "RobotConfiguration",
                              "robotIO", "Configuration");
    robotWidgetFactory->Configure();

    // ROS CRTK bridge
    mts_ros_crtk_robot_io_bridge * crtk_bridge
        = new mts_ros_crtk_robot_io_bridge("robot_io_crtk_bridge", rosNode,
                                           rosPeriod, tfPeriod);
    componentManager->AddComponent(crtk_bridge);
    componentManager->Connect("robot_io_crtk_bridge", "RobotConfiguration",
                              "robotIO", "Configuration");

    crtk_bridge->Configure();

    // custom user components
    if (!componentManager->ConfigureJSON(managerConfig)) {
        CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager, check cisstLog for error messages" << std::endl;
        return -1;
    }

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    // run Qt app
    application.exec();

    // cleanup
    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    // delete dvgc robot
    delete robotWidgetFactory;
    delete robotIO;

    // stop all logs
    cmnLogger::Kill();

    // stop ROS node
    cisst_ral::shutdown();

    return 0;
}
