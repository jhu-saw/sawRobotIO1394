/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-01-09

  (C) Copyright 2014-2015 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <sawRobotIO1394/osaConfiguration1394.h>
#include <sawRobotIO1394/osaXML1394.h>
#include <sawRobotIO1394/osaPort1394.h>
#include <sawRobotIO1394/osaRobot1394.h>


int main(int argc, char * argv[])
{
    cmnCommandLineOptions options;
    int portNumber = 0;
    int actuatorIndex = 0;
    int startIndex = 0;
    int endIndex = startIndex + 4;
    size_t numberOfIterations;
    double sleepBetweenReads = 0.3 * cmn_ms;
    std::string configFile;
    options.AddOptionOneValue("c", "config",
                              "configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &configFile);
    options.AddOptionOneValue("a", "actuator",
                              "actuator index",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &actuatorIndex);
    options.AddOptionOneValue("p", "port",
                              "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &portNumber);
    options.AddOptionOneValue("n", "number-iterations",
                              "number of iterations",
                              cmnCommandLineOptions::REQUIRED_OPTION, &numberOfIterations);
    options.AddOptionOneValue("s", "sleep-between-reads",
                              "sleep between reads",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &sleepBetweenReads);

    const size_t nbIterationsToStart = 10000;

    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }

    if (!cmnPath::Exists(configFile)) {
        std::cerr << "Can't find file \"" << configFile << "\"." << std::endl;
        return -1;
    }
    std::cout << "Configuration file: " << configFile << std::endl
              << "Port: " << portNumber << std::endl;

    // add arrays here for all data collected....
    std::cout << "Allocation memory for " << numberOfIterations << " samples." << std::endl;
    size_t allIterations[numberOfIterations];
    //    double * allActuatorTimeStamps = new double[numberOfIterations];
    double allCPUTimes[numberOfIterations];
    double allPositions[numberOfIterations][4];
    double allVelocities[numberOfIterations][4];
    double allAccelerations[numberOfIterations][4];
    double allVelocitiesSoftware[numberOfIterations][4];
    
    std::cout << "Loading config file ..." << std::endl;
    sawRobotIO1394::osaPort1394Configuration config;
    sawRobotIO1394::osaXML1394ConfigurePort(configFile, config);

    std::cout << "Creating robot ..." << std::endl;
    if (config.Robots.size() == 0) {
        std::cerr << "Error: the config file doesn't define a robot." << std::endl;
        return -1;
    }
    if (config.Robots.size() != 1) {
        std::cerr << "Error: the config file defines more than one robot." << std::endl;
        return -1;
    }
    sawRobotIO1394::osaRobot1394 * robot = new sawRobotIO1394::osaRobot1394(config.Robots[0]);

    std::cout << "Creating port ..." << std::endl;
    sawRobotIO1394::osaPort1394 * port = new sawRobotIO1394::osaPort1394(portNumber);
    port->AddRobot(robot);
   
    // make sure we have at least one set of pots values
    try {
        port->Read();
    } catch (const std::runtime_error & e) {
        std::cerr << "Caught exception: " << e.what() << std::endl;
    }
    // preload encoders
    robot->CalibrateEncoderOffsetsFromPots();

    std::cout << "Starting data collection." << std::endl;

    size_t percent = nbIterationsToStart / 100;
    size_t progress = 0;

    for (size_t iter = 0;
         iter < nbIterationsToStart;
         ++iter) {
        port->Read();
        // display progress
        progress++;
        if (progress == percent) {
            std::cout << "-" << std::flush;
            progress = 0;
        }
    }

    // main loop
    percent = numberOfIterations / 100;
    progress = 0;

    for (size_t iter = 0;
         iter < numberOfIterations;
         ++iter) {
        port->Read();
        // save index
        allIterations[iter] = iter;

        // CPU time
        allCPUTimes[iter] = mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime();

        // get time from FPGA
        // allActuatorTimeStamps[iter] =
        //robot->ActuatorTimeStamp()[actuatorIndex];

        // get positions, see osaRobot1394.cpp, line ~1000
        for (actuatorIndex = startIndex; actuatorIndex < endIndex; ++actuatorIndex)
            {
                allPositions[iter][actuatorIndex-startIndex] =
                    robot->EncoderPosition()[actuatorIndex];

                allVelocities[iter][actuatorIndex-startIndex] =
                    robot->EncoderVelocity()[actuatorIndex];
     
                allAccelerations[iter][actuatorIndex-startIndex] =
                    robot->EncoderAcceleration()[actuatorIndex];
                
                allVelocitiesSoftware[iter][actuatorIndex-startIndex] =
                    robot->EncoderVelocitySoftware()[actuatorIndex];
            }
        
        // display progress
        progress++;
        if (progress == percent) {
            std::cout << "." << std::flush;
            progress = 0;
        }

        // finally sleep as requested
        osaSleep(sleepBetweenReads);
    }
    std::cout << std::endl;

    // save to csv file
    std::string currentDateTime;
    osaGetDateTimeString(currentDateTime);
    std::string fileName = "data-" + currentDateTime + ".csv";

    std::cout << "Saving to file: " << fileName << std::endl;
    std::ofstream output;
    output.open(fileName.c_str());

    output << "iteration,"
           << "cpu-time,";
        // << "fpga-dtime,"

    // header
    for(actuatorIndex = startIndex; actuatorIndex < endIndex; ++actuatorIndex)
        {
            output << "encoder-pos-" << actuatorIndex << ","
                   << "encoder-vel-" << actuatorIndex << ","
                   << "encoder-acc-" << actuatorIndex << ","
                   << "software-vel-" << actuatorIndex << ",";
        }
    output << std::endl;

    output << std::setprecision(17);
    for (size_t iter = 0;
         iter < numberOfIterations;
         ++iter) {
        output << allIterations[iter] << ","
               << allCPUTimes[iter] << ",";
        //     << allActuatorTimeStamps[iter] << ","
        for(actuatorIndex = 0; actuatorIndex < 4; ++actuatorIndex)
            {
                output << allPositions[iter][actuatorIndex] << ","
                       << allVelocities[iter][actuatorIndex] << ","
                       << allAccelerations[iter][actuatorIndex] << ","
                       << allVelocitiesSoftware[iter][actuatorIndex] << ",";
            }
        output << std::endl;
    }
    output.close();

    delete port;
    return 0;
}
