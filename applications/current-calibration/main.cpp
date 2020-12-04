/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-12-20

  (C) Copyright 2013-2020 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnXMLPath.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobot1394.h>

using namespace sawRobotIO1394;

const double WatchdogPeriod = 100.0 * cmn_ms;
enum PowerType {ALL, BOARD, ACTUATOR, BRAKE};
bool brakes;
mtsRobot1394 * robot;
mtsRobotIO1394 * port;
vctDoubleVec zeros;
size_t numberOfAxis = 0;

// Struct to hold sample readings
struct Samples {
    vctDoubleVec averageAllSamples;
    vctDoubleVec stdDeviation;
    size_t validSamples;
    size_t totalSamples;
    vctDoubleVec averageValidSamples;
};

bool enablePower(PowerType type) {
    if (brakes) {
        robot->SetBrakeCurrent(zeros);
    } else {
        robot->SetActuatorCurrent(zeros);
    }

    switch (type) {
    case ALL:
        std::cout << "Enabling power to all..." << std::endl;
        robot->PowerOnSequence();
        break;
    case BOARD:
        std::cout << "Enabling power to the QLA board..." << std::endl;
        robot->WriteSafetyRelay(true);
        robot->WritePowerEnable(true);
        break;
    case ACTUATOR:
        std::cout << "Enabling amplifiers for the actuators..." << std::endl;
        robot->SetActuatorAmpEnable(true);
        break;
    case BRAKE:
        std::cout << "Enabling amplifiers for the brakes..." << std::endl;
        robot->SetBrakeAmpEnable(true);
        break;
    }

    port->Write();

    if (brakes) {
        robot->SetBrakeCurrent(zeros);
    } else {
        robot->SetActuatorCurrent(zeros);
    }

    port->Write();

    // wait a bit to make sure current stabilizes, 500 * 10 ms = 5 seconds
    for (size_t i = 0; i < 500; ++i) {
        osaSleep(10.0 * cmn_ms);
        port->Read();
        port->Write();
    }

    // check that power is on
    if (!robot->PowerStatus()) {
        std::cerr << "Error: unable to power on controllers, make sure E-Stop is ok." << std::endl;
        return false;
    }
    if (brakes && (type == ALL || type == BRAKE)) {
        if (!robot->BrakeAmpStatus().All()) {
            std::cerr << "Error: failed to turn on brake amplifiers: " << robot->BrakeAmpStatus() << std::endl;
            return false;
        }
    } else if (!brakes && (type == ALL || type == ACTUATOR)) {
        if (!robot->ActuatorAmpStatus().All()) {
            std::cerr << "Error: failed to turn on actuator amplifiers" << std::endl
                      << " - status: " << robot->ActuatorAmpStatus() << std::endl
                      << " - desired: " << robot->ActuatorAmpEnable() << std::endl;
            return false;
        }
    }
    return true;
}

Samples collectSamples(void) {
    // collect samples
    const size_t totalSamples = 50000;
    std::vector<vctDoubleVec> samples;
    samples.resize(totalSamples);
    vctDoubleVec sumSamples, averageAllSamples;
    sumSamples.SetSize(numberOfAxis);
    averageAllSamples.SetSize(numberOfAxis);
    sumSamples.SetAll(0.0);
    for (size_t index = 0; index < totalSamples; ++index) {
        // write to make sure watchdog is not tripped
        if (brakes) {
            robot->SetBrakeCurrent(zeros);
        } else {
            robot->SetActuatorCurrent(zeros);
        }
        port->Write();
        port->Read();
        if (brakes) {
            samples[index].ForceAssign(robot->BrakeCurrentFeedback());
        } else {
            samples[index].ForceAssign(robot->ActuatorCurrentFeedback());
        }
        samples[index].Multiply(1000.0); // convert all values to mA to be easier to read
        sumSamples.Add(samples[index]);
    }

    port->Write();

    // compute simple average
    averageAllSamples.Assign(sumSamples);
    averageAllSamples.Divide(totalSamples);

    // compute standard deviation
    vctDoubleVec sumDifferencesSquared(numberOfAxis);
    vctDoubleVec difference(numberOfAxis);
    for (size_t index = 0; index < totalSamples; ++index) {
        difference.DifferenceOf(samples[index], averageAllSamples);
        sumDifferencesSquared.AddElementwiseProductOf(difference, difference);
    }
    vctDoubleVec stdDeviation(sumDifferencesSquared);
    stdDeviation.Divide(totalSamples);
    for (size_t index = 0; index < stdDeviation.size(); ++index) {
        stdDeviation[index] = sqrt(stdDeviation[index]);
    }

    // eliminate outliers
    vctDoubleVec lower(numberOfAxis);
    lower.DifferenceOf(averageAllSamples, stdDeviation);
    vctDoubleVec upper(numberOfAxis);
    upper.SumOf(averageAllSamples, stdDeviation);
    size_t validSamples = 0;
    vctDoubleVec averageValidSamples(numberOfAxis);
    sumSamples.SetAll(0.0);
    for (size_t index = 0; index < totalSamples; ++index) {
        if (samples[index].ElementwiseLesserOrEqual(upper).All()
            && samples[index].ElementwiseGreaterOrEqual(lower).All()) {
            sumSamples.Add(samples[index]);
            validSamples++;
        }
    }
    averageValidSamples.Assign(sumSamples);
    averageValidSamples.Divide(validSamples);

    Samples curSamples;
    curSamples.averageAllSamples = averageAllSamples;
    curSamples.stdDeviation = stdDeviation;
    curSamples.validSamples = validSamples;
    curSamples.totalSamples = totalSamples;
    curSamples.averageValidSamples = averageValidSamples;
    return curSamples;
}

int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    cmnCommandLineOptions options;
    int portNumber = 0;
    std::string configFile;
    options.AddOptionOneValue("c", "config",
                              "configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &configFile);
    options.AddOptionOneValue("p", "port",
                              "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &portNumber);
    options.AddOptionNoValue("b", "brakes",
                             "calibrate current feedback on brakes instead of actuators",
                             cmnCommandLineOptions::OPTIONAL_OPTION);

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

    std::cout << "Make sure:" << std::endl
              << " - your computer is connected to the firewire controller." << std::endl
              << " - the arm corresponding to the configuration file \"" << configFile << "\" is connected to the controller." << std::endl
              << " - the E-Stop is closed, i.e. will let the controller power on." << std::endl
              << " - you have no other device connected to the firewire chain." << std::endl
              << " - you have no other program trying to communicate with the controller." << std::endl
              << std::endl
              << "Press any key to start." << std::endl;
    cmnGetChar();

    std::cout << "Loading config file ..." << std::endl;
    port = new mtsRobotIO1394("io", 1.0 * cmn_ms, portNumber);
    port->SkipConfigurationCheck(true);
    port->Configure(configFile);

    std::cout << "Creating robot ..." << std::endl;
    int numberOfRobots;
    port->GetNumberOfRobots(numberOfRobots);
    if (numberOfRobots == 0) {
        std::cerr << "Error: the config file doesn't define a robot." << std::endl;
        return -1;
    }
    if (numberOfRobots != 1) {
        std::cerr << "Error: the config file defines more than one robot." << std::endl;
        return -1;
    }
    robot = port->Robot(0);
    brakes = false;
    if (options.IsSet("brakes")) {
        brakes = true;
        numberOfAxis = robot->NumberOfBrakes();
    } else {
        brakes = false;
        numberOfAxis = robot->NumberOfActuators();
    }
    zeros.SetSize(numberOfAxis);
    zeros.SetAll(0.0);

    // make sure we have at least one set of pots values
    try {
        port->Read();
    } catch (const std::runtime_error & e) {
        std::cerr << "Caught exception: " << e.what() << std::endl
                  << "--> This is normal if the encoders have not yet been calibrated using the potentiometers"
                  << std::endl;
    }
    // preload encoders
    try {
        robot->CalibrateEncoderOffsetsFromPots();
    } catch (const std::runtime_error & e) {
        std::cerr << "Caught exception: " << e.what() << std::endl
                  << "--> This is normal if that robot doesn't use the potentiometers to calibrate the encoders"
                  << std::endl;
    }

    std::cout << std::endl
              << "Ready to power?  Press any key to start." << std::endl;
    cmnGetChar();

    robot->SetWatchdogPeriod(300.0 * cmn_ms);
    if (!enablePower(BOARD)) {
        robot->PowerOffSequence();
        delete port;
        return -1;
    }

    std::cout << "Status: power seems fine." << std::endl
              << "Starting calibration ..." << std::endl;
    Samples samplesFbErr = collectSamples();
    // display results
    std::cout << "Measured current error statistics" << std::endl
              << "Status: average current feedback in mA: " << samplesFbErr.averageAllSamples << std::endl
              << "Status: standard deviation in mA:       " << samplesFbErr.stdDeviation << std::endl
              << "Status: kept " << samplesFbErr.validSamples << " samples out of " << samplesFbErr.totalSamples << std::endl
              << "Status: new average in mA:              " << samplesFbErr.averageValidSamples << std::endl
              << std::endl;

    if (brakes) {
        if (!enablePower(BRAKE)) {
            robot->PowerOffSequence();
            delete port;
            return -1;
        }
    } else {
        if (!enablePower(ACTUATOR)) {
            robot->PowerOffSequence();
            delete port;
            return -1;
        }
    }

    std::cout << "Status: power seems fine." << std::endl
              << "Starting calibration ..." << std::endl;
    Samples samplesCmdErr = collectSamples();
    // display results
    std::cout << "Commanded current error statistics" << std::endl
              << "Status: average current feedback in mA: " << samplesCmdErr.averageAllSamples << std::endl
              << "Status: standard deviation in mA:       " << samplesCmdErr.stdDeviation << std::endl
              << "Status: kept " << samplesCmdErr.validSamples << " samples out of " << samplesCmdErr.totalSamples << std::endl
              << "Status: new average in mA:              " << samplesCmdErr.averageValidSamples << std::endl
              << std::endl << std::endl;

    // disable power
    robot->PowerOffSequence();

    // correct cmd (commanded) using corrected fb (feedback)
    vctDoubleVec averageValidSamples(numberOfAxis);
    for (size_t ind = 0; ind < numberOfAxis; ++ind){
        averageValidSamples[ind] = samplesCmdErr.averageValidSamples[ind] - samplesFbErr.averageValidSamples[ind];
    }

    // display results
    std::cout << "Measured current offsets:" << std::endl
              << samplesFbErr.averageValidSamples << std::endl
              << "Command current offsets (corrected using measured current offsets):" << std::endl
              << averageValidSamples << std::endl
              << std::endl
              << "Do you want to update the config file with these values? [Y/y]" << std::endl;

    // save if needed
    char key;
    key = cmnGetChar();
    if ((key == 'y') || (key == 'Y')) {
        cmnXMLPath xmlConfig;
        xmlConfig.SetInputSource(configFile);
        std::string
            xmlQueryCmdOffset,
            xmlQueryCmdScale,
            xmlQueryFbOffset;
        if (brakes) {
            xmlQueryCmdOffset = "Robot[1]/Actuator[%d]/AnalogBrake/AmpsToBits/@Offset";
            xmlQueryCmdScale  = "Robot[1]/Actuator[%d]/AnalogBrake/AmpsToBits/@Scale";
            xmlQueryFbOffset =  "Robot[1]/Actuator[%d]/AnalogBrake/BitsToFeedbackAmps/@Offset";
        } else {
            xmlQueryCmdOffset = "Robot[1]/Actuator[%d]/Drive/AmpsToBits/@Offset";
            xmlQueryCmdScale  = "Robot[1]/Actuator[%d]/Drive/AmpsToBits/@Scale";
            xmlQueryFbOffset =  "Robot[1]/Actuator[%d]/Drive/BitsToFeedbackAmps/@Offset";
        }

        // query previous current offset and scales
        vctDoubleVec previousCmdOffsets(numberOfAxis, 0.0);
        vctDoubleVec previousCmdScales(numberOfAxis, 0.0);
        vctDoubleVec previousFbOffsets(numberOfAxis, 0.0);
        for (size_t index = 0; index < numberOfAxis; ++index) {
            char path[64];
            const char * context = "Config";
            sprintf(path, xmlQueryCmdOffset.c_str(), static_cast<int>(index + 1));
            xmlConfig.GetXMLValue(context, path, previousCmdOffsets[index]);
            sprintf(path, xmlQueryCmdScale.c_str(), static_cast<int>(index + 1));
            xmlConfig.GetXMLValue(context, path, previousCmdScales[index]);
            sprintf(path, xmlQueryFbOffset.c_str(), static_cast<int>(index + 1));
            xmlConfig.GetXMLValue(context, path, previousFbOffsets[index]);

        }
        // compute new offsets
        vctDoubleVec newCmdOffsets(numberOfAxis);
        newCmdOffsets.Assign(averageValidSamples);
        newCmdOffsets.Divide(-1000.0); // convert back to Amps and negate
        newCmdOffsets.ElementwiseMultiply(previousCmdScales);
        newCmdOffsets.Add(previousCmdOffsets);

        vctDoubleVec newFbOffsets(numberOfAxis);
        newFbOffsets.Assign(previousFbOffsets);
        newFbOffsets.Subtract(samplesFbErr.averageValidSamples / 1000.0);

        // ask one last confirmation from user
        std::cout << "Status: commanded current offsets in XML configuration file: " << previousCmdOffsets << std::endl
                  << "Status: new commanded current offsets:                       " << newCmdOffsets << std::endl
                  << "Status: measured current offsets in XML configuration file: " << previousFbOffsets << std::endl
                  << "Status: new measured current offsets:                       " << newFbOffsets << std::endl
                  << std::endl
                  << "Do you want to save these values? [S/s]" << std::endl;
        key = cmnGetChar();
        if ((key == 's') || (key == 'S')) {
            vctIntVec newCmdOffsetsInt(newCmdOffsets);
            vctDoubleVec newFbOffsetsInt(newFbOffsets);
            for (size_t index = 0; index < numberOfAxis; ++index) {
                char path[64];
                const char * context = "Config";
                sprintf(path, xmlQueryCmdOffset.c_str(), static_cast<int>(index + 1));
                xmlConfig.SetXMLValue(context, path, newCmdOffsetsInt[index]);
                sprintf(path, xmlQueryFbOffset.c_str(), static_cast<int>(index + 1));
                xmlConfig.SetXMLValue(context, path, newFbOffsetsInt[index]);
            }
            std::string newConfigFile = configFile + "-new";
            xmlConfig.SaveAs(newConfigFile);
            std::cout << "Status: new config file is \"" << newConfigFile << "\"" << std::endl
                      << "You can copy the new file over the old one using:\n  cp -i "
                      << newConfigFile << " " << configFile << std::endl;
        } else {
            std::cout << "Status: user didn't want to save new offsets." << std::endl;
        }
    } else {
        std::cout << "Status: no data saved in config file." << std::endl;
    }

    delete port;
    return 0;
}
