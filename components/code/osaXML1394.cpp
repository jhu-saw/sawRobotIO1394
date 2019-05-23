/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Jonathan Bohren, Anton Deguet
  Created on: 2013-06-29

  (C) Copyright 2013-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#include <sawRobotIO1394/osaXML1394.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstNumerical/nmrInverse.h>

namespace sawRobotIO1394 {

    void osaXML1394ConfigurePort(const std::string & filename, osaPort1394Configuration & config)
    {
        cmnXMLPath xmlConfig;
        xmlConfig.SetInputSource(filename);

        // get an check version number
        int version;
        bool versionFound = osaXML1394GetValue(xmlConfig, "Config", "@Version", version);
        if (!versionFound) {
            CMN_LOG_INIT_ERROR << "Configure: Config/Version is missing in file: "
                               << filename << std::endl
                               << "Make sure you generate your XML files with the latest config generator." << std::endl;
            exit(EXIT_FAILURE);
            return;
        } else {
            const int minimumVersion = 4; // backward compatibility
            if (version < minimumVersion) {
                CMN_LOG_INIT_ERROR << "Configure: Config/Version must be at least " << minimumVersion
                                   << ", version found is " << version << std::endl
                                   << "File: " << filename << std::endl
                                   << "Make sure you generate your XML files with the latest config generator." << std::endl;
                exit(EXIT_FAILURE);
                return;
            }
            const int currentVersion = 4;
            if (version > currentVersion) {
                CMN_LOG_INIT_ERROR << "Configure: current Config/Version is " << currentVersion
                                   << ", version found is " << version << ", you might want to upgrade your code" << std::endl
                                   << "File: " << filename << std::endl
                                   << "Make sure you generate your XML files with the latest config generator." << std::endl;
            }
        }

        // Get the number of robot elements
        int numRobots = 0;
        xmlConfig.GetXMLValue("", "count(/Config/Robot)", numRobots);

        for (int i = 0; i < numRobots; i++) {
            osaRobot1394Configuration robot;

            // Store the robot in the config if it's succesfully parsed
            if (osaXML1394ConfigureRobot(xmlConfig, i + 1, robot)) {
                config.Robots.push_back(robot);
            } else {
                CMN_LOG_INIT_WARNING << "ConfigurePort: failed to configure robot from file \""
                                     << filename << "\"" << std::endl;
                exit(EXIT_FAILURE);
            }
        }

        // Get the number of digital input elements
        int numDigitalInputs = 0;
        xmlConfig.GetXMLValue("", "count(/Config/DigitalIn)", numDigitalInputs);

        for (int i = 0; i < numDigitalInputs; i++) {
            osaDigitalInput1394Configuration digitalInput;

            // Store the digitalInput in the config if it's succesfully parsed
            if (osaXML1394ConfigureDigitalInput(xmlConfig, i + 1, digitalInput)) {
                config.DigitalInputs.push_back(digitalInput);
            } else {
                CMN_LOG_INIT_WARNING << "ConfigurePort: failed to configure digital input from file \""
                                     << filename << "\"" << std::endl;
                exit(EXIT_FAILURE);
            }
        }

        // Get the number of digital output elements
        int numDigitalOutputs = 0;
        xmlConfig.GetXMLValue("", "count(/Config/DigitalOut)", numDigitalOutputs);

        for (int i = 0; i < numDigitalOutputs; i++) {
            osaDigitalOutput1394Configuration digitalOutput;

            // Store the digitalOutput in the config if it's succesfully parsed
            if (osaXML1394ConfigureDigitalOutput(xmlConfig, i + 1, digitalOutput)) {
                config.DigitalOutputs.push_back(digitalOutput);
            } else {
                CMN_LOG_INIT_WARNING << "ConfigurePort: failed to configure digital output from file \""
                                     << filename << "\"" << std::endl;
                exit(EXIT_FAILURE);
            }
        }

        // Get the number of Dallas chip elements
        int numDallasChips = 0;
        xmlConfig.GetXMLValue("", "count(/Config/DallasChip)", numDallasChips);

        for (int i = 0; i < numDallasChips; i++) {
            osaDallasChip1394Configuration dallasChip;

            // Store the dallasChip in the config if it's succesfully parsed
            if (osaXML1394ConfigureDallasChip(xmlConfig, i + 1, dallasChip)) {
                config.DallasChips.push_back(dallasChip);
            } else {
                CMN_LOG_INIT_WARNING << "ConfigurePort: failed to configure Dallas chip from file \""
                                     << filename << "\"" << std::endl;
                exit(EXIT_FAILURE);
            }
        }

        // Check to make sure something was found
        if ((numRobots + numDigitalInputs + numDigitalOutputs + numDallasChips) == 0) {
            CMN_LOG_INIT_ERROR << "osaXML1394ConfigurePort: file " << filename
                               << " doesn't contain any Config/Robot, Config/DigitalIn, Config/DigitalOut or Config/DallasChip" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    bool osaXML1394ConfigureRobot(cmnXMLPath & xmlConfig,
                                  const int robotIndex,
                                  osaRobot1394Configuration & robot)
    {
        bool good = true;
        char path[256];
        const char * context = "Config";

        robot.NumberOfBrakes = 0;

        sprintf(path, "Robot[%d]/@Name", robotIndex);
        good &= osaXML1394GetValue(xmlConfig, context, path, robot.Name);

        sprintf(path, "Robot[%d]/@NumOfActuator", robotIndex);
        good &= osaXML1394GetValue(xmlConfig, context, path, robot.NumberOfActuators);

        sprintf(path, "Robot[%d]/@NumOfJoint", robotIndex);
        good &= osaXML1394GetValue(xmlConfig, context, path, robot.NumberOfJoints);

        std::string type;
        sprintf(path, "Robot[%d]/@Type", robotIndex);
        if (xmlConfig.GetXMLValue(context, path, type)) {
            if (type == std::string("io-only")) {
                robot.OnlyIO = true;
            } else if (type == std::string("robot")) {
                robot.OnlyIO = false;
            } else {
                CMN_LOG_INIT_ERROR << "osaXML1394ConfigureRobot: Type must be \"io-only\" or \"robot\", not "
                                   << type << std::endl;
                good = false;
            }
        } else {
            // default is robot
            robot.OnlyIO = false;
        }

        sprintf(path, "Robot[%d]/@SN", robotIndex);
        robot.SerialNumber = 0;
        good &= osaXML1394GetValue(xmlConfig, context, path, robot.SerialNumber, false); // not required

        for (int i = 0; i < robot.NumberOfActuators; i++) {
            osaActuator1394Configuration actuator;
            int actuatorIndex = i + 1;

            sprintf(path, "Robot[%d]/Actuator[%d]/@BoardID", robotIndex, actuatorIndex);
            good &= osaXML1394GetValue(xmlConfig, context, path, actuator.BoardID);
            if ((actuator.BoardID < 0) || (actuator.BoardID >= (int)MAX_BOARDS)) {
                CMN_LOG_INIT_ERROR << "Configure: invalid board number " << actuator.BoardID
                                   << " for board " << i << std::endl;
                return false;
            }

            sprintf(path, "Robot[%d]/Actuator[%d]/@AxisID", robotIndex, actuatorIndex);
            good &= osaXML1394GetValue(xmlConfig, context, path, actuator.AxisID);
            if ((actuator.AxisID < 0) || (actuator.AxisID >= (int)MAX_AXES)) {
                CMN_LOG_INIT_ERROR << "Configure: invalid axis number " << actuator.AxisID
                                   << " for actuator " << i << std::endl;
                return false;
            }

            std::string actuatorType = "";
            sprintf(path, "Robot[%d]/Actuator[%d]/@Type", robotIndex, actuatorIndex);
            xmlConfig.GetXMLValue(context, path, actuatorType);
            if (actuatorType == "") {
                CMN_LOG_INIT_WARNING << "Configure: no actuator type specified " << actuator.AxisID
                                     << " for actuator " << i << " set to Revolute by default" << std::endl;
                actuatorType = "Revolute";
            }
            double unitPosConversion;
            if (actuatorType == "Revolute") {
                // deg to radian
                unitPosConversion = cmnPI_180;
                actuator.JointType = PRM_JOINT_REVOLUTE;
            } else if (actuatorType == "Prismatic") {
                unitPosConversion = cmn_mm;
                actuator.JointType = PRM_JOINT_PRISMATIC;
            }

            sprintf(path, "Robot[%i]/Actuator[%d]/Drive/AmpsToBits/@Scale", robotIndex, actuatorIndex);
            good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Drive.CurrentToBitsScale);

            sprintf(path, "Robot[%i]/Actuator[%d]/Drive/AmpsToBits/@Offset", robotIndex, actuatorIndex);
            good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Drive.CurrentToBitsOffset);

            sprintf(path, "Robot[%i]/Actuator[%d]/Drive/BitsToFeedbackAmps/@Scale", robotIndex, actuatorIndex);
            good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Drive.BitsToCurrentScale);

            sprintf(path, "Robot[%i]/Actuator[%d]/Drive/BitsToFeedbackAmps/@Offset", robotIndex, actuatorIndex);
            good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Drive.BitsToCurrentOffset);

            sprintf(path, "Robot[%i]/Actuator[%d]/Drive/NmToAmps/@Scale", robotIndex, actuatorIndex);
            good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Drive.EffortToCurrentScale, !robot.OnlyIO);

            sprintf(path, "Robot[%i]/Actuator[%d]/Drive/MaxCurrent/@Value", robotIndex, actuatorIndex);
            good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Drive.CurrentCommandLimit);

            // looking for brakes
            sprintf(path, "Robot[%i]/Actuator[%d]/AnalogBrake", robotIndex, actuatorIndex);
            int analogBrake;
            xmlConfig.GetXMLValue(context, path, analogBrake, -1);
            if (analogBrake != -1) {
                osaAnalogBrake1394Configuration * brake = new osaAnalogBrake1394Configuration;
                actuator.Brake = brake;
                robot.NumberOfBrakes++;

                sprintf(path, "Robot[%i]/Actuator[%d]/AnalogBrake/@BoardID", robotIndex, actuatorIndex);
                good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Brake->BoardID);

                sprintf(path, "Robot[%i]/Actuator[%d]/AnalogBrake/@AxisID", robotIndex, actuatorIndex);
                good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Brake->AxisID);

                sprintf(path, "Robot[%i]/Actuator[%d]/AnalogBrake/AmpsToBits/@Scale", robotIndex, actuatorIndex);
                good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Brake->Drive.CurrentToBitsScale);

                sprintf(path, "Robot[%i]/Actuator[%d]/AnalogBrake/AmpsToBits/@Offset", robotIndex, actuatorIndex);
                good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Brake->Drive.CurrentToBitsOffset);

                sprintf(path, "Robot[%i]/Actuator[%d]/AnalogBrake/BitsToFeedbackAmps/@Scale", robotIndex, actuatorIndex);
                good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Brake->Drive.BitsToCurrentScale);

                sprintf(path, "Robot[%i]/Actuator[%d]/AnalogBrake/BitsToFeedbackAmps/@Offset", robotIndex, actuatorIndex);
                good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Brake->Drive.BitsToCurrentOffset);

                sprintf(path, "Robot[%i]/Actuator[%d]/AnalogBrake/MaxCurrent/@Value", robotIndex, actuatorIndex);
                good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Brake->Drive.CurrentCommandLimit);

                sprintf(path, "Robot[%i]/Actuator[%d]/AnalogBrake/ReleaseCurrent/@Value", robotIndex, actuatorIndex);
                good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Brake->ReleaseCurrent);

                sprintf(path, "Robot[%i]/Actuator[%d]/AnalogBrake/ReleaseTime/@Value", robotIndex, actuatorIndex);
                good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Brake->ReleaseTime);

                sprintf(path, "Robot[%i]/Actuator[%d]/AnalogBrake/ReleasedCurrent/@Value", robotIndex, actuatorIndex);
                good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Brake->ReleasedCurrent);

                sprintf(path, "Robot[%i]/Actuator[%d]/AnalogBrake/EngagedCurrent/@Value", robotIndex, actuatorIndex);
                good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Brake->EngagedCurrent);

            } else {
                // no brake found
                actuator.Brake = 0;
            }

            sprintf(path, "Robot[%i]/Actuator[%d]/Encoder/BitsToPosSI/@Scale", robotIndex, actuatorIndex);
            good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Encoder.BitsToPositionScale, !robot.OnlyIO);
            actuator.Encoder.BitsToPositionScale *= unitPosConversion; // -------------------------------------------- adeguet1, make sure these are degrees
            if (robot.OnlyIO) {
                actuator.Encoder.BitsToPositionScale = 1.0;
            }

            sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/BitsToVolts/@Scale", robotIndex, actuatorIndex);
            good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Pot.BitsToVoltageScale);

            sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/BitsToVolts/@Offset", robotIndex, actuatorIndex);
            good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Pot.BitsToVoltageOffset);

            sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/VoltsToPosSI/@Scale", robotIndex, actuatorIndex);
            good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Pot.VoltageToPositionScale, !robot.OnlyIO);
            actuator.Pot.VoltageToPositionScale *= unitPosConversion; // -------------------------------------------- adeguet1, make sure these are degrees

            sprintf(path, "Robot[%i]/Actuator[%d]/AnalogIn/VoltsToPosSI/@Offset", robotIndex, actuatorIndex);
            good &= osaXML1394GetValue(xmlConfig, context, path, actuator.Pot. VoltageToPositionOffset, !robot.OnlyIO);
            actuator.Pot.VoltageToPositionOffset *= unitPosConversion; // -------------------------------------------- adeguet1, make sure these are degrees

            // Add the actuator
            robot.Actuators.push_back(actuator);
        }

        // verify that all amps offsets are different from each other
        if (robot.Actuators.size() > 2) {
            bool allEqual = true;
            const double defaultOffset = robot.Actuators[0].Drive.CurrentToBitsOffset;
            for (size_t index = 1;
                 index < robot.Actuators.size();
                 ++index) {
                if (robot.Actuators[index].Drive.CurrentToBitsOffset != defaultOffset) {
                    allEqual = false;
                }
            }
            if (allEqual) {
                CMN_LOG_INIT_ERROR << "All offsets equal, it is very unlikely that the current calibration has been performed for this arm:"
                                   << "  " << robot.Name << std::endl;
            }
        }

        // look for potentiometers position, if any
        std::string potentiometerPosition;
        robot.PotLocation = POTENTIOMETER_UNDEFINED;
        sprintf(path,"Robot[%d]/Potentiometers/@Position", robotIndex);
        if (xmlConfig.GetXMLValue(context, path, potentiometerPosition)) {
            if (potentiometerPosition == "Actuators") {
                robot.PotLocation = POTENTIOMETER_ON_ACTUATORS;
            } else if (potentiometerPosition == "Joints") {
                robot.PotLocation = POTENTIOMETER_ON_JOINTS;
            } else {
                CMN_LOG_INIT_ERROR << "Configure: invalid <Potentiometers Position=\"\"> value, must be either \"Joints\" or \"Actuators\" for robot number "
                                   << robotIndex << std::endl;
            }
        } else {
            CMN_LOG_INIT_VERBOSE << "Configure: no <Potentiometers Position=\"\"> found." << std::endl;
        }

        // load pot tolerances
        if ((robot.PotLocation == POTENTIOMETER_ON_ACTUATORS)
            || robot.PotLocation == POTENTIOMETER_ON_JOINTS) {
            int numberOfPots = 0;
            if (robot.PotLocation == POTENTIOMETER_ON_ACTUATORS) {
                numberOfPots = robot.NumberOfActuators;
            } else {
                numberOfPots = robot.NumberOfJoints;
            }
            for (int potIndex = 0; potIndex < numberOfPots; ++potIndex) {
                osaPotTolerance1394Configuration pot;
                int xmlPotIndex = potIndex + 1;
                // check that axis index is valid
                int axis = -1;
                sprintf(path, "Robot[%i]/Potentiometers/Tolerance[%d]/@Axis", robotIndex, xmlPotIndex);
                good &= osaXML1394GetValue(xmlConfig, context, path, axis);
                if (axis != potIndex) {
                    CMN_LOG_INIT_ERROR << "Configure: invalid <Potentiometers><Tolerance Axis=\"\"> must be provided in order, for tolerance "
                                       << potIndex << " Axis should match but found " << axis << " for robot "
                                       << robotIndex << std::endl;
                    good = false;
                }
                // get data
                sprintf(path, "Robot[%i]/Potentiometers/Tolerance[%d]/@Distance", robotIndex, xmlPotIndex);
                good &= osaXML1394GetValue(xmlConfig, context, path, pot.Distance);
                sprintf(path, "Robot[%i]/Potentiometers/Tolerance[%d]/@Latency", robotIndex, xmlPotIndex);
                good &= osaXML1394GetValue(xmlConfig, context, path, pot.Latency);
                std::string units;
                sprintf(path, "Robot[%i]/Potentiometers/Tolerance[%d]/@Unit", robotIndex, xmlPotIndex);
                good &= osaXML1394GetValue(xmlConfig, context, path, units);
                if (units == "deg") {
                    pot.Distance *= cmnPI_180;
                } else if (units == "rad") {
                    // SI ok
                } else if (units == "mm") {
                    pot.Distance *= cmn_mm;
                } else if (units == "m") {
                    pot.Distance *= cmn_m;
                } else {
                    CMN_LOG_INIT_ERROR << "Configure: invalid <Potentiometers><Tolerance Unit=\"\"> must be rad, deg, mm, m but found \""
                                       << units << "\" for Axis " << axis << " for robot "
                                       << robotIndex << std::endl;
                    good = false;
                }
                robot.PotTolerances.push_back(pot);
            }
        }

        // Configure Coupling
        if (!osaXML1394ConfigureCoupling(xmlConfig, robotIndex, robot)) {
            return false;
        }
        return good;
    }

    bool osaXML1394ConfigureCoupling(cmnXMLPath & xmlConfig,
                                     const int robotIndex,
                                     osaRobot1394Configuration & robot)
    {
        char path[64];
        const char * context = "Config";
        //The Coupling Value must be equal to 1 for this configuration to work.
        int coupling = 0;
        sprintf(path, "Robot[%i]/Coupling/@Value", robotIndex);
        xmlConfig.GetXMLValue(context, path, coupling);
        robot.HasActuatorToJointCoupling = (coupling == 1);

        if (robot.HasActuatorToJointCoupling) {
            bool parse_success = true;
            parse_success &= osaXML1394ConfigureCouplingMatrix(xmlConfig, robotIndex, "ActuatorToJointPosition",
                                                               robot.NumberOfJoints, robot.NumberOfActuators,
                                                               robot.Coupling.ActuatorToJointPosition());

            parse_success &= osaXML1394ConfigureCouplingMatrix(xmlConfig, robotIndex, "JointToActuatorPosition",
                                                               robot.NumberOfActuators, robot.NumberOfJoints,
                                                               robot.Coupling.JointToActuatorPosition());
            if (robot.Coupling.JointToActuatorPosition().size() == 0) {
                robot.Coupling.JointToActuatorPosition()
                    .ForceAssign(robot.Coupling.ActuatorToJointPosition());
                nmrInverse(robot.Coupling.JointToActuatorPosition());
            }

            parse_success &= osaXML1394ConfigureCouplingMatrix(xmlConfig, robotIndex, "ActuatorToJointTorque",
                                                               robot.NumberOfJoints, robot.NumberOfActuators,
                                                               robot.Coupling.ActuatorToJointEffort());
            if (robot.Coupling.ActuatorToJointEffort().size() == 0) {
                robot.Coupling.ActuatorToJointEffort()
                    .ForceAssign(robot.Coupling.JointToActuatorPosition().Transpose());
            }

            parse_success &= osaXML1394ConfigureCouplingMatrix(xmlConfig, robotIndex, "JointToActuatorTorque",
                                                               robot.NumberOfActuators, robot.NumberOfJoints,
                                                               robot.Coupling.JointToActuatorEffort());
            if (robot.Coupling.JointToActuatorEffort().size() == 0) {
                robot.Coupling.JointToActuatorEffort()
                    .ForceAssign(robot.Coupling.ActuatorToJointEffort());
                nmrInverse(robot.Coupling.JointToActuatorEffort());
            }

            if (!parse_success) {
                return false;
            }

            // make sure the coupling matrices make sense
            vctDoubleMat product, identity;
            identity.ForceAssign(vctDoubleMat::Eye(robot.NumberOfActuators));
            product.SetSize(robot.NumberOfActuators, robot.NumberOfActuators);

            product.ProductOf(robot.Coupling.ActuatorToJointPosition(),
                              robot.Coupling.JointToActuatorPosition());

            if (!product.AlmostEqual(identity, 0.001)) {
                CMN_LOG_INIT_ERROR << "ConfigureCoupling: product of position coupling matrices not identity:"
                                   << std::endl << product << std::endl;
                return false;
            }

            product.ProductOf(robot.Coupling.ActuatorToJointEffort(),
                              robot.Coupling.JointToActuatorEffort());
            if (!product.AlmostEqual(identity, 0.001)) {
                CMN_LOG_INIT_ERROR << "ConfigureCoupling: product of torque coupling matrices not identity:"
                                   << std::endl << product << std::endl;
                return false;
            }
        } // has actuator coupling
        return true;
    }

    bool osaXML1394ConfigureCouplingMatrix(cmnXMLPath & xmlConfig,
                                           const int robotIndex,
                                           const char * couplingString,
                                           int numRows,
                                           int numCols,
                                           vctDoubleMat & resultMatrix)
    {
        std::string context = "Config";

        // Construct XPath for matrix
        std::ostringstream matrixPath;
        matrixPath << "Robot[" << robotIndex << "]/Coupling/" << couplingString;

        // if it doesn't exist, parsing is fine, matrix is set to size 0, 0
        if (!xmlConfig.Exists(context + '/'+ matrixPath.str())) {
            resultMatrix.SetSize(0.0, 0.0, 0.0);
            return true;
        }

        resultMatrix.SetSize(numRows, numCols, 0.0);
        for (int i = 0; i < numRows; i++) {
            // Construct XPath for matrix row
            std::ostringstream rowPath;
            rowPath << matrixPath.str() << "/Row[" << i + 1 << "]/@Val";

            // Get the matrix row text
            std::string rowAsString = "";
            xmlConfig.GetXMLValue(context, rowPath.str(), rowAsString);

            // Convert the text to a cisstVector row
            std::stringstream rowAsStringStream;
            rowAsStringStream.str(rowAsString);

            vctDoubleVec row;
            row.SetSize(numCols);

            if (!row.FromStreamRaw(rowAsStringStream)) {
                CMN_LOG_INIT_ERROR << "Row vector Assign failed on row " << i << ", path: " << rowPath.str() << std::endl;
                return false;
            }
            resultMatrix.Row(i).Assign(row);
        }
        return true;
    }

    bool osaXML1394ConfigureDigitalInput(cmnXMLPath & xmlConfig,
                                         const int inputIndex,
                                         osaDigitalInput1394Configuration & digitalInput)
    {
        // Digital Input Setup Stage
        char path[64];
        const char * context = "Config";
        bool tagsFound = true;

        //Check there is digital input entry. Return boolean result for success/fail.
        sprintf(path,"DigitalIn[%i]/@Name", inputIndex);
        tagsFound &= xmlConfig.GetXMLValue(context, path, digitalInput.Name);
        sprintf(path,"DigitalIn[%i]/@BoardID", inputIndex);
        tagsFound &= xmlConfig.GetXMLValue(context, path, digitalInput.BoardID);
        sprintf(path,"DigitalIn[%i]/@BitID", inputIndex);
        tagsFound &= xmlConfig.GetXMLValue(context, path, digitalInput.BitID);

        if (!tagsFound) {
            CMN_LOG_INIT_ERROR << "Configuration for " << path << " failed. Stopping config." << std::endl;
            return false;
        }

        int pressed_value = 0;
        sprintf(path, "DigitalIn[%i]/@Pressed", inputIndex);
        xmlConfig.GetXMLValue(context, path, pressed_value);
        digitalInput.PressedValue = bool(pressed_value);

        std::string trigger_modes;
        sprintf(path, "DigitalIn[%i]/@Trigger", inputIndex);
        xmlConfig.GetXMLValue(context, path, trigger_modes);

        digitalInput.TriggerWhenPressed = false;
        digitalInput.TriggerWhenReleased = false;

        if (trigger_modes == "all") {
            digitalInput.TriggerWhenPressed = true;
            digitalInput.TriggerWhenReleased = true;
        }
        else if (trigger_modes == "press") {
            digitalInput.TriggerWhenPressed = true;
        }
        else if (trigger_modes == "release") {
            digitalInput.TriggerWhenReleased = true;
        }
        else if (trigger_modes != "none") {
            // Should not come here during init.
            CMN_LOG_INIT_ERROR << "Unacceptable Trigger argument: " << trigger_modes << "." << std::endl
                               << "Trigger argument should be one of these: [all,press,release,none]." << std::endl;
            return false;
        }

        double debounce = 0.0;
        sprintf(path, "DigitalIn[%i]/@Debounce", inputIndex);
        xmlConfig.GetXMLValue(context, path, debounce, 0.0);
        if (debounce < 0.0) {
            debounce = 0.0;
            CMN_LOG_INIT_ERROR << "Configuration for " << path << " failed, you can't have a negative debounce value. Stopping config." << std::endl;
            return false;
        }
        digitalInput.DebounceThreshold = debounce;

        return true;
    }


    bool osaXML1394ConfigureDigitalOutput(cmnXMLPath & xmlConfig,
                                          const int outputIndex,
                                          osaDigitalOutput1394Configuration & digitalOutput)
    {
        // Digital Input Setup Stage
        char path[64];
        const char * context = "Config";
        bool tagsFound = true;

        // defaults
        digitalOutput.IsPWM = false;
        digitalOutput.HighDuration = 0.0;
        digitalOutput.LowDuration = 0.0;

        // Check there is digital output entry. Return boolean result for success/fail.
        sprintf(path,"DigitalOut[%i]/@Name", outputIndex);
        tagsFound &= xmlConfig.GetXMLValue(context, path, digitalOutput.Name);
        sprintf(path,"DigitalOut[%i]/@BoardID", outputIndex);
        tagsFound &= xmlConfig.GetXMLValue(context, path, digitalOutput.BoardID);
        sprintf(path,"DigitalOut[%i]/@BitID", outputIndex);
        tagsFound &= xmlConfig.GetXMLValue(context, path, digitalOutput.BitID);

        if (!tagsFound) {
            CMN_LOG_INIT_ERROR << "Configuration for " << path << " failed. Stopping config." << std::endl;
            return false;
        }

        // look for high/low duration
        sprintf(path,"DigitalOut[%i]/@HighDuration", outputIndex);
        tagsFound = xmlConfig.GetXMLValue(context, path, digitalOutput.HighDuration);
        sprintf(path,"DigitalOut[%i]/@LowDuration", outputIndex);
        tagsFound = xmlConfig.GetXMLValue(context, path, digitalOutput.LowDuration);

        // look for PWM settings
        sprintf(path,"DigitalOut[%i]/@Frequency", outputIndex);
        tagsFound = xmlConfig.GetXMLValue(context, path, digitalOutput.PWMFrequency);
        if (tagsFound) {
            digitalOutput.IsPWM = true;
        }
        return true;
    }


    bool osaXML1394ConfigureDallasChip(cmnXMLPath & xmlConfig,
                                       const int dallasIndex,
                                       osaDallasChip1394Configuration & dallasChip)
    {
        // Digital Input Setup Stage
        char path[64];
        const char * context = "Config";
        bool tagsFound = true;

        // Check there is digital output entry. Return boolean result for success/fail.
        sprintf(path,"DallasChip[%i]/@Name", dallasIndex);
        tagsFound &= xmlConfig.GetXMLValue(context, path, dallasChip.Name);
        sprintf(path,"DallasChip[%i]/@BoardID", dallasIndex);
        tagsFound &= xmlConfig.GetXMLValue(context, path, dallasChip.BoardID);

        if (!tagsFound) {
            CMN_LOG_INIT_ERROR << "Configuration for " << path << " failed. Stopping config." << std::endl;
            return false;
        }
        return true;
    }

}
