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

#ifndef _osaConfiguration1394_h
#define _osaConfiguration1394_h

#ifndef SAW_ROBOT_IO_1394_WO_CISST
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstParameterTypes/prmJointType.h>
#include <cisstParameterTypes/prmActuatorJointCoupling.h>
#include <sawRobotIO1394/sawRobotIO1394Revision.h>

#else // ifndef SAW_ROBOT_IO_1394_WO_CISST
#include <sawRobotIO1394/EigenWrapper.h>
#include <sawRobotIO1394/MinimalPrm.h>
#include <sawRobotIO1394/MinimalCmn.h>
#endif // ifndef SAW_ROBOT_IO_1394_WO_CISST

#include <stdexcept>

class AmpIO;

namespace sawRobotIO1394 {

    const size_t MAX_BOARDS = 16;
    const size_t MAX_AXES = 4;

    //! Exceptions for error handling
    class osaRuntimeError1394: public std::runtime_error {
    public:
        explicit osaRuntimeError1394(const std::string & what):
            std::runtime_error(what) { };
    };


    //! Defines where the potentiometers are positioned, if any.
    typedef enum {
        POTENTIOMETER_UNDEFINED,
        POTENTIOMETER_ON_ACTUATORS,
        POTENTIOMETER_ON_JOINTS
    } osaPot1394Location;

    struct osaDrive1394Configuration {
        double EffortToCurrentScale;
        double CurrentToBitsScale;
        double CurrentToBitsOffset;
        double BitsToCurrentScale;
        double BitsToCurrentOffset;
        double EffortCommandLimit;
        double CurrentCommandLimit;
    };

    struct osaEncoder1394Configuration {
        double BitsToPositionScale;
    };

    struct osaPot1394Configuration {
        double BitsToVoltageScale;
        double BitsToVoltageOffset;
        double VoltageToPositionScale;
        double VoltageToPositionOffset;
    };

    struct osaAnalogBrake1394Configuration {
        int BoardID;
        int AxisID;
        osaDrive1394Configuration Drive;
        double ReleaseCurrent;
        double ReleaseTime;
        double ReleasedCurrent;
        double EngagedCurrent;
    };

    struct osaActuator1394Configuration {
        int BoardID;
        int AxisID;
        prmJointType JointType;
        osaDrive1394Configuration Drive;
        osaEncoder1394Configuration Encoder;
        osaPot1394Configuration Pot;
        osaAnalogBrake1394Configuration * Brake; // 0 pointer for no brake
    };

    struct osaPotTolerance1394Configuration {
        int AxisID;
        double Distance;
        double Latency;
    };

    struct osaRobot1394Configuration {
        std::string Name;
        int NumberOfActuators;
        int NumberOfJoints;
        int SerialNumber;
        int NumberOfBrakes;
        bool OnlyIO;
        bool HasActuatorToJointCoupling;

        osaPot1394Location PotLocation;
        std::vector<osaPotTolerance1394Configuration> PotTolerances;

        std::vector<osaActuator1394Configuration> Actuators;

        prmActuatorJointCoupling Coupling;
    };

    struct osaDigitalInput1394Configuration {
        std::string Name;
        int BoardID;
        int BitID;
        bool TriggerWhenPressed;
        bool TriggerWhenReleased;
        bool PressedValue;
        double DebounceThreshold;
    };

    struct osaDigitalOutput1394Configuration {
        std::string Name;
        int BoardID;
        int BitID;
        double HighDuration;
        double LowDuration;
        bool IsPWM;
        double PWMFrequency;
    };

    struct osaDallasChip1394Configuration {
        std::string Name;
        int BoardID;
    };

    struct osaPort1394Configuration {
        std::vector<osaRobot1394Configuration> Robots;
        std::vector<osaDigitalInput1394Configuration> DigitalInputs;
        std::vector<osaDigitalOutput1394Configuration> DigitalOutputs;
        std::vector<osaDallasChip1394Configuration> DallasChips;
    };

    // Maps an actuator to the hardware (board and axis)
    struct osaActuatorMapping {
        AmpIO * Board;
        int Axis;
    };

    // maps a brake to the hardware
    struct osaBrakeMapping {
        AmpIO * Board;
        int Axis;
    };

} // namespace sawRobotIO1394

#endif // _osaConfiguration1394_h
