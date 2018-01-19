/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides, Anton Deguet
  Created on: 2011-06-10

  (C) Copyright 2011-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _osaRobot1394_h
#define _osaRobot1394_h

#include <vector>
#include <map>

#ifndef SAW_ROBOT_IO_1394_WO_CISST
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstParameterTypes/prmJointType.h>
#else
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "EigenWrapper.h"
#include "MinimalPrm.h"
#endif

#include <sawRobotIO1394/sawRobotIO1394ForwardDeclarations.h>
#include <sawRobotIO1394/osaConfiguration1394.h>
#include <sawRobotIO1394/sawRobotIO1394Export.h>

namespace sawRobotIO1394 {

    /**
     * IO1394 Robot Abstraction Layer
     * This class is responsible for robot-level actuation and unit
     * conversion as well as safety monitoring for the QLA robot control
     * architecture.
     **/
class CISST_EXPORT osaRobot1394
{
public:
    //! Watchdog counts per ms (note counter width, e.g. 16 bits)
    static const size_t WATCHDOG_MS_TO_COUNT = 192;

    /** \name Lifecycle
     *\{**/
    osaRobot1394(const osaRobot1394Configuration & config,
                 const size_t maxConsecutiveCurrentSafetyViolations = 100);

    void Configure(const osaRobot1394Configuration & config);

    void SetBoards(const std::vector<osaActuatorMapping> & actuatorBoards,
                   const std::vector<osaBrakeMapping> & brakeBoards);
    /**}**/

    /** \name State Update Functions
     * These functions interact with the lower-level hardware to query
     * information only and update this class' members.
     *\{**/
    void PollValidity(void);
    void PollState(void);
    void ConvertState(void);
    void CheckState(void);
    /**}**/

    void SetCoupling(const prmActuatorJointCoupling & coupling);

    /** \name Command Functions
     * These functions interact with the lower-level hardware when called to
     * change its state in some way. Note that these functions do not have
     * any side-effects in the class.
     *\{**/
    //
    //! Power / Safety Control
    void EnablePower(void);
    void EnableBoardsPower(void);
    void DisablePower(void);
    void DisableBoardPower(void);
    void WriteSafetyRelay(const bool & enabled);
    void SetWatchdogPeriod(const double & periodInSeconds);

    void SetActuatorPower(const bool & enabled);
    void SetActuatorPower(const vctBoolVec & enabled);
    void SetBrakePower(const bool & enabled);
    void SetBrakePower(const vctBoolVec & enabled);

    //! Encoder Control
    void SetEncoderPosition(const vctDoubleVec & pos);
    void SetEncoderPositionBits(const vctIntVec & bits);
    void SetSingleEncoderPosition(const int index, const double pos = 0);
    void SetSingleEncoderPositionBits(const int index, const int bits = 0);

    //! Pots to encoder safety check
    void UsePotsForSafetyCheck(const bool & usePotsForSafetyCheck);

    //! Actuator Control
    void SetJointEffort(const vctDoubleVec & efforts);
    void SetActuatorEffort(const vctDoubleVec & efforts);
    void SetActuatorCurrent(const vctDoubleVec & currents);
    void SetActuatorCurrentBits(const vctIntVec & bits);

    //! Brake Control
    void SetBrakeCurrent(const vctDoubleVec & currents);
    void SetBrakeCurrentBits(const vctIntVec & bits);
    void BrakeRelease(void);
    void BrakeEngage(void);
    /**}**/


    /** \name State Accessors
     * These accessors only access data which is contained in this class, i.e.
     * they do not interact with the lower-level hardware. To update these data
     * from the lower-level system, you must call \ref poll_state.
     *\{**/
    bool Valid(void) const;
    bool PowerStatus(void) const;
    bool SafetyRelay(void) const;
    bool WatchdogStatus(void) const;
    const vctBoolVec & ActuatorPowerStatus(void) const;
    const vctBoolVec & BrakePowerStatus(void) const;
    const vctDoubleVec & ActuatorCurrentFeedback(void) const;
    const vctDoubleVec & BrakeCurrentFeedback(void) const;
    const vctDoubleVec & PotPosition(void) const;
    const vctDoubleVec & ActuatorTimeStamp(void) const;
    const vctDoubleVec & BrakeTimeStamp(void) const;
    const vctDoubleVec & EncoderPosition(void) const;
    const vctDoubleVec & EncoderVelocity(void) const;
    const vctDoubleVec & EncoderAcceleration(void) const;
    const vctDoubleVec & EncoderVelocitySoftware(void) const;
    /**}**/

    /** \name Parameter Accessors
     *\{**/
    osaRobot1394Configuration GetConfiguration(void) const;
    std::string Name(void) const;
    size_t NumberOfJoints(void) const;
    size_t NumberOfActuators(void) const;
    size_t SerialNumber(void) const;
    size_t NumberOfBrakes(void) const;
    void GetJointTypes(prmJointTypeVec & jointTypes) const;
    void GetJointEffortCommandLimits(vctDoubleVec & limits) const;
    void GetActuatorEffortCommandLimits(vctDoubleVec & limits) const;
    void GetActuatorCurrentCommandLimits(vctDoubleVec & limits) const;
    /**}**/

    /** \name Bias Calibration Functions
     *\{**/
    void CalibrateEncoderOffsetsFromPots(void);
    /**}**/

    /** \name Conversion Functions
     * These functions convert data between units for various purposes. They
     * have no side-effects.
     *\{**/
    //! Conversions for encoders
    void EncoderPositionToBits(const vctDoubleVec & pos, vctIntVec & bits) const;
    void EncoderBitsToPosition(const vctIntVec & bits, vctDoubleVec & pos) const;
    void EncoderBitsToVelocity(vctDoubleVec & vel) const;
    
    //! Conversions for actuator current commands and measurements
    void ActuatorEffortToCurrent(const vctDoubleVec & efforts, vctDoubleVec & currents) const;
    void ActuatorCurrentToBits(const vctDoubleVec & currents, vctIntVec & bits) const;
    void ActuatorBitsToCurrent(const vctIntVec & bits, vctDoubleVec & currents) const;
    void ActuatorCurrentToEffort(const vctDoubleVec & currents, vctDoubleVec & efforts) const;

    //! Conversions for brake commands
    void BrakeCurrentToBits(const vctDoubleVec & currents, vctIntVec & bits) const;
    void BrakeBitsToCurrent(const vctIntVec & bits, vctDoubleVec & currents) const;

    //! Conversions for potentiometers
    void PotBitsToVoltage(const vctIntVec & bits, vctDoubleVec & voltages) const;
    void PotVoltageToPosition(const vctDoubleVec & voltages, vctDoubleVec & pos) const;
    /**}**/

protected:

    void ClipActuatorEffort(vctDoubleVec & efforts);
    void ClipActuatorCurrent(vctDoubleVec & currents);
    void ClipBrakeCurrent(vctDoubleVec & currents);

    //! Board Objects
    std::vector<osaActuatorMapping> mActuatorInfo;
    std::vector<osaBrakeMapping> mBrakeInfo;
    std::map<int, AmpIO*> mUniqueBoards;
    typedef std::map<int, AmpIO*>::iterator unique_board_iterator;
    typedef std::map<int, AmpIO*>::const_iterator unique_board_const_iterator;

    //! Robot Configuration
    osaRobot1394Configuration mConfiguration;
    std::string mName;
    size_t mNumberOfActuators;
    size_t mNumberOfJoints;
    size_t mNumberOfBrakes;
    size_t mSerialNumber;

    // state of brakes
    bool mBrakeReleasing;
    vctDoubleVec mBrakeReleasingTimer;

    //! Vectors of actuator properties
    vctDoubleVec
        mEffortToCurrentScales,
        mActuatorCurrentToBitsScales,
        mBrakeCurrentToBitsScales,
        mActuatorCurrentToBitsOffsets,
        mBrakeCurrentToBitsOffsets,
        mActuatorBitsToCurrentScales,
        mBrakeBitsToCurrentScales,
        mActuatorBitsToCurrentOffsets,
        mBrakeBitsToCurrentOffsets,
        mBitsToPositionScales,
        mBitsToVoltageScales,
        mBitsToVoltageOffsets,
        mVoltageToPositionScales,
        mVoltageToPositionOffsets;

    vctDoubleVec
        mJointEffortCommandLimits,
        mActuatorEffortCommandLimits,
        mActuatorCurrentCommandLimits,
        mBrakeCurrentCommandLimits,
        mActuatorCurrentFeedbackLimits, // limit used to trigger error
        mBrakeCurrentFeedbackLimits,    // limit used to trigger error
        mPotsToEncodersTolerance;       // maximum error between encoders and pots

    //! Robot type
    prmJointTypeVec mJointType;
    osaPot1394Location mPotType;
    bool mUsePotsForSafetyCheck;

    //! State Members
    bool
        mValid,
        mPowerStatus,
        mPreviousPowerStatus,
        mWatchdogStatus,
        mPreviousWatchdogStatus;

    double
        mWatchdogPeriod;

    unsigned int mLowestFirmWareVersion;

    bool mSafetyRelay;

    vctBoolVec
        mActuatorPowerStatus,
        mBrakePowerStatus,
        mActuatorPowerEnabled,
        mBrakePowerEnabled,
        mPotValid,
        mPreviousEncoderOverflow,
        mEncoderOverflow,
        mDigitalInputs,
        mEncoderChannelsA;

    vctIntVec
        mPotBits,
        mEncoderPositionBits,
        mEncoderPositionBitsPrev,
        mEncoderDPositionBits,
        mEncoderDTimeBits;
    
    vctIntVec
        mActuatorCurrentBitsCommand,
        mBrakeCurrentBitsCommand,
        mActuatorCurrentBitsFeedback,
        mBrakeCurrentBitsFeedback;

    vctDoubleVec
        mActuatorTimestamp,
        mActuatorTimestampChange, // cumulated time since last encoder changed
        mVelocitySlopeToZero, // slope used to reduced velocity to zero when no encoder count change
        mBrakeTimestamp,
        mPotVoltage,
        mPotPosition,
        mEncoderPosition,
        mEncoderVelocity,
        mEncoderVelocityBits,     // latched velocity
        mEncoderAcceleration,
        mEncoderVelocitySoftware,
        mJointPosition,
        mJointVelocity,
        mJointTorque,
        mActuatorCurrentCommand,
        mBrakeCurrentCommand,
        mActuatorEffortCommand,
        mActuatorCurrentFeedback,
        mPotToleranceLatency,
        mPotToleranceDistance,
        mPotErrorDuration,
        mBrakeCurrentFeedback,
        mActuatorEffortFeedback,
        mActuatorTemperature,
        mBrakeTemperature,
        mBrakeReleaseCurrent,
        mBrakeReleaseTime,
        mBrakeReleasedCurrent,
        mBrakeEngagedCurrent;

    size_t
        mCurrentSafetyViolationsCounter,
        mCurrentSafetyViolationsMaximum;

    size_t mInvalidReadCounter;
    };

} // namespace sawRobotIO1394

#endif // _osaRobot1394_h
