/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2011-06-10

  (C) Copyright 2011-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsRobot1394_h
#define _mtsRobot1394_h

#include <cisstParameterTypes/prmJointType.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmVelocityJointGet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>

namespace sawRobotIO1394 {

    class mtsRobot1394 {
    public:
        /*! Pointer on existing services.  This allows to use the class
          name and level of detail of another class, e.g. the class that
          owns this map.  To set the "Owner", use the method SetOwner
          after the cmnNamedMap is constructed. */
        const cmnClassServicesBase * OwnerServices;

        /*! Method used to emulate the cmnGenericObject interface used by
          CMN_LOG_CLASS macros. */
        //@{
        inline const cmnClassServicesBase * Services(void) const {
            return this->OwnerServices;
        }

        inline cmnLogger::StreamBufType * GetLogMultiplexer(void) const {
            return cmnLogger::GetMultiplexer();
        }
        //@}

        mtsRobot1394(const cmnGenericObject & owner,
                     const osaRobot1394Configuration & config);
        ~mtsRobot1394();

        void Configure(const osaRobot1394Configuration & config);

        bool SetupStateTables(const size_t stateTableSize,
                              mtsStateTable * & stateTableRead,
                              mtsStateTable * & stateTableWrite);
        void SetupInterfaces(mtsInterfaceProvided * robotInterface,
                             mtsInterfaceProvided * actuatorInterface);

        void StartReadStateTable(void);
        void AdvanceReadStateTable(void);
        void StartWriteStateTable(void);
        void AdvanceWriteStateTable(void);
        bool CheckConfiguration(void);

        // Wrapper of osa methods to match command signatures
        void GetNumberOfActuators(int & num_actuators) const;
        void GetNumberOfJoints(int & num_joints) const;
        void GetSerialNumber(int & serialNumber) const;
        void SetTorqueJoint(const prmForceTorqueJointSet & jointTorques);
        void ResetSingleEncoder(const int & index);
        void SetCoupling(const prmActuatorJointCoupling & coupling);

        /*! \name Bias Calibration */
        void CalibrateEncoderOffsetsFromPots(const int & numberOfSamples);

        //! Watchdog counts per ms (note counter width, e.g. 16 bits)
        static const size_t WATCHDOG_MS_TO_COUNT = 192;

        /** \name Lifecycle
         *\{**/
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
        const vctDoubleVec & EncoderVelocityPredicted(void) const;
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
        void EncoderBitsToVelocityPredicted(vctDoubleVec & vel) const;

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
            mFirstWatchdog,
            mWatchdogStatus,
            mPreviousWatchdogStatus;

        double mWatchdogPeriod;

        unsigned int mLowestFirmWareVersion;
        unsigned int mHighestFirmWareVersion;

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
            mEncoderDPositionBits;

        vctIntVec
            mActuatorCurrentBitsCommand,
            mBrakeCurrentBitsCommand,
            mActuatorCurrentBitsFeedback,
            mBrakeCurrentBitsFeedback;

        vctDoubleVec
            mActuatorTimestamp,
            mActuatorTimestampChange, // cumulated time since last encoder changed
            mActuatorPreviousTimestampChange,
            mVelocitySlopeToZero, // slope used to reduced velocity to zero when no encoder count change
            mBrakeTimestamp,
            mPotVoltage,
            mPotPosition,
            mEncoderPosition,

            mEncoderVelocityCountsPerSecond,  // velocity based on FPGA measurement of time between encoder edges (period)
            mEncoderVelocity,                 // velocity passed to higher level (SI units)
            mEncoderVelocityDelay,            // assumed delay in velocity measurement (period/2)
            mEncoderVelocityPredicted,        // velocity based on FPGA measurement, combined with prediction based on acceleration (SI units)
            mEncoderVelocitySoftware,         // velocity based on backward difference of position (SI units)
            mEncoderAccelerationCountsPerSecSec, // acceleration based on FPGA measurement (firmware rev 6)
            mEncoderAcceleration,             // acceleration in SI units (firmware rev 6)
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

        mtsStateTable * mStateTableRead;
        mtsStateTable * mStateTableWrite;
        bool mUserExpectsPower;

        prmForceTorqueJointSet mTorqueJoint;
        prmPositionJointGet mPositionJointGet;
        prmPositionJointGet mPositionActuatorGet;
        prmVelocityJointGet mVelocityJointGet;

        // Functions for events
        struct {
            mtsFunctionWrite PowerStatus;
            mtsFunctionWrite WatchdogStatus;
            mtsFunctionWrite WatchdogPeriod;
            mtsFunctionWrite Coupling;
            mtsFunctionWrite BiasEncoder;
            mtsFunctionWrite UsePotsForSafetyCheck;
        } EventTriggers;

        int mSamplesForCalibrateEncoderOffsetsFromPots;
        int mSamplesForCalibrateEncoderOffsetsFromPotsRequested;
        mtsStateTable::Accessor<vctDoubleVec> * mPotPositionAccessor;
        mtsStateTable::Accessor<prmPositionJointGet> * mPositionActuatorGetAccessor;

    public:
        mtsInterfaceProvided * mInterface;
    };

} // namespace sawRobotIO1394

#endif // _mtsRobot1394_h





#if TO_BE_PORTED

    };



#endif // _osaRobot1394_h
