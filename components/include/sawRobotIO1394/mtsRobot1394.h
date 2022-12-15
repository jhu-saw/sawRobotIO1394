/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2011-06-10

  (C) Copyright 2011-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsRobot1394_h
#define _mtsRobot1394_h

#include <cisstParameterTypes/prmMaskedVector.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>

#include <sawRobotIO1394/osaConfiguration1394.h>
#include <sawRobotIO1394/sawRobotIO1394ForwardDeclarations.h>

// Always include last
#include <sawRobotIO1394/sawRobotIO1394Export.h>

namespace sawRobotIO1394 {

    class CISST_EXPORT mtsRobot1394 {
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
        void SetupInterfaces(mtsInterfaceProvided * robotInterface);

        void StartReadStateTable(void);
        void AdvanceReadStateTable(void);
        void StartWriteStateTable(void);
        void AdvanceWriteStateTable(void);
        bool CheckConfiguration(void);

        // Wrapper of osa methods to match command signatures
        void GetNumberOfActuators(size_t & num_actuators) const;
        void GetSerialNumber(int & serialNumber) const;
        void servo_jf(const prmForceTorqueJointSet & jointTorques);
        void SetSomeEncoderPosition(const prmMaskedDoubleVec & values);
        void ActuatorToJointPosition(const vctDoubleVec & actuators,
                                     vctDoubleVec & joints) const;

        /*! \name Bias Calibration */
        void CalibrateEncoderOffsetsFromPots(const int & numberOfSamples);


        /** \name Lifecycle
         *\{**/
        void SetBoards(const std::vector<osaActuatorMapping> & actuatorBoards,
                       const std::vector<osaBrakeMapping> & brakeBoards);

        void GetFirmwareRange(unsigned int & lowest, unsigned int & highest) const;
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
        void PowerOnSequence(void);
        void PowerOffSequence(const bool & openSafetyRelays = true);
        void WritePowerEnable(const bool & power);
        void WriteSafetyRelay(const bool & close);
        void SetWatchdogPeriod(const double & periodInSeconds);

        void SetActuatorAmpEnable(const bool & enable);
        void SetActuatorAmpEnable(const vctBoolVec & enable);
        void SetBrakeAmpEnable(const bool & enable);
        void SetBrakeAmpEnable(const vctBoolVec & enable);

        //! Encoder Control
        void SetEncoderPosition(const vctDoubleVec & pos);
        void SetEncoderPositionBits(const vctIntVec & bits);
        void SetSingleEncoderPosition(const int index, const double pos = 0);
        void SetSingleEncoderPositionBits(const int index, const int bits = 0);

        //! Pots to encoder safety check
        void UsePotsForSafetyCheck(const bool & usePotsForSafetyCheck);

        //! Actuator Control
        void SetActuatorEffort(const vctDoubleVec & efforts);
        void SetActuatorCurrent(const vctDoubleVec & currents);
        void SetActuatorCurrentBits(const vctIntVec & bits);
        void SetActuatorVoltageRatio(const vctDoubleVec & ratios);

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
        inline bool Valid(void) const {
            return mValid;
        }

        //! All power is good, i.e. safety relays, power and amps status
        inline bool FullyPowered(void) const {
            return mFullyPowered;
        }
        //! Desired state from FPGA for power (board level)
        inline bool PowerEnable(void) const {
            return mPowerEnable;
        }
        //! Power status, as detected by MV good (board level)
        inline bool PowerStatus(void) const {
            return mPowerStatus;
        }
        //! Power fault (board level)
        inline bool PowerFault(void) const {
            return mPowerFault;
        }
        //! Safety relay desired state
        inline bool SafetyRelay(void) const {
            return mSafetyRelay;
        }
        //! Safety relay status
        inline bool SafetyRelayStatus(void) const {
            return mSafetyRelayStatus;
        }
        //! Watchdog timeout status, true for triggered
        bool WatchdogTimeoutStatus(void) const;

        inline const vctBoolVec & ActuatorAmpStatus(void) const {
            return mActuatorAmpStatus;
        }

        inline const vctBoolVec & ActuatorAmpEnable(void) const {
            return mActuatorAmpEnable;
        }

        inline const vctBoolVec & BrakeAmpStatus(void) const {
            return mBrakeAmpStatus;
        }

        inline const vctBoolVec & BrakeAmpEnable(void) const {
            return mBrakeAmpEnable;
        }

        const vctDoubleVec & ActuatorCurrentFeedback(void) const;
        const vctDoubleVec & ActuatorCurrentCommand(void) const;
        const vctDoubleVec & ActuatorEffortCommand(void) const;
        const vctDoubleVec & BrakeCurrentFeedback(void) const;
        const vctIntVec & PotBits(void) const;
        const vctDoubleVec & PotPosition(void) const;
        const vctDoubleVec & ActuatorTimestamp(void) const;
        const vctDoubleVec & BrakeTimestamp(void) const;
        const vctDoubleVec & ActuatorEncoderAcceleration(void) const;
        const vctDoubleVec & EncoderAcceleration(void) const;
        const prmStateJoint & ActuatorJointState(void) const;
        /**}**/

        /** \name Parameter Accessors
         *\{**/
        osaRobot1394Configuration GetConfiguration(void) const;
        std::string Name(void) const;
        size_t NumberOfActuators(void) const;
        size_t SerialNumber(void) const;
        size_t NumberOfBrakes(void) const;
        void configuration_js(prmConfigurationJoint & jointConfig) const;
        void configure_js(const prmConfigurationJoint & jointConfig);
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

        //! Robot Configuration
        osaRobot1394Configuration mConfiguration;
        std::string mName;
        size_t mNumberOfActuators;
        size_t mNumberOfBrakes;
        size_t mSerialNumber;
        bool mHasEncoderPreload;

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
            mBitsToPositionOffsets, // this is used only if the hardware doesn't allow encoder pre-loading
            mBitsToVoltageScales,
            mBitsToVoltageOffsets,
            mSensorToPositionScales,
            mSensorToPositionOffsets;

        vctDoubleVec
            mActuatorEffortCommandLimits,
            mActuatorCurrentCommandLimits,
            mBrakeCurrentCommandLimits,
            mActuatorCurrentFeedbackLimits, // limit used to trigger error
            mBrakeCurrentFeedbackLimits,    // limit used to trigger error
            mPotsToEncodersTolerance;       // maximum error between encoders and pots

        //! Robot type
        prmConfigurationJoint mConfigurationJoint;
        int mPotType = 0; // 0 for undefined, 1 for analog, 2 for digital (dVRK S)
        vctDoubleMat mPotCoupling;
        bool mUsePotsForSafetyCheck;

        //! State Members
        bool
            mValid,
            mFullyPowered,
            mPreviousFullyPowered,
            mPowerEnable,
            mPowerStatus,
            mPowerFault,
            mPreviousPowerFault,
            mWatchdogTimeoutStatus,
            mPreviousWatchdogTimeoutStatus;

        double mWatchdogPeriod;

        unsigned int mLowestFirmWareVersion;
        unsigned int mHighestFirmWareVersion;

        bool mSafetyRelay, mSafetyRelayStatus;
        bool mSafetyAmpDisabled = false; // disabled at firmware level

        vctBoolVec
            mActuatorAmpStatus,
            mBrakeAmpStatus,
            mActuatorAmpEnable,
            mBrakeAmpEnable,
            mPotValid,
            mPreviousEncoderOverflow,
            mEncoderOverflow,
            mDigitalInputs,
            mEncoderChannelsA;

        vctIntVec
            mPotBits,
            mEncoderPositionBits,
            mPreviousEncoderPositionBits;
            // mEncoderDPositionBits;

        vctIntVec
            mActuatorCurrentBitsCommand,
            mBrakeCurrentBitsCommand,
            mActuatorCurrentBitsFeedback,
            mBrakeCurrentBitsFeedback;

        vctDoubleVec
            mActuatorTimestamp,
            mBrakeTimestamp,
            mPotVoltage,
            mActuatorTimestampChange, // software velocity: cumulated time since last encoder changed
            mVelocitySlopeToZero,     // software velocity: slope used to reduced velocity to zero when no encoder count change
            mEncoderVelocityPredictedCountsPerSec, // velocity based on FPGA velocity estimation, including prediction
            mEncoderAccelerationCountsPerSecSec,   // acceleration based on FPGA measurement (firmware rev 6)
            mEncoderAcceleration,                  // acceleration in SI units (firmware rev 6)
            mActuatorEncoderAcceleration,
            mActuatorCurrentCommand,
            mBrakeCurrentCommand,
            mActuatorEffortCommand,
            mActuatorCurrentFeedback,
            mPotToleranceLatency,
            mPotToleranceDistance,
            mPotErrorDuration,
            mBrakeCurrentFeedback,
            mActuatorTemperature,
            mBrakeTemperature,
            mBrakeReleaseCurrent,
            mBrakeReleaseTime,
            mBrakeReleasedCurrent,
            mBrakeEngagedCurrent;

        vctDynamicVector<vctDoubleVec> mPotLookupTable;

        size_t
            mCurrentSafetyViolationsCounter,
            mCurrentSafetyViolationsMaximum;

        size_t
            mInvalidReadCounter = 0;

        double
            mTimeLastTemperatureWarning = sawRobotIO1394::TimeBetweenTemperatureWarnings;

        mtsStateTable * mStateTableRead;
        mtsStateTable * mStateTableWrite;
        bool mUserExpectsPower;
        double mPoweringStartTime;

        prmForceTorqueJointSet mTorqueJoint;
        prmStateJoint m_measured_js, m_software_measured_js, m_raw_pot_measured_js, m_pot_measured_js;

        // Functions for events
        struct {
            mtsFunctionWrite SafetyRelay;
            mtsFunctionWrite SafetyRelayStatus;
            mtsFunctionWrite FullyPowered;
            mtsFunctionWrite PowerFault;
            mtsFunctionWrite WatchdogTimeoutStatus;
            mtsFunctionWrite WatchdogPeriod;
            mtsFunctionWrite BiasEncoder;
            mtsFunctionWrite UsePotsForSafetyCheck;
        } EventTriggers;

        struct {
            int SamplesFromPots = 0;
            int SamplesFromPotsRequested;
            bool Performed = false;
            int PostCalibrationCounter = -1; // -1: nothing to do, 0: emit event, anything else: decrement
        } CalibrateEncoderOffsets;

        mtsStateTable::Accessor<prmStateJoint> * m_pot_measured_js_accessor;
        mtsStateTable::Accessor<prmStateJoint> * m_measured_js_accessor;

    public:
        mtsInterfaceProvided * mInterface;
    };

} // namespace sawRobotIO1394

#endif // _mtsRobot1394_h
