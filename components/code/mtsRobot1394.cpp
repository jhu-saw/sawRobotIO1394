/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides, Anton Deguet
  Created on: 2011-06-10

  (C) Copyright 2011-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cmath>

#include <cisstNumerical/nmrInverse.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsStateTable.h>

#include <Amp1394/AmpIORevision.h>
#if ((Amp1394_VERSION_MAJOR < 1) || ((Amp1394_VERSION_MAJOR == 1) && (Amp1394_VERSION_MINOR < 1)))
#error "Version 1.1 or higher of libAmpIO is required (change to signed encoder positions)"
#endif

#include <AmpIO.h>

#include <sawRobotIO1394/mtsRobot1394.h>

using namespace sawRobotIO1394;

mtsRobot1394::mtsRobot1394(const cmnGenericObject & owner,
                           const osaRobot1394Configuration & config):
    OwnerServices(owner.Services()),
    // IO Structures
    mActuatorInfo(),
    mUniqueBoards(),
    // State Initialization
    mValid(false),
    mPowerStatus(false),
    mPreviousPowerStatus(false),
    mWatchdogStatus(true),
    mPreviousWatchdogStatus(true),
    mWatchdogPeriod(15.0),
    mSafetyRelay(false),
    mCurrentSafetyViolationsCounter(0),
    mCurrentSafetyViolationsMaximum(100),
    mStateTableRead(0),
    mStateTableWrite(0),
    mSamplesForCalibrateEncoderOffsetsFromPots(0)
{
    this->Configure(config);
}

mtsRobot1394::~mtsRobot1394()
{
    delete mStateTableRead;
    delete mStateTableWrite;
}

bool mtsRobot1394::SetupStateTables(const size_t stateTableSize,
                                    mtsStateTable * & stateTableRead,
                                    mtsStateTable * & stateTableWrite)
{
    if (mStateTableRead || mStateTableWrite) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupStateTables: state tables have already been created for robot: "
                                 << this->Name() << std::endl;
        return false;
    }

    mStateTableRead = new mtsStateTable(stateTableSize, this->Name() + "Read");
    mStateTableRead->SetAutomaticAdvance(false);
    mStateTableWrite = new mtsStateTable(stateTableSize, this->Name() + "Write");
    mStateTableWrite->SetAutomaticAdvance(false);

    mStateTableRead->AddData(mEncoderChannelsA, "EncoderChannelA");
    mStateTableRead->AddData(mValid, "Valid");
    mStateTableRead->AddData(mPowerStatus, "PowerStatus");
    mStateTableRead->AddData(mSafetyRelay, "SafetyRelay");
    mStateTableRead->AddData(mWatchdogStatus, "WatchdogStatus");
    mStateTableRead->AddData(mWatchdogPeriod, "WatchdogPeriod");
    mStateTableRead->AddData(mActuatorTemperature, "ActuatorTemperature");
    mStateTableRead->AddData(mActuatorPowerStatus, "ActuatorPowerStatus");
    mStateTableRead->AddData(mActuatorPowerEnabled, "ActuatorPowerEnabled");
    mStateTableRead->AddData(mEncoderPositionBits, "PositionEncoderRaw");
    mStateTableRead->AddData(mEncoderAcceleration, "Accel");
    mStateTableRead->AddData(mPotBits, "AnalogInRaw");
    mStateTableRead->AddData(mPotVoltage, "AnalogInVolts");
    mStateTableRead->AddData(mPotPosition, "AnalogInPosSI");
    mStateTableRead->AddData(mActuatorCurrentBitsFeedback, "ActuatorFeedbackCurrentRaw");
    mStateTableRead->AddData(mActuatorCurrentFeedback, "ActuatorFeedbackCurrent");

    mStateTableRead->AddData(mMeasuredJS, "measured_js");
    mStateTableRead->AddData(mActuatorMeasuredJS, "actuator_measured_js");

    mStateTableWrite->AddData(mActuatorCurrentBitsCommand, "ActuatorControlCurrentRaw");
    mStateTableWrite->AddData(mActuatorCurrentCommand, "ActuatorControlCurrent");

    mStateTableRead->AddData(mBrakePowerStatus, "BrakePowerStatus");
    mStateTableRead->AddData(mBrakePowerEnabled, "BrakePowerEnabled");
    mStateTableWrite->AddData(mBrakeCurrentBitsCommand, "BrakeControlCurrentRaw");
    mStateTableWrite->AddData(mBrakeCurrentCommand, "BrakeControlCurrent");
    mStateTableRead->AddData(mBrakeCurrentFeedback, "BrakeFeedbackCurrent");
    mStateTableRead->AddData(mBrakeTemperature, "BrakeTemperature");

    mtsStateTable::AccessorBase * accessorBase;
    accessorBase = mStateTableRead->GetAccessor(mPotPosition);
    CMN_ASSERT(accessorBase);
    mPotPositionAccessor = dynamic_cast<mtsStateTable::Accessor<vctDoubleVec>* >(accessorBase);
    CMN_ASSERT(mPotPositionAccessor);
    accessorBase = mStateTableRead->GetAccessor(mActuatorMeasuredJS);
    CMN_ASSERT(accessorBase);
    mActuatorStateJointAccessor = dynamic_cast<mtsStateTable::Accessor<prmStateJoint>*>(accessorBase);
    CMN_ASSERT(mActuatorStateJointAccessor);

    stateTableRead = mStateTableRead;
    stateTableWrite = mStateTableWrite;
    return true;
}

void mtsRobot1394::StartReadStateTable(void) {
    mStateTableRead->Start();
}

void mtsRobot1394::AdvanceReadStateTable(void) {
    mStateTableRead->Advance();
}

void mtsRobot1394::StartWriteStateTable(void) {
    mStateTableWrite->Start();
}

void mtsRobot1394::AdvanceWriteStateTable(void) {
    mStateTableWrite->Advance();
}

void mtsRobot1394::GetNumberOfActuators(int & numberOfActuators) const {
    numberOfActuators = this->NumberOfActuators();
}

void mtsRobot1394::GetNumberOfJoints(int & numberOfJoints) const {
    numberOfJoints = this->NumberOfJoints();
}

void mtsRobot1394::GetSerialNumber(int & serialNumber) const {
    serialNumber = this->SerialNumber();
}

void mtsRobot1394::UsePotsForSafetyCheck(const bool & usePotsForSafetyCheck)
{
    mUsePotsForSafetyCheck = usePotsForSafetyCheck;
    mPotErrorDuration.SetAll(0.0);
    mPotValid.SetAll(true);
    // trigger mts event
    EventTriggers.UsePotsForSafetyCheck(usePotsForSafetyCheck);
}

void mtsRobot1394::servo_jf(const prmForceTorqueJointSet & efforts) {
    this->SetJointEffort(efforts.ForceTorque());
}

void mtsRobot1394::SetSomeEncoderPosition(const prmMaskedDoubleVec & values) {
    for (size_t index = 0; index < values.Mask().size(); ++index) {
        if (values.Mask().at(index)) {
            this->SetSingleEncoderPosition(index, values.Data().at(index));
        }
    }
}

void mtsRobot1394::SetCoupling(const prmActuatorJointCoupling & coupling)
{
    // check sizes
    if ((coupling.ActuatorToJointPosition().rows() != mNumberOfJoints) ||
        (coupling.ActuatorToJointPosition().cols() != mNumberOfActuators)) {
        cmnThrow("SetCoupling: invalid size for ActuatorToJointPosition");
    }
    mConfiguration.Coupling.ActuatorToJointPosition()
        .ForceAssign(coupling.ActuatorToJointPosition());

    // if we get an empty matrix, compute the inverse
    if (coupling.JointToActuatorPosition().size() == 0) {
        mConfiguration.Coupling.JointToActuatorPosition()
            .ForceAssign(coupling.ActuatorToJointPosition());
        nmrInverse(mConfiguration.Coupling.JointToActuatorPosition());
    } else {
        if ((coupling.JointToActuatorPosition().rows() != mNumberOfActuators) ||
            (coupling.JointToActuatorPosition().cols() != mNumberOfJoints)) {
            cmnThrow("SetCoupling: invalid size for JointToActuatorPosition");
        }
        mConfiguration.Coupling.JointToActuatorPosition()
            .ForceAssign(coupling.JointToActuatorPosition());
    }

    // if we get an empty matrix, compute the transpose
    if (coupling.ActuatorToJointEffort().size() == 0) {
        mConfiguration.Coupling.ActuatorToJointEffort()
            .ForceAssign(mConfiguration.Coupling.JointToActuatorPosition().Transpose());
    } else {
        if ((coupling.ActuatorToJointEffort().rows() != mNumberOfJoints) ||
            (coupling.ActuatorToJointEffort().cols() != mNumberOfActuators)) {
            cmnThrow("SetCoupling: invalid size for ActuatorToJointEffort");
        }
        mConfiguration.Coupling.ActuatorToJointEffort()
            .ForceAssign(coupling.ActuatorToJointEffort());
    }

    // if we get an empty matrix, compute the transpose
    if (coupling.JointToActuatorEffort().size() == 0) {
        mConfiguration.Coupling.JointToActuatorEffort()
            .ForceAssign(mConfiguration.Coupling.ActuatorToJointEffort());
        nmrInverse(mConfiguration.Coupling.JointToActuatorEffort());
    } else {
        if ((coupling.JointToActuatorEffort().rows() != mNumberOfActuators) ||
            (coupling.JointToActuatorEffort().cols() != mNumberOfJoints)) {
            cmnThrow("SetCoupling: invalid size for JointToActuatorEffort");
        }
        mConfiguration.Coupling.JointToActuatorEffort()
            .ForceAssign(coupling.JointToActuatorEffort());
    }

    // assign values
    mConfiguration.HasActuatorToJointCoupling = true;

    // check for identity using inverse
    const vctDoubleMat identity = vctDoubleMat::Eye(mNumberOfActuators);
    vctDoubleMat product;
    product.SetSize(mNumberOfActuators, mNumberOfActuators);
    product.ProductOf(mConfiguration.Coupling.ActuatorToJointPosition(),
                      mConfiguration.Coupling.JointToActuatorPosition());
    if (!product.AlmostEqual(identity, 0.001)) {
        cmnThrow("SetCoupling: product of position coupling matrices not identity");
    }
    product.ProductOf(mConfiguration.Coupling.ActuatorToJointEffort(),
                      mConfiguration.Coupling.JointToActuatorEffort());
    if (!product.AlmostEqual(identity, 0.001)) {
        cmnThrow("ConfigureCoupling: product of torque coupling matrices not identity");
    }

    // start state table and get new data
    StartReadStateTable();
    {
        PollValidity();
        PollState();
        ConvertState();
        CheckState();
    }
    AdvanceReadStateTable();
    // finally let users the coupling has changed
    EventTriggers.Coupling(coupling);
}

void mtsRobot1394::EnablePower(void)
{
    mUserExpectsPower = true;
    mTimeLastTemperatureWarning = sawRobotIO1394::TimeBetweenTemperatureWarnings;
    EnableBoardsPower();
    SetActuatorPower(true);
    SetBrakePower(true);
}

void mtsRobot1394::EnableBoardsPower(void)
{
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        board->second->WriteSafetyRelay(true);
        board->second->WritePowerEnable(true);
    }
}

void mtsRobot1394::DisablePower(void)
{
    mUserExpectsPower = false;
    // write to boards directly
    // disable all axes
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        board->second->WriteAmpEnable(0x0f, 0x00);
    }

    // disable all boards
    DisableBoardPower();
    mPreviousPowerStatus = false;
}

void mtsRobot1394::DisableBoardPower(void)
{
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        board->second->WritePowerEnable(false);
        board->second->WriteSafetyRelay(false);
    }
}

void mtsRobot1394::SetWatchdogPeriod(const double & periodInSeconds)
{
    uint32_t periodCount;
    if (periodInSeconds == 0.0) {
        // Disable watchdog
        periodCount = 0;
    } else {
        // Use at least one tick just to make sure we don't accidentaly disable
        // the truth is that the count will be so low that watchdog will
        // continuously trigger.
        periodCount = (periodInSeconds * 1000.0) * WATCHDOG_MS_TO_COUNT;
        periodCount = std::max(periodCount, static_cast<uint32_t>(1));
    }

    // update local copy of watchdog period based on final period count
    mWatchdogPeriod = (periodCount / static_cast<double>(WATCHDOG_MS_TO_COUNT)) * 0.001;

    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        board->second->WriteWatchdogPeriod(periodCount);
    }

    EventTriggers.WatchdogPeriod(mWatchdogPeriod);
}

void mtsRobot1394::CalibrateEncoderOffsetsFromPots(const int & numberOfSamples)
{
    // if the number of samples is negative, user wants to first check
    // if the encoders are not already preloaded
    if (numberOfSamples < 0) {
        // get encoder preload values re. midrange
        bool isMidrange = true;
        for (size_t i = 0; i < mNumberOfActuators; i++) {
            bool thisAxis;
            mActuatorInfo[i].Board->IsEncoderPreloadMidrange(mActuatorInfo[i].Axis, thisAxis);
            isMidrange = (isMidrange && thisAxis);
        }
        // if all encoders are at midrange, assume encoder bias on pots is already done
        if (!isMidrange) {
            mSamplesForCalibrateEncoderOffsetsFromPots = 0;
            EventTriggers.BiasEncoder(-1);
            return;
        }
    }
    mSamplesForCalibrateEncoderOffsetsFromPots = std::abs(numberOfSamples) + 1;
    mSamplesForCalibrateEncoderOffsetsFromPotsRequested = std::abs(numberOfSamples);
}

void mtsRobot1394::SetupInterfaces(mtsInterfaceProvided * robotInterface,
                                   mtsInterfaceProvided * actuatorInterface)
{
    mInterface = robotInterface;
    robotInterface->AddMessageEvents();

    robotInterface->AddCommandRead(&mtsRobot1394::GetNumberOfActuators, this,
                                   "GetNumberOfActuators");
    robotInterface->AddCommandRead(&mtsRobot1394::GetNumberOfJoints, this,
                                   "GetNumberOfJoints");
    robotInterface->AddCommandRead(&mtsRobot1394::GetSerialNumber, this,
                                   "GetSerialNumber");
    robotInterface->AddCommandReadState(*mStateTableRead, this->mValid,
                                        "IsValid");

    robotInterface->AddCommandWrite(&mtsRobot1394::SetCoupling, this,
                                    "SetCoupling");

    // Enable // Disable
    robotInterface->AddCommandVoid(&mtsRobot1394::EnablePower, this,
                                   "EnablePower");
    robotInterface->AddCommandVoid(&mtsRobot1394::DisablePower, this,
                                   "DisablePower");

    robotInterface->AddCommandWrite(&mtsRobot1394::SetWatchdogPeriod, this,
                                    "SetWatchdogPeriod");

    robotInterface->AddCommandReadState(*mStateTableRead, mActuatorPowerEnabled,
                                        "GetActuatorAmpEnable"); // vector[bool]
    robotInterface->AddCommandReadState(*mStateTableRead, mActuatorPowerStatus,
                                        "GetActuatorAmpStatus"); // vector[bool]

    robotInterface->AddCommandReadState(*mStateTableRead, mPowerStatus,
                                        "GetPowerStatus"); // bool
    robotInterface->AddCommandReadState(*mStateTableRead, mSafetyRelay,
                                        "GetSafetyRelay"); // unsigned short
    robotInterface->AddCommandReadState(*mStateTableRead, mWatchdogStatus,
                                        "GetWatchdogStatus"); // bool
    robotInterface->AddCommandReadState(*mStateTableRead, mWatchdogPeriod,
                                        "GetWatchdogPeriod"); // double
    robotInterface->AddCommandReadState(*mStateTableRead, mActuatorTemperature,
                                        "GetActuatorAmpTemperature"); // vector[double]

    robotInterface->AddCommandReadState(*mStateTableRead, mEncoderChannelsA,
                                        "GetEncoderChannelA"); // vector[bool]
    robotInterface->AddCommandReadState(*mStateTableRead, mEncoderPositionBits,
                                        "GetPositionEncoderRaw"); // vector[int]

    robotInterface->AddCommandReadState(*mStateTableRead, mEncoderAcceleration,
                                        "GetAcceleration"); // vector[double]

    robotInterface->AddCommandReadState(*mStateTableRead, this->mMeasuredJS,
                                        "measured_js");
    robotInterface->AddCommandReadState(*mStateTableRead, mActuatorMeasuredJS,
                                        "actuator_measured_js");

    robotInterface->AddCommandReadState(*mStateTableRead, mPotBits,
                                        "GetAnalogInputRaw");
    robotInterface->AddCommandReadState(*mStateTableRead, mPotVoltage,
                                        "GetAnalogInputVolts");
    robotInterface->AddCommandReadState(*mStateTableRead, mPotPosition,
                                        "GetAnalogInputPosSI");

    robotInterface->AddCommandReadState(*mStateTableRead, mActuatorCurrentBitsFeedback,
                                        "GetActuatorFeedbackCurrentRaw");
    robotInterface->AddCommandReadState(*mStateTableRead, mActuatorCurrentFeedback,
                                        "GetActuatorFeedbackCurrent");
    robotInterface->AddCommandReadState(*mStateTableWrite, mActuatorCurrentCommand,
                                        "GetActuatorRequestedCurrent");

    robotInterface->AddCommandWrite(&mtsRobot1394::UsePotsForSafetyCheck, this,
                                    "UsePotsForSafetyCheck", mUsePotsForSafetyCheck);

    robotInterface->AddCommandWrite<mtsRobot1394, vctBoolVec>(&mtsRobot1394::SetBrakePower, this,
                                                              "SetBrakeAmpEnable", mBrakePowerEnabled); // vector[bool]
    robotInterface->AddCommandReadState(*mStateTableRead, mBrakePowerEnabled,
                                        "GetBrakeAmpEnable"); // vector[bool]
    robotInterface->AddCommandReadState(*mStateTableRead, mBrakePowerStatus,
                                        "GetBrakeAmpStatus"); // vector[bool]
    robotInterface->AddCommandReadState(*mStateTableRead, mBrakeCurrentFeedback,
                                        "GetBrakeFeedbackCurrent");
    robotInterface->AddCommandReadState(*mStateTableWrite, mBrakeCurrentCommand,
                                        "GetBrakeRequestedCurrent");
    robotInterface->AddCommandReadState(*mStateTableRead, mBrakeTemperature,
                                        "GetBrakeAmpTemperature"); // vector[double]

    robotInterface->AddCommandWrite(&mtsRobot1394::servo_jf, this,
                                    "servo_jf", mTorqueJoint);
    robotInterface->AddCommandRead(&mtsRobot1394::GetJointEffortCommandLimits, this,
                                   "GetTorqueJointMax", mJointEffortCommandLimits);

    robotInterface->AddCommandWrite(&mtsRobot1394::SetActuatorCurrentBits, this,
                                    "SetActuatorCurrentRaw", mActuatorCurrentBitsCommand);
    robotInterface->AddCommandWrite(&mtsRobot1394::SetActuatorCurrent, this,
                                    "SetActuatorCurrent", mActuatorCurrentCommand);
    robotInterface->AddCommandRead(&mtsRobot1394::GetActuatorCurrentCommandLimits, this,
                                   "GetActuatorCurrentMax", mActuatorCurrentCommandLimits);
    robotInterface->AddCommandRead(&mtsRobot1394::configuration_js, this,
                                   "configuration_js", mConfigurationJoint);
    robotInterface->AddCommandWrite(&mtsRobot1394::configure_js, this,
                                    "configure_js", mConfigurationJoint);

    robotInterface->AddCommandWrite(&mtsRobot1394::SetEncoderPositionBits, this,
                                    "SetEncoderPositionRaw");
    robotInterface->AddCommandWrite(&mtsRobot1394::SetEncoderPosition, this,
                                    "SetEncoderPosition");

    robotInterface->AddCommandVoid(&mtsRobot1394::BrakeRelease, this,
                                   "BrakeRelease");
    robotInterface->AddCommandVoid(&mtsRobot1394::BrakeEngage, this,
                                   "BrakeEngage");

    // unit conversion methods (Qualified Read)
    robotInterface->AddCommandQualifiedRead(&mtsRobot1394::EncoderBitsToPosition, this,
                                            "EncoderRawToSI", vctIntVec(), vctDoubleVec());
    robotInterface->AddCommandQualifiedRead(&mtsRobot1394::EncoderPositionToBits, this,
                                            "EncoderSIToRaw", vctDoubleVec(), vctIntVec());
    robotInterface->AddCommandQualifiedRead(&mtsRobot1394::ActuatorCurrentToEffort, this,
                                            "DriveAmpsToNm", mActuatorCurrentCommand, mActuatorEffortCommand);
    robotInterface->AddCommandQualifiedRead(&mtsRobot1394::ActuatorEffortToCurrent, this,
                                            "DriveNmToAmps", mActuatorEffortCommand, mActuatorCurrentCommand);
    robotInterface->AddCommandQualifiedRead(&mtsRobot1394::PotBitsToVoltage, this,
                                            "AnalogInBitsToVolts", mPotBits, mPotVoltage);

    //
    robotInterface->AddCommandWrite(&mtsRobot1394::CalibrateEncoderOffsetsFromPots,
                                    this, "BiasEncoder");
    robotInterface->AddCommandWrite(&mtsRobot1394::SetSomeEncoderPosition, this,
                                    "SetSomeEncoderPosition");

    // Events
    robotInterface->AddEventWrite(EventTriggers.PowerStatus, "PowerStatus", false);
    robotInterface->AddEventWrite(EventTriggers.WatchdogStatus, "WatchdogStatus", false);
    robotInterface->AddEventWrite(EventTriggers.WatchdogPeriod, "WatchdogPeriod", 15.0 * cmn_ms);
    robotInterface->AddEventWrite(EventTriggers.Coupling, "Coupling", prmActuatorJointCoupling());
    robotInterface->AddEventWrite(EventTriggers.BiasEncoder, "BiasEncoder", 0);
    robotInterface->AddEventWrite(EventTriggers.UsePotsForSafetyCheck, "UsePotsForSafetyCheck", false);

    // fine tune power, board vs. axis
    actuatorInterface->AddCommandVoid(&mtsRobot1394::EnableBoardsPower, this,
                                      "EnableBoardsPower");
    actuatorInterface->AddCommandVoid(&mtsRobot1394::DisableBoardPower, this,
                                      "DisableBoardsPower");
    actuatorInterface->AddCommandWrite<mtsRobot1394, vctBoolVec>(&mtsRobot1394::SetActuatorPower, this,
                                                                 "SetAmpEnable", mActuatorPowerEnabled); // vector[bool]
    actuatorInterface->AddCommandWrite(&mtsRobot1394::SetSomeEncoderPosition, this,
                                       "SetSomeEncoderPosition");

    actuatorInterface->AddCommandReadState(*mStateTableRead, mActuatorPowerEnabled,
                                           "GetAmpEnable"); // vector[bool]
    actuatorInterface->AddCommandReadState(*mStateTableRead, mActuatorPowerStatus,
                                           "GetAmpStatus"); // vector[bool]
    actuatorInterface->AddCommandReadState(*mStateTableRead, mActuatorMeasuredJS,
                                           "measured_js");

    actuatorInterface->AddCommandQualifiedRead(&mtsRobot1394::ActuatorCurrentToBits, this,
                                               "DriveAmpsToBits", mActuatorCurrentFeedback, mActuatorCurrentBitsFeedback);
    actuatorInterface->AddCommandQualifiedRead(&mtsRobot1394::PotVoltageToPosition, this,
                                               "AnalogInVoltsToPosSI", mPotVoltage, mPotPosition);
}

bool mtsRobot1394::CheckConfiguration(void)
{
    if ((NumberOfActuators() > 2)
        && mActuatorCurrentToBitsOffsets.Equal(mActuatorCurrentToBitsOffsets[0])) {
        CMN_LOG_CLASS_INIT_ERROR << "CheckConfiguration: all currents to bits offsets are equal, please calibrate the current offsets for arm: "
                                 << this->Name() << std::endl;
        return false;
    }
    return true;
}

void mtsRobot1394::Configure(const osaRobot1394Configuration & config)
{
    // Store the configuration
    mConfiguration = config;

    //  info
    mName = config.Name;
    mNumberOfActuators = config.NumberOfActuators;
    mNumberOfJoints = config.NumberOfJoints;
    mSerialNumber = config.SerialNumber;
    mPotType = config.PotLocation;

    // Low-level API
    mActuatorInfo.resize(mNumberOfActuators);

    // Initialize state vectors to the appropriate sizes
    mActuatorPowerStatus.SetSize(mNumberOfActuators);
    mActuatorPowerEnabled.SetSize(mNumberOfActuators);
    mDigitalInputs.SetSize(mNumberOfActuators);
    mEncoderChannelsA.SetSize(mNumberOfActuators);
    mPotBits.SetSize(mNumberOfActuators);
    mEncoderOverflow.SetSize(mNumberOfActuators);
    if (mConfiguration.OnlyIO) {
        mEncoderOverflow.SetAll(false);
    }
    mPreviousEncoderOverflow.SetSize(mNumberOfActuators);
    mPreviousEncoderOverflow.SetAll(false);
    mEncoderPositionBits.SetSize(mNumberOfActuators);
    mEncoderPositionBitsPrev.SetSize(mNumberOfActuators);
    mActuatorCurrentBitsCommand.SetSize(mNumberOfActuators);
    mActuatorCurrentBitsFeedback.SetSize(mNumberOfActuators);

    mActuatorTimestamp.SetSize(mNumberOfActuators);
    mActuatorTimestampChange.SetSize(mNumberOfActuators);
    mActuatorTimestampChange.SetAll(0.0);
    mActuatorPreviousTimestampChange.SetSize(mNumberOfActuators);
    mActuatorPreviousTimestampChange.SetAll(0.0);
    mVelocitySlopeToZero.SetSize(mNumberOfActuators);
    mVelocitySlopeToZero.SetAll(0.0);
    mPotVoltage.SetSize(mNumberOfActuators);
    mPotPosition.SetSize(mNumberOfActuators);
    mEncoderVelocityCountsPerSecond.SetSize(mNumberOfActuators);
    mEncoderVelocityDelay.SetSize(mNumberOfActuators);
    mEncoderVelocityPredicted.SetSize(mNumberOfActuators);
    mEncoderAccelerationCountsPerSecSec.SetSize(mNumberOfActuators);
    mEncoderAcceleration.SetSize(mNumberOfActuators);
    mEncoderVelocitySoftware.SetSize(mNumberOfActuators);
    mActuatorCurrentCommand.SetSize(mNumberOfActuators);
    mActuatorEffortCommand.SetSize(mNumberOfActuators);
    mActuatorCurrentFeedback.SetSize(mNumberOfActuators);

    // Initialize property vectors to the appropriate sizes
    mConfigurationJoint.Type().SetSize(mNumberOfJoints);
    mMeasuredJS.Position().SetSize(mNumberOfJoints);
    mMeasuredJS.Velocity().SetSize(mNumberOfJoints);
    mMeasuredJS.Effort().SetSize(mNumberOfJoints);
    mActuatorMeasuredJS.Position().SetSize(mNumberOfActuators);
    mActuatorMeasuredJS.Velocity().SetSize(mNumberOfActuators);
    mActuatorMeasuredJS.Effort().SetSize(mNumberOfActuators);

    mEffortToCurrentScales.SetSize(mNumberOfActuators);
    mActuatorCurrentToBitsScales.SetSize(mNumberOfActuators);
    mActuatorCurrentToBitsOffsets.SetSize(mNumberOfActuators);
    mActuatorBitsToCurrentScales.SetSize(mNumberOfActuators);
    mActuatorBitsToCurrentOffsets.SetSize(mNumberOfActuators);
    mActuatorEffortCommandLimits.SetSize(mNumberOfActuators);
    mJointEffortCommandLimits.SetSize(mNumberOfJoints);
    mActuatorCurrentCommandLimits.SetSize(mNumberOfActuators);
    mActuatorCurrentFeedbackLimits.SetSize(mNumberOfActuators);
    if (mPotType == POTENTIOMETER_ON_ACTUATORS) {
        mPotToleranceLatency.SetSize(mNumberOfActuators);
        mPotToleranceDistance.SetSize(mNumberOfActuators);
        for (size_t i = 0; i < mNumberOfActuators; ++i) {
            mPotToleranceLatency.at(i) = config.PotTolerances.at(i).Latency;
            mPotToleranceDistance.at(i) = config.PotTolerances.at(i).Distance;
        }
        mPotErrorDuration.SetSize(mNumberOfActuators);
        mPotValid.SetSize(mNumberOfActuators);
    } else if (mPotType == POTENTIOMETER_ON_JOINTS) {
        mPotToleranceLatency.SetSize(mNumberOfJoints);
        mPotToleranceDistance.SetSize(mNumberOfJoints);
        for (size_t i = 0; i < mNumberOfJoints; ++i) {
            mPotToleranceLatency.at(i) = config.PotTolerances.at(i).Latency;
            mPotToleranceDistance.at(i) = config.PotTolerances.at(i).Distance;
        }
        mPotErrorDuration.SetSize(mNumberOfJoints);
        mPotValid.SetSize(mNumberOfJoints);
    }
    mPotErrorDuration.SetAll(0.0);
    mPotValid.SetAll(true);
    mUsePotsForSafetyCheck = false;

    mBitsToPositionScales.SetSize(mNumberOfActuators);

    mBitsToVoltageScales.SetSize(mNumberOfActuators);
    mBitsToVoltageOffsets.SetSize(mNumberOfActuators);
    mVoltageToPositionScales.SetSize(mNumberOfActuators);
    mVoltageToPositionOffsets.SetSize(mNumberOfActuators);

    mActuatorTemperature.SetSize(mNumberOfActuators);

    mNumberOfBrakes = 0;
    mBrakeReleasing = false;

    // Construct property vectors
    for (size_t i = 0; i < mNumberOfActuators; i++) {

        // Local references to the config properties
        const osaActuator1394Configuration & actuator = config.Actuators.at(i);
        const osaDrive1394Configuration & drive = actuator.Drive;
        const osaEncoder1394Configuration & encoder = actuator.Encoder;
        const osaPot1394Configuration & pot = actuator.Pot;

        mConfigurationJoint.Type().at(i) = actuator.JointType;

        mEffortToCurrentScales.at(i)         = drive.EffortToCurrentScale;
        mActuatorCurrentToBitsScales.at(i)   = drive.CurrentToBitsScale;
        mActuatorCurrentToBitsOffsets.at(i)  = drive.CurrentToBitsOffset;
        mActuatorBitsToCurrentScales.at(i)   = drive.BitsToCurrentScale;
        mActuatorBitsToCurrentOffsets.at(i)  = drive.BitsToCurrentOffset;
        mActuatorEffortCommandLimits.at(i)   = drive.EffortCommandLimit;
        mActuatorCurrentCommandLimits.at(i)  = drive.CurrentCommandLimit;
        // 120% of command current is in the acceptable range
        // Add 50 mA for non motorized actuators due to a2d noise
        mActuatorCurrentFeedbackLimits.at(i) = 1.2 * mActuatorCurrentCommandLimits.at(i) + (50.0 / 1000.0);

        mBitsToPositionScales.at(i)   = encoder.BitsToPositionScale;

        mBitsToVoltageScales.at(i)      = pot.BitsToVoltageScale;
        mBitsToVoltageOffsets.at(i)     = pot.BitsToVoltageOffset;
        mVoltageToPositionScales.at(i)  = pot.VoltageToPositionScale;
        mVoltageToPositionOffsets.at(i) = pot.VoltageToPositionOffset;

        // Initialize state vectors
        mActuatorMeasuredJS.Position().at(i) = 0.0;
        mActuatorCurrentCommand.at(i) = 0.0;
        mActuatorCurrentFeedback.at(i) = 0.0;

        // Count number of brakes
        if (actuator.Brake) {
            mNumberOfBrakes++;
        }
    }

    // Update brake data
    mBrakeInfo.resize(mNumberOfBrakes);
    mBrakeReleasingTimer.resize(mNumberOfBrakes);

    mBrakeCurrentToBitsScales.SetSize(mNumberOfBrakes);
    mBrakeCurrentToBitsOffsets.SetSize(mNumberOfBrakes);
    mBrakeBitsToCurrentScales.SetSize(mNumberOfBrakes);
    mBrakeBitsToCurrentOffsets.SetSize(mNumberOfBrakes);
    mBrakeCurrentCommandLimits.SetSize(mNumberOfBrakes);
    mBrakeCurrentFeedbackLimits.SetSize(mNumberOfBrakes);
    mBrakePowerStatus.SetSize(mNumberOfBrakes);
    mBrakePowerEnabled.SetSize(mNumberOfBrakes);
    mBrakeCurrentBitsCommand.SetSize(mNumberOfBrakes);
    mBrakeCurrentBitsFeedback.SetSize(mNumberOfBrakes);
    mBrakeTimestamp.SetSize(mNumberOfBrakes);
    mBrakeCurrentCommand.SetSize(mNumberOfBrakes);
    mBrakeCurrentFeedback.SetSize(mNumberOfBrakes);
    mBrakeTemperature.SetSize(mNumberOfBrakes);
    mBrakeReleaseCurrent.SetSize(mNumberOfBrakes);
    mBrakeReleaseTime.SetSize(mNumberOfBrakes);
    mBrakeReleasedCurrent.SetSize(mNumberOfBrakes);
    mBrakeEngagedCurrent.SetSize(mNumberOfBrakes);

    // Construct property vectors for brakes
    size_t currentBrake = 0;
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        const osaActuator1394Configuration & actuator = config.Actuators.at(i);

        // Count number of brakes
        if (actuator.Brake) {
            const osaAnalogBrake1394Configuration * brake = actuator.Brake;
            const osaDrive1394Configuration & drive = brake->Drive;
            mBrakeCurrentToBitsScales[currentBrake]   = drive.CurrentToBitsScale;
            mBrakeCurrentToBitsOffsets[currentBrake]  = drive.CurrentToBitsOffset;
            mBrakeBitsToCurrentScales[currentBrake]   = drive.BitsToCurrentScale;
            mBrakeBitsToCurrentOffsets[currentBrake]  = drive.BitsToCurrentOffset;
            mBrakeCurrentCommandLimits[currentBrake]  = drive.CurrentCommandLimit;
            // 120% of command current is in the acceptable range
            // Add 50 mA for a2d noise around 0
            mBrakeCurrentFeedbackLimits[currentBrake] = 1.2 * mBrakeCurrentCommandLimits[currentBrake] + (50.0 / 1000.0);

            mBrakeReleaseCurrent[currentBrake]  = brake->ReleaseCurrent;
            mBrakeReleaseTime[currentBrake]     = brake->ReleaseTime;
            mBrakeReleasedCurrent[currentBrake] = brake->ReleasedCurrent;
            mBrakeEngagedCurrent[currentBrake]  = brake->EngagedCurrent;

            // Initialize defaults
            mBrakeCurrentCommand[currentBrake] = 0.0;
            mBrakeCurrentFeedback[currentBrake] = 0.0;

            currentBrake++;
        }
    }

    // Compute effort command limits
    if (mConfiguration.HasActuatorToJointCoupling) {
        mJointEffortCommandLimits.ProductOf(mConfiguration.Coupling.ActuatorToJointEffort(),
                                            mActuatorEffortCommandLimits);
    } else {
        mJointEffortCommandLimits.Assign(mActuatorEffortCommandLimits);
    }
}

void mtsRobot1394::SetBoards(const std::vector<osaActuatorMapping> & actuatorBoards,
                             const std::vector<osaBrakeMapping> & brakeBoards)
{
    if (actuatorBoards.size() != mNumberOfActuators) {
        cmnThrow(osaRuntimeError1394(this->Name() + ": number of actuator boards different than the number of actuators."));
    }

    if (brakeBoards.size() != mNumberOfBrakes) {
        cmnThrow(osaRuntimeError1394(this->Name() + ": number of brake boards different than the number of brakes."));
    }

    for (size_t i = 0; i < mNumberOfActuators; i++) {
        // Store this board
        mActuatorInfo.at(i).Board = actuatorBoards.at(i).Board;
        mActuatorInfo.at(i).Axis = actuatorBoards.at(i).Axis;
        // Construct a list of unique boards
        mUniqueBoards[actuatorBoards.at(i).Board->GetBoardId()] = actuatorBoards.at(i).Board;
    }

    for (size_t i = 0; i < mNumberOfBrakes; i++) {
        // Store this board
        mBrakeInfo.at(i).Board = brakeBoards.at(i).Board;
        mBrakeInfo.at(i).Axis = brakeBoards.at(i).Axis;
        // Construct a list of unique boards
        mUniqueBoards[brakeBoards.at(i).Board->GetBoardId()] = brakeBoards.at(i).Board;
    }

    mLowestFirmWareVersion = 999999;
    mHighestFirmWareVersion = 0;
    size_t boardCounter = 0;
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board, ++boardCounter) {
        AmpIO_UInt32 fversion = board->second->GetFirmwareVersion();
        if (fversion == 0) {
            CMN_LOG_CLASS_INIT_ERROR << "SetBoards: " << this->mName
                                     << ", unable to get firmware version for board: " << boardCounter
                                     << ", Id: " << static_cast<int>(board->second->GetBoardId())
                                     << ".  Make sure the controller is powered and connected" << std::endl;
            exit(EXIT_FAILURE);
        }
        std::string serialQLA = board->second->GetQLASerialNumber();
        if (serialQLA.empty()) {
            serialQLA = "unknown";
        }
        std::string serialFPGA = board->second->GetFPGASerialNumber();
        if (serialFPGA.empty()) {
            serialFPGA = "unknown";
        }
        if (fversion < mLowestFirmWareVersion) {
            mLowestFirmWareVersion = fversion;
        }
        if (fversion > mHighestFirmWareVersion) {
            mHighestFirmWareVersion = fversion;
        }
        CMN_LOG_CLASS_INIT_WARNING << "SetBoards: " << this->mName
                                   << ", board: " << boardCounter
                                   << ", Id: " << static_cast<int>(board->second->GetBoardId())
                                   << ", firmware: " << fversion
                                   << ", FPGA serial: " << serialFPGA
                                   << ", QLA serial: " << serialQLA
                                   << std::endl;
    }

    const AmpIO_UInt32 currentFirmwareRevision = 7;
    const AmpIO_UInt32 lowestFirmwareSupported = 6;

    std::stringstream message;
    bool fatal = false;
    bool firmwareSuggested = false;
    // supported
    if ((mLowestFirmWareVersion >= lowestFirmwareSupported)
        && (mLowestFirmWareVersion < currentFirmwareRevision)) {
        message << "mtsRobot1394::SetBoards" << std::endl
                << "----------------------------------------------------" << std::endl
                << " Suggestion:" << std::endl
                << "   Please upgrade all boards firmware to version " << currentFirmwareRevision << "." << std::endl
                << "   Lowest version found is " << mLowestFirmWareVersion << "." << std::endl
                << "----------------------------------------------------" << std::endl;
        firmwareSuggested = true;
    }
    // too low
    if (mLowestFirmWareVersion < lowestFirmwareSupported) {
        message << "mtsRobot1394::SetBoards" << std::endl
                << "----------------------------------------------------" << std::endl
                << " Error:" << std::endl
                << "   Please upgrade all boards firmware to version " << currentFirmwareRevision << "." << std::endl
                << "   Lowest version found is " << mLowestFirmWareVersion << "." << std::endl
                << "   This software supports firmware revision(s) " << lowestFirmwareSupported << " to " << currentFirmwareRevision << std::endl
                << "----------------------------------------------------" << std::endl;
        fatal = true;
    }
    // too high
    if (mHighestFirmWareVersion > currentFirmwareRevision) {
        message << "SetBoards" << std::endl
                << "----------------------------------------------------" << std::endl
                << " Error:" << std::endl
                << "   Highest firmware version found is " << mHighestFirmWareVersion << "." << std::endl
                << "   The highest firmware revision supported by this software is " << currentFirmwareRevision << "." << std::endl
                << "   Please update this software or downgrade your firmware." << std::endl
                << "----------------------------------------------------" << std::endl;
        fatal = true;
    }
    if (fatal || firmwareSuggested) {
        message << " To upgrade (or downgrade) the FPGA firmware, please follow instructions from:" << std::endl
                << "  https://github.com/jhu-cisst/mechatronics-firmware/wiki/FPGA-Program" << std::endl
                << "----------------------------------------------------" << std::endl;
        if (fatal) {
            std::cerr << message.str();
            CMN_LOG_CLASS_INIT_ERROR << message.str();
            exit(EXIT_FAILURE);
        } else {
            CMN_LOG_CLASS_INIT_WARNING << message.str();
        }
    }
}


void mtsRobot1394::PollValidity(void)
{
    // Make sure the boards have been configured
    if (mNumberOfActuators != mActuatorInfo.size()) {
        cmnThrow(osaRuntimeError1394(this->Name() + ": number of boards different than the number of actuators."));
    }

    // Store previous state
    mPreviousPowerStatus = mPowerStatus;
    mPreviousWatchdogStatus = mWatchdogStatus;

    // Initialize flags
    mValid = true;
    mPowerStatus = true;
    mSafetyRelay = true;
    mWatchdogStatus = false;

    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        mValid &= board->second->ValidRead();
        mPowerStatus &= board->second->GetPowerStatus();
        mSafetyRelay &= board->second->GetSafetyRelayStatus();
        mWatchdogStatus |= board->second->GetWatchdogTimeoutStatus();
    }

    if (!mValid) {
        if (mInvalidReadCounter == 0) {
            mInvalidReadCounter++;
            std::stringstream message;
            message << this->Name() << ": port read error on board(s) ";
            for (unique_board_iterator board = mUniqueBoards.begin();
                 board != mUniqueBoards.end();
                 ++board) {
                if (!board->second->ValidRead()) {
                    message << static_cast<int>(board->second->GetBoardId()) << " ";
                }
            }
            cmnThrow(osaRuntimeError1394(message.str()));
        } else {
            mInvalidReadCounter++;
            if (mInvalidReadCounter == 10000) {
                mInvalidReadCounter = 0;
                cmnThrow(osaRuntimeError1394(this->Name() + ": port read errors, occurred 10,000 times"));
            }
        }
    } else {
        mInvalidReadCounter = 0;
    }
}

void mtsRobot1394::PollState(void)
{
    // Poll data
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        AmpIO * board = mActuatorInfo[i].Board;
        int axis = mActuatorInfo[i].Axis;

        if (!board || (axis < 0)) continue; // We probably don't need this check any more

        mActuatorTimestamp[i] = board->GetTimestamp() * 1.0 / 49125000.0;
        mDigitalInputs[i] = board->GetDigitalInput();

        // vectors of bits
        if (!mConfiguration.OnlyIO) {
            mEncoderOverflow[i] = board->GetEncoderOverflow(axis);
        }
        mEncoderChannelsA[i] = board->GetEncoderChannelA(axis);

        // convert from 24 bits signed stored in 32 unsigned to 32 signed
        mEncoderPositionBits[i] = board->GetEncoderPosition(axis);
        mEncoderVelocityDelay[i] = board->GetEncoderVelocityDelay(axis);
        // Second argument below is how much quantization error do we accept
        mEncoderAccelerationCountsPerSecSec[i] = board->GetEncoderAcceleration(axis, 0.0005);
        mEncoderVelocityCountsPerSecond[i] = board->GetEncoderVelocityCountsPerSecond(axis);

        mPotBits[i] = board->GetAnalogInput(axis);

        mActuatorCurrentBitsFeedback[i] = board->GetMotorCurrent(axis);
        mActuatorPowerEnabled[i] = board->GetAmpEnable(axis);
        mActuatorPowerStatus[i] = board->GetAmpStatus(axis);

        // first temperature corresponds to first 2 actuators, second to last 2
        // board reports temperature in celsius * 2
        mActuatorTemperature[i] = (board->GetAmpTemperature(axis / 2)) / 2.0;
    }

    for (size_t i = 0; i < mNumberOfBrakes; i++) {
        AmpIO * board = mBrakeInfo[i].Board;
        int axis = mBrakeInfo[i].Axis;

        if (!board || (axis < 0)) continue; // We probably don't need this check any more

        mBrakeTimestamp[i] = board->GetTimestamp() * 1.0 / 49125000.0;
        mBrakeCurrentBitsFeedback[i] = board->GetMotorCurrent(axis);
        mBrakePowerEnabled[i] = board->GetAmpEnable(axis);
        mBrakePowerStatus[i] = board->GetAmpStatus(axis);

        // first temperature corresponds to first 2 brakes, second to last 2
        // board reports temperature in celsius * 2
        mBrakeTemperature[i] = (board->GetAmpTemperature(axis / 2)) / 2.0;
    }

}

void mtsRobot1394::ConvertState(void)
{
    // Perform read conversions
    EncoderBitsToPosition(mEncoderPositionBits,
                          mActuatorMeasuredJS.Position());
    if (mConfiguration.HasActuatorToJointCoupling) {
        mMeasuredJS.Position().ProductOf(mConfiguration.Coupling.ActuatorToJointPosition(),
                                         mActuatorMeasuredJS.Position());
    } else {
        mMeasuredJS.Position().Assign(mActuatorMeasuredJS.Position());
    }

    // Scale velocity from counts/sec to SI units
    // Note that this velocity may not get used; depending on the firmware version,
    // the system will compute the joint velocities from mEncoderVelocitySoftware or
    // mEncoderVelocityPredicted.
    mActuatorMeasuredJS.Velocity().ElementwiseProductOf(mBitsToPositionScales, mEncoderVelocityCountsPerSecond);

    // Scale acceleration from counts/sec**2 to SI units
    mEncoderAcceleration.ElementwiseProductOf(mBitsToPositionScales, mEncoderAccelerationCountsPerSecSec);

    // Velocity computation

    // If we have firmware 6, use FPGA acceleration to predict current velocity
    if ((mLowestFirmWareVersion >= 6) && (mHighestFirmWareVersion >= 6)) {
        EncoderBitsToVelocityPredicted(mEncoderVelocityPredicted);
    }

    // In any case, compute velocities in "software"
    // using iterator for efficiency and going over all actuators
    const double timeToZeroVelocity = 1.0 * cmn_s;
    const vctIntVec::const_iterator end = mEncoderPositionBits.end();
    vctIntVec::const_iterator currentEncoder, previousEncoder;
    vctDoubleVec::const_iterator currentTimestamp, bitsToPos;
    vctDoubleVec::const_iterator encoderVelocity;
    vctDoubleVec::iterator lastChangeTimestamp, slope, velocity;
    size_t index = 0;
    for (currentEncoder = mEncoderPositionBits.begin(),
             previousEncoder = mEncoderPositionBitsPrev.begin(),
             currentTimestamp = mActuatorTimestamp.begin(),
             bitsToPos = mBitsToPositionScales.begin(),
             lastChangeTimestamp = mActuatorTimestampChange.begin(),
             slope = mVelocitySlopeToZero.begin(),
             velocity = mEncoderVelocitySoftware.begin();
         // end
         currentEncoder != end;
         // increment
         ++currentEncoder,
             ++previousEncoder,
             ++currentTimestamp,
             ++bitsToPos,
             ++lastChangeTimestamp,
             ++slope,
             ++velocity,
             ++index) {
        // first see if there has been any change
        const int difference = (*currentEncoder) - (*previousEncoder);
        if (difference == 0) {
            if (*lastChangeTimestamp < timeToZeroVelocity) {
                *velocity -= (*slope) * (*currentTimestamp);
            } else {
                *velocity = 0.0;
            }
            *lastChangeTimestamp += (*currentTimestamp);
        } else {
            *lastChangeTimestamp += (*currentTimestamp);
            // if we only have one bit change compute velocity since last change
            if ((difference == 1) || (difference == -1)) {
                *velocity = (difference / (*lastChangeTimestamp))
                    * (*bitsToPos);
            } else {
                if (difference > 1) {
                    // we know all but 1 bit difference happened in last Dt, other bit change happened between now and last change
                    *velocity = ((difference - 1.0) / (*currentTimestamp) + 1.0 / (*lastChangeTimestamp))
                        * (*bitsToPos);
                } else {
                    *velocity = ((difference + 1.0) / (*currentTimestamp) - 1.0 / (*lastChangeTimestamp))
                        * (*bitsToPos);
                }
            }
            // keep record of this change
            *lastChangeTimestamp = 0.0;
            *slope = (*velocity) / (timeToZeroVelocity);
        }
    }

    // finally save previous encoder bits position
    mEncoderPositionBitsPrev.Assign(mEncoderPositionBits);

    // Figure out which velocity to use based on firmware version
    vctDoubleVec *velToUse;
    if ((mLowestFirmWareVersion >= 6) && (mHighestFirmWareVersion >= 6)) {
        velToUse = &mEncoderVelocityPredicted;   // velocity based on FPGA measurement of velocity and acceleration
    } else {
        velToUse = &mEncoderVelocitySoftware;    // velocity based on software computation (from position)
    }

    if (mConfiguration.HasActuatorToJointCoupling) {
        mMeasuredJS.Velocity().ProductOf(mConfiguration.Coupling.ActuatorToJointPosition(), *velToUse);
    } else {
        mMeasuredJS.Velocity().Assign(*velToUse);
    }

    // Effort computation
    ActuatorBitsToCurrent(mActuatorCurrentBitsFeedback,
                          mActuatorCurrentFeedback);
    ActuatorCurrentToEffort(mActuatorCurrentFeedback,
                            mActuatorMeasuredJS.Effort());
    if (mConfiguration.HasActuatorToJointCoupling) {
        mMeasuredJS.Effort().ProductOf(mConfiguration.Coupling.ActuatorToJointEffort(),
                                       mActuatorMeasuredJS.Effort());
    } else {
        mMeasuredJS.Effort().Assign(mActuatorMeasuredJS.Effort());
    }

    BrakeBitsToCurrent(mBrakeCurrentBitsFeedback, mBrakeCurrentFeedback);

    PotBitsToVoltage(mPotBits, mPotVoltage);
    PotVoltageToPosition(mPotVoltage, mPotPosition);
}

void mtsRobot1394::CheckState(void)
{
    // set data as invalid by default
    mMeasuredJS.SetValid(false);
    mActuatorMeasuredJS.SetValid(false);

    // If we had a read error, all checks are pretty much useless
    if (mInvalidReadCounter > 0) {
        return;
    }

    // Perform safety checks
    bool currentSafetyViolation = false;
    // actuators
    {
        const vctDoubleVec::const_iterator end = mActuatorCurrentFeedback.end();
        vctDoubleVec::const_iterator feedback = mActuatorCurrentFeedback.begin();
        vctDoubleVec::const_iterator limit = mActuatorCurrentFeedbackLimits.begin();
        size_t index = 0;
        for (; feedback < end;
             ++feedback,
                 ++limit,
                 ++index) {
            if (fabs(*feedback) >= *limit) {
                CMN_LOG_CLASS_RUN_WARNING << "CheckState: " << this->mName << ", actuator " << index
                                          << " power: " << *feedback
                                          << " > limit: " << *limit << std::endl;
                currentSafetyViolation = true;
            }
        }
    }
    // brakes
    {
        const vctDoubleVec::const_iterator end = mBrakeCurrentFeedback.end();
        vctDoubleVec::const_iterator feedback = mBrakeCurrentFeedback.begin();
        vctDoubleVec::const_iterator limit = mBrakeCurrentFeedbackLimits.begin();
        size_t index = 0;
        for (; feedback < end;
             ++feedback,
                 ++limit,
                 ++index) {
            if (fabs(*feedback) >= *limit) {
                CMN_LOG_CLASS_RUN_WARNING << "CheckState: " << this->mName << ", brake " << index
                                          << " power: " << *feedback
                                          << " > limit: " << *limit << std::endl;
                currentSafetyViolation = true;
            }
        }
    }

    if (currentSafetyViolation) {
        mCurrentSafetyViolationsCounter++;
    } else {
        mCurrentSafetyViolationsCounter = 0;
    }

    if (mCurrentSafetyViolationsCounter > mCurrentSafetyViolationsMaximum) {
        this->DisablePower();
        cmnThrow(osaRuntimeError1394(this->Name() + ": too many consecutive current safety violations.  Power has been disabled."));
    }

    // check safety amp disable
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        AmpIO_UInt32 safetyAmpDisable = board->second->GetSafetyAmpDisable();
        if (safetyAmpDisable) {
            cmnThrow(osaRuntimeError1394(this->Name() + ": hardware current safety amp disable tripped." + mActuatorTimestamp.ToString()));
        }
    }

    // check temperature when powered
    if (mUserExpectsPower) {
        bool temperatureError = false;
        bool temperatureWarning = false;
        double temperatureTrigger = 0.0;
        // actuators
        {
            const vctDoubleVec::const_iterator end = mActuatorTemperature.end();
            vctDoubleVec::const_iterator temperature = mActuatorTemperature.begin();
            size_t index = 0;
            for (; temperature < end;
                 ++temperature,
                     ++index) {
                if (*temperature > sawRobotIO1394::TemperatureErrorThreshold) {
                    CMN_LOG_CLASS_RUN_ERROR << "CheckState: " << this->mName << ", actuator " << index
                                            << " temperature: " << *temperature
                                            << " greater than error threshold: " << sawRobotIO1394::TemperatureErrorThreshold << std::endl;
                    temperatureError = true;
                    temperatureTrigger = *temperature;
                } else {
                    if (*temperature > sawRobotIO1394::TemperatureWarningThreshold) {
                        CMN_LOG_CLASS_RUN_DEBUG << "CheckState: " << this->mName << ", actuator " << index
                                                << " temperature: " << *temperature
                                                << " greater than warning threshold: " << sawRobotIO1394::TemperatureWarningThreshold << std::endl;
                        temperatureWarning = true;
                        temperatureTrigger = *temperature;
                    }
                }
            }
        }
        // brakes
        {
            const vctDoubleVec::const_iterator end = mBrakeTemperature.end();
            vctDoubleVec::const_iterator temperature = mBrakeTemperature.begin();
            size_t index = 0;
            for (; temperature < end;
                 ++temperature,
                     ++index) {
                if (*temperature > sawRobotIO1394::TemperatureErrorThreshold) {
                    CMN_LOG_CLASS_RUN_ERROR << "CheckState: " << this->mName << ", brake " << index
                                            << " temperature: " << *temperature
                                            << " greater than error threshold: " << sawRobotIO1394::TemperatureErrorThreshold << std::endl;
                    temperatureError = true;
                    temperatureTrigger = *temperature;
                } else {
                    if (*temperature > sawRobotIO1394::TemperatureWarningThreshold) {
                        CMN_LOG_CLASS_RUN_DEBUG << "CheckState: " << this->mName << ", brake " << index
                                                << " temperature: " << *temperature
                                                << " greater than warning threshold: " << sawRobotIO1394::TemperatureWarningThreshold << std::endl;
                        temperatureWarning = true;
                        temperatureTrigger = *temperature;
                    }
                }
            }
        }

        if (temperatureError) {
            this->DisablePower();
            std::stringstream message;
            message << "IO: " << this->Name() << " controller measured temperature is " << temperatureTrigger
                    << "ºC, error threshold is set to " << sawRobotIO1394::TemperatureErrorThreshold << "ºC";
            mInterface->SendError(message.str());
        } else if (temperatureWarning) {
            if (mTimeLastTemperatureWarning >= sawRobotIO1394::TimeBetweenTemperatureWarnings) {
                std::stringstream message;
                message << "IO: " << this->Name() << " controller measured temperature is " << temperatureTrigger
                        << "ºC, warning threshold is set to " << sawRobotIO1394::TemperatureWarningThreshold << "ºC";
                mInterface->SendWarning(message.str());
                mTimeLastTemperatureWarning = 0.0;
            }
            double time = 0.0;
            if (!mActuatorTimestamp.empty()) {
                time = *(mActuatorTimestamp.begin());
            } else if (!mBrakeTimestamp.empty()) {
                time = *(mBrakeTimestamp.begin());
            }
            mTimeLastTemperatureWarning+= time;
        } else {
            // reset time so next time we hit a warning it displays immediately
            mTimeLastTemperatureWarning = sawRobotIO1394::TimeBetweenTemperatureWarnings;
        }
    }

    // Check if brakes are releasing/released
    if (mBrakeReleasing) {
        bool allReleased = true;
        // check how much time per brake, set all to high (release current)
        mBrakeCurrentCommand.Assign(mBrakeReleaseCurrent);
        mBrakeReleasingTimer.Add(mBrakeTimestamp);
        for (size_t index = 0; index < mNumberOfBrakes; ++index) {
            if (mBrakeReleasingTimer[index] > mBrakeReleaseTime[index]) {
                // lower to releaseD current
                mBrakeCurrentCommand[index] = mBrakeReleasedCurrent[index];
            } else {
                allReleased = false;
            }
        }
        SetBrakeCurrent(mBrakeCurrentCommand);
        mBrakeReleasing = !allReleased;
    }

    // Check if encoders and potentiometers agree
    if (mUsePotsForSafetyCheck) {
        switch (mPotType) {
        case POTENTIOMETER_UNDEFINED:
            break;
        case POTENTIOMETER_ON_ACTUATORS:
        case POTENTIOMETER_ON_JOINTS:
            {
                vctDynamicVectorRef<double> encoderRef;
                if (mPotType == POTENTIOMETER_ON_ACTUATORS) {
                    encoderRef.SetRef(mActuatorMeasuredJS.Position());
                } else {
                    encoderRef.SetRef(mMeasuredJS.Position());
                }
                bool statusChanged = false;
                bool error = false;
                vctDoubleVec::const_iterator pot = mPotPosition.begin();
                vctDynamicVectorRef<double>::const_iterator enc = encoderRef.begin();
                const vctDoubleVec::const_iterator potEnd = mPotPosition.end();
                vctDoubleVec::const_iterator potLatency = mPotToleranceLatency.begin();
                vctDoubleVec::const_iterator potError = mPotToleranceDistance.begin();
                vctDoubleVec::const_iterator potTimestamp = mActuatorTimestamp.begin(); // this is a bit approximative when there's coupling
                vctDoubleVec::iterator potDuration = mPotErrorDuration.begin();
                vctBoolVec::iterator potValid = mPotValid.begin();

                for (;
                     pot != potEnd;
                     ++pot,
                         ++enc,
                         ++potLatency,
                         ++potError,
                         ++potTimestamp,
                         ++potDuration,
                         ++potValid) {
                    // if tolerance set to 0, disable check for that joint
                    if (*potError == 0.0) {
                        *potValid = true;
                    } else {
                        // check for error
                        double delta = std::abs(*pot - *enc);
                        if (delta > *potError) {
                            *potDuration += *potTimestamp;
                            // check how long have we been off
                            if (*potDuration > *potLatency) {
                                // now we have a problem,
                                this->DisablePower();
                                // maybe it's not new, used for reporting
                                if (*potValid) {
                                    // this is new
                                    statusChanged = true;
                                    error = true;
                                    *potValid = false;
                                }
                            }
                        } else {
                            // back to normal, reset status if needed
                            *potDuration = 0.0;
                            if (! *potValid) {
                                statusChanged = true;
                                *potValid = true;
                            }
                        }
                    }
                }
                // if status has changed
                if (statusChanged) {
                    if (error) {
                        std::string errorMessage = "IO: " + this->Name() + ": inconsistency between encoders and potentiometers\nencoders:\n";
                        errorMessage.append(encoderRef.ToString());
                        errorMessage.append("\npotentiomers:\n");
                        errorMessage.append(mPotPosition.ToString());
                        errorMessage.append("\ntolerance distance:\n");
                        errorMessage.append(mPotToleranceDistance.ToString());
                        errorMessage.append("\nvalid pots:\n");
                        errorMessage.append(mPotValid.ToString());
                        errorMessage.append("\ntolerance latency:\n");
                        errorMessage.append(mPotToleranceLatency.ToString());
                        errorMessage.append("\nerror duration:\n");
                        errorMessage.append(mPotErrorDuration.ToString());
                        cmnThrow(osaRuntimeError1394(errorMessage));
                    } else {
                        CMN_LOG_CLASS_RUN_VERBOSE << "IO: " << this->Name()
                                                  << ": check between encoders and potentiomenters, recovery.  Valid pots:" << std::endl
                                                  << mPotValid << std::endl;
                    }
                }
            }
            break;
        default:
            break;
        }
    }

    // Check for encoder overflow
    if (mEncoderOverflow.Any()) {
        this->DisablePower();
        this->SetEncoderPosition(vctDoubleVec(mNumberOfActuators, 0.0));
        if (mEncoderOverflow.NotEqual(mPreviousEncoderOverflow)) {
            mPreviousEncoderOverflow.Assign(mEncoderOverflow);
            std::string errorMessage = this->Name() + ": detected encoder overflow: ";
            errorMessage.append(mEncoderOverflow.ToString());
            cmnThrow(osaRuntimeError1394(errorMessage));
        }
    }

    mActuatorMeasuredJS.SetValid(true);
    mMeasuredJS.SetValid(true);

    if (mPreviousPowerStatus != mPowerStatus) {
        EventTriggers.PowerStatus(mPowerStatus);
        if (!mPowerStatus && mUserExpectsPower) {
            mInterface->SendError("IO: " + this->Name() + " lost power");
            mPreviousPowerStatus = false;
        }
    }

    if (mPreviousWatchdogStatus != mWatchdogStatus) {
        EventTriggers.WatchdogStatus(mWatchdogStatus);
        if (mWatchdogStatus) {
            mInterface->SendError("IO: " + this->Name() + " watchdog triggered");
        } else {
            mInterface->SendStatus("IO: " + this->Name() + " watchdog ok");
        }
    }

    // if nb samples > 0, need to countdown
    if (mSamplesForCalibrateEncoderOffsetsFromPots > 0) {
        // if count down is at 1, compute average of encoders and potentiometers
        if (mSamplesForCalibrateEncoderOffsetsFromPots > 1) {
            mSamplesForCalibrateEncoderOffsetsFromPots--;
        } else {
            // data read from state table
            vctDoubleVec potentiometers(mNumberOfActuators, 0.0);
            mtsGenericObjectProxy<vctDoubleVec> newPot;
            newPot.Data.SetSize(mNumberOfActuators);
            vctDoubleVec encoderRef(mNumberOfActuators, 0.0);
            vctDoubleVec encoderDelta(mNumberOfActuators);
            prmStateJoint newEnc;
            newEnc.Position().SetSize(mNumberOfActuators);

            int nbElements = 0;
            mtsStateIndex index = mStateTableRead->GetIndexReader();
            bool validIndex = true;
            while (validIndex && (nbElements < mSamplesForCalibrateEncoderOffsetsFromPotsRequested)) {
                mPotPositionAccessor->Get(index, newPot);
                mActuatorStateJointAccessor->Get(index, newEnc);
                if (nbElements == 0) {
                    // find reference encoder value
                    encoderRef.Assign(newEnc.Position());
                } else {
                    // correct pot using encoder delta
                    encoderDelta.DifferenceOf(newEnc.Position(), encoderRef);
                    newPot.Data.Subtract(encoderDelta);
                    potentiometers.Add(newPot.Data);
                }
                ++nbElements;
                --index;
                // check if index is still valid, would happen if reached size of state table
                validIndex = mStateTableRead->ValidateReadIndex(index);
            }

            // compute average
            potentiometers.Divide(nbElements);

            // determine where pots are
            vctDoubleVec actuatorPosition(mNumberOfActuators);
            switch(mPotType) {
            case POTENTIOMETER_UNDEFINED:
                cmnThrow("mtsRobot1394::CheckState: can't set encoder offset, potentiometer's position undefined");
                break;
            case POTENTIOMETER_ON_JOINTS:
                if (mConfiguration.HasActuatorToJointCoupling) {
                    actuatorPosition.ProductOf(mConfiguration.Coupling.JointToActuatorPosition(),
                                               potentiometers);
                } else {
                    actuatorPosition.Assign(potentiometers);
                }
                SetEncoderPosition(actuatorPosition);
                break;
            case POTENTIOMETER_ON_ACTUATORS:
                SetEncoderPosition(potentiometers);
                break;
            }
            EventTriggers.BiasEncoder(nbElements);
            mSamplesForCalibrateEncoderOffsetsFromPots = 0; // not needed anymore
        }
    }
}

void mtsRobot1394::WriteSafetyRelay(const bool & enabled)
{
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        AmpIO * boardPointer = board->second;
        boardPointer->WriteSafetyRelay(enabled);
    }
}

void mtsRobot1394::SetActuatorPower(const bool & enabled)
{
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        mActuatorInfo[i].Board->SetAmpEnable(mActuatorInfo[i].Axis, enabled);
    }
}

void mtsRobot1394::SetActuatorPower(const vctBoolVec & enabled)
{
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        mActuatorInfo[i].Board->SetAmpEnable(mActuatorInfo[i].Axis, enabled[i]);
    }
}

void mtsRobot1394::SetBrakePower(const bool & enabled)
{
    for (size_t i = 0; i < mNumberOfBrakes; i++) {
        mBrakeInfo[i].Board->SetAmpEnable(mBrakeInfo[i].Axis, enabled);
    }
}

void mtsRobot1394::SetBrakePower(const vctBoolVec & enabled)
{
    for (size_t i = 0; i < mNumberOfBrakes; i++) {
        mBrakeInfo[i].Board->SetAmpEnable(mBrakeInfo[i].Axis, enabled[i]);
    }
}

void mtsRobot1394::SetEncoderPosition(const vctDoubleVec & pos)
{
    vctIntVec bits(mNumberOfActuators);
    this->EncoderPositionToBits(pos, bits);
    this->SetEncoderPositionBits(bits);
}

void mtsRobot1394::SetEncoderPositionBits(const vctIntVec & bits)
{
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        mActuatorInfo[i].Board->WriteEncoderPreload(mActuatorInfo[i].Axis, bits[i]);
    }
    // initialize previous bits value
    mEncoderPositionBitsPrev.Assign(bits);
    mActuatorTimestampChange.SetAll(0.0);
    mVelocitySlopeToZero.SetAll(0.0);
}

void mtsRobot1394::SetSingleEncoderPosition(const int index, const double pos)
{
    SetSingleEncoderPositionBits(index, pos / mBitsToPositionScales[index]);
}

void mtsRobot1394::SetSingleEncoderPositionBits(const int index, const int bits)
{
    mActuatorInfo[index].Board->WriteEncoderPreload(mActuatorInfo[index].Axis, bits);

    // initialize previous bits value
    mEncoderPositionBitsPrev.Element(index) = bits;
    mActuatorTimestampChange.Element(index) = 0.0;
    mVelocitySlopeToZero.Element(index) = 0.0;
}

void mtsRobot1394::ClipActuatorEffort(vctDoubleVec & efforts)
{
    efforts.ElementwiseClipIn(mActuatorEffortCommandLimits);
}

void mtsRobot1394::ClipActuatorCurrent(vctDoubleVec & currents)
{
    currents.ElementwiseClipIn(mActuatorCurrentCommandLimits);
}

void mtsRobot1394::ClipBrakeCurrent(vctDoubleVec & currents)
{
    currents.ElementwiseClipIn(mBrakeCurrentCommandLimits);
}

void mtsRobot1394::SetJointEffort(const vctDoubleVec & efforts)
{
    vctDoubleVec actuatorEfforts(mNumberOfActuators);
    if (mConfiguration.HasActuatorToJointCoupling) {
        actuatorEfforts.ProductOf(mConfiguration.Coupling.JointToActuatorEffort(), efforts);
    } else {
        actuatorEfforts.Assign(efforts);
    }
    this->SetActuatorEffort(actuatorEfforts);
}

void mtsRobot1394::SetActuatorEffort(const vctDoubleVec & efforts)
{
    // Convert efforts to bits and set the command
    vctDoubleVec clipped_efforts = efforts;
    vctDoubleVec currents(mNumberOfActuators);

    // this->clip_actuator_efforts(clipped_efforts);

    this->ActuatorEffortToCurrent(clipped_efforts, currents);
    this->SetActuatorCurrent(currents);
}

void mtsRobot1394::SetActuatorCurrent(const vctDoubleVec & currents)
{
    // Convert amps to bits and set the command
    vctDoubleVec clipped_amps = currents;
    vctIntVec bits(mNumberOfActuators);

    this->ClipActuatorCurrent(clipped_amps);
    this->ActuatorCurrentToBits(clipped_amps, bits);
    this->SetActuatorCurrentBits(bits);

    // Store commanded amps
    mActuatorCurrentCommand = clipped_amps;
}

void mtsRobot1394::SetActuatorCurrentBits(const vctIntVec & bits)
{
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        mActuatorInfo[i].Board->SetMotorCurrent(mActuatorInfo[i].Axis, bits[i]);
    }

    // Store commanded bits
    mActuatorCurrentBitsCommand = bits;
}

void mtsRobot1394::SetBrakeCurrent(const vctDoubleVec & currents)
{
    // Convert amps to bits and set the command
    vctDoubleVec clipped_amps = currents;
    vctIntVec bits(mNumberOfBrakes);

    this->ClipBrakeCurrent(clipped_amps);
    this->BrakeCurrentToBits(clipped_amps, bits);
    this->SetBrakeCurrentBits(bits);

    // Store commanded amps
    mBrakeCurrentCommand = clipped_amps;
}

void mtsRobot1394::SetBrakeCurrentBits(const vctIntVec & bits)
{
    for (size_t i = 0; i < mNumberOfBrakes; i++) {
        mBrakeInfo[i].Board->SetMotorCurrent(mBrakeInfo[i].Axis, bits[i]);
    }

    // Store commanded bits
    mBrakeCurrentBitsCommand = bits;
}

void mtsRobot1394::BrakeRelease(void)
{
    if (mNumberOfBrakes != 0) {
        mBrakeReleasing = true;
        mBrakeReleasingTimer.SetAll(0.0);
        SetBrakeCurrent(mBrakeReleaseCurrent);
    }
}

void mtsRobot1394::BrakeEngage(void)
{
    if (mNumberOfBrakes != 0) {
        mBrakeReleasing = false;
        SetBrakeCurrent(mBrakeEngagedCurrent);
    }
}

void mtsRobot1394::CalibrateEncoderOffsetsFromPots(void)
{
    vctDoubleVec actuatorPosition(mNumberOfActuators);

    switch(mPotType) {

    case POTENTIOMETER_UNDEFINED:
        cmnThrow("mtsRobot1394::CalibrateEncoderOffsetsFromPots: can't set encoder offset, potentiometer's position undefined");
        break;

    case POTENTIOMETER_ON_JOINTS:
        if (mConfiguration.HasActuatorToJointCoupling) {
            actuatorPosition.ProductOf(mConfiguration.Coupling.JointToActuatorPosition(),
                                       mPotPosition);
        } else {
            actuatorPosition.Assign(mPotPosition);
        }
        SetEncoderPosition(actuatorPosition);
        break;

    case POTENTIOMETER_ON_ACTUATORS:
        SetEncoderPosition(mPotPosition);
        break;
    };
}

bool mtsRobot1394::Valid(void) const {
    return mValid;
}

bool mtsRobot1394::PowerStatus(void) const {
    return mPowerStatus;
}

bool mtsRobot1394::SafetyRelay(void) const {
    return mSafetyRelay;
}

bool mtsRobot1394::WatchdogStatus(void) const {
    return mWatchdogStatus;
}

const vctBoolVec & mtsRobot1394::ActuatorPowerStatus(void) const {
    return mActuatorPowerStatus;
}

const vctBoolVec & mtsRobot1394::BrakePowerStatus(void) const {
    return mBrakePowerStatus;
}

const vctDoubleVec & mtsRobot1394::ActuatorCurrentFeedback(void) const {
    return mActuatorCurrentFeedback;
}

const vctDoubleVec & mtsRobot1394::ActuatorCurrentCommand(void) const {
    return mActuatorCurrentCommand;
}

const vctDoubleVec & mtsRobot1394::ActuatorEffortCommand(void) const {
    return mActuatorEffortCommand;
}

const vctDoubleVec & mtsRobot1394::BrakeCurrentFeedback(void) const {
    return mBrakeCurrentFeedback;
}

const vctDoubleVec & mtsRobot1394::PotPosition(void) const {
    return mPotPosition;
}

const vctDoubleVec & mtsRobot1394::ActuatorTimeStamp(void) const {
    return mActuatorTimestamp;
}

const vctDoubleVec & mtsRobot1394::BrakeTimeStamp(void) const {
    return mBrakeTimestamp;
}

const vctDoubleVec & mtsRobot1394::EncoderVelocityPredicted(void) const {
    return mEncoderVelocityPredicted;
}

const vctDoubleVec & mtsRobot1394::EncoderAcceleration(void) const {
    return mEncoderAcceleration;
}

const vctDoubleVec & mtsRobot1394::EncoderVelocitySoftware(void) const {
    return mEncoderVelocitySoftware;
}

const prmStateJoint & mtsRobot1394::ActuatorJointState(void) const {
    return mActuatorMeasuredJS;
}

const prmStateJoint & mtsRobot1394::JointState(void) const {
    return mMeasuredJS;
}

osaRobot1394Configuration mtsRobot1394::GetConfiguration(void) const {
    return mConfiguration;
}

std::string mtsRobot1394::Name(void) const {
    return mName;
}

size_t mtsRobot1394::NumberOfJoints(void) const {
    return mNumberOfJoints;
}

size_t mtsRobot1394::NumberOfActuators(void) const {
    return mNumberOfActuators;
}

size_t mtsRobot1394::SerialNumber(void) const {
    return mSerialNumber;
}

size_t mtsRobot1394::NumberOfBrakes(void) const {
    return mNumberOfBrakes;
}

void mtsRobot1394::configuration_js(prmConfigurationJoint & jointConfig) const
{
    jointConfig = mConfigurationJoint;
}

void mtsRobot1394::configure_js(const prmConfigurationJoint & jointConfig)
{
    // we assume the types are loaded from the XML config file, we
    // need the names from above.  This is not very elegant.  We use
    // SetSize followed by Assign as a way to enforce that sizes
    // match.
    mConfigurationJoint.Name().SetSize(mNumberOfActuators);
    mConfigurationJoint.Name().Assign(jointConfig.Name());
    mMeasuredJS.Name().SetSize(mNumberOfActuators);
    mMeasuredJS.Name() = jointConfig.Name();
}

void mtsRobot1394::GetJointEffortCommandLimits(vctDoubleVec & limits) const
{
    limits = mJointEffortCommandLimits;
}

void mtsRobot1394::GetActuatorEffortCommandLimits(vctDoubleVec & limits) const
{
    limits = mActuatorEffortCommandLimits;
}

void mtsRobot1394::GetActuatorCurrentCommandLimits(vctDoubleVec & limits) const
{
    limits = mActuatorCurrentCommandLimits;
}

void mtsRobot1394::EncoderPositionToBits(const vctDoubleVec & pos, vctIntVec & bits) const
{
    const vctDoubleVec::const_iterator end = pos.end();
    vctDoubleVec::const_iterator position = pos.begin();
    vctDoubleVec::const_iterator scale = mBitsToPositionScales.begin();
    vctIntVec::iterator bit = bits.begin();
    for (; position != end;
         ++position,
             ++scale,
             ++bit) {
        *bit = static_cast<int>(*position / *scale);
    }
}

void mtsRobot1394::EncoderBitsToPosition(const vctIntVec & bits, vctDoubleVec & pos) const
{
    const vctIntVec::const_iterator end = bits.end();
    vctIntVec::const_iterator bit = bits.begin();
    vctDoubleVec::const_iterator scale = mBitsToPositionScales.begin();
    vctDoubleVec::iterator position = pos.begin();
    for (; bit != end;
         ++bit,
             ++scale,
             ++position) {
        *position = static_cast<double>(*bit) * *scale;
    }
}

void mtsRobot1394::EncoderBitsToVelocityPredicted(vctDoubleVec & vel) const
{
    if ((mLowestFirmWareVersion >= 6) && (mHighestFirmWareVersion >= 6)) {
        const vctDoubleVec::iterator end = vel.end();
        vctDoubleVec::iterator velocity = vel.begin();
        vctDoubleVec::const_iterator enc_vel_cnts_per_sec = mEncoderVelocityCountsPerSecond.begin();
        vctDoubleVec::const_iterator enc_acc_cnts_per_sec_sec = mEncoderAccelerationCountsPerSecSec.begin();
        vctDoubleVec::const_iterator enc_vel_delay = mEncoderVelocityDelay.begin();
        vctDoubleVec::const_iterator scale = mBitsToPositionScales.begin();
        for (; velocity != end;
             ++velocity,
                 ++enc_vel_cnts_per_sec,
                 ++enc_acc_cnts_per_sec_sec,
                 ++enc_vel_delay,
                 ++scale) {
            const double vel_term = *enc_vel_cnts_per_sec;
            const double acc_term = *enc_acc_cnts_per_sec_sec * *enc_vel_delay;
            // Don't decelerate past a zero-crossing; first expression checks whether vel_term and
            // acc_term have different signs (i.e., one positive and one negative).
            if ((vel_term * acc_term < 0.0) && (abs(acc_term) > abs(vel_term))){
                *velocity = 0.0;
            }
            else {
                *velocity = *scale * (vel_term + acc_term);
            }
        }
    }
}

void mtsRobot1394::ActuatorEffortToCurrent(const vctDoubleVec & efforts, vctDoubleVec & currents) const {
    currents.ElementwiseProductOf(efforts, mEffortToCurrentScales);
}

void mtsRobot1394::ActuatorCurrentToBits(const vctDoubleVec & currents, vctIntVec & bits) const
{
    const vctDoubleVec::const_iterator end = currents.end();
    vctDoubleVec::const_iterator current = currents.begin();
    vctDoubleVec::const_iterator scale = mActuatorCurrentToBitsScales.begin();
    vctDoubleVec::const_iterator offset = mActuatorCurrentToBitsOffsets.begin();
    vctIntVec::iterator bit = bits.begin();
    for (; current != end;
         ++current,
             ++scale,
             ++offset,
             ++bit) {
        *bit = static_cast<int>(*current * *scale + *offset);
    }
}

void mtsRobot1394::ActuatorBitsToCurrent(const vctIntVec & bits, vctDoubleVec & currents) const
{
    const vctIntVec::const_iterator end = bits.end();
    vctIntVec::const_iterator bit = bits.begin();
    vctDoubleVec::const_iterator scale =  mActuatorBitsToCurrentScales.begin();
    vctDoubleVec::const_iterator offset = mActuatorBitsToCurrentOffsets.begin();
    vctDoubleVec::iterator current = currents.begin();
    for (; bit != end;
         ++bit,
             ++scale,
             ++offset,
             ++current) {
        *current = static_cast<double>(*bit) * *scale + *offset;
    }
}

void mtsRobot1394::ActuatorCurrentToEffort(const vctDoubleVec & currents, vctDoubleVec & efforts) const {
    efforts.ElementwiseRatioOf(currents, mEffortToCurrentScales);
}

void mtsRobot1394::BrakeCurrentToBits(const vctDoubleVec & currents, vctIntVec & bits) const
{
    const vctDoubleVec::const_iterator end = currents.end();
    vctDoubleVec::const_iterator current = currents.begin();
    vctDoubleVec::const_iterator scale = mBrakeCurrentToBitsScales.begin();
    vctDoubleVec::const_iterator offset = mBrakeCurrentToBitsOffsets.begin();
    vctIntVec::iterator bit = bits.begin();
    for (; current != end;
         ++current,
             ++scale,
             ++offset,
             ++bit) {
        *bit = static_cast<int>(*current * *scale + *offset);
    }
}

void mtsRobot1394::BrakeBitsToCurrent(const vctIntVec & bits, vctDoubleVec & currents) const
{
    const vctIntVec::const_iterator end = bits.end();
    vctIntVec::const_iterator bit = bits.begin();
    vctDoubleVec::const_iterator scale =  mBrakeBitsToCurrentScales.begin();
    vctDoubleVec::const_iterator offset = mBrakeBitsToCurrentOffsets.begin();
    vctDoubleVec::iterator current = currents.begin();
    for (; bit != end;
         ++bit,
             ++scale,
             ++offset,
             ++current) {
        *current = static_cast<double>(*bit) * *scale + *offset;
    }
}

void mtsRobot1394::PotBitsToVoltage(const vctIntVec & bits, vctDoubleVec & voltages) const
{
    const vctIntVec::const_iterator end = bits.end();
    vctIntVec::const_iterator bit = bits.begin();
    vctDoubleVec::const_iterator scale = mBitsToVoltageScales.begin();
    vctDoubleVec::const_iterator offset = mBitsToVoltageOffsets.begin();
    vctDoubleVec::iterator voltage = voltages.begin();

    for (; bit != end;
         ++bit,
             ++scale,
             ++offset,
             ++voltage) {
        *voltage = static_cast<double>(*bit) * *scale + *offset;
    }
}

void mtsRobot1394::PotVoltageToPosition(const vctDoubleVec & voltages, vctDoubleVec & pos) const
{
    pos.ElementwiseProductOf(voltages, mVoltageToPositionScales);
    pos.SumOf(pos, mVoltageToPositionOffsets);
}
