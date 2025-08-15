/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides, Anton Deguet
  Created on: 2011-06-10

  (C) Copyright 2011-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cmath>
#include <cctype>

#include <cisstCommon/cmnPath.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsStateTable.h>

#include <Amp1394/AmpIORevision.h>
#if ((Amp1394_VERSION_MAJOR < 1) || ((Amp1394_VERSION_MAJOR == 1) && (Amp1394_VERSION_MINOR < 1)))
#error "Version 1.1 or higher of libAmpIO is required (change to signed encoder positions)"
#endif

#include <AmpIO.h>
#include <BasePort.h>

#include <sawRobotIO1394/mtsRobot1394.h>

using namespace sawRobotIO1394;

mtsRobot1394::mtsRobot1394(const cmnGenericObject & owner,
                           const osaRobot1394Configuration & config,
                           const bool calibrationMode):
    OwnerServices(owner.Services()),
    mCalibrationMode(calibrationMode),
    // IO Structures
    mActuatorInfo(),
    m_unique_boards(),
    // State Initialization
    mValid(false),
    mFullyPowered(false),
    mPreviousFullyPowered(false),
    mPowerEnable(false),
    mPowerStatus(false),
    mPowerFault(false),
    mPreviousPowerFault(false),
    mWatchdogTimeoutStatus(true),
    mPreviousWatchdogTimeoutStatus(true),
    mWatchdogPeriod(sawRobotIO1394::WatchdogTimeout),
    mSafetyRelay(false),
    mSafetyRelayStatus(false),
    mCurrentSafetyViolationsCounter(0),
    mCurrentSafetyViolationsMaximum(100)
{
    this->mCalibrationMode = calibrationMode;
    this->Configure(config);
}

mtsRobot1394::~mtsRobot1394()
{
    delete m_state_table_read;
    delete m_state_table_write;
}

bool mtsRobot1394::SetupStateTables(const size_t stateTableSize,
                                    mtsStateTable * & stateTableRead,
                                    mtsStateTable * & stateTableWrite)
{
    if (m_state_table_read || m_state_table_write) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupStateTables: state tables have already been created for robot: "
                                 << this->Name() << std::endl;
        return false;
    }

    m_state_table_read = new mtsStateTable(stateTableSize, this->Name() + "Read");
    m_state_table_read->SetAutomaticAdvance(false);
    m_state_table_write = new mtsStateTable(stateTableSize, this->Name() + "Write");
    m_state_table_write->SetAutomaticAdvance(false);

    m_state_table_read->AddData(mEncoderChannelsA, "EncoderChannelA");
    m_state_table_read->AddData(mValid, "Valid");
    m_state_table_read->AddData(mFullyPowered, "FullyPowered");
    m_state_table_read->AddData(mPowerEnable, "PowerEnable");
    m_state_table_read->AddData(mPowerStatus, "PowerStatus");
    m_state_table_read->AddData(mPowerFault, "PowerFault");
    m_state_table_read->AddData(mSafetyRelay, "SafetyRelay");
    m_state_table_read->AddData(mSafetyRelayStatus, "SafetyRelayStatus");
    m_state_table_read->AddData(mWatchdogTimeoutStatus, "WatchdogTimeoutStatus");
    m_state_table_read->AddData(mWatchdogPeriod, "WatchdogPeriod");
    m_state_table_read->AddData(mActuatorTimestamp, "ActuatorTimestamp");
    m_state_table_read->AddData(mActuatorTemperature, "ActuatorTemperature");
    m_state_table_read->AddData(mActuatorAmpStatus, "ActuatorAmpStatus");
    m_state_table_read->AddData(mActuatorAmpEnable, "ActuatorAmpEnable");
    m_state_table_read->AddData(mEncoderPositionBits, "PositionEncoderRaw");
    m_state_table_read->AddData(mPotentiometerBits, "AnalogInRaw");
    m_state_table_read->AddData(mPotentiometerVoltage, "AnalogInVolts");
    m_state_table_read->AddData(m_raw_pot_measured_js, "raw_pot_measured_js"); // wherever pots are mounted
    m_state_table_read->AddData(m_pot_measured_js, "pot_measured_js"); // in actuator space
    m_state_table_read->AddData(mActuatorCurrentBitsFeedback, "ActuatorFeedbackCurrentRaw");
    m_state_table_read->AddData(mActuatorCurrentFeedback, "ActuatorFeedbackCurrent");

    m_state_table_read->AddData(m_measured_js, "measured_js");
    m_state_table_read->AddData(mEncoderAcceleration, "measured_ja");
    m_state_table_read->AddData(m_firmware_measured_js, "firmware_measured_js");
    m_state_table_read->AddData(m_software_measured_js, "software_measured_js");

    m_state_table_write->AddData(mActuatorCurrentBitsCommand, "ActuatorControlCurrentRaw");
    m_state_table_write->AddData(mActuatorCurrentCommand, "ActuatorControlCurrent");

    m_state_table_read->AddData(mBrakeAmpStatus, "BrakeAmpStatus");
    m_state_table_read->AddData(mBrakeAmpEnable, "BrakeAmpEnable");
    m_state_table_write->AddData(mBrakeCurrentBitsCommand, "BrakeControlCurrentRaw");
    m_state_table_write->AddData(mBrakeCurrentCommand, "BrakeControlCurrent");
    m_state_table_read->AddData(mBrakeCurrentFeedback, "BrakeFeedbackCurrent");
    m_state_table_read->AddData(mBrakeTemperature, "BrakeTemperature");

    mtsStateTable::AccessorBase * accessor_base;
    accessor_base = m_state_table_read->GetAccessorByInstance(m_pot_measured_js);
    CMN_ASSERT(accessor_base);
    m_pot_measured_js_accessor = dynamic_cast<mtsStateTable::Accessor<prmStateJoint>* >(accessor_base);
    CMN_ASSERT(m_pot_measured_js_accessor);
    accessor_base = m_state_table_read->GetAccessorByInstance(m_measured_js);
    CMN_ASSERT(accessor_base);
    m_measured_js_accessor = dynamic_cast<mtsStateTable::Accessor<prmStateJoint>*>(accessor_base);
    CMN_ASSERT(m_measured_js_accessor);

    // return pointers to state tables
    stateTableRead = m_state_table_read;
    stateTableWrite = m_state_table_write;
    return true;
}

void mtsRobot1394::SetupInterfaces(mtsInterfaceProvided * robotInterface)
{
    mInterface = robotInterface;
    robotInterface->AddMessageEvents();

    robotInterface->AddCommandRead(&mtsRobot1394::GetNumberOfActuators, this,
                                   "GetNumberOfActuators");
    robotInterface->AddCommandRead(&mtsRobot1394::GetSerialNumber, this,
                                   "GetSerialNumber");
    robotInterface->AddCommandReadState(*m_state_table_read, this->mValid,
                                        "IsValid");

    // safety relays
    robotInterface->AddCommandReadState(*m_state_table_read, mSafetyRelay,
                                        "GetSafetyRelay"); // bool
    robotInterface->AddCommandReadState(*m_state_table_read, mSafetyRelayStatus,
                                        "GetSafetyRelayStatus"); // bool
    robotInterface->AddCommandWrite(&mtsRobot1394::WriteSafetyRelay, this,
                                    "WriteSafetyRelay"); // bool

    // Enable // Disable
    robotInterface->AddCommandReadState(*m_state_table_read, mFullyPowered,
                                        "GetFullyPowered"); // bool
    robotInterface->AddCommandVoid(&mtsRobot1394::PowerOnSequence, this,
                                   "PowerOnSequence");
    robotInterface->AddCommandWrite(&mtsRobot1394::PowerOffSequence, this,
                                    "PowerOffSequence"); // bool, true to open safety relays
    robotInterface->AddCommandVoid(&mtsRobot1394::Explain, this,
                                   "Explain");
    robotInterface->AddCommandWrite(&mtsRobot1394::set_LED_pattern, this,
                                    "set_LED_pattern");

    robotInterface->AddCommandReadState(*m_state_table_read, mPowerEnable,
                                        "GetPowerEnable"); // bool
    robotInterface->AddCommandReadState(*m_state_table_read, mPowerStatus,
                                        "GetPowerStatus"); // bool
    robotInterface->AddCommandWrite(&mtsRobot1394::WritePowerEnable, this,
                                    "WritePowerEnable"); // bool

    robotInterface->AddCommandReadState(*m_state_table_read, mPowerFault,
                                        "GetPowerFault"); // bool

    robotInterface->AddCommandWrite(&mtsRobot1394::SetWatchdogPeriod, this,
                                    "SetWatchdogPeriod");

    robotInterface->AddCommandReadState(*m_state_table_read, mActuatorAmpEnable,
                                        "GetActuatorAmpEnable"); // vector[bool]
    robotInterface->AddCommandReadState(*m_state_table_read, mActuatorAmpStatus,
                                        "GetActuatorAmpStatus"); // vector[bool]

    robotInterface->AddCommandReadState(*m_state_table_read, mWatchdogTimeoutStatus,
                                        "GetWatchdogTimeoutStatus"); // bool
    robotInterface->AddCommandReadState(*m_state_table_read, mWatchdogPeriod,
                                        "GetWatchdogPeriod"); // double
    robotInterface->AddCommandReadState(*m_state_table_read, mActuatorTimestamp,
                                        "GetActuatorTimestamp");
    robotInterface->AddCommandReadState(*m_state_table_read, mActuatorTemperature,
                                        "GetActuatorAmpTemperature"); // vector[double]

    robotInterface->AddCommandReadState(*m_state_table_read, mEncoderChannelsA,
                                        "GetEncoderChannelA"); // vector[bool]
    robotInterface->AddCommandReadState(*m_state_table_read, mEncoderPositionBits,
                                        "GetPositionEncoderRaw"); // vector[int]

    robotInterface->AddCommandReadState(*m_state_table_read, mEncoderAcceleration,
                                        "GetAcceleration"); // vector[double]

    robotInterface->AddCommandReadState(*m_state_table_read, m_measured_js,
                                        "measured_js");
    robotInterface->AddCommandReadState(*m_state_table_read, m_firmware_measured_js,
                                        "firmware/measured_js");
    robotInterface->AddCommandReadState(*m_state_table_read, m_software_measured_js,
                                        "software/measured_js");
    robotInterface->AddCommandReadState(*m_state_table_read, mPotentiometerBits,
                                        "GetAnalogInputRaw");
    robotInterface->AddCommandReadState(*m_state_table_read, mPotentiometerVoltage,
                                        "GetAnalogInputVolts");
    robotInterface->AddCommandReadState(*m_state_table_read, m_raw_pot_measured_js,
                                        "raw_pot/measured_js");
    robotInterface->AddCommandReadState(*m_state_table_read, m_pot_measured_js,
                                        "pot/measured_js");

    robotInterface->AddCommandReadState(*m_state_table_read, mActuatorCurrentBitsFeedback,
                                        "GetActuatorFeedbackCurrentRaw");
    robotInterface->AddCommandReadState(*m_state_table_read, mActuatorCurrentFeedback,
                                        "GetActuatorFeedbackCurrent");
    robotInterface->AddCommandReadState(*m_state_table_write, mActuatorCurrentCommand,
                                        "GetActuatorRequestedCurrent");

    robotInterface->AddCommandWrite(&mtsRobot1394::UsePotentiometersForSafetyCheck, this,
                                    "UsePotsForSafetyCheck", mUsePotentiometersForSafetyCheck);

    robotInterface->AddCommandWrite<mtsRobot1394, vctBoolVec>(&mtsRobot1394::SetBrakeAmpEnable, this,
                                                              "SetBrakeAmpEnable", mBrakeAmpEnable); // vector[bool]
    robotInterface->AddCommandReadState(*m_state_table_read, mBrakeAmpEnable,
                                        "GetBrakeAmpEnable"); // vector[bool]
    robotInterface->AddCommandReadState(*m_state_table_read, mBrakeAmpStatus,
                                        "GetBrakeAmpStatus"); // vector[bool]
    robotInterface->AddCommandReadState(*m_state_table_read, mBrakeCurrentFeedback,
                                        "GetBrakeFeedbackCurrent");
    robotInterface->AddCommandReadState(*m_state_table_write, mBrakeCurrentCommand,
                                        "GetBrakeRequestedCurrent");
    robotInterface->AddCommandReadState(*m_state_table_read, mBrakeTemperature,
                                        "GetBrakeAmpTemperature"); // vector[double]

    robotInterface->AddCommandWrite(&mtsRobot1394::servo_jf, this,
                                    "servo_jf", mTorqueJoint);

    robotInterface->AddCommandWrite(&mtsRobot1394::SetActuatorCurrentBits, this,
                                    "SetActuatorCurrentRaw", mActuatorCurrentBitsCommand);
    robotInterface->AddCommandWrite(&mtsRobot1394::SetActuatorCurrent, this,
                                    "SetActuatorCurrent", mActuatorCurrentCommand);
    robotInterface->AddCommandRead(&mtsRobot1394::GetActuatorCurrentCommandLimits, this,
                                   "GetActuatorCurrentMax", vctDoubleVec());
    robotInterface->AddCommandRead(&mtsRobot1394::configuration_js, this,
                                   "configuration_js", m_configuration_js);
    robotInterface->AddCommandWrite(&mtsRobot1394::configure_js, this,
                                    "configure_js", m_configuration_js);

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
    robotInterface->AddCommandQualifiedRead(&mtsRobot1394::PotentiometerBitsToVoltage, this,
                                            "AnalogInBitsToVolts", mPotentiometerBits, mPotentiometerVoltage);

    //
    robotInterface->AddCommandWrite(&mtsRobot1394::CalibrateEncoderOffsetsFromPotentiometers,
                                    this, "BiasEncoder");
    robotInterface->AddCommandWrite(&mtsRobot1394::SetSomeEncoderPosition, this,
                                    "SetSomeEncoderPosition");

    // Events
    robotInterface->AddEventWrite(EventTriggers.FullyPowered, "FullyPowered", false);
    robotInterface->AddEventWrite(EventTriggers.PowerFault, "PowerFault", false);
    robotInterface->AddEventWrite(EventTriggers.WatchdogTimeoutStatus, "WatchdogTimeoutStatus", false);
    robotInterface->AddEventWrite(EventTriggers.WatchdogPeriod, "WatchdogPeriod", sawRobotIO1394::WatchdogTimeout);
    robotInterface->AddEventWrite(EventTriggers.BiasEncoder, "BiasEncoder", 0);
    robotInterface->AddEventWrite(EventTriggers.UsePotentiometersForSafetyCheck, "UsePotentiometersForSafetyCheck", false);

    // from old actuator interface
    // todo: are these used anywhere?
    robotInterface->AddCommandWrite<mtsRobot1394, vctBoolVec>(&mtsRobot1394::SetActuatorAmpEnable, this,
                                                              "SetActuatorAmpEnable", mActuatorAmpEnable); // vector[bool]
    robotInterface->AddCommandQualifiedRead(&mtsRobot1394::ActuatorCurrentToBits, this,
                                            "DriveAmpsToBits", mActuatorCurrentFeedback, mActuatorCurrentBitsFeedback);
    robotInterface->AddCommandQualifiedRead(&mtsRobot1394::PotentiometerVoltageToPosition, this,
                                            "AnalogInVoltsToPosSI", mPotentiometerVoltage, m_raw_pot_measured_js.Position());
}

void mtsRobot1394::Startup(void)
{
    if (m_configuration.hardware_version == osa1394::dRA1) {
        // do an encoder preload since we always use the lookup table
        SetEncoderPosition(vctDoubleVec(m_number_of_actuators, 0.0));
        // check the serial number
        std::string calFileName = m_unique_boards.begin()->second->ReadRobotSerialNumber();
        std::string expectedCalFileName =
            static_cast<char>(std::tolower(this->Name().at(0)))
            + this->SerialNumber() + ".cal";
        if (calFileName != expectedCalFileName) {
            std::string message = "Arm was configured using the name \"" + this->Name()
                + "\" and the serial number \"" + this->SerialNumber()
                + "\". Therefore the original cal file should have been \""
                + expectedCalFileName + "\" but we found \""
                + calFileName + "\".  Make sure your serial numbers are correct!";
            mInterface->SendError(message);
            exit(EXIT_FAILURE);
        } else {
            mInterface->SendStatus("IO: " + this->Name() + " arm correctly identified by serial number ("
                                   + expectedCalFileName + ")");
        }
    }
}


void mtsRobot1394::StartReadStateTable(void) {
    m_state_table_read->Start();
}


void mtsRobot1394::AdvanceReadStateTable(void) {
    m_state_table_read->Advance();
}


void mtsRobot1394::StartWriteStateTable(void) {
    m_state_table_write->Start();
}


void mtsRobot1394::AdvanceWriteStateTable(void) {
    m_state_table_write->Advance();
}

void mtsRobot1394::GetNumberOfActuators(size_t & numberOfActuators) const {
    numberOfActuators = this->NumberOfActuators();
}

void mtsRobot1394::GetSerialNumber(std::string & serialNumber) const {
    serialNumber = this->SerialNumber();
}


void mtsRobot1394::UsePotentiometersForSafetyCheck(const bool & usePotentiometersForSafetyCheck)
{
    mUsePotentiometersForSafetyCheck = usePotentiometersForSafetyCheck;
    mPotentiometerErrorDuration.SetAll(0.0);
    mPotentiometerValid.SetAll(true);
    // trigger mts event
    EventTriggers.UsePotentiometersForSafetyCheck(usePotentiometersForSafetyCheck);
}


void mtsRobot1394::servo_jf(const prmForceTorqueJointSet & efforts) {
    this->SetActuatorEffort(efforts.ForceTorque());
}


void mtsRobot1394::SetSomeEncoderPosition(const prmMaskedDoubleVec & values) {
    for (size_t index = 0; index < values.Mask().size(); ++index) {
        if (values.Mask().at(index)) {
            this->SetSingleEncoderPosition(index, values.Data().at(index));
        }
    }
}


void mtsRobot1394::CalibrateEncoderOffsetsFromPotentiometers(const int & numberOfSamples)
{
    // if the number of samples is negative, user wants to first check
    // if the encoders are not already preloaded
    if (numberOfSamples < 0) {
        // get encoder preload values re. midrange
        bool anyAtMidRange = false;
        for (size_t i = 0; i < m_number_of_actuators; i++) {
            bool thisAxis;
            mActuatorInfo[i].board->IsEncoderPreloadMidrange(mActuatorInfo[i].axis, thisAxis);
            if (thisAxis) {
                anyAtMidRange = true;
            }
        }
        // if none at midrange, assume encoder bias on pots is already done
        if (!anyAtMidRange) {
            CalibrateEncoderOffsets.SamplesFromPotentiometers = 0;
            CalibrateEncoderOffsets.Performed = true;
            EventTriggers.BiasEncoder(-1);
            return;
        }
    }
    CalibrateEncoderOffsets.SamplesFromPotentiometers = std::abs(numberOfSamples) + 1;
    CalibrateEncoderOffsets.SamplesFromPotentiometersRequested = std::abs(numberOfSamples);
}


bool mtsRobot1394::CheckConfiguration(void)
{
    if ((m_configuration.hardware_version != osa1394::dRA1)
        && (NumberOfActuators() > 2)) {
        const int first_offset = m_configuration.actuators[0].drive.current_to_bits.offset;
        bool allEqual = true;
        for (const auto & actuator : m_configuration.actuators) {
            allEqual &= (first_offset == actuator.drive.current_to_bits.offset);
        }
        if (allEqual) {
            CMN_LOG_CLASS_INIT_ERROR << "CheckConfiguration: all currents to bits offsets are equal, please calibrate the current offsets for arm: "
                                     << this->Name() << std::endl;
            return false;
        }
    }
    return true;
}


void mtsRobot1394::LoadPotentiometerLookupTable(void)
{
    if (m_number_of_actuators == 0) {
        CMN_LOG_CLASS_INIT_ERROR << "LoadPotentiometerLookupTable for arm: "
                                 << this->Name() << " must be call by Configure" << std::endl;
        exit(EXIT_FAILURE);
    }

    // make sure the filename has the serial number in it
    // to prevent mismatch
    if (m_configuration.serial_number == "") {
        CMN_LOG_CLASS_INIT_ERROR << "LoadPotentiometerLookupTable: the robot serial number must be defined for "
                                 << this->Name() << std::endl;
        exit(EXIT_FAILURE);
    }
    const std::string::size_type serialNumberFound
        = m_configuration.potentiometers.lookup_table_file.find(m_configuration.serial_number);
    if (serialNumberFound == std::string::npos) {
        CMN_LOG_CLASS_INIT_ERROR << "LoadPotentiometerLookupTable: the potentiometer lookup table file name \""
                                 << m_configuration.potentiometers.lookup_table_file << "\" for "
                                 << Name() << " doesn't contain the serial number ("
                                 << m_configuration.serial_number << ").  You're likely using the wrong lookup file"
                                 << std::endl;
        exit(EXIT_FAILURE);
    }

    // load the file
    cmnPath path;
    for (const auto & dir : m_configuration.path) {
        path.Add(dir, cmnPath::TAIL);
    }
    path.Add(cmnPath::GetWorkingDirectory(), cmnPath::TAIL);

    std::string filename = path.Find(m_configuration.potentiometers.lookup_table_file);
    if (filename == "") {
        CMN_LOG_CLASS_INIT_ERROR << "Unable to find the potentiometer lookup table file \""
                                 << m_configuration.potentiometers.lookup_table_file << "\" for "
                                 << Name() << " in path " << path << std::endl;
        exit(EXIT_FAILURE);
    }

    try {
        std::ifstream jsonStream;
        Json::Value jsonValue;
        Json::Reader jsonReader;

        jsonStream.open(filename.c_str());
        if (!jsonReader.parse(jsonStream, jsonValue)) {
            CMN_LOG_CLASS_INIT_ERROR << "LoadPotentioneterLookupTable: error found while parsing \""
                                     << filename << "\":"
                                     << jsonReader.getFormattedErrorMessages();
            exit(EXIT_FAILURE);
        }
        // check the serial number in the file
        std::string inFileSerial = jsonValue["serial"].asString();
        if (inFileSerial != m_configuration.serial_number) {
            CMN_LOG_CLASS_INIT_ERROR << "LoadPotentiometerLookupTable: serial number found lookup table file ("
                                     << inFileSerial << ") doesn't match the arm one ("
                                     << m_configuration.serial_number << ")" << std::endl;
            exit(EXIT_FAILURE);
        }
        cmnDataJSON<vctDoubleMat>::DeSerializeText(mPotentiometerLookupTable, jsonValue["lookup"]);
        // make sure the table size makes sense
        if ((mPotentiometerLookupTable.rows() != m_configuration.actuators.size())
            || (mPotentiometerLookupTable.cols() == 0)) {
            CMN_LOG_CLASS_INIT_ERROR << "LoadPotentiometerLookupTable: size of lookup table for " << Name()
                                     << " from " << m_configuration.potentiometers.lookup_table_file
                                     << " doesn't match the number of actuators, found "
                                     << mPotentiometerLookupTable.rows() << " but was expecting " << m_configuration.actuators.size()
                                     << std::endl;
            exit(EXIT_FAILURE);
        }
        // for logs
        CMN_LOG_CLASS_INIT_VERBOSE << "LoadPotentiometerLookupTable: loaded potentiometer lookup table from file \""
                                   << filename << "\" for arm " << Name()
                                   << " (" << m_configuration.serial_number << ")" << std::endl;
    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "LoadPotentiometerLookupTable: error found while parsing \""
                                 << m_configuration.potentiometers.lookup_table_file << "\", make sure the file is in JSON format."
                                 << std::endl;
        exit(EXIT_FAILURE);
    }
}


void mtsRobot1394::Configure(const osaRobot1394Configuration & config)
{
    // Store the configuration
    m_configuration = config;

    m_number_of_actuators = m_configuration.actuators.size();

    // Low-level API
    mActuatorInfo.resize(m_number_of_actuators);

    // Initialize state vectors to the appropriate sizes
    mActuatorAmpStatus.SetSize(m_number_of_actuators);
    mActuatorAmpEnable.SetSize(m_number_of_actuators);
    mDigitalInputs.SetSize(m_number_of_actuators);
    mEncoderChannelsA.SetSize(m_number_of_actuators);
    mPotentiometerBits.SetSize(m_number_of_actuators);
    mEncoderOverflow.SetSize(m_number_of_actuators);
    if (m_configuration.only_IO) {
        mEncoderOverflow.SetAll(false);
    }
    mPreviousEncoderOverflow.SetSize(m_number_of_actuators);
    mPreviousEncoderOverflow.SetAll(false);
    mEncoderPositionBits.SetSize(m_number_of_actuators);
    mActuatorCurrentBitsCommand.SetSize(m_number_of_actuators);
    mActuatorCurrentBitsFeedback.SetSize(m_number_of_actuators);

    mActuatorTimestamp.SetSize(m_number_of_actuators);
    mPotentiometerVoltage.SetSize(m_number_of_actuators);
    m_raw_pot_measured_js.Position().SetSize(m_number_of_actuators);
    m_pot_measured_js.Position().SetSize(m_number_of_actuators);
    mEncoderVelocityPredictedCountsPerSec.SetSize(m_number_of_actuators);
    mEncoderAccelerationCountsPerSecSec.SetSize(m_number_of_actuators);
    mEncoderAcceleration.SetSize(m_number_of_actuators);

    // software velocity variables
    m_firmware_measured_js.Velocity().SetSize(m_number_of_actuators);
    m_software_measured_js.Velocity().SetSize(m_number_of_actuators);
    mPreviousEncoderPositionBits.SetSize(m_number_of_actuators);
    mActuatorTimestampChange.SetSize(m_number_of_actuators);
    mActuatorTimestampChange.SetAll(0.0);
    mVelocitySlopeToZero.SetSize(m_number_of_actuators);
    mVelocitySlopeToZero.SetAll(0.0);

    mActuatorCurrentCommand.SetSize(m_number_of_actuators);
    mActuatorEffortCommand.SetSize(m_number_of_actuators);
    mActuatorCurrentFeedback.SetSize(m_number_of_actuators);

    // Initialize property vectors to the appropriate sizes
    m_configuration_js.Type().SetSize(m_number_of_actuators);
    m_configuration_js.PositionMin().SetSize(m_number_of_actuators);
    m_configuration_js.PositionMax().SetSize(m_number_of_actuators);
    m_configuration_js.EffortMin().SetSize(m_number_of_actuators);
    m_configuration_js.EffortMax().SetSize(m_number_of_actuators);
    m_measured_js.Position().SetSize(m_number_of_actuators);
    m_measured_js.Velocity().SetSize(m_number_of_actuators);
    m_measured_js.Effort().SetSize(m_number_of_actuators);

    // names
    m_measured_js.Name().resize(m_number_of_actuators);
    for (size_t index = 0; index < m_number_of_actuators; ++index) {
        m_measured_js.Name().at(index) = "actuator_" + std::to_string(index);
    }
    cmnDataCopy(m_firmware_measured_js.Name(), m_measured_js.Name());
    cmnDataCopy(m_software_measured_js.Name(), m_measured_js.Name());
    cmnDataCopy(m_pot_measured_js.Name(), m_measured_js.Name());
    // these names might be confusing since we name the pots based on
    // actuator names
    cmnDataCopy(m_raw_pot_measured_js.Name(), m_measured_js.Name());

    mActuatorCurrentFeedbackLimits.SetSize(m_number_of_actuators);
    mPotentiometerErrorDuration.SetSize(m_number_of_actuators);
    mPotentiometerValid.SetSize(m_number_of_actuators);
    mPotentiometerErrorDuration.SetAll(0.0);
    mPotentiometerValid.SetAll(true);
    mUsePotentiometersForSafetyCheck = false;

    mActuatorTemperature.SetSize(m_number_of_actuators);

    mBrakeReleasing = false;

    // Construct property vectors
    for (size_t i = 0; i < m_number_of_actuators; i++) {

        // Local references to the config properties
        const osaActuator1394Configuration & actuator = config.actuators.at(i);
        const osaDrive1394Configuration & drive = actuator.drive;
        const osaEncoder1394Configuration & encoder = actuator.encoder;

        m_configuration_js.Type().at(i) = actuator.joint_type;
        m_configuration_js.PositionMin().at(i) = encoder.position_limits_soft.lower;
        m_configuration_js.PositionMax().at(i) = encoder.position_limits_soft.upper;
        m_configuration_js.EffortMin().at(i) = -drive.maximum_current / drive.effort_to_current.scale;
        m_configuration_js.EffortMax().at(i) =  drive.maximum_current / drive.effort_to_current.scale;

        // 120% of command current is in the acceptable range
        // Add 50 mA for non motorized actuators due to a2d noise
        mActuatorCurrentFeedbackLimits.at(i) = 1.2 * actuator.drive.maximum_current + (50.0 / 1000.0);

        // Initialize state vectors
        m_measured_js.Position().at(i) = 0.0;
        mActuatorCurrentCommand.at(i) = 0.0;
        mActuatorCurrentFeedback.at(i) = 0.0;
    }

    // check if pots are digital
    if (config.potentiometers.potentiometers_type == osaPotentiometers1394Configuration::DIGITAL) {
        if (m_configuration.potentiometers.lookup_table_file != "") {
            LoadPotentiometerLookupTable();
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: potentiometers.lookup_table_file is not defined for arm: "
                                     << this->Name() << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // Update brake data
    mBrakeInfo.resize(m_number_of_brakes);
    mBrakeReleasingTimer.resize(m_number_of_brakes);

    mBrakeCurrentFeedbackLimits.SetSize(m_number_of_brakes);
    mBrakeAmpStatus.SetSize(m_number_of_brakes);
    mBrakeAmpEnable.SetSize(m_number_of_brakes);
    mBrakeCurrentBitsCommand.SetSize(m_number_of_brakes);
    mBrakeCurrentBitsFeedback.SetSize(m_number_of_brakes);
    mBrakeTimestamp.SetSize(m_number_of_brakes);
    mBrakeCurrentCommand.SetSize(m_number_of_brakes);
    mBrakeCurrentFeedback.SetSize(m_number_of_brakes);
    mBrakeTemperature.SetSize(m_number_of_brakes);

    // Construct property vectors for brakes
    for (size_t i = 0; i < m_number_of_brakes; i++) {
        const osaDrive1394Configuration & drive = config.brakes.at(i).drive;
        // 120% of command current is in the acceptable range
        // Add 50 mA for a2d noise around 0
        mBrakeCurrentFeedbackLimits[i] = 1.2 * drive.maximum_current + (50.0 / 1000.0);
        // Initialize defaults
        mBrakeCurrentCommand[i] = 0.0;
        mBrakeCurrentFeedback[i] = 0.0;
    }
}


void mtsRobot1394::SetBoards(const std::vector<osaActuatorMapping> & actuatorBoards,
                             const std::vector<osaBrakeMapping> & brakeBoards)
{
    if (actuatorBoards.size() != m_number_of_actuators) {
        cmnThrow(this->Name() + ": number of actuator boards different than the number of actuators.");
    }

    if (brakeBoards.size() != m_number_of_brakes) {
        cmnThrow(this->Name() + ": number of brake boards different than the number of brakes.");
    }

    for (size_t i = 0; i < m_number_of_actuators; i++) {
        // Store this board
        mActuatorInfo.at(i).board = actuatorBoards.at(i).board;
        mActuatorInfo.at(i).board_id = actuatorBoards.at(i).board_id;
        mActuatorInfo.at(i).axis = actuatorBoards.at(i).axis;
        // Construct a list of unique boards
        m_unique_boards[actuatorBoards.at(i).board->GetBoardId()] = actuatorBoards.at(i).board;
    }

    for (size_t i = 0; i < m_number_of_brakes; i++) {
        // Store this board
        mBrakeInfo.at(i).board = brakeBoards.at(i).board;
        mBrakeInfo.at(i).board_id = brakeBoards.at(i).board_id;
        mBrakeInfo.at(i).axis = brakeBoards.at(i).axis;
        // Construct a list of unique boards
        m_unique_boards[brakeBoards.at(i).board->GetBoardId()] = brakeBoards.at(i).board;
    }

    mLowestFirmWareVersion = 999999;
    mHighestFirmWareVersion = 0;
    size_t boardCounter = 0;
    for (auto board = m_unique_boards.begin();
         board != m_unique_boards.end();
         ++board, ++boardCounter) {
        // check the hardware version vs version specified in configuration file
        const auto hardwareVersion = board->second->GetHardwareVersion();
        if (!((hardwareVersion == QLA1_String && m_configuration.hardware_version == osa1394::QLA1)
              || (hardwareVersion == DQLA_String && m_configuration.hardware_version == osa1394::DQLA)
              || (hardwareVersion == dRA1_String && m_configuration.hardware_version == osa1394::dRA1))) {
            if (hardwareVersion == BCFG_String) {
                CMN_LOG_CLASS_INIT_ERROR << "SetBoards: " << this->Name()
                                         << ", hardware version query reported BCFG (boot configuration)." << std::endl
                                         << "Maybe you just need to wait for the controller to boot or forgot to insert the SD with a valid firmware." << std::endl;
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "SetBoards: " << this->Name()
                                         << ", hardware version doesn't match value from configuration file for board: " << boardCounter
                                         << ", Id: " << static_cast<int>(board->second->GetBoardId())
                                         << ".  Hardware found: " << board->second->GetHardwareVersionString()
                                         << ".  Configuration file value: " << osa1394::hardware_tToString(m_configuration.hardware_version) << std::endl;
            }
            exit(EXIT_FAILURE);
        }
        // then get firmware
        uint32_t fversion = board->second->GetFirmwareVersion();
        if (fversion == 0) {
            CMN_LOG_CLASS_INIT_ERROR << "SetBoards: " << this->Name()
                                     << ", unable to get firmware version for board: " << boardCounter
                                     << ", Id: " << static_cast<int>(board->second->GetBoardId())
                                     << ".  Make sure the controller is powered and connected" << std::endl;
            exit(EXIT_FAILURE);
        }
        std::string serialQLA;
        if (m_configuration.hardware_version == osa1394::DQLA) {
            serialQLA = board->second->GetQLASerialNumber(1) +
                ", " +  board->second->GetQLASerialNumber(2);
        }
        else {
            serialQLA = board->second->GetQLASerialNumber();
        }
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
        CMN_LOG_CLASS_INIT_WARNING << "SetBoards: " << this->Name()
                                   << ", board: " << boardCounter
                                   << ", Id: " << static_cast<int>(board->second->GetBoardId())
                                   << ", firmware: " << fversion
                                   << ", FPGA serial: " << serialFPGA
                                   << ", QLA serial: " << serialQLA
                                   << std::endl;
    }
}

void mtsRobot1394::GetFirmwareRange(unsigned int & lowest, unsigned int & highest) const
{
    lowest = mLowestFirmWareVersion;
    highest = mHighestFirmWareVersion;
}

void mtsRobot1394::PollValidity(void)
{
    // Make sure the boards have been configured
    if (m_number_of_actuators != mActuatorInfo.size()) {
        cmnThrow(this->Name() + ": number of boards different than the number of actuators.");
    }

    // Store previous state
    mPreviousFullyPowered = mFullyPowered;
    mPreviousPowerFault = mPowerFault;
    mPreviousWatchdogTimeoutStatus = mWatchdogTimeoutStatus;

    // Initialize flags
    mValid = true;
    mPowerEnable = true;
    mPowerStatus = true;
    mPowerFault = false;
    mSafetyRelay = true;
    mSafetyRelayStatus = true;
    mWatchdogTimeoutStatus = false;

    // Get status from boards
    for (auto & board : m_unique_boards) {
        mValid &= board.second->ValidRead();
        mPowerEnable &= board.second->GetPowerEnable();
        mPowerStatus &= board.second->GetPowerStatus();
        mPowerFault |= board.second->GetPowerFault();
        mSafetyRelay &= board.second->GetSafetyRelay();
        mSafetyRelayStatus &= board.second->GetSafetyRelayStatus();
        mWatchdogTimeoutStatus |= board.second->GetWatchdogTimeoutStatus();
    }

    mFullyPowered = mPowerStatus && !mPowerFault && mSafetyRelay && mSafetyRelayStatus && !mWatchdogTimeoutStatus;

    if (!mValid) {
        if (mInvalidReadCounter == 0) {
            mInvalidReadCounter++;
            std::stringstream message;
            message << this->Name() << ": port read error on board(s) ";
            for (auto & board : m_unique_boards) {
                if (!board.second->ValidRead()) {
                    message << static_cast<int>(board.second->GetBoardId()) << " ";
                }
            }
            cmnThrow(message.str());
        } else {
            mInvalidReadCounter++;
            if (mInvalidReadCounter == 10000) {
                mInvalidReadCounter = 0;
                cmnThrow(this->Name() + ": port read errors, occurred 10,000 times");
            }
        }
    } else {
        mInvalidReadCounter = 0;
    }
}

void mtsRobot1394::PollState(void)
{
    // Poll data
    for (size_t i = 0; i < m_number_of_actuators; i++) {
        AmpIO * board = mActuatorInfo[i].board;
        int axis = mActuatorInfo[i].axis;

        if (!board || (axis < 0)) continue; // We probably don't need this check any more

        mActuatorTimestamp[i] = board->GetTimestampSeconds();
        mDigitalInputs[i] = board->GetDigitalInput();

        // vectors of bits
        if (!m_configuration.only_IO) {
            mEncoderOverflow[i] = board->GetEncoderOverflow(axis);
        }
        mEncoderChannelsA[i] = board->GetEncoderChannelA(axis);

        // convert from 24 bits signed stored in 32 unsigned to 32 signed
        mEncoderPositionBits[i] = board->GetEncoderPosition(axis);

        // Get estimation of acceleration, not used internally but maybe users could
        mEncoderAccelerationCountsPerSecSec[i] = board->GetEncoderAcceleration(axis);

        // Second argument below is how much quantization error do we accept
        mEncoderVelocityPredictedCountsPerSec[i] = board->GetEncoderVelocityPredicted(axis, 0.0005);

        mPotentiometerBits[i] = board->GetAnalogInput(axis);

        mActuatorCurrentBitsFeedback[i] = board->GetMotorCurrent(axis);
        mActuatorAmpEnable[i] = board->GetAmpEnable(axis);
        mActuatorAmpStatus[i] = board->GetAmpStatus(axis);

        // first temperature corresponds to first 2 actuators, second to last 2
        // board reports temperature in celsius * 2
        mActuatorTemperature[i] = (board->GetAmpTemperature(axis / 2)) / 2.0;
    }

    for (size_t i = 0; i < m_number_of_brakes; i++) {
        AmpIO * board = mBrakeInfo[i].board;
        int axis = mBrakeInfo[i].axis;

        if (!board || (axis < 0)) continue; // We probably don't need this check any more

        mBrakeTimestamp[i] = board->GetTimestampSeconds();
        mBrakeCurrentBitsFeedback[i] = board->GetMotorCurrent(axis);
        mBrakeAmpEnable[i] = board->GetAmpEnable(axis);
        mBrakeAmpStatus[i] = board->GetAmpStatus(axis);

        // first temperature corresponds to first 2 brakes, second to last 2
        // board reports temperature in celsius * 2
        mBrakeTemperature[i] = (board->GetAmpTemperature(axis / 2)) / 2.0;
    }

}

void mtsRobot1394::ConvertState(void)
{
    // Perform read conversions
    EncoderBitsToPosition(mEncoderPositionBits,
                          m_measured_js.Position());

    // Velocities
    const auto f_vel_end = m_firmware_measured_js.Velocity().end();
    auto f_vel =  m_firmware_measured_js.Velocity().begin();
    auto conf = m_configuration.actuators.cbegin();
    auto enc_vel_pred_cps = mEncoderVelocityPredictedCountsPerSec.cbegin();
    auto enc_acc_pred_cpss = mEncoderAccelerationCountsPerSecSec.cbegin();
    auto enc_acc = mEncoderAcceleration.begin();
    for (; f_vel != f_vel_end;
         ++f_vel, ++conf, ++enc_vel_pred_cps, ++enc_acc_pred_cpss, ++enc_acc) {
        // Firmware
        *f_vel =   conf->encoder.bits_to_position.scale * *enc_vel_pred_cps;
        *enc_acc = conf->encoder.bits_to_position.scale * *enc_acc_pred_cpss;
    }

    // Effort computation
    ActuatorBitsToCurrent(mActuatorCurrentBitsFeedback,
                          mActuatorCurrentFeedback);
    ActuatorCurrentToEffort(mActuatorCurrentFeedback,
                            m_measured_js.Effort());

    BrakeBitsToCurrent(mBrakeCurrentBitsFeedback, mBrakeCurrentFeedback);

    // Software based velocity estimation
    const double timeToZeroVelocity = 1.0 * cmn_s;
    const auto end = mEncoderPositionBits.cend();
    vctIntVec::const_iterator currentEncoder, previousEncoder;
    vctDoubleVec::const_iterator currentTimestamp;
    vctDoubleVec::const_iterator encoderVelocity;
    vctDoubleVec::iterator lastChangeTimestamp, slope, velocity;
    conf = m_configuration.actuators.begin();
    for (currentEncoder = mEncoderPositionBits.begin(),
             previousEncoder = mPreviousEncoderPositionBits.begin(),
             currentTimestamp = mActuatorTimestamp.begin(),
             lastChangeTimestamp = mActuatorTimestampChange.begin(),
             slope = mVelocitySlopeToZero.begin(),
             velocity = m_software_measured_js.Velocity().begin();
         // end
         currentEncoder != end;
         // increment
         ++currentEncoder,
             ++previousEncoder,
             ++currentTimestamp,
             ++conf,
             ++lastChangeTimestamp,
             ++slope,
             ++velocity) {
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
                    * (conf->encoder.bits_to_position.scale);
            } else {
                if (difference > 1) {
                    // we know all but 1 bit difference happened in last Dt, other bit change happened between now and last change
                    *velocity = ((difference - 1.0) / (*currentTimestamp) + 1.0 / (*lastChangeTimestamp))
                        * (conf->encoder.bits_to_position.scale);
                } else {
                    *velocity = ((difference + 1.0) / (*currentTimestamp) - 1.0 / (*lastChangeTimestamp))
                        * (conf->encoder.bits_to_position.scale);
                }
            }
            // keep record of this change
            *lastChangeTimestamp = 0.0;
            *slope = (*velocity) / (timeToZeroVelocity);
        }
    }
    // Finally save previous encoder bits position and populate position/effort
    mPreviousEncoderPositionBits.Assign(mEncoderPositionBits);

    // fill all measured_js
    m_firmware_measured_js.Position().ForceAssign(m_measured_js.Position());
    m_firmware_measured_js.Effort().ForceAssign(m_measured_js.Effort());
    m_software_measured_js.Position().ForceAssign(m_measured_js.Position());
    m_software_measured_js.Effort().ForceAssign(m_measured_js.Effort());
    const auto end_v = m_measured_js.Velocity().end();
    auto measured_v = m_measured_js.Velocity().begin();
    auto firm_v = m_firmware_measured_js.Velocity().cbegin();
    auto soft_v = m_software_measured_js.Velocity().cbegin();
    auto act_conf = m_configuration.actuators.cbegin();
    for (;
         // end
         measured_v != end_v;
         // increment
         ++measured_v, ++firm_v, ++soft_v, ++act_conf) {
        // pick
        if (act_conf->encoder.velocity_source == osaEncoder1394Configuration::FIRMWARE) {
            *measured_v = *firm_v;
        } else {
            *measured_v = *soft_v;
        }
    }

    // Potentiometers
    switch (m_configuration.potentiometers.potentiometers_type) {
    case osaPotentiometers1394Configuration::ANALOG:
        {
            PotentiometerBitsToVoltage(mPotentiometerBits, mPotentiometerVoltage);
            PotentiometerVoltageToPosition(mPotentiometerVoltage, m_raw_pot_measured_js.Position());
        }
        break;
    case osaPotentiometers1394Configuration::DIGITAL:
        {
            mPotentiometerVoltage.Assign(mPotentiometerBits);
            auto raw = mPotentiometerBits.cbegin();
            const auto end = mPotentiometerBits.cend();
            size_t index = 0;
            auto si = m_raw_pot_measured_js.Position().begin();
            for (; raw != end;
                 ++raw,
                     ++index,
                     ++si) {
                // look up in table
                *si = mPotentiometerLookupTable.Row(index).at(*raw);
            }
        }
        break;
    default:
        break;
    }

    // Potentiometers, convert to actuator space if the coupling matrix is defined
    vctDoubleMat & coupling = m_configuration.potentiometers.coupling.JointToActuatorPosition();
    if (coupling.size() != 0) {
        m_pot_measured_js.Position().ProductOf(coupling,
                                               m_raw_pot_measured_js.Position());
    } else {
        m_pot_measured_js.Position().Assign(m_raw_pot_measured_js.Position());
    }
}


void mtsRobot1394::CheckState(void)
{
    // set data as invalid by default
    m_measured_js.SetValid(false);
    m_firmware_measured_js.SetValid(false);
    m_software_measured_js.SetValid(false);
    m_raw_pot_measured_js.SetValid(false);
    m_pot_measured_js.SetValid(false);

    // If we had a read error, all checks are pretty much useless
    if (mInvalidReadCounter > 0) {
        return;
    }

    // Perform safety checks
    bool currentSafetyViolation = false;
    // actuators
    {
        const auto end = mActuatorCurrentFeedback.cend();
        auto feedback = mActuatorCurrentFeedback.cbegin();
        auto limit = mActuatorCurrentFeedbackLimits.cbegin();
        auto enabled = mActuatorAmpEnable.cbegin();
        size_t index = 0;
        for (; feedback < end;
             ++feedback,
                 ++limit,
                 ++enabled,
                 ++index) {
            double actual_limit;
            if (*enabled) {
                actual_limit = *limit;
            } else {
                if (mCalibrationMode || !mPowerEnable) {
                    actual_limit = 0.2; // noise + poor calibration
                } else {
                    actual_limit = 0.1; // 100 mA for noise in a2d
                }
            }
            if (fabs(*feedback) >= actual_limit) {
                CMN_LOG_CLASS_RUN_WARNING << "CheckState: " << this->Name() << ", actuator " << index
                                          << " power: " << *feedback
                                          << " > limit: " << actual_limit << std::endl;
                currentSafetyViolation = true;
            }
        }
    }
    // brakes
    {
        const auto end = mBrakeCurrentFeedback.cend();
        auto feedback = mBrakeCurrentFeedback.cbegin();
        auto limit = mBrakeCurrentFeedbackLimits.cbegin();
        size_t index = 0;
        for (; feedback < end;
             ++feedback,
                 ++limit,
                 ++index) {
            if (fabs(*feedback) >= *limit) {
                CMN_LOG_CLASS_RUN_WARNING << "CheckState: " << this->Name() << ", brake " << index
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
        this->PowerOffSequenceOnError(false /* do no open safety relays */);
        cmnThrow(this->Name() + ": too many consecutive current safety violations.  Power has been disabled.");
    }

    // check safety amp disable
    bool newSafetyAmpDisabled = false;
    for (auto & board : m_unique_boards) {
        uint32_t safetyAmpDisable = board.second->GetSafetyAmpDisable();
        if (safetyAmpDisable) {
            newSafetyAmpDisabled = true;
        }
    }
    if (newSafetyAmpDisabled && !mSafetyAmpDisabled) {
        // update status - this needs to be here, throw will interrupt execution...
        mSafetyAmpDisabled = newSafetyAmpDisabled;
        // throw only if this is new
        cmnThrow(this->Name() + ": hardware current safety amp disable tripped." + mActuatorTimestamp.ToString());
    } else {
        // update status
        mSafetyAmpDisabled = newSafetyAmpDisabled;
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
                    CMN_LOG_CLASS_RUN_ERROR << "CheckState: " << this->Name() << ", actuator " << index
                                            << " temperature: " << *temperature
                                            << " greater than error threshold: " << sawRobotIO1394::TemperatureErrorThreshold << std::endl;
                    temperatureError = true;
                    temperatureTrigger = *temperature;
                } else {
                    if (*temperature > sawRobotIO1394::TemperatureWarningThreshold) {
                        CMN_LOG_CLASS_RUN_DEBUG << "CheckState: " << this->Name() << ", actuator " << index
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
                    CMN_LOG_CLASS_RUN_ERROR << "CheckState: " << this->Name() << ", brake " << index
                                            << " temperature: " << *temperature
                                            << " greater than error threshold: " << sawRobotIO1394::TemperatureErrorThreshold << std::endl;
                    temperatureError = true;
                    temperatureTrigger = *temperature;
                } else {
                    if (*temperature > sawRobotIO1394::TemperatureWarningThreshold) {
                        CMN_LOG_CLASS_RUN_DEBUG << "CheckState: " << this->Name() << ", brake " << index
                                                << " temperature: " << *temperature
                                                << " greater than warning threshold: " << sawRobotIO1394::TemperatureWarningThreshold << std::endl;
                        temperatureWarning = true;
                        temperatureTrigger = *temperature;
                    }
                }
            }
        }

        if (temperatureError) {
            this->PowerOffSequenceOnError(false /* do not open safety relays */);
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
            mTimeLastTemperatureWarning += time;
        } else {
            // reset time so next time we hit a warning it displays immediately
            mTimeLastTemperatureWarning = sawRobotIO1394::TimeBetweenTemperatureWarnings;
        }
    }

    // Check if brakes are releasing/released
    if (mBrakeReleasing) {
        bool allReleased = true;
        // check how much time per brake, set all to high (release current)
        mBrakeReleasingTimer.Add(mBrakeTimestamp);
        for (size_t index = 0; index < m_number_of_brakes; ++index) {
            mBrakeCurrentCommand[index] = m_configuration.brakes[index].release_current;
            if (mBrakeReleasingTimer[index] >  m_configuration.brakes[index].release_time) {
                // lower to releaseD current
                mBrakeCurrentCommand[index] = m_configuration.brakes[index].released_current;
            } else {
                allReleased = false;
            }
        }
        SetBrakeCurrent(mBrakeCurrentCommand);
        mBrakeReleasing = !allReleased;
    }


    // For dRAC based arms, make sure the pots value are meaningfull
    if (m_configuration.hardware_version == osa1394::dRA1 && !mCalibrationMode) {
        bool foundMissingPotentiometer = false;
        for (const auto & potValue : m_pot_measured_js.Position()) {
            if (mtsRobot1394::IsMissingPotentiometerValue(potValue)) {
                foundMissingPotentiometer = true;
            }
        }
        if (foundMissingPotentiometer) {
            this->PowerOffSequenceOnError();
            // send error message without flooding the UI
            if (mTimeLastPotentiometerMissingError >= sawRobotIO1394::TimeBetweenPotentiometerMissingErrors) {
                mInterface->SendError("IO: " + this->Name() + " detected an unknow pot position, make sure you're using the correct lookup configuration file or recalibrate your potentiometers");
                mTimeLastPotentiometerMissingError = 0.0;
            }
            double time = 0.0;
            if (!mActuatorTimestamp.empty()) {
                time = *(mActuatorTimestamp.begin());
            } else if (!mBrakeTimestamp.empty()) {
                time = *(mBrakeTimestamp.begin());
            }
            mTimeLastPotentiometerMissingError += time;
        } else {
            // reset time so next time we hit a warning it displays immediately
            mTimeLastPotentiometerMissingError = sawRobotIO1394::TimeBetweenPotentiometerMissingErrors;
        }
    }

    // Check if encoders and potentiometers agree
    if (mUsePotentiometersForSafetyCheck) {
        vctDynamicVectorRef<double> encoderRef;
        // todo: multiply pots by PotCoupling so everything is in actuator space
        encoderRef.SetRef(m_measured_js.Position());

        bool statusChanged = false;
        bool error = false;
        auto pot = m_pot_measured_js.Position().cbegin();
        const auto potEnd = m_pot_measured_js.Position().cend();
        auto enc = encoderRef.cbegin();
        auto tolerance = m_configuration.potentiometers.tolerances.cbegin();
        auto potTimestamp = mActuatorTimestamp.cbegin();
        auto potDuration = mPotentiometerErrorDuration.begin();
        auto potValid = mPotentiometerValid.begin();

        for (;
             pot != potEnd;
             ++pot,
                 ++enc,
                 ++tolerance,
                 ++potTimestamp,
                 ++potDuration,
                 ++potValid) {
            // if tolerance set to 0, disable check for that joint
            if (tolerance->distance == 0.0) {
                *potValid = true;
            } else {
                // check for error
                double delta = std::abs(*pot - *enc);
                if (delta > tolerance->distance) {
                    *potDuration += *potTimestamp;
                    // check how long have we been off
                    if (*potDuration > tolerance->latency) {
                        // now we have a problem,
                        this->PowerOffSequenceOnError(false); // don't open safety relays
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
                this->PowerOffSequenceOnError(false /* do not open safety relays */);
                std::string errorMessage = "IO: " + this->Name() + ": inconsistency between encoders and potentiometers";
                std::string warningMessage = errorMessage + "\nencoders:\n";
                warningMessage.append(encoderRef.ToString());
                warningMessage.append("\npotentiomers:\n");
                warningMessage.append(m_pot_measured_js.Position().ToString());
                warningMessage.append("\nvalid pots:\n");
                warningMessage.append(mPotentiometerValid.ToString());
                warningMessage.append("\nerror duration:\n");
                warningMessage.append(mPotentiometerErrorDuration.ToString());
                warningMessage.append("\nconfiguration:\n");
                warningMessage.append(m_configuration.potentiometers.HumanReadable());
                mInterface->SendWarning(warningMessage);
                cmnThrow(errorMessage);
            } else {
                CMN_LOG_CLASS_RUN_VERBOSE << "IO: " << this->Name()
                                          << ": check between encoders and potentiomenters, recovery.  Valid pots:" << std::endl
                                          << mPotentiometerValid << std::endl;
            }
        }
    }

    // Check for encoder overflow
    if (mEncoderOverflow.Any()) {
        this->PowerOffSequenceOnError(false /* do not open safety relays */);
        this->SetEncoderPosition(vctDoubleVec(m_number_of_actuators, 0.0));
        if (mEncoderOverflow.NotEqual(mPreviousEncoderOverflow)) {
            mPreviousEncoderOverflow.Assign(mEncoderOverflow);
            std::string errorMessage = this->Name() + ": encoder overflow detected: ";
            errorMessage.append(mEncoderOverflow.ToString());
            // if we have already performed encoder calibration, this is really bad
            if (CalibrateEncoderOffsets.Performed) {
                cmnThrow(errorMessage);
            } else {
                mInterface->SendError("IO: " + this->Name() + " encoder overflow detected");
            }
        }
    }

    m_measured_js.SetValid(true);
    m_firmware_measured_js.SetValid(true);
    m_software_measured_js.SetValid(true);
    m_raw_pot_measured_js.SetValid(true);
    m_pot_measured_js.SetValid(true);

    if (mPreviousFullyPowered != mFullyPowered) {
        EventTriggers.FullyPowered(mFullyPowered);
        // this might be an issues
        if (!mFullyPowered && mUserExpectsPower) {
            // give some time to power, if greater then it's an issue
            if ((m_state_table_read->Tic - mPoweringStartTime) > sawRobotIO1394::MaximumTimeToPower) {
                this->PowerOffSequenceOnError(false /* do not open safety relays */);
                mInterface->SendError("IO: " + this->Name() + " power is unexpectedly off");
            }
        }
    }

    if (mPreviousPowerFault != mPowerFault) {
        EventTriggers.PowerFault(mPowerFault);
        if (mPowerFault) {
            // give some time to power, if greater then it's an issue
            if ((m_state_table_read->Tic - mPoweringStartTime) > sawRobotIO1394::MaximumTimeForMVGood) {
                this->PowerOffSequenceOnError(false /* do not open safety relays */);
                mInterface->SendError("IO: " + this->Name() + " detected power fault");
            }
        }
    }

    if (mPreviousWatchdogTimeoutStatus != mWatchdogTimeoutStatus) {
        EventTriggers.WatchdogTimeoutStatus(mWatchdogTimeoutStatus);
        if (mWatchdogTimeoutStatus) {
            this->PowerOffSequenceOnError(false /* do not open safety relays */);
            mInterface->SendError("IO: " + this->Name() + " watchdog triggered");
        } else {
            mInterface->SendStatus("IO: " + this->Name() + " watchdog ok");
        }
    }

    // if nb samples > 0, need to countdown
    if (CalibrateEncoderOffsets.SamplesFromPotentiometers > 0) {
        // if count down is at 1, compute average of encoders and potentiometers
        if (CalibrateEncoderOffsets.SamplesFromPotentiometers > 1) {
            CalibrateEncoderOffsets.SamplesFromPotentiometers--;
        } else {
            // data read from state table
            vctDoubleVec potentiometers(m_number_of_actuators, 0.0);
            prmStateJoint newPot;
            newPot.Position().SetSize(m_number_of_actuators);
            vctDoubleVec encoderRef(m_number_of_actuators, 0.0);
            vctDoubleVec encoderDelta(m_number_of_actuators);
            prmStateJoint newEnc;
            newEnc.Position().SetSize(m_number_of_actuators);

            int nbElements = 0;
            mtsStateIndex index = m_state_table_read->GetIndexReader();
            bool validIndex = true;
            while (validIndex && (nbElements < CalibrateEncoderOffsets.SamplesFromPotentiometersRequested)) {
                m_pot_measured_js_accessor->Get(index, newPot);
                m_measured_js_accessor->Get(index, newEnc);
                if (nbElements == 0) {
                    // find reference encoder value
                    encoderRef.Assign(newEnc.Position());
                } else {
                    // correct pot using encoder delta
                    encoderDelta.DifferenceOf(newEnc.Position(), encoderRef);
                    newPot.Position().Subtract(encoderDelta);
                    potentiometers.Add(newPot.Position());
                }
                ++nbElements;
                --index;
                // check if index is still valid, would happen if reached size of state table
                validIndex = m_state_table_read->ValidateReadIndex(index);
            }

            // compute average
            potentiometers.Divide(nbElements);

            // determine where pots are
            vctDoubleVec actuatorPosition(m_number_of_actuators);
            SetEncoderPosition(potentiometers);

            // samples from pots not needed anymore
            CalibrateEncoderOffsets.SamplesFromPotentiometers = 0;
            // one cycle to write encoder preload, one to get encoder
            // with new offsets.  then send event so higher level
            // classes have calibrated positions
            CalibrateEncoderOffsets.PostCalibrationCounter = 2;
        }
    }

    // post encoder calibration event
    if (CalibrateEncoderOffsets.PostCalibrationCounter >= 0) {
        if (CalibrateEncoderOffsets.PostCalibrationCounter == 0) {
            // ready to send event
            CalibrateEncoderOffsets.PostCalibrationCounter = -1;
            EventTriggers.BiasEncoder(CalibrateEncoderOffsets.SamplesFromPotentiometersRequested);
        } else {
            // not ready, keep decrementing
            CalibrateEncoderOffsets.PostCalibrationCounter--;
        }
    }
}

void mtsRobot1394::PowerOnSequence(void)
{
    mUserExpectsPower = true;
    mPoweringStartTime = m_state_table_read->Tic;
    mTimeLastTemperatureWarning = sawRobotIO1394::TimeBetweenTemperatureWarnings;
    WriteSafetyRelay(true);
    WritePowerEnable(true);
    SetActuatorAmpEnable(true);
    SetBrakeAmpEnable(true);
}

void mtsRobot1394::PowerOffSequence(const bool & openSafetyRelays)
{
    mUserExpectsPower = false;
    // write to boards directly
    // disable all axes
    for (auto & board : m_unique_boards) {
        board.second->WriteAmpEnable(0x0f, 0x00);
    }

    // disable all boards
    WritePowerEnable(false);
    if (openSafetyRelays) {
        WriteSafetyRelay(false);
    }
}

void mtsRobot1394::PowerOffSequenceOnError(const bool & openSafetyRelays)
{
    if (mUserExpectsPower && (m_configuration.hardware_version == osa1394::dRA1)) {
        this->Explain();
    }
    PowerOffSequence(openSafetyRelays);
}

void mtsRobot1394::Explain(void)
{
    for (auto board : m_unique_boards) {
        CMN_LOG_CLASS_RUN_ERROR << "Explain: " << this->Name()
                                << " - " << board.second->ExplainSiFault() << std::endl;
    }
}


void mtsRobot1394::set_LED_pattern(const prmInputData & pattern)
{
    if ((pattern.AnalogInputs().size() != 2)
        || (pattern.DigitalInputs().size() != 2)) {
        CMN_LOG_CLASS_RUN_WARNING << "set_LED_pattern for " << this->Name() << " has incorrect number of values" << std::endl;
        return;
    }
    for (auto board : m_unique_boards) {
        board.second->WriteRobotLED(static_cast<uint32_t>(pattern.AnalogInputs().at(0)),
                                    static_cast<uint32_t>(pattern.AnalogInputs().at(1)),
                                    pattern.DigitalInputs().at(0),
                                    pattern.DigitalInputs().at(1));
    }
}


void mtsRobot1394::SetWatchdogPeriod(const double & periodInSeconds)
{
    mWatchdogPeriod = periodInSeconds;
    for (auto & board : m_unique_boards) {
        board.second->WriteWatchdogPeriodInSeconds(periodInSeconds);
    }
    EventTriggers.WatchdogPeriod(mWatchdogPeriod);
}

void mtsRobot1394::WriteSafetyRelay(const bool & close)
{
    for (auto & board : m_unique_boards) {
        board.second->WriteSafetyRelay(close);
    }
}

void mtsRobot1394::WritePowerEnable(const bool & power)
{
    if (power) {
        mSafetyAmpDisabled = false;
    }
    for (auto & board : m_unique_boards) {
        board.second->WritePowerEnable(power);
    }
}

void mtsRobot1394::SetActuatorAmpEnable(const bool & enable)
{
    if (m_configuration.hardware_version == osa1394::dRA1 && mCalibrationMode) {
        mInterface->SendWarning("IO: " + this->Name() + " can't power actuator since we're in calibration mode");
        return;
    }
    mSafetyAmpDisabled = false;
    for (size_t i = 0; i < m_number_of_actuators; i++) {
        mActuatorInfo[i].board->SetAmpEnable(mActuatorInfo[i].axis, enable);
    }
}

void mtsRobot1394::SetActuatorAmpEnable(const vctBoolVec & enable)
{
    if (m_configuration.hardware_version == osa1394::dRA1 && mCalibrationMode) {
        mInterface->SendWarning("IO: " + this->Name() + " can't power actuator since we're in calibration mode");
        return;
    }
    mSafetyAmpDisabled = false;
    for (size_t i = 0; i < m_number_of_actuators; i++) {
        mActuatorInfo[i].board->SetAmpEnable(mActuatorInfo[i].axis, enable[i]);
    }
}

void mtsRobot1394::SetBrakeAmpEnable(const bool & enable)
{
    mSafetyAmpDisabled = false;
    for (size_t i = 0; i < m_number_of_brakes; i++) {
        mBrakeInfo[i].board->SetAmpEnable(mBrakeInfo[i].axis, enable);
    }
}

void mtsRobot1394::SetBrakeAmpEnable(const vctBoolVec & enable)
{
    mSafetyAmpDisabled = false;
    for (size_t i = 0; i < m_number_of_brakes; i++) {
        mBrakeInfo[i].board->SetAmpEnable(mBrakeInfo[i].axis, enable[i]);
    }
}

void mtsRobot1394::SetEncoderPosition(const vctDoubleVec & pos)
{
    vctIntVec bits(m_number_of_actuators);
    this->EncoderPositionToBits(pos, bits);
    this->SetEncoderPositionBits(bits);
}

void mtsRobot1394::SetEncoderPositionBits(const vctIntVec & bits)
{
    for (size_t i = 0; i < m_number_of_actuators; i++) {
        mActuatorInfo[i].board->WriteEncoderPreload(mActuatorInfo[i].axis, bits[i]);
    }
    // initialize software based velocity variables
    mPreviousEncoderPositionBits.Assign(bits);
    mActuatorTimestampChange.SetAll(0.0);
    mVelocitySlopeToZero.SetAll(0.0);
}

void mtsRobot1394::SetSingleEncoderPosition(const int index, const double pos)
{
    int bits = pos
        / m_configuration.actuators[index].encoder.bits_to_position.scale;
    SetSingleEncoderPositionBits(index, bits);

}

void mtsRobot1394::SetSingleEncoderPositionBits(const int index, const int bits)
{
    mActuatorInfo[index].board->WriteEncoderPreload(mActuatorInfo[index].axis, bits);
    // initialize software based velocity variables
    mPreviousEncoderPositionBits.Element(index) = bits;
    mActuatorTimestampChange.Element(index) = 0.0;
    mVelocitySlopeToZero.Element(index) = 0.0;
}


void mtsRobot1394::ClipActuatorEffort(vctDoubleVec & efforts)
{
    efforts.ElementwiseClipIn(m_configuration_js.EffortMax());
}


void mtsRobot1394::ClipActuatorCurrent(vctDoubleVec & currents)
{
    auto current = currents.begin();
    for (const auto & actuator : m_configuration.actuators) {
        *current = std::clamp(*current,
                              - actuator.drive.maximum_current,
                              actuator.drive.maximum_current);
        ++current;
    }
}


void mtsRobot1394::ClipBrakeCurrent(vctDoubleVec & currents)
{
    auto current = currents.begin();
    for (const auto & brake : m_configuration.brakes) {
        *current = std::clamp(*current,
                              - brake.drive.maximum_current,
                              brake.drive.maximum_current);
        ++current;
    }
}


void mtsRobot1394::SetActuatorEffort(const vctDoubleVec & efforts)
{
    // Convert efforts to bits and set the command
    vctDoubleVec clipped_efforts = efforts;
    vctDoubleVec currents(m_number_of_actuators);

    // this->clip_actuator_efforts(clipped_efforts);

    this->ActuatorEffortToCurrent(clipped_efforts, currents);
    this->SetActuatorCurrent(currents);
}

void mtsRobot1394::SetActuatorCurrent(const vctDoubleVec & currents)
{
    // Convert amps to bits and set the command
    vctDoubleVec clipped_amps = currents;
    vctIntVec bits(m_number_of_actuators);

    this->ClipActuatorCurrent(clipped_amps);
    this->ActuatorCurrentToBits(clipped_amps, bits);
    this->SetActuatorCurrentBits(bits);

    // Store commanded amps
    mActuatorCurrentCommand = clipped_amps;
}

void mtsRobot1394::SetActuatorCurrentBits(const vctIntVec & bits)
{
    for (size_t i = 0; i < m_number_of_actuators; i++) {
        mActuatorInfo[i].board->SetMotorCurrent(mActuatorInfo[i].axis, bits[i]);
    }

    // Store commanded bits
    mActuatorCurrentBitsCommand = bits;
}

void mtsRobot1394::SetActuatorVoltageRatio(const vctDoubleVec & ratios)
{
    for (size_t i = 0; i < m_number_of_actuators; i++) {
        mActuatorInfo[i].board->SetMotorVoltageRatio(mActuatorInfo[i].axis, ratios[i]);
    }
}

void mtsRobot1394::SetBrakeCurrent(const vctDoubleVec & currents)
{
    // Convert amps to bits and set the command
    vctDoubleVec clipped_amps = currents;
    vctIntVec bits(m_number_of_brakes);

    this->ClipBrakeCurrent(clipped_amps);
    this->BrakeCurrentToBits(clipped_amps, bits);
    this->SetBrakeCurrentBits(bits);

    // Store commanded amps
    mBrakeCurrentCommand = clipped_amps;
}


void mtsRobot1394::SetBrakeCurrentBits(const vctIntVec & bits)
{
    for (size_t i = 0; i < m_number_of_brakes; i++) {
        mBrakeInfo[i].board->SetMotorCurrent(mBrakeInfo[i].axis, bits[i]);
    }

    // Store commanded bits
    mBrakeCurrentBitsCommand = bits;
}


void mtsRobot1394::BrakeRelease(void)
{
    if (m_number_of_brakes != 0) {
        mBrakeReleasing = true;
        mBrakeReleasingTimer.SetAll(0.0);
        vctDoubleVec currents(m_configuration.number_of_brakes);
        auto current = currents.begin();
        for (const auto & brake : m_configuration.brakes) {
            *current = brake.release_current;
            ++current;
        }
        SetBrakeCurrent(currents);
    }
}


void mtsRobot1394::BrakeEngage(void)
{
    if (m_number_of_brakes != 0) {
        mBrakeReleasing = false;
        vctDoubleVec currents(m_configuration.number_of_brakes);
        auto current = currents.begin();
        for (const auto & brake : m_configuration.brakes) {
            *current = brake.engaged_current;
            ++current;
        }
        SetBrakeCurrent(currents);
    }
}


void mtsRobot1394::CalibrateEncoderOffsetsFromPotentiometers(void)
{
    vctDoubleVec actuatorPosition(m_number_of_actuators);
    SetEncoderPosition(m_pot_measured_js.Position());
    CalibrateEncoderOffsets.Performed = true;
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


const vctIntVec & mtsRobot1394::PotentiometerBits(void) const {
    return mPotentiometerBits;
}


const vctDoubleVec & mtsRobot1394::PotentiometerPosition(void) const {
    return m_pot_measured_js.Position();
}


const vctDoubleVec & mtsRobot1394::ActuatorTimestamp(void) const {
    return mActuatorTimestamp;
}


const vctDoubleVec & mtsRobot1394::BrakeTimestamp(void) const {
    return mBrakeTimestamp;
}


const vctDoubleVec & mtsRobot1394::EncoderAcceleration(void) const {
    return mEncoderAcceleration;
}


const prmStateJoint & mtsRobot1394::ActuatorJointState(void) const {
    return m_measured_js;
}


osaRobot1394Configuration mtsRobot1394::GetConfiguration(void) const {
    return m_configuration;
}


std::string mtsRobot1394::Name(void) const {
    return m_configuration.name;
}


size_t mtsRobot1394::NumberOfActuators(void) const {
    return m_number_of_actuators;
}


std::string mtsRobot1394::SerialNumber(void) const {
    return m_configuration.serial_number;
}


size_t mtsRobot1394::NumberOfBrakes(void) const {
    return m_number_of_brakes;
}


void mtsRobot1394::configuration_js(prmConfigurationJoint & jointConfig) const
{
    jointConfig = m_configuration_js;
}


void mtsRobot1394::configure_js(const prmConfigurationJoint & jointConfig)
{
    if (jointConfig.Name().size() != m_number_of_actuators) {
        cmnThrow(this->Name() + ": configure_js, incorrect number of actuators.");
    }
    cmnDataCopy(m_configuration_js.Name(), jointConfig.Name());
    cmnDataCopy(m_measured_js.Name(), jointConfig.Name());
    cmnDataCopy(m_firmware_measured_js.Name(), jointConfig.Name());
    cmnDataCopy(m_software_measured_js.Name(), jointConfig.Name());
    cmnDataCopy(m_pot_measured_js.Name(), jointConfig.Name());
    cmnDataCopy(m_raw_pot_measured_js.Name(), jointConfig.Name());
}


void mtsRobot1394::GetActuatorCurrentCommandLimits(vctDoubleVec & limits) const
{
    auto limit = limits.begin();
    for (const auto & actuator : m_configuration.actuators) {
        *limit = actuator.drive.maximum_current;
        ++limit;
    }
}


void mtsRobot1394::EncoderPositionToBits(const vctDoubleVec & pos, vctIntVec & bits) const
{
    const auto end = pos.cend();
    auto position = pos.cbegin();
    auto conf = m_configuration.actuators.cbegin();
    auto bit = bits.begin();
    for (; position != end;
         ++position, ++conf, ++bit) {
        *bit = static_cast<int>((*position - conf->encoder.bits_to_position.offset)
                                / conf->encoder.bits_to_position.scale);
    }
}


void mtsRobot1394::EncoderBitsToPosition(const vctIntVec & bits, vctDoubleVec & pos) const
{
    const auto end = bits.cend();
    auto bit = bits.cbegin();
    auto conf = m_configuration.actuators.cbegin();
    auto position = pos.begin();
    for (; bit != end;
         ++bit, ++conf, ++position) {
        *position = conf->encoder.bits_to_position.offset
            + static_cast<double>(*bit) * conf->encoder.bits_to_position.scale;
    }
}


void mtsRobot1394::ActuatorEffortToCurrent(const vctDoubleVec & efforts, vctDoubleVec & currents) const
{
    const auto end = efforts.cend();
    auto effort = efforts.cbegin();
    auto conf = m_configuration.actuators.cbegin();
    auto current = currents.begin();
    for (; effort != end;
         ++effort, ++conf, ++current) {
        *current = *effort * conf->drive.effort_to_current.scale;
    }
}


void mtsRobot1394::ActuatorCurrentToBits(const vctDoubleVec & currents, vctIntVec & bits) const
{
    const auto end = currents.cend();
    auto current = currents.cbegin();
    auto actuator = m_configuration.actuators.cbegin();
    auto bit = bits.begin();
    for (; current != end;
         ++current, ++actuator, ++bit) {
        *bit = static_cast<int>(*current * actuator->drive.current_to_bits.scale
                                + actuator->drive.current_to_bits.offset);
    }
}


void mtsRobot1394::ActuatorBitsToCurrent(const vctIntVec & bits, vctDoubleVec & currents) const
{
    const auto end = bits.cend();
    auto bit = bits.cbegin();
    auto actuator = m_configuration.actuators.cbegin();
    auto current = currents.begin();
    for (; bit != end;
         ++bit, ++actuator, ++current) {
        *current = static_cast<double>(*bit) * actuator->drive.bits_to_current.scale
            + actuator->drive.bits_to_current.offset;
    }
}


void mtsRobot1394::ActuatorCurrentToEffort(const vctDoubleVec & currents, vctDoubleVec & efforts) const {
    const auto end = currents.cend();
    auto current = currents.cbegin();
    auto conf = m_configuration.actuators.cbegin();
    auto effort = efforts.begin();
    for (; current != end;
         ++current, ++conf, ++effort) {
        *effort = *current / conf->drive.effort_to_current.scale;
    }
}


void mtsRobot1394::BrakeCurrentToBits(const vctDoubleVec & currents, vctIntVec & bits) const
{
    const auto end = currents.cend();
    auto current = currents.cbegin();
    auto brake = m_configuration.brakes.cbegin();
    auto bit = bits.begin();
    for (; current != end;
         ++current, ++brake, ++bit) {
        *bit = static_cast<int>(*current * brake->drive.current_to_bits.scale
                                + brake->drive.current_to_bits.offset);
    }
}


void mtsRobot1394::BrakeBitsToCurrent(const vctIntVec & bits, vctDoubleVec & currents) const
{
    const auto end = bits.cend();
    auto bit = bits.cbegin();
    auto brake = m_configuration.brakes.cbegin();
    auto current = currents.begin();
    for (; bit != end;
         ++bit, ++brake, ++current) {
        *current = static_cast<double>(*bit) * brake->drive.bits_to_current.scale
            + brake->drive.bits_to_current.offset;
    }
}


void mtsRobot1394::PotentiometerBitsToVoltage(const vctIntVec & bits, vctDoubleVec & voltages) const
{
    const auto end = bits.cend();
    auto bit = bits.cbegin();
    auto actuator = m_configuration.actuators.cbegin();
    auto voltage = voltages.begin();
    for (; bit != end;
         ++bit, ++actuator, ++voltage) {
        *voltage =
            static_cast<double>(*bit) * actuator->potentiometer->bits_to_voltage.scale
            + actuator->potentiometer->bits_to_voltage.offset;
    }
}


void mtsRobot1394::PotentiometerVoltageToPosition(const vctDoubleVec & voltages, vctDoubleVec & pos) const
{
    const auto end = voltages.cend();
    auto voltage = voltages.cbegin();
    auto actuator = m_configuration.actuators.cbegin();
    auto position = pos.begin();
    for (; voltage != end;
         ++voltage, ++actuator, ++position) {
        *position =
            *voltage * actuator->potentiometer->voltage_to_position.scale
            + actuator->potentiometer->voltage_to_position.offset;
    }
}


double mtsRobot1394::GetMissingPotentiometerValue(void)
{
    return 31.4159; // 31 meters or 5 full rotations
}


bool mtsRobot1394::IsMissingPotentiometerValue(const double & potValue)
{
    // add a fat margin for floating point precision
    return (potValue > (mtsRobot1394::GetMissingPotentiometerValue() - 0.4159));
}
