/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2012-07-31

  (C) Copyright 2011-2020 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#include <iostream>

#include <cisstBuildType.h>
#include <cisstCommon/cmnLogger.h>
#include <cisstCommon/cmnUnits.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsDigitalInput1394.h>
#include <sawRobotIO1394/mtsDigitalOutput1394.h>
#include <sawRobotIO1394/mtsDallasChip1394.h>
#include <sawRobotIO1394/mtsRobot1394.h>
#include <sawRobotIO1394/osaXML1394.h>

#include <Amp1394/AmpIORevision.h>
#if Amp1394_HAS_RAW1394
#include "FirewirePort.h"
#endif
#include "EthUdpPort.h"
// Could support raw Ethernet port if desired
//#if Amp1394_HAS_PCAP
//#include "EthRawPort.h"
//#endif
#include "AmpIO.h"

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsRobotIO1394, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg)

using namespace sawRobotIO1394;

mtsRobotIO1394::mtsRobotIO1394(const std::string & name, const double periodInSeconds, const std::string & port):
    mtsTaskPeriodic(name, periodInSeconds)
{
    Init(port);
}

mtsRobotIO1394::mtsRobotIO1394(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg)
{
    Init(mtsRobotIO1394::DefaultPort());
}

mtsRobotIO1394::~mtsRobotIO1394()
{
    // delete robots before deleting boards
    for (auto & robot : mRobots) {
        if (robot != 0) {
            delete robot;
        }
    }
    mRobots.clear();
    mRobotsByName.clear();

    // delete digital inputs before deleting boards
    for (auto & input : mDigitalInputs) {
        if (input != 0) {
            delete input;
        }
    }
    mDigitalInputs.clear();
    mDigitalInputsByName.clear();

    // delete digital outputs before deleting boards
    for (auto & output : mDigitalOutputs) {
        if (output != 0) {
            delete output;
        }
    }
    mDigitalOutputs.clear();
    mDigitalOutputsByName.clear();

    // delete Dallas chips before deleting boards
    for (auto & dallas : mDallasChips) {
        if (dallas != 0) {
            delete dallas;
        }
    }
    mDallasChips.clear();
    mDallasChipsByName.clear();

    // delete board structures
    for (board_iterator iter = mBoards.begin();
         iter != mBoards.end();
         ++iter) {
        if (iter->second != 0) {
            mPort->RemoveBoard(iter->first);
            delete iter->second;
        }
    }
    mBoards.clear();

    // delete firewire port
    if (mPort != 0) {
        delete mPort;
    }

    // delete message stream
    delete MessageStream;
}

void mtsRobotIO1394::SetProtocol(const sawRobotIO1394::ProtocolType & protocol)
{
    bool ok = false;
    switch (protocol) {
    case PROTOCOL_SEQ_RW:
        ok = mPort->SetProtocol(BasePort::PROTOCOL_SEQ_RW);
        break;
    case PROTOCOL_SEQ_R_BC_W:
        ok = mPort->SetProtocol(BasePort::PROTOCOL_SEQ_R_BC_W);
        break;
    case PROTOCOL_BC_QRW:
        ok = mPort->SetProtocol(BasePort::PROTOCOL_BC_QRW);
        break;
    default:
        break;
    }
    if (!ok) {
        CMN_LOG_CLASS_INIT_ERROR << "mtsRobot1394::SetProtocol failed" << std::endl;
        exit(EXIT_FAILURE);
    }
}

void mtsRobotIO1394::SetWatchdogPeriod(const double & periodInSeconds)
{
    mWatchdogPeriod = periodInSeconds;
    for (auto & robot : mRobots) {
        robot->SetWatchdogPeriod(periodInSeconds);
    }
}

void mtsRobotIO1394::Init(const std::string & port)
{
    // write warning to cerr if not compiled in Release mode
    if (std::string(CISST_BUILD_TYPE) != "Release") {
        std::cerr << "---------------------------------------------------- " << std::endl
                  << " Warning:                                            " << std::endl
                  << "   It seems that \"cisst\" has not been compiled in  " << std::endl
                  << "   Release mode.  Make sure your CMake configuration " << std::endl
                  << "   or catkin profile is configured to compile in     " << std::endl
                      << "   Release mode for better performance and stability " << std::endl
                  << "---------------------------------------------------- " << std::endl;
    }

    // default watchdog period
    mWatchdogPeriod = sawRobotIO1394::WatchdogTimeout;
    mSkipConfigurationCheck = false;

    // add state tables for stats
    mStateTableRead = new mtsStateTable(100, this->GetName() + "Read");
    mStateTableRead->SetAutomaticAdvance(false);
    mStateTableWrite = new mtsStateTable(100, this->GetName() + "Write");
    mStateTableWrite->SetAutomaticAdvance(false);

    // extract port type, number, IP
    BasePort::PortType portType; // firewire or udp
    int portNumber = 0; // default firewire port
    std::string address(ETH_UDP_DEFAULT_IP); // default IP
    BasePort::ParseOptions(port.c_str(), portType, portNumber, address);

    // construct port
    MessageStream = new std::ostream(this->GetLogMultiplexer());
    try {
        switch (portType) {
        case BasePort::PORT_FIREWIRE:
#if Amp1394_HAS_RAW1394
            // Construct handle to firewire port
            mPort = new FirewirePort(portNumber, *MessageStream);

            // Check number of port users
            if (mPort->NumberOfUsers() > 1) {
                std::ostringstream oss;
                oss << "osaIO1394Port: Found more than one user on firewire port: " << portNumber;
                cmnThrow(osaRuntimeError1394(oss.str()));
            }
#else
            CMN_LOG_CLASS_INIT_ERROR << "Init: can't use port: " << port
                                     << ", FireWire support is not available (set Amp1394_HAS_RAW1394 in CMake for AmpIO)."
                                     << std::endl;
            exit(EXIT_FAILURE);
#endif
            break;
        case BasePort::PORT_ETH_UDP:
            // Open Ethernet UDP port
            mPort = new EthUdpPort(portNumber, address, *MessageStream);
            break;
        default:
            CMN_LOG_CLASS_INIT_ERROR << "Init: unknown port type: " << port
                                     << ", port can be: " << std::endl
                                     << "  - a single number (implicitly a FireWire port)" << std::endl
                                     << "  - fw[X] for a FireWire port" << std::endl
                                     << "  - udp[xx.xx.xx.xx] for raw UDP (IP is optional)"
                                     << std::endl;
            exit(EXIT_FAILURE);
            break;
        }
    } catch (std::runtime_error &err) {
        CMN_LOG_CLASS_INIT_ERROR << err.what();
        exit(EXIT_FAILURE);
    }

    mtsInterfaceProvided * mainInterface = AddInterfaceProvided("MainInterface");
    if (mainInterface) {
        mainInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfBoards, this, "GetNumberOfBoards");
        mainInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfRobots, this, "GetNumberOfRobots");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "Init: failed to create provided interface \"MainInterface\", method Init should be called only once."
                                 << std::endl;
    }

    // At this stage, the robot interfaces and the digital input interfaces should be ready.
    // Add on Configuration provided interface with functionWrite with vector of strings.
    // Provide names of robot, names of digital inputs, and name of this member.

    // All previous interfaces are ready. Good start. Let's make a new provided interface.
    mtsInterfaceProvided * configurationInterface   = this->AddInterfaceProvided("Configuration");
    if (configurationInterface) {
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetRobotNames, this,
                                               "GetRobotNames");
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfActuatorsPerRobot, this,
                                               "GetNumActuators");
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfBrakesPerRobot, this,
                                               "GetNumBrakes");
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfRobots, this,
                                               "GetNumRobots");
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfDigitalInputs, this,
                                               "GetNumDigitalInputs");
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetDigitalInputNames, this,
                                               "GetDigitalInputNames");
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfDigitalOutputs, this,
                                               "GetNumDigitalOutputs");
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetDigitalOutputNames, this,
                                               "GetDigitalOutputNames");
        configurationInterface->AddCommandRead<mtsComponent>(&mtsComponent::GetName, this,
                                                             "GetName");
        configurationInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                                    "period_statistics");
        configurationInterface->AddCommandReadState(*mStateTableRead, mStateTableRead->PeriodStats,
                                                    "GetPeriodStatisticsRead");
        configurationInterface->AddCommandReadState(*mStateTableWrite, mStateTableWrite->PeriodStats,
                                                    "GetPeriodStatisticsWrite");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to create configuration interface." << std::endl;
    }

    // Set callback for interval statistics
    StateTable.PeriodStats.SetCallback(&mtsRobotIO1394::IntervalStatisticsCallback, this);
}

void mtsRobotIO1394::SkipConfigurationCheck(const bool skip)
{
    mSkipConfigurationCheck = skip;
}

void mtsRobotIO1394::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring from " << filename << std::endl;

    osaPort1394Configuration config;
    osaXML1394ConfigurePort(filename, config);

    // Add all the robots
    for (const auto & configRobot : config.Robots) {
        // Create a new robot
        mtsRobot1394 * robot = new mtsRobot1394(*this, configRobot);
        // Check the configuration if needed
        if (!mSkipConfigurationCheck) {
            if (!robot->CheckConfiguration()) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: error in configuration file \""
                                         << filename << "\" for robot \""
                                         << robot->Name() << "\"" << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        // Set up the cisstMultiTask interfaces
        if (!this->SetupRobot(robot)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to setup interface(s) for robot \""
                                     << robot->Name() << "\"" << std::endl;
            delete robot;
        } else {
            AddRobot(robot);
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: added robot \""
                                       << robot->Name() << "\"" << std::endl;

        }
    }

    // Add all the digital inputs
    for (const auto & configInput : config.DigitalInputs) {
        // Create a new digital input
        mtsDigitalInput1394 * digitalInput = new mtsDigitalInput1394(*this, configInput);
        // Set up the cisstMultiTask interfaces
        if (!this->SetupDigitalInput(digitalInput)) {
            delete digitalInput;
        } else {
            AddDigitalInput(digitalInput);
        }
    }

    // Add all the digital outputs
    for (const auto & configOutput : config.DigitalOutputs) {
        // Create a new digital input
        mtsDigitalOutput1394 * digitalOutput = new mtsDigitalOutput1394(*this, configOutput);
        // Set up the cisstMultiTask interfaces
        if (!this->SetupDigitalOutput(digitalOutput)) {
            delete digitalOutput;
        } else {
            AddDigitalOutput(digitalOutput);
        }
    }

    // Add all the Dallas chips
    for (const auto & configDallas : config.DallasChips) {
        // Create a new digital input
        mtsDallasChip1394 * dallasChip = new mtsDallasChip1394(*this, configDallas);
        // Set up the cisstMultiTask interfaces
        if (!this->SetupDallasChip(dallasChip)) {
            delete dallasChip;
        } else {
            AddDallasChip(dallasChip);
        }
    }

    // Check firmware versions used so far
    if (!CheckFirmwareVersions()) {
        exit(EXIT_FAILURE);
    }
}

bool mtsRobotIO1394::SetupRobot(mtsRobot1394 * robot)
{
    mtsStateTable * stateTableRead;
    mtsStateTable * stateTableWrite;

    // Configure StateTable for this Robot
    if (!robot->SetupStateTables(2000, // hard coded number of elements in state tables
                                 stateTableRead, stateTableWrite)) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupRobot: unable to setup state tables" << std::endl;
        return false;
    }

    this->AddStateTable(stateTableRead);
    this->AddStateTable(stateTableWrite);

    // Add new InterfaceProvided for this Robot with Name.
    // Ensure all names from XML Config file are UNIQUE!
    mtsInterfaceProvided * robotInterface = this->AddInterfaceProvided(robot->Name());
    if (!robotInterface) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupRobot: failed to create robot interface \""
                                 << robot->Name() << "\", do we have multiple robots with the same name?" << std::endl;
        return false;
    }
    // we show statistics for the whole component using the main state table
    robotInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                        "period_statistics");

    // Create actuator interface
    std::string actuatorInterfaceName = robot->Name();
    actuatorInterfaceName.append("Actuators");
    mtsInterfaceProvided * actuatorInterface = this->AddInterfaceProvided(actuatorInterfaceName);
    if (!actuatorInterface) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupRobot: failed to create robot actuator interface \""
                                 << actuatorInterfaceName << "\", do we have multiple robots with the same name?" << std::endl;
        return false;
    }

    // Setup the MTS interfaces
    robot->SetupInterfaces(robotInterface, actuatorInterface);

    return true;
}

bool mtsRobotIO1394::SetupDigitalInput(mtsDigitalInput1394 * digitalInput)
{
    // Configure pressed active direction and edge detection
    digitalInput->SetupStateTable(this->StateTable);

    mtsInterfaceProvided * digitalInInterface = this->AddInterfaceProvided(digitalInput->Name());

    digitalInput->SetupProvidedInterface(digitalInInterface, this->StateTable);
    return true;
}

bool mtsRobotIO1394::SetupDigitalOutput(mtsDigitalOutput1394 * digitalOutput)
{
    // Configure pressed active direction and edge detection
    digitalOutput->SetupStateTable(this->StateTable);

    mtsInterfaceProvided * digitalOutInterface = this->AddInterfaceProvided(digitalOutput->Name());

    digitalOutput->SetupProvidedInterface(digitalOutInterface, this->StateTable);
    return true;
}

bool mtsRobotIO1394::SetupDallasChip(mtsDallasChip1394 * dallasChip)
{
    // Configure pressed active direction and edge detection
    dallasChip->SetupStateTable(this->StateTable);

    mtsInterfaceProvided * dallasChipInterface = this->AddInterfaceProvided(dallasChip->Name());

    dallasChip->SetupProvidedInterface(dallasChipInterface, this->StateTable);
    return true;
}

void mtsRobotIO1394::Startup(void)
{
    // Use preferred watchdog timeout
    SetWatchdogPeriod(mWatchdogPeriod);
}

void mtsRobotIO1394::PreRead(void)
{
    mStateTableRead->Start();
    for (auto & robot : mRobots) {
        robot->StartReadStateTable();
    }
}

void mtsRobotIO1394::Read(void)
{
    // Read from all boards on the port
    mPort->ReadAllBoards();

    // Poll the state for each robot
    for (auto & robot : mRobots) {
        // Poll the board validity
        robot->PollValidity();

        // Poll this robot's state
        robot->PollState();

        // Convert bits to usable numbers
        robot->ConvertState();
    }
    // Poll the state for each digital input
    for (auto & input : mDigitalInputs) {
        input->PollState();
    }
    // Poll the state for each digital output
    for (auto & output : mDigitalOutputs) {
        output->PollState();
    }
    // Poll the state for each Dallas chip
    for (auto & dallas: mDallasChips) {
        dallas->PollState();
    }
}

void mtsRobotIO1394::PostRead(void)
{
    mStateTableRead->Advance();
    // Trigger robot events
    for (auto & robot : mRobots) {
        try {
            robot->CheckState();
        } catch (std::exception & stdException) {
            CMN_LOG_CLASS_RUN_ERROR << "PostRead: " << robot->Name() << ": standard exception \"" << stdException.what() << "\"" << std::endl;
            robot->mInterface->SendError("IO exception: " + robot->Name() + ", " + stdException.what());
        } catch (...) {
            CMN_LOG_CLASS_RUN_ERROR << "PostRead: " << robot->Name() << ": unknown exception" << std::endl;
            robot->mInterface->SendError("IO unknown exception: " + robot->Name());
        }
        robot->AdvanceReadStateTable();
    }
    // Trigger digital input events
    for (auto & input : mDigitalInputs) {
        input->CheckState();
    }
}

bool mtsRobotIO1394::IsOK(void) const
{
    return mPort->IsOK();
}

void mtsRobotIO1394::PreWrite(void)
{
    mStateTableWrite->Start();
    for (auto & robot : mRobots) {
        robot->StartWriteStateTable();
    }
}

void mtsRobotIO1394::Write(void)
{
    // Write to all boards
    mPort->WriteAllBoards();
}

void mtsRobotIO1394::PostWrite(void)
{
    mStateTableWrite->Advance();
    // Trigger robot events
    for (auto & robot : mRobots) {
        robot->AdvanceWriteStateTable();
    }
}

void mtsRobotIO1394::Run(void)
{
    // Read from all boards
    bool gotException = false;
    std::string message;

    PreRead();
    try {
        Read();
    } catch (sawRobotIO1394::osaRuntimeError1394 & sawException) {
        gotException = true;
        message = this->Name + ": sawRobotIO1394 exception \"" + sawException.what() + "\"";
    } catch (std::exception & stdException) {
        gotException = true;
        message = this->Name + ": standard exception \"" + stdException.what() + "\"";
    } catch (...) {
        gotException = true;
        message = this->Name + ": unknown exception";
    }
    if (gotException) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: port read, " << message << std::endl;
        // Trigger robot events
        for (auto & robot : mRobots) {
            robot->mInterface->SendError(message);
        }
    }
    PostRead(); // this performs all state conversions and checks

    // Invoke connected components (if any)
    this->RunEvent();

    // Process queued commands (e.g., to set motor current)
    this->ProcessQueuedCommands();

    // Write to all boards
    PreWrite();
    Write();
    PostWrite();
}

void mtsRobotIO1394::Cleanup(void)
{
    for (size_t i = 0; i < mRobots.size(); i++) {
        if (mRobots[i]->Valid()) {
            mRobots[i]->PowerOffSequence(true /* open safety relays */);
        }
    }
    // Write to all boards
    Write();
}

void mtsRobotIO1394::GetNumberOfDigitalInputs(int & placeHolder) const
{
    placeHolder = mDigitalInputs.size();
}

void mtsRobotIO1394::GetNumberOfDigitalOutputs(int & placeHolder) const
{
    placeHolder = mDigitalOutputs.size();
}

void mtsRobotIO1394::GetNumberOfBoards(int & placeHolder) const
{
    placeHolder = mBoards.size();
}

void mtsRobotIO1394::GetNumberOfRobots(int & placeHolder) const
{
    placeHolder = mRobots.size();
}

mtsRobot1394 * mtsRobotIO1394::Robot(const size_t index)
{
    return mRobots.at(index);
}

std::string mtsRobotIO1394::DefaultPort(void)
{
    return BasePort::DefaultPort();
}

const mtsRobot1394 * mtsRobotIO1394::Robot(const size_t index) const
{
    return mRobots.at(index);
}

void mtsRobotIO1394::GetNumberOfActuatorsPerRobot(vctIntVec & placeHolder) const
{
    size_t numRobots = mRobots.size();
    placeHolder.resize(numRobots);
    for (size_t i = 0; i < numRobots; i++) {
        placeHolder[i] = mRobots[i]->NumberOfActuators();
    }
}

void mtsRobotIO1394::GetNumberOfBrakesPerRobot(vctIntVec & placeHolder) const
{
    size_t numRobots = mRobots.size();
    placeHolder.resize(numRobots);
    for (size_t i = 0; i < numRobots; i++) {
        placeHolder[i] = mRobots[i]->NumberOfBrakes();
    }
}

void mtsRobotIO1394::AddRobot(mtsRobot1394 * robot)
{
    if (robot == 0) {
        cmnThrow(osaRuntimeError1394("mtsRobotIO1394::AddRobot: Robot pointer is null."));
    }

    const osaRobot1394Configuration & config = robot->GetConfiguration();

    // Check to make sure this robot isn't already added
    if (mRobotsByName.count(config.Name) > 0) {
        cmnThrow(osaRuntimeError1394(robot->Name() + ": robot name is not unique."));
    }

    // Construct a vector of boards relevant to this robot
    std::vector<osaActuatorMapping> actuatorBoards(config.NumberOfActuators);
    std::vector<osaBrakeMapping> brakeBoards(config.NumberOfBrakes);
    int currentBrake = 0;

    for (int i = 0; i < config.NumberOfActuators; i++) {

        // Board for the actuator
        int boardId = config.Actuators[i].BoardID;

        // If the board hasn't been created, construct it and add it to the port
        if (mBoards.count(boardId) == 0) {
            mBoards[boardId] = new AmpIO(boardId);
            mPort->AddBoard(mBoards[boardId]);
        }

        // Add the board to the list of boards relevant to this robot
        actuatorBoards[i].Board = mBoards[boardId];
        actuatorBoards[i].BoardID = boardId;
        actuatorBoards[i].Axis = config.Actuators[i].AxisID;

        // Board for the brake if any
        osaAnalogBrake1394Configuration * brake = config.Actuators[i].Brake;
        if (brake) {
            // Board for the brake
            boardId = brake->BoardID;

            // If the board hasn't been created, construct it and add it to the port
            if (mBoards.count(boardId) == 0) {
                mBoards[boardId] = new AmpIO(boardId);
                mPort->AddBoard(mBoards[boardId]);
            }

            // Add the board to the list of boards relevant to this robot
            brakeBoards[currentBrake].Board = mBoards[boardId];
            brakeBoards[currentBrake].BoardID = boardId;
            brakeBoards[currentBrake].Axis = brake->AxisID;
            currentBrake++;
        }
    }

    // Set the robot boards
    robot->SetBoards(actuatorBoards, brakeBoards);

    // Store the robot by name
    mRobots.push_back(robot);
    mRobotsByName[config.Name] = robot;
}

void mtsRobotIO1394::AddDigitalInput(mtsDigitalInput1394 * digitalInput)
{
    if (digitalInput == 0) {
        cmnThrow(osaRuntimeError1394("mtsRobotIO1394::AddDigitalInput: digital input pointer is null."));
    }

    const osaDigitalInput1394Configuration & config = digitalInput->Configuration();

    // Check to make sure this digital input isn't already added
    if (mDigitalInputsByName.count(config.Name) > 0) {
        cmnThrow(osaRuntimeError1394(digitalInput->Name() + ": digital input name is not unique."));
    }

    // Construct a vector of boards relevant to this digital input
    int boardID = config.BoardID;

    // If the board hasn't been created, construct it and add it to the port
    if (mBoards.count(boardID) == 0) {
        mBoards[boardID] = new AmpIO(boardID);
        mPort->AddBoard(mBoards[boardID]);
    }

    // Assign the board to the digital input
    digitalInput->SetBoard(mBoards[boardID]);

    // Store the digital input by name
    mDigitalInputs.push_back(digitalInput);
    mDigitalInputsByName[config.Name] = digitalInput;
}

void mtsRobotIO1394::AddDigitalOutput(mtsDigitalOutput1394 * digitalOutput)
{
    if (digitalOutput == 0) {
        cmnThrow(osaRuntimeError1394("mtsRobotIO1394::AddDigitalOutput: digital output pointer is null."));
    }

    const osaDigitalOutput1394Configuration & config = digitalOutput->Configuration();

    // Check to make sure this digital output isn't already added
    if (mDigitalOutputsByName.count(config.Name) > 0) {
        cmnThrow(osaRuntimeError1394(digitalOutput->Name() + ": digital output name is not unique."));
    }

    // Construct a vector of boards relevant to this digital output
    int boardID = config.BoardID;

    // If the board hasn't been created, construct it and add it to the port
    if (mBoards.count(boardID) == 0) {
        mBoards[boardID] = new AmpIO(boardID);
        mPort->AddBoard(mBoards[boardID]);
    }

    // Assign the board to the digital output
    digitalOutput->SetBoard(mBoards[boardID]);

    // Store the digital output by name
    mDigitalOutputs.push_back(digitalOutput);
    mDigitalOutputsByName[config.Name] = digitalOutput;
}

void mtsRobotIO1394::AddDallasChip(mtsDallasChip1394 * dallasChip)
{
    if (dallasChip == 0) {
        cmnThrow(osaRuntimeError1394("mtsRobotIO1394::AddDallasChip: Dallas chip pointer is null."));
    }

    const osaDallasChip1394Configuration & config = dallasChip->Configuration();

    // Check to make sure this Dallas chip isn't already added
    if (mDallasChipsByName.count(config.Name) > 0) {
        cmnThrow(osaRuntimeError1394(dallasChip->Name() + ": Dallas chip name is not unique."));
    }

    // Construct a vector of boards relevant to this Dallas chip
    int boardID = config.BoardID;

    // If the board hasn't been created, construct it and add it to the port
    if (mBoards.count(boardID) == 0) {
        mBoards[boardID] = new AmpIO(boardID);
        mPort->AddBoard(mBoards[boardID]);
    }

    // Assign the board to the Dallas chip
    dallasChip->SetBoard(mBoards[boardID]);

    // Store the digital output by name
    mDallasChips.push_back(dallasChip);
    mDallasChipsByName[config.Name] = dallasChip;
}

bool mtsRobotIO1394::CheckFirmwareVersions(void)
{
    unsigned int lowest = 99999;
    unsigned int highest = 0;
    for (const auto & robot : mRobots) {
        unsigned int robotLow, robotHigh;
        robot->GetFirmwareRange(robotLow, robotHigh);
        if (robotLow < lowest) {
            lowest = robotLow;
        }
        if (robotHigh > highest) {
            highest = robotHigh;
        }
    }

    const AmpIO_UInt32 currentFirmwareRevision = 7;
    const AmpIO_UInt32 lowestFirmwareSupported = 6;

    std::stringstream message;
    bool fatal = false;
    bool firmwareSuggested = false;
    // supported
    if ((lowest >= lowestFirmwareSupported)
        && (lowest < currentFirmwareRevision)) {
        message << "mtsRobot1394::SetBoards" << std::endl
                << "----------------------------------------------------" << std::endl
                << " Suggestion:" << std::endl
                << "   Please upgrade all boards firmware to version " << currentFirmwareRevision << "." << std::endl
                << "   Lowest version found is " << lowest << "." << std::endl
                << "----------------------------------------------------" << std::endl;
        firmwareSuggested = true;
    }
    // too low
    if (lowest < lowestFirmwareSupported) {
        message << "mtsRobot1394::SetBoards" << std::endl
                << "----------------------------------------------------" << std::endl
                << " Error:" << std::endl
                << "   Please upgrade all boards firmware to version " << currentFirmwareRevision << "." << std::endl
                << "   Lowest version found is " << lowest << "." << std::endl
                << "   This software supports firmware revision(s) " << lowestFirmwareSupported << " to " << currentFirmwareRevision << std::endl
                << "----------------------------------------------------" << std::endl;
        fatal = true;
    }
    // too high
    if (highest > currentFirmwareRevision) {
        message << "mtsRobot1394::SetBoards" << std::endl
                << "----------------------------------------------------" << std::endl
                << " Error:" << std::endl
                << "   Highest firmware version found is " << highest << "." << std::endl
                << "   The highest firmware revision supported by this software is " << currentFirmwareRevision << "." << std::endl
                << "   Please update this software or downgrade your firmware." << std::endl
                << "----------------------------------------------------" << std::endl;
        fatal = true;
    }
    // different
    if (highest != lowest) {
        message << "mtsRobot1394::SetBoards" << std::endl
                << "----------------------------------------------------" << std::endl
                << " Error:" << std::endl
                << "   Found different firmware versions," << std::endl
                << "   ranging from " << lowest << " to " << highest << "." << std::endl
                << "   Please try to use the same firmware on all boards." << std::endl
                << "----------------------------------------------------" << std::endl;
    }
    // message if needed
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
    return !fatal;
}

void mtsRobotIO1394::GetRobotNames(std::vector<std::string> & names) const
{
    names.clear();
    for (const auto & robot : mRobots) {
        names.push_back(robot->Name());
    }
}

void mtsRobotIO1394::GetDigitalInputNames(std::vector<std::string> & names) const
{
    names.clear();
    for (const auto & input : mDigitalInputs) {
        names.push_back(input->Name());
    }
}

void mtsRobotIO1394::GetDigitalOutputNames(std::vector<std::string> & names) const
{
    names.clear();
    for (const auto & output : mDigitalOutputs) {
        names.push_back(output->Name());
    }
}

void mtsRobotIO1394::IntervalStatisticsCallback(void)
{
    // if the data is recent, arbitrary 10 seconds, ignore stats
    const double now = StateTable.PeriodStats.Timestamp();
    if (now < 10.0 * cmn_s) {
        return;
    }

    // check if the average is somewhat close to the expected period
    // linux/mtsTask periodic always introduces a small time delay in
    // sleep.  Cause is not known so far
    const double expectedDelay = 0.06 * cmn_ms;
    const double expectedPeriod = GetPeriodicity() + expectedDelay;
    bool sendingMessage = false;
    bool error = false;
    std::stringstream message;
    message.precision(2);

    // check periodicity
    if (StateTable.PeriodStats.PeriodAvg() > sawRobotIO1394::TimingMaxRatio * expectedPeriod) {
        sendingMessage = true;
        error = true;
        message << "average period (" << cmnInternalTo_ms(StateTable.PeriodStats.PeriodAvg())
                << " ms) exceeded " << sawRobotIO1394::TimingMaxRatio << " time expected period ("
                << cmnInternalTo_ms(expectedPeriod) << " ms)";
    }

    // check load
    if (!error) {
        if (StateTable.PeriodStats.ComputeTimeAvg() > expectedPeriod) {
            if (now >= (mTimeLastTimingWarning + sawRobotIO1394::TimeBetweenTimingWarnings)) {
                sendingMessage = true;
                message << "average compute time (" << cmnInternalTo_ms(StateTable.PeriodStats.ComputeTimeAvg())
                        << " ms) exceeds expected period ("
                        << cmnInternalTo_ms(expectedPeriod) << " ms)";
                mTimeLastTimingWarning = now;
            }
        } else {
            // reset time so next time we hit a warning it displays immediately
            mTimeLastTimingWarning = 0.0;
        }
    }

    // send message as needed
    if (sendingMessage) {
        std::cerr << StateTable.PeriodStats << std::endl;
        std::string messageString = " IO: " + message.str();
        for (auto & robot : mRobots) {
            if (error) {
                robot->mInterface->SendError(robot->Name() + messageString);
            } else {
                robot->mInterface->SendWarning(robot->Name() + messageString);
            }
        }
    }
}
