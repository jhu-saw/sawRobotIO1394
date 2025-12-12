/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2012-07-31

  (C) Copyright 2011-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <iostream>
#include <fstream>

#include <cisstBuildType.h>
#include <cisstCommon/cmnLogger.h>
#include <cisstCommon/cmnUnits.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsDigitalInput1394.h>
#include <sawRobotIO1394/mtsDigitalOutput1394.h>
#include <sawRobotIO1394/mtsDallasChip1394.h>
#include <sawRobotIO1394/mtsRobot1394.h>

#include <Amp1394/AmpIORevision.h>
#include "PortFactory.h"
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
    for (auto & robot : m_robots) {
        if (robot != 0) {
            delete robot;
        }
    }
    m_robots.clear();
    m_robots_by_name.clear();

    // delete digital inputs before deleting boards
    for (auto & input : m_digital_inputs) {
        if (input != 0) {
            delete input;
        }
    }
    m_digital_inputs.clear();
    m_digital_inputs_by_name.clear();

    // delete digital outputs before deleting boards
    for (auto & output : m_digital_outputs) {
        if (output != 0) {
            delete output;
        }
    }
    m_digital_outputs.clear();
    m_digital_outputs_by_name.clear();

    // delete Dallas chips before deleting boards
    for (auto & dallas : m_dallas_chips) {
        if (dallas != 0) {
            delete dallas;
        }
    }
    m_dallas_chips.clear();
    m_dallas_chips_by_name.clear();

    // delete board structures
    for (board_iterator iter = m_boards.begin();
         iter != m_boards.end();
         ++iter) {
        if (iter->second != 0) {
            m_port->RemoveBoard(iter->first);
            delete iter->second;
        }
    }
    m_boards.clear();

    // delete firewire port
    if (m_port != 0) {
        delete m_port;
    }

    // delete message stream
    delete m_message_stream;
}

void mtsRobotIO1394::SetProtocol(const std::string & protocol)
{
    BasePort::ProtocolType protocolType;
    bool ok = false;
    if (BasePort::ParseProtocol(protocol.c_str(),
                                protocolType,
                                *m_message_stream)) {
        ok = m_port->SetProtocol(protocolType);
    }
    if (!ok) {
        CMN_LOG_CLASS_INIT_ERROR << "mtsRobot1394::SetProtocol failed" << std::endl;
        exit(EXIT_FAILURE);
    }
}

void mtsRobotIO1394::SetWatchdogPeriod(const double & periodInSeconds)
{
    m_watchdog_period = periodInSeconds;
    for (auto & robot : m_robots) {
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
    m_watchdog_period = sawRobotIO1394::WatchdogTimeout;
    m_skip_configuration_check = false;

    // add state tables for stats
    m_state_table_read = new mtsStateTable(100, this->GetName() + "Read");
    m_state_table_read->SetAutomaticAdvance(false);
    m_state_table_write = new mtsStateTable(100, this->GetName() + "Write");
    m_state_table_write->SetAutomaticAdvance(false);

    // create port
    m_message_stream = new std::ostream(this->GetLogMultiplexer());
    m_port = PortFactory(port.c_str(), *m_message_stream);
    if (!m_port) {
        CMN_LOG_CLASS_INIT_ERROR << "Init: unknown port type: " << port
                                 << ", port can be: " << std::endl
                                 << "  - a single number (implicitly a FireWire port)" << std::endl
                                 << "  - fw[:X] for a FireWire port" << std::endl
                                 << "  - udp[:xx.xx.xx.xx] for raw UDP (IP is optional)"
                                 << std::endl;
        exit(EXIT_FAILURE);
    }
    // test port
    if (!m_port->IsOK()) {
        CMN_LOG_CLASS_INIT_ERROR << "Init: failed to initialize " << m_port->GetPortTypeString() << std::endl;
        exit(EXIT_FAILURE);
    }
    // check number of port users
    if (m_port->NumberOfUsers() > 1) {
        CMN_LOG_CLASS_INIT_ERROR << "Init: found more than one user on firewire port: " << port << std::endl;;
        exit(EXIT_FAILURE);
    }

    // store if the port is simulated hardware
    m_is_hw_simulated = (m_port->GetPortType() == BasePort::PORT_SIMULATION);

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
    mConfigurationInterface = this->AddInterfaceProvided("Configuration");
    if (mConfigurationInterface) {
        mConfigurationInterface->AddMessageEvents();
        mConfigurationInterface->AddCommandRead(&mtsRobotIO1394::GetRobotNames, this,
                                                "GetRobotNames");
        mConfigurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfActuatorsPerRobot, this,
                                                "GetNumActuators");
        mConfigurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfBrakesPerRobot, this,
                                                "GetNumBrakes");
        mConfigurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfRobots, this,
                                                "GetNumRobots");
        mConfigurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfDigitalInputs, this,
                                                "GetNumDigitalInputs");
        mConfigurationInterface->AddCommandRead(&mtsRobotIO1394::GetDigitalInputNames, this,
                                                "GetDigitalInputNames");
        mConfigurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfDigitalOutputs, this,
                                                "GetNum_digital_outputs");
        mConfigurationInterface->AddCommandRead(&mtsRobotIO1394::GetDigitalOutputNames, this,
                                                "GetDigitalOutputNames");
        mConfigurationInterface->AddCommandRead<mtsComponent>(&mtsComponent::GetName, this,
                                                              "GetName");
        mConfigurationInterface->AddCommandVoid(&mtsRobotIO1394::close_all_relays, this,
                                                "close_all_relays");
        mConfigurationInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                                     "period_statistics");
        mConfigurationInterface->AddCommandReadState(*m_state_table_read, m_state_table_read->PeriodStats,
                                                     "period_statistics_read");
        mConfigurationInterface->AddCommandReadState(*m_state_table_write, m_state_table_write->PeriodStats,
                                                     "period_statistics_write");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to create configuration interface." << std::endl;
    }

    // Set callback for interval statistics
    StateTable.PeriodStats.SetCallback(&mtsRobotIO1394::IntervalStatisticsCallback, this);
}


void mtsRobotIO1394::SkipConfigurationCheck(const bool skip)
{
    m_skip_configuration_check = skip;
}


void mtsRobotIO1394::set_calibration_mode(const bool & mode)
{
    m_calibration_mode = mode;
    for (auto & robot : m_robots) {
        robot->set_calibration_mode(mode);
    }
}


void mtsRobotIO1394::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring from " << filename << std::endl;

    osaPort1394Configuration config;

    try {
        std::ifstream json_stream;
        Json::Value json_config;
        Json::Reader json_reader;

        json_stream.open(filename.c_str());
        if (!json_reader.parse(json_stream, json_config)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": failed to parse configuration file \""
                                     << filename << "\"\n"
                                     << json_reader.getFormattedErrorMessages();
            exit(EXIT_FAILURE);
        }

        // id & version check
        const std::string id = json_config["$id"].asString();
        const std::string id_expected = "saw-robot-io.schema.json";
        if (id != id_expected) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: file " << filename
                                     << " has incorrect or missing $id, found \"" << id
                                     << "\", expected \"" << id_expected << "\"" << std::endl;
            exit(EXIT_FAILURE);
        }
        const std::string version = json_config["$version"].asString();
        if (version != "6") {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: file " << filename
                                     << " has incorrect or missing $version, found \"" << version
                                     << "\", expected \"6\"" << std::endl;
            exit(EXIT_FAILURE);
        }

        // base component configuration
        mtsComponent::ConfigureJSON(json_config);

        // using cmnData traits
        cmnDataJSON<osaPort1394Configuration>::DeSerializeText(config, json_config);

        CMN_LOG_CLASS_INIT_VERBOSE << "Configure " << this->GetName()
                                   << ": content of configuration file" << std::endl
                                   << "------------ file ------------" << std::endl
                                   << config
                                   << "----------end of file --------" << std::endl;

    } catch (std::exception & e) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() << ": parsing file \""
                                 << filename << "\", got error: " << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }

    // Add all the robots
    for (const auto & configRobot : config.robots) {
        // Create a new robot
        mtsRobot1394 * robot = new mtsRobot1394(*this);
        robot->SetHwSimulation(m_is_hw_simulated);
        robot->set_calibration_mode(m_calibration_mode);
        robot->Configure(configRobot);
        // Check the configuration if needed
        if (!m_skip_configuration_check) {
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
    for (const auto & configInput : config.digital_inputs) {
        // Create a new digital input
        mtsDigitalInput1394 * digitalInput = new mtsDigitalInput1394(*this);
        digitalInput->Configure(configInput);
        // Set up the cisstMultiTask interfaces
        if (!this->SetupDigitalInput(digitalInput)) {
            delete digitalInput;
        } else {
            AddDigitalInput(digitalInput);
        }
    }

    // Add all the digital outputs
    for (const auto & configOutput : config.digital_outputs) {
        // Create a new digital input
        mtsDigitalOutput1394 * digitalOutput = new mtsDigitalOutput1394(*this);
        digitalOutput->Configure(configOutput);
        // Set up the cisstMultiTask interfaces
        if (!this->SetupDigitalOutput(digitalOutput)) {
            delete digitalOutput;
        } else {
            AddDigitalOutput(digitalOutput);
        }
    }

    // Add all the Dallas chips
    for (const auto & configDallas : config.dallas_chips) {
        // Create a new Dallas chip
        mtsDallasChip1394 * dallasChip = new mtsDallasChip1394(*this);
        dallasChip->Configure(configDallas);
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

    // Read all the boards, a easy solution to the issue that specific board cannot be read when using boardcast-read-write
    m_port->ReadAllBoards();
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

    // Setup the MTS interfaces
    robot->SetupInterfaces(robotInterface);

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
    SetWatchdogPeriod(m_watchdog_period);

    // Robot Startup
    for (auto & robot : m_robots) {
        robot->Startup();
    }
}

void mtsRobotIO1394::PreRead(void)
{
    m_state_table_read->Start();
    for (auto & robot : m_robots) {
        robot->StartReadStateTable();
    }
}

void mtsRobotIO1394::Read(void)
{
    // Read from all boards on the port
    m_port->ReadAllBoards();

    // Poll the state for each robot
    for (auto & robot : m_robots) {
        // Poll the board validity
        robot->PollValidity();

        // Poll this robot's state
        robot->PollState();

        // Convert bits to usable numbers
        robot->ConvertState();
    }
    // Poll the state for each digital input
    for (auto & input : m_digital_inputs) {
        input->PollState();
    }
    // Poll the state for each digital output
    for (auto & output : m_digital_outputs) {
        output->PollState();
    }
    // Poll the state for each Dallas chip
    for (auto & dallas: m_dallas_chips) {
        dallas->PollState();
    }
}

void mtsRobotIO1394::PostRead(void)
{
    m_state_table_read->Advance();
    // Trigger robot events
    for (auto & robot : m_robots) {
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
    for (auto & input : m_digital_inputs) {
        input->CheckState();
    }
}

bool mtsRobotIO1394::IsOK(void) const
{
    return m_port->IsOK();
}

void mtsRobotIO1394::PreWrite(void)
{
    m_state_table_write->Start();
    for (auto & robot : m_robots) {
        robot->StartWriteStateTable();
    }
}

void mtsRobotIO1394::Write(void)
{
    // Write to all boards
    m_port->WriteAllBoards();
}

void mtsRobotIO1394::PostWrite(void)
{
    m_state_table_write->Advance();
    // Trigger robot events
    for (auto & robot : m_robots) {
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
        for (auto & robot : m_robots) {
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
    for (size_t i = 0; i < m_robots.size(); i++) {
        if (m_robots[i]->Valid()) {
            m_robots[i]->PowerOffSequence(true /* open safety relays */);
        }
    }
    // Write to all boards
    Write();
}

void mtsRobotIO1394::GetNumberOfDigitalInputs(size_t & placeHolder) const
{
    placeHolder = m_digital_inputs.size();
}

mtsDigitalInput1394 * mtsRobotIO1394::DigitalInput(const size_t index)
{
    return m_digital_inputs.at(index);
}

const mtsDigitalInput1394 * mtsRobotIO1394::DigitalInput(const size_t index) const
{
    return m_digital_inputs.at(index);
}


void mtsRobotIO1394::GetNumberOfDigitalOutputs(size_t & placeHolder) const
{
    placeHolder = m_digital_outputs.size();
}


void mtsRobotIO1394::GetNumberOfBoards(size_t & placeHolder) const
{
    placeHolder = m_boards.size();
}


void mtsRobotIO1394::GetNumberOfRobots(size_t & placeHolder) const
{
    placeHolder = m_robots.size();
}


mtsRobot1394 * mtsRobotIO1394::Robot(const size_t index)
{
    return m_robots.at(index);
}


const mtsRobot1394 * mtsRobotIO1394::Robot(const size_t index) const
{
    return m_robots.at(index);
}


std::string mtsRobotIO1394::DefaultPort(void)
{
    return BasePort::DefaultPort();
}


void mtsRobotIO1394::GetNumberOfActuatorsPerRobot(vctIntVec & placeHolder) const
{
    const size_t _num_robots = m_robots.size();
    placeHolder.resize(_num_robots);
    for (size_t i = 0; i < _num_robots; i++) {
        placeHolder[i] = m_robots[i]->NumberOfActuators();
    }
}


void mtsRobotIO1394::GetNumberOfBrakesPerRobot(vctIntVec & placeHolder) const
{
    const size_t _num_robots = m_robots.size();
    placeHolder.resize(_num_robots);
    for (size_t i = 0; i < _num_robots; i++) {
        placeHolder[i] = m_robots[i]->NumberOfBrakes();
    }
}


void mtsRobotIO1394::AddRobot(mtsRobot1394 * robot)
{
    if (robot == 0) {
        cmnThrow("mtsRobotIO1394::AddRobot: Robot pointer is null.");
    }

    const osaRobot1394Configuration & config = robot->GetConfiguration();

    // Check to make sure this robot isn't already added
    if (m_robots_by_name.count(config.name) > 0) {
        cmnThrow(robot->Name() + ": robot name is not unique.");
    }

    // Construct a vector of boards relevant to this robot
    std::vector<osaActuatorMapping> actuatorBoards(config.number_of_actuators);
    std::vector<osaBrakeMapping> brakeBoards(config.number_of_brakes);

    for (size_t i = 0; i < config.number_of_actuators; i++) {
        int _board_id = config.actuators[i].board_id;
        // If the board hasn't been created, construct it and add it to the port
        if (m_boards.count(_board_id) == 0) {
            m_boards[_board_id] = new AmpIO(_board_id);
            m_port->AddBoard(m_boards[_board_id]);
        }
        // Add the board to the list of boards relevant to this robot
        actuatorBoards[i].board = m_boards[_board_id];
        actuatorBoards[i].board_id = _board_id;
        actuatorBoards[i].axis = config.actuators[i].axis_id;
    }

    for (size_t i = 0; i < config.number_of_brakes; i++) {
        int _board_id = config.brakes[i].board_id;
        // If the board hasn't been created, construct it and add it to the port
        if (m_boards.count(_board_id) == 0) {
            m_boards[_board_id] = new AmpIO(_board_id);
            m_port->AddBoard(m_boards[_board_id]);
        }
        // Add the board to the list of boards relevant to this robot
        brakeBoards[i].board = m_boards[_board_id];
        brakeBoards[i].board_id = _board_id;
        brakeBoards[i].axis = config.brakes[i].axis_id;
    }

    // Set the robot boards
    robot->SetBoards(actuatorBoards, brakeBoards);

    // Store the robot by name
    m_robots.push_back(robot);
    m_robots_by_name[config.name] = robot;
}


void mtsRobotIO1394::AddDigitalInput(mtsDigitalInput1394 * digitalInput)
{
    if (digitalInput == 0) {
        cmnThrow("mtsRobotIO1394::AddDigitalInput: digital input pointer is null.");
    }

    const osaDigitalInput1394Configuration & config = digitalInput->Configuration();

    // Check to make sure this digital input isn't already added
    if (m_digital_inputs_by_name.count(config.name) > 0) {
        cmnThrow(digitalInput->Name() + ": digital input name is not unique.");
    }

    // Construct a vector of boards relevant to this digital input
    int board_id = config.board_id;

    // If the board hasn't been created, construct it and add it to the port
    if (m_boards.count(board_id) == 0) {
        m_boards[board_id] = new AmpIO(board_id);
        m_port->AddBoard(m_boards[board_id]);
    }

    // Assign the board to the digital input
    digitalInput->SetBoard(m_boards[board_id]);

    // Store the digital input by name
    m_digital_inputs.push_back(digitalInput);
    m_digital_inputs_by_name[config.name] = digitalInput;
}

void mtsRobotIO1394::AddDigitalOutput(mtsDigitalOutput1394 * digitalOutput)
{
    if (digitalOutput == 0) {
        cmnThrow("mtsRobotIO1394::AddDigitalOutput: digital output pointer is null.");
    }

    const osaDigitalOutput1394Configuration & config = digitalOutput->Configuration();

    // Check to make sure this digital output isn't already added
    if (m_digital_outputs_by_name.count(config.name) > 0) {
        cmnThrow(digitalOutput->Name() + ": digital output name is not unique.");
    }

    // Construct a vector of boards relevant to this digital output
    int board_id = config.board_id;

    // If the board hasn't been created, construct it and add it to the port
    if (m_boards.count(board_id) == 0) {
        m_boards[board_id] = new AmpIO(board_id);
        m_port->AddBoard(m_boards[board_id]);
    }

    // Assign the board to the digital output
    digitalOutput->SetBoard(m_boards[board_id]);

    // Store the digital output by name
    m_digital_outputs.push_back(digitalOutput);
    m_digital_outputs_by_name[config.name] = digitalOutput;
}

void mtsRobotIO1394::AddDallasChip(mtsDallasChip1394 * dallasChip)
{
    if (dallasChip == 0) {
        cmnThrow("mtsRobotIO1394::AddDallasChip: Dallas chip pointer is null.");
    }

    const osaDallasChip1394Configuration & config = dallasChip->Configuration();

    // Check to make sure this Dallas chip isn't already added
    if (m_dallas_chips_by_name.count(config.name) > 0) {
        cmnThrow(dallasChip->Name() + ": Dallas chip name is not unique.");
    }

    // Construct a vector of boards relevant to this Dallas chip
    int board_id = config.board_id;

    // If the board hasn't been created, construct it and add it to the port
    if (m_boards.count(board_id) == 0) {
        m_boards[board_id] = new AmpIO(board_id);
        m_port->AddBoard(m_boards[board_id]);
    }

    // Assign the board to the Dallas chip
    dallasChip->SetBoard(m_boards[board_id]);

    // Store the digital output by name
    m_dallas_chips.push_back(dallasChip);
    m_dallas_chips_by_name[config.name] = dallasChip;
}

bool mtsRobotIO1394::CheckFirmwareVersions(void)
{
    unsigned int lowest = 99999;
    unsigned int highest = 0;
    for (const auto & robot : m_robots) {
        unsigned int robotLow, robotHigh;
        robot->GetFirmwareRange(robotLow, robotHigh);
        if (robotLow < lowest) {
            lowest = robotLow;
        }
        if (robotHigh > highest) {
            highest = robotHigh;
        }
    }

    const uint32_t currentFirmwareRevision = 9;
    const uint32_t lowestFirmwareSupported = 6;

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
    for (const auto & robot : m_robots) {
        names.push_back(robot->Name());
    }
}

void mtsRobotIO1394::GetDigitalInputNames(std::vector<std::string> & names) const
{
    names.clear();
    for (const auto & input : m_digital_inputs) {
        names.push_back(input->Name());
    }
}

void mtsRobotIO1394::GetDigitalOutputNames(std::vector<std::string> & names) const
{
    names.clear();
    for (const auto & output : m_digital_outputs) {
        names.push_back(output->Name());
    }
}

void mtsRobotIO1394::close_all_relays(void)
{
    if (m_port) {
        AmpIO::WriteSafetyRelayAll(m_port, true);
        mConfigurationInterface->SendStatus("Closed all safety relays");
    } else {
        mConfigurationInterface->SendError("Failed to close all safety relays, port has not been created yet");
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
            if (now >= (m_time_last_timing_warning + sawRobotIO1394::TimeBetweenTimingWarnings)) {
                sendingMessage = true;
                message << "average compute time (" << cmnInternalTo_ms(StateTable.PeriodStats.ComputeTimeAvg())
                        << " ms) exceeds expected period ("
                        << cmnInternalTo_ms(expectedPeriod) << " ms)";
                m_time_last_timing_warning = now;
            }
        } else {
            // reset time so next time we hit a warning it displays immediately
            m_time_last_timing_warning = 0.0;
        }
    }

    // send message as needed
    if (sendingMessage) {
        std::string messageString = " IO: " + message.str();
        for (auto & robot : m_robots) {
            if (error) {
                robot->mInterface->SendError(robot->Name() + messageString);
            } else {
                robot->mInterface->SendWarning(robot->Name() + messageString);
            }
        }
    }
}
