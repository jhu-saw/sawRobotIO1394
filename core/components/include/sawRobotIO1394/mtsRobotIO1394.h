/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2011-06-10

  (C) Copyright 2011-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsRobotIO1394_h
#define _mtsRobotIO1394_h

#include <ostream>
#include <iostream>
#include <vector>

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <sawRobotIO1394/sawRobotIO1394ForwardDeclarations.h>
#include <sawRobotIO1394/osaConfiguration1394.h>

// Always include last!
#include <sawRobotIO1394/sawRobotIO1394Export.h>

class CISST_EXPORT mtsRobotIO1394 : public mtsTaskPeriodic {

    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);
public:
    enum { MAX_BOARDS = 16 };

protected:

    std::ostream * m_message_stream = nullptr; // Stream provided to the low level boards for messages, redirected to cmnLogger

    BasePort * m_port = nullptr;

    double m_watchdog_period = sawRobotIO1394::WatchdogTimeout; // prefered watchdog period for all boards
    bool m_skip_configuration_check = false;
    bool m_calibration_mode = false;

    std::map<int, AmpIO*> m_boards;
    typedef std::map<int, AmpIO*>::iterator board_iterator;
    typedef std::map<int, AmpIO*>::const_iterator board_const_iterator;

    std::vector<sawRobotIO1394::mtsRobot1394*> m_robots;
    std::map<std::string, sawRobotIO1394::mtsRobot1394*> m_robots_by_name;

    std::vector<sawRobotIO1394::mtsDigitalInput1394*> m_digital_inputs;
    std::map<std::string, sawRobotIO1394::mtsDigitalInput1394*> m_digital_inputs_by_name;

    std::vector<sawRobotIO1394::mtsDigitalOutput1394*> m_digital_outputs;
    std::map<std::string, sawRobotIO1394::mtsDigitalOutput1394*> m_digital_outputs_by_name;

    std::vector<sawRobotIO1394::mtsDallasChip1394*> m_dallas_chips;
    std::map<std::string, sawRobotIO1394::mtsDallasChip1394*> m_dallas_chips_by_name;

    // state tables for statistics
    mtsStateTable * m_state_table_read;
    mtsStateTable * m_state_table_write;

    ///////////// Public Class Methods ///////////////////////////
public:
    // Constructor & Destructor
    mtsRobotIO1394(const std::string & name, const double periodInSeconds, const std::string & port);
    mtsRobotIO1394(const mtsTaskPeriodicConstructorArg & arg); // TODO: add port_num
    virtual ~mtsRobotIO1394();

    void SetProtocol(const std::string & protocol);
    void SetWatchdogPeriod(const double & periodInSeconds);

    void Init(const std::string & port);

    void SkipConfigurationCheck(const bool skip); // must be called before Configure
    void SetCalibrationMode(const bool & mode); // must be called before Configure.  When calibrating, some values might be missing (e.g. lookup table to Si pots
    void Configure(const std::string & filename) override;
    bool SetupRobot(sawRobotIO1394::mtsRobot1394 * robot);
    bool SetupDigitalInput(sawRobotIO1394::mtsDigitalInput1394 * digitalInput);
    bool SetupDigitalOutput(sawRobotIO1394::mtsDigitalOutput1394 * digitalOutput);
    bool SetupDallasChip(sawRobotIO1394::mtsDallasChip1394 * dallasChip);
    void AddRobot(sawRobotIO1394::mtsRobot1394 * Robot);
    void AddDigitalInput(sawRobotIO1394::mtsDigitalInput1394 * digitalInput);
    void AddDigitalOutput(sawRobotIO1394::mtsDigitalOutput1394 * digitalInput);
    void AddDallasChip(sawRobotIO1394::mtsDallasChip1394 * dallasChip);

    bool CheckFirmwareVersions(void);

    void Startup(void) override;
    void Run(void) override;
    void Cleanup(void) override;
    void GetNumberOfDigitalInputs(size_t & placeHolder) const;
    sawRobotIO1394::mtsDigitalInput1394 * DigitalInput(const size_t index);
    const sawRobotIO1394::mtsDigitalInput1394* DigitalInput(const size_t index) const;
    void GetNumberOfDigitalOutputs(size_t & placeHolder) const;

    // public so these can be used outside cisstMultiTask
    bool IsOK(void) const;
    void Read(void);
    void Write(void);
    void GetNumberOfRobots(size_t & placeHolder) const;
    sawRobotIO1394::mtsRobot1394 * Robot(const size_t index);
    const sawRobotIO1394::mtsRobot1394 * Robot(const size_t index) const;

    static std::string DefaultPort(void);
    void close_all_relays(void);

protected:
    void GetNumberOfBoards(size_t & placeHolder) const;
    void GetNumberOfActuatorsPerRobot(vctIntVec & placeHolder) const;
    void GetNumberOfBrakesPerRobot(vctIntVec & placeHolder) const;

    mtsInterfaceProvided * mConfigurationInterface = nullptr;
    void GetRobotNames(std::vector<std::string> & names) const;
    void GetDigitalInputNames(std::vector<std::string> & names) const;
    void GetDigitalOutputNames(std::vector<std::string> & names) const;

    void PreRead(void);
    void PostRead(void);
    void PreWrite(void);
    void PostWrite(void);

    void IntervalStatisticsCallback(void);
private:
    double m_time_last_timing_warning = 0.0;

private:
    // Make uncopyable
    mtsRobotIO1394(const mtsRobotIO1394 &);
    mtsRobotIO1394 & operator = (const mtsRobotIO1394 &);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRobotIO1394);

#endif // _mtsRobotIO1394_h
