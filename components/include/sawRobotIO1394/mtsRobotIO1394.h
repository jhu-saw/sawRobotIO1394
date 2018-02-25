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

#ifndef _mtsRobotIO1394_h
#define _mtsRobotIO1394_h

#include <ostream>
#include <iostream>
#include <vector>

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <sawRobotIO1394/sawRobotIO1394ForwardDeclarations.h>
#include <sawRobotIO1394/sawRobotIO1394Export.h>

class CISST_EXPORT mtsRobotIO1394 : public mtsTaskPeriodic {

    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);
public:
    enum { MAX_BOARDS = 16 };

protected:

    std::ostream * MessageStream;             // Stream provided to the low level boards for messages, redirected to cmnLogger

            //! Board Objects
    BasePort * mPort;

    double mWatchdogPeriod; // prefered watchdog period for all boards

    std::map<int, AmpIO*> mBoards;
    typedef std::map<int, AmpIO*>::iterator board_iterator;
    typedef std::map<int, AmpIO*>::const_iterator board_const_iterator;
    
    typedef std::vector<sawRobotIO1394::mtsRobot1394*> RobotsType;
    typedef RobotsType::iterator robot_iterator;
    typedef RobotsType::const_iterator robot_const_iterator;
    RobotsType mRobots;
    std::map<std::string, sawRobotIO1394::mtsRobot1394*> mRobotsByName;

    typedef std::vector<sawRobotIO1394::mtsDigitalInput1394*> DigitalInputsType;
    typedef DigitalInputsType::iterator digital_input_iterator;
    typedef DigitalInputsType::const_iterator digital_input_const_iterator;
    DigitalInputsType mDigitalInputs;
    std::map<std::string, sawRobotIO1394::mtsDigitalInput1394*> mDigitalInputsByName;

    typedef std::vector<sawRobotIO1394::mtsDigitalOutput1394*> DigitalOutputsType;
    typedef DigitalOutputsType::iterator digital_output_iterator;
    typedef DigitalOutputsType::const_iterator digital_output_const_iterator;
    DigitalOutputsType mDigitalOutputs;
    std::map<std::string, sawRobotIO1394::mtsDigitalOutput1394*> mDigitalOutputsByName;

    // state tables for statistics
    mtsStateTable * mStateTableRead;
    mtsStateTable * mStateTableWrite;

    ///////////// Public Class Methods ///////////////////////////
public:
    // Constructor & Destructor
    mtsRobotIO1394(const std::string & name, const double periodInSeconds, const int portNumber);
    mtsRobotIO1394(const mtsTaskPeriodicConstructorArg & arg); // TODO: add port_num
    virtual ~mtsRobotIO1394();

    void SetProtocol(const sawRobotIO1394::ProtocolType & protocol);
    void SetWatchdogPeriod(const double & periodInSeconds);

    void Init(const int portNumber);

    void Configure(const std::string & filename);
    bool SetupRobot(sawRobotIO1394::mtsRobot1394 * robot);
    bool SetupDigitalInput(sawRobotIO1394::mtsDigitalInput1394 * digitalInput);
    bool SetupDigitalOutput(sawRobotIO1394::mtsDigitalOutput1394 * digitalOutput);
    void AddRobot(sawRobotIO1394::mtsRobot1394 * Robot);
    void AddDigitalInput(sawRobotIO1394::mtsDigitalInput1394 * digitalInput);
    void AddDigitalOutput(sawRobotIO1394::mtsDigitalOutput1394 * digitalInput);

    void Startup(void);
    void Run(void);
    void Cleanup(void);
    void GetNumberOfDigitalInputs(int & placeHolder) const;
    void GetNumberOfDigitalOutputs(int & placeHolder) const;

protected:
    void GetNumberOfBoards(int & placeHolder) const;
    void GetNumberOfRobots(int & placeHolder) const;
    void GetNumberOfActuatorsPerRobot(vctIntVec & placeHolder) const;
    void GetNumberOfBrakesPerRobot(vctIntVec & placeHolder) const;

    void GetRobotNames(std::vector<std::string> & names) const;
    void GetDigitalInputNames(std::vector<std::string> & names) const;
    void GetDigitalOutputNames(std::vector<std::string> & names) const;

    void PreRead(void);
    void Read(void);
    void PostRead(void);
    void PreWrite(void);
    void Write(void);
    void PostWrite(void);

    ////////////// Private Class Methods /////////////////////////////
private:
    // Make uncopyable
    mtsRobotIO1394(const mtsRobotIO1394 &);
    mtsRobotIO1394 & operator = (const mtsRobotIO1394 &);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRobotIO1394);

#endif // _mtsRobotIO1394_h


#if TO_BE_PORTED

         * robot control architecture. It is also responsible for some low-level error
         * handling.
         */

    public:

        osaPort1394(int portNumber, std::ostream & messageStream = std::cerr);
        ~osaPort1394();

        //! Configure the port using the specified configuration file. This method is not
        //  used by the mtsRobotIO1394 SAW component.
        void Configure(const osaPort1394Configuration & config);

        //! Robot Accessors
        osaRobot1394 * Robot(const std::string & name);
        const osaRobot1394 * Robot(const std::string & name) const;

        osaRobot1394 * Robot(const int index);
        const osaRobot1394 * Robot(const int index) const;



        int NumberOfBoards(void) const;
        int NumberOfRobots(void) const;
        int NumberOfDigitalInputs(void) const;
        int NumberOfDigitalOutputs(void) const;

    protected:


        std::map<int, AmpIO*> mBoards;
        typedef std::map<int, AmpIO*>::iterator board_iterator;
        typedef std::map<int, AmpIO*>::const_iterator board_const_iterator;

        //! Robot Objects
        std::vector<osaRobot1394*> mRobots;
        typedef std::vector<osaRobot1394*>::iterator robot_iterator;
        typedef std::vector<osaRobot1394*>::const_iterator robot_const_iterator;
        typedef std::map<std::string, osaRobot1394*>::iterator robotByName_iterator;
        typedef std::map<std::string, osaRobot1394*>::const_iterator robotByName_const_iterator;

        std::vector<osaDigitalInput1394*> mDigitalInputs;
        typedef std::vector<osaDigitalInput1394*>::iterator digital_input_iterator;
        typedef std::vector<osaDigitalInput1394*>::const_iterator digital_input_const_iterator;

        std::vector<osaDigitalOutput1394*> mDigitalOutputs;
        typedef std::vector<osaDigitalOutput1394*>::iterator digital_output_iterator;
        typedef std::vector<osaDigitalOutput1394*>::const_iterator digital_output_const_iterator;
    };

} // namespace sawRobotIO1394

#endif // TO_BE_PORTED
