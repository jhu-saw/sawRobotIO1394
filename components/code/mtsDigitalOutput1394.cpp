/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-11-06

  (C) Copyright 2014-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsStateTable.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawRobotIO1394/mtsDigitalOutput1394.h>

#include "AmpIO.h"

namespace sawRobotIO1394 {
    class mtsDigitalOutput1394Data {
    public:
        mtsDigitalOutput1394Data():
            DigitalOutputBits(0x0)
        {};
        AmpIO_UInt32 BitMask;       // BitMask for this output. From DigitalOutput Stream.
        AmpIO_UInt32 DigitalOutputBits; // BitMask for this output. From DigitalOutput Stream.
    };
}

using namespace sawRobotIO1394;

mtsDigitalOutput1394::mtsDigitalOutput1394(const cmnGenericObject & owner,
                                           const osaDigitalOutput1394Configuration & config):
    OwnerServices(owner.Services()),
    mData(0),
    mValue(false)
{
    mData = new mtsDigitalOutput1394Data;
    Configure(config);
}

mtsDigitalOutput1394::~mtsDigitalOutput1394()
{
    if (mData) {
        delete mData;
    }
}

void mtsDigitalOutput1394::SetupStateTable(mtsStateTable & stateTable)
{
    stateTable.AddData(mValue, mName + "Value");
}

void mtsDigitalOutput1394::SetupProvidedInterface(mtsInterfaceProvided * interfaceProvided, mtsStateTable & stateTable)
{
    interfaceProvided->AddCommandReadState(stateTable, this->mValue, "GetValue");
    interfaceProvided->AddCommandWrite(&mtsDigitalOutput1394::SetValue, this,
                                       "SetValue");
    if (mConfiguration.IsPWM) {
        interfaceProvided->AddCommandWrite(&mtsDigitalOutput1394::SetPWMDutyCycle, this,
                                           "SetPWMDutyCycle");
    }
}

void mtsDigitalOutput1394::CheckState(void)
{
    std::cerr << CMN_LOG_DETAILS
              << " --- nothing here?   Can we have outputs changed on us for no reason and we should emit event" << std::endl;
}

void mtsDigitalOutput1394::Configure(const osaDigitalOutput1394Configuration & config)
{
    // Store configuration
    mConfiguration = config;
    mName = config.Name;
    mBitID = config.BitID;
    mData->BitMask = 0x1 << mBitID;

    // Set the value
    mValue = false;
}

void mtsDigitalOutput1394::SetBoard(AmpIO * board)
{
    if (board == 0) {
        cmnThrow(osaRuntimeError1394(this->Name() + ": invalid board pointer."));
    }
    mBoard = board;
    mBoard->WriteDoutControl(mBitID,
                             mBoard->GetDoutCounts(mConfiguration.HighDuration),
                             mBoard->GetDoutCounts(mConfiguration.LowDuration));
}

void mtsDigitalOutput1394::PollState(void)
{
    // Get the new value
    mData->DigitalOutputBits = mBoard->GetDigitalOutput();

    // Use masked bit
    mValue = (mData->DigitalOutputBits & mData->BitMask);
}

const osaDigitalOutput1394Configuration & mtsDigitalOutput1394::Configuration(void) const
{
    return mConfiguration;
}

const std::string & mtsDigitalOutput1394::Name(void) const
{
    return mName;
}

const bool & mtsDigitalOutput1394::Value(void) const
{
    return mValue;
}

void mtsDigitalOutput1394::SetValue(const bool & newValue)
{
    // read from the boards
    mData->DigitalOutputBits = mBoard->GetDigitalOutput();
    if (newValue) {
        mData->DigitalOutputBits |= mData->BitMask;
    } else {
        mData->DigitalOutputBits &= ~(mData->BitMask);
    }
    mBoard->WriteDigitalOutput(0x0f, mData->DigitalOutputBits);
}

void mtsDigitalOutput1394::SetPWMDutyCycle(const double & dutyCycle)
{
    if ((dutyCycle > 0.0) && (dutyCycle < 1.0)) {
        mBoard->WritePWM(mBitID, mConfiguration.PWMFrequency, dutyCycle);
    } else {
        mBoard->WriteDoutControl(mBitID, 0, 0);
    }
}
