/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2019-05-23

  (C) Copyright 2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsStateTable.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawRobotIO1394/mtsDallasChip1394.h>

#include "AmpIO.h"

namespace sawRobotIO1394 {
    class mtsDallasChip1394Data {
    public:
        mtsDallasChip1394Data():
            DigitalOutputBits(0x0)
        {};
        AmpIO_UInt32 BitMask;       // BitMask for this output. From DigitalOutput Stream.
        AmpIO_UInt32 DigitalOutputBits; // BitMask for this output. From DigitalOutput Stream.
    };
}

using namespace sawRobotIO1394;

mtsDallasChip1394::mtsDallasChip1394(const cmnGenericObject & owner,
                                           const osaDallasChip1394Configuration & config):
    OwnerServices(owner.Services()),
    mData(0)
{
    mData = new mtsDallasChip1394Data;
    Configure(config);
}

mtsDallasChip1394::~mtsDallasChip1394()
{
    if (mData) {
        delete mData;
    }
}

void mtsDallasChip1394::SetupStateTable(mtsStateTable & stateTable)
{
    stateTable.AddData(mToolType, "ToolType");
}

void mtsDallasChip1394::SetupProvidedInterface(mtsInterfaceProvided * interfaceProvided, mtsStateTable & stateTable)
{
    interfaceProvided->AddCommandReadState(stateTable, this->mToolType, "GetToolType");
    interfaceProvided->AddCommandVoid(&mtsDallasChip1394::TriggerRead, this, "TriggerRead");
}

void mtsDallasChip1394::CheckState(void)
{
    std::cerr << CMN_LOG_DETAILS
              << " --- nothing here?   Can we have outputs changed on us for no reason and we should emit event" << std::endl;
}

void mtsDallasChip1394::Configure(const osaDallasChip1394Configuration & config)
{
    // Store configuration
    mConfiguration = config;
    mName = config.Name;
    mToolType = UndefinedToolType;
}

void mtsDallasChip1394::SetBoard(AmpIO * board)
{
    if (board == 0) {
        cmnThrow(osaRuntimeError1394(this->Name() + ": invalid board pointer."));
    }
    mBoard = board;
#if 0
    mBoard->WriteDoutControl(mBitID,
                             mBoard->GetDoutCounts(mConfiguration.HighDuration),
                             mBoard->GetDoutCounts(mConfiguration.LowDuration));
#endif
}

void mtsDallasChip1394::PollState(void)
{
#if 0
    // Get the new value
    mData->DallasChipBits = mBoard->GetDallasChip();

    // Use masked bit
    mValue = (mData->DallasChipBits & mData->BitMask);
#endif
}

const osaDallasChip1394Configuration & mtsDallasChip1394::Configuration(void) const
{
    return mConfiguration;
}

const std::string & mtsDallasChip1394::Name(void) const
{
    return mName;
}

const std::string & mtsDallasChip1394::ToolType(void) const
{
    return mToolType;
}

void mtsDallasChip1394::TriggerRead(void)
{
#if 0
    // read from the boards
    mData->DallasChipBits = mBoard->GetDallasChip();
    if (newValue) {
        mData->DallasChipBits |= mData->BitMask;
    } else {
        mData->DallasChipBits &= ~(mData->BitMask);
    }
    mBoard->WriteDallasChip(0x0f, mData->DallasChipBits);
#endif
}
