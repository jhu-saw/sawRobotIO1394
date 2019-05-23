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
    interfaceProvided->AddEventWrite(ToolTypeEvent, "ToolType", UndefinedToolType);
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
    mStatus = 0; // nothing happened so far
}

void mtsDallasChip1394::SetBoard(AmpIO * board)
{
    if (board == 0) {
        cmnThrow(osaRuntimeError1394(this->Name() + ": invalid board pointer."));
    }
    mBoard = board;
}

void mtsDallasChip1394::PollState(void)
{
    if (mStatus > 0) {
        if (mStatus == 1) {
            AmpIO_UInt32 status;
            if (!mBoard->DallasReadStatus(status)) {
                std::cerr << CMN_LOG_DETAILS << "DallasReadStatus failed" << std::endl;
                mStatus = 0;
                return;
            } else {
                if ((status&0x000000F0) == 0) {
                    std::cerr << CMN_LOG_DETAILS << " ------- debug, ready to read!" << std::endl;
                    mStatus = 2;
                }
            }
        } else if (mStatus == 2) {

        }
    }
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
    std::cerr << CMN_LOG_DETAILS << " ---- read started " << std::endl;

    if (mStatus > 0) {
        std::cerr << CMN_LOG_DETAILS << " read is already in progress, ignoring" << std::endl;
        return;
    }

    AmpIO_UInt32 fver = mBoard->GetFirmwareVersion();
    if (fver < 7) {
        std::cerr << CMN_LOG_DETAILS << "Instrument read requires firmware version 7+ (detected version " << fver << ")" << std::endl;
        return;
    }
    AmpIO_UInt32 status = mBoard->ReadStatus();
    // Check whether bi-directional I/O is available
    if ((status & 0x00300000) != 0x00300000) {
        std::cerr << CMN_LOG_DETAILS << "QLA does not support bidirectional I/O (QLA Rev 1.4+ required)" << std::endl;
        return;
    }

    // Address to read tool info
    unsigned short address = 0x160; // offset in Dallas chip with tool type string
    if (!(mBoard->DallasWriteControl( (address<<16)|2 ))) {
        std::cerr << CMN_LOG_DETAILS << "DallasWriteControl failed" << std::endl;
        return;
    }

    mStatus = 1;
}
