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

using namespace sawRobotIO1394;

mtsDallasChip1394::mtsDallasChip1394(const cmnGenericObject & owner,
                                     const osaDallasChip1394Configuration & config):
    OwnerServices(owner.Services())
{
    Configure(config);
}

mtsDallasChip1394::~mtsDallasChip1394()
{
}

void mtsDallasChip1394::SetupStateTable(mtsStateTable & stateTable)
{
    stateTable.AddData(mToolType, "ToolType");
}

void mtsDallasChip1394::SetupProvidedInterface(mtsInterfaceProvided * interfaceProvided,
                                               mtsStateTable & stateTable)
{
    mInterface = interfaceProvided;
    mInterface->AddMessageEvents();
    mInterface->AddCommandReadState(stateTable, this->mToolType, "GetToolType");
    mInterface->AddCommandVoid(&mtsDallasChip1394::TriggerRead, this, "TriggerRead");
    mInterface->AddEventWrite(ToolTypeEvent, "ToolType", ToolTypeUndefined);
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
    mToolType = ToolTypeUndefined;
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
                mInterface->SendWarning(mName + ": DallasReadStatus failed");
                ToolTypeEvent(ToolTypeError);
                mStatus = 0;
                return;
            } else {
                if ((status&0x000000F0) == 0) {
                    // Check family_code, dout_cfg_bidir, ds_reset, and ds_enable
                    if ((status & 0xFF00000F) != 0x0B00000B) {
                        mInterface->SendWarning(mName + ": check family_code, dout_cfg_bidir, ds_reset and/or ds_enable failed");
                        ToolTypeEvent(ToolTypeError);
                        mStatus = 0;
                        return;
                    }
                    mInterface->SendStatus(mName + ": reading tool info");
                    mStatus = 2;
                }
            }
        } else if (mStatus == 2) {
            nodeaddr_t address = 0x6000;
            char buffer[256];
            // Read first block of data (up to 256 bytes)
            if (!mBoard->ReadBlock(address, reinterpret_cast<quadlet_t *>(buffer), 256)) {
                mInterface->SendWarning(mName + "ReadBlock failed");
                ToolTypeEvent(ToolTypeError);
                mStatus = 0;
                return;
            } else {
                mToolType.Data = std::string(buffer);
                mInterface->SendStatus(mName + ": found tool type \"" + mToolType.Data + "\"");
                ToolTypeEvent(mToolType);
                mStatus = 0;
            }
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
    if (mStatus > 0) {
        mInterface->SendWarning(mName + ": tool info read is already in progress, ignoring");
        ToolTypeEvent(ToolTypeError);
        return;
    }

    AmpIO_UInt32 fver = mBoard->GetFirmwareVersion();
    if (fver < 7) {
        mInterface->SendWarning(mName + ": tool info read requires firmware version 7 or greater");
        ToolTypeEvent(ToolTypeError);
        return;
    }
    AmpIO_UInt32 status = mBoard->ReadStatus();
    // Check whether bi-directional I/O is available
    if ((status & 0x00300000) != 0x00300000) {
        mInterface->SendWarning(mName + ": QLA does not support bidirectional I/O (QLA Rev 1.4+ required)");
        ToolTypeEvent(ToolTypeError);
        return;
    }

    // Address to read tool info
    unsigned short address = 0x160; // offset in Dallas chip with tool type string
    if (!(mBoard->DallasWriteControl( (address<<16)|2 ))) {
        mInterface->SendWarning(mName + ": DallasWriteControl failed");
        ToolTypeEvent(ToolTypeError);
        return;
    }

    mStatus = 1;
}
