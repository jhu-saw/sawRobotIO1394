/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2019-05-23

  (C) Copyright 2019-2022 Johns Hopkins University (JHU), All Rights Reserved.

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

#include "BasePort.h"
#include "AmpIO.h"

#define DALLAS_START_READ     0x80
#define DALLAS_MODEL_OFFSET   0xa4
#define DALLAS_VERSION_OFFSET 0xa8
#define DALLAS_NAME_OFFSET    0x160
#define DALLAS_NAME_END       0x17c

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
    mWaiting = false;
}

void mtsDallasChip1394::SetBoard(AmpIO * board)
{
    if (board == 0) {
        cmnThrow(this->Name() + ": invalid board pointer.");
    }
    mBoard = board;
}

void mtsDallasChip1394::PollState(void)
{
    if (!mWaiting) {
        return;
    }
    TriggerRead();
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

void mtsDallasChip1394::TriggerToolTypeEvent(const unsigned int & model,
                                             const unsigned int & version,
                                             const std::string & name)
{
    std::string sanitizedName = name;
    // replace spaces with "_" and use upper case (see mtsIntuitiveResearchKitToolTypes.cdg)
    std::replace(sanitizedName.begin(), sanitizedName.end(), ' ', '_');
    std::transform(sanitizedName.begin(), sanitizedName.end(), sanitizedName.begin(), ::toupper);
    // concatenate name, model and version
    std::stringstream toolType;
    toolType << sanitizedName
             << ":" << model
             << "[" << version << "]";
    mToolType.Data = toolType.str();
    // send info
    mInterface->SendStatus(mName + ": found tool type \"" + mToolType.Data + "\"");
    ToolTypeEvent(mToolType);
}

void mtsDallasChip1394::TriggerRead(void)
{
    AmpIO_UInt32 model;
    AmpIO_UInt8 version;
    std::string name;
    AmpIO::DallasStatus status
        = mBoard->DallasReadTool(model, version, name);
    switch (status) {
    case AmpIO::DALLAS_OK:
        TriggerToolTypeEvent(model, version, name);
        mWaiting = false;
        break;
    case AmpIO::DALLAS_IO_ERROR:
        mInterface->SendError(mName + ": tool read IO error");
        mWaiting = false;
        break;
    case AmpIO::DALLAS_WAIT:
        // check if we were not already waiting
        if (!mWaiting) {
            mInterface->SendStatus(mName + ": requested tool read");
            mWaiting = true;
        }
        break;
    case AmpIO::DALLAS_NONE:
        mInterface->SendError(mName + ": tool read not supported on this system (hardware or firmware too old)");
        mWaiting = false;
        break;
    case AmpIO::DALLAS_TIMEOUT:
        mInterface->SendError(mName + ": tool read timeout");
        mWaiting = false;
        break;
    case AmpIO::DALLAS_DATA_ERROR:
        mInterface->SendError(mName + ": tool read received some unexpected data");
        mWaiting = false;
        break;
    default:
        mInterface->SendError(mName + ": tool read unknown error");
        mWaiting = false;
        break;
    }
}
