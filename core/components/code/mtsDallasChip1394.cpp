/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2019-05-23

  (C) Copyright 2019-2023 Johns Hopkins University (JHU), All Rights Reserved.

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

using namespace sawRobotIO1394;

mtsDallasChip1394::mtsDallasChip1394(const cmnGenericObject & owner):
    m_owner_services(owner.Services())
{
}

mtsDallasChip1394::~mtsDallasChip1394()
{
}

void mtsDallasChip1394::SetupStateTable(mtsStateTable & stateTable)
{
    stateTable.AddData(m_tool_type, "ToolType");
}

void mtsDallasChip1394::SetupProvidedInterface(mtsInterfaceProvided * interfaceProvided,
                                               mtsStateTable & stateTable)
{
    m_interface = interfaceProvided;
    m_interface->AddMessageEvents();
    m_interface->AddCommandReadState(stateTable, this->m_tool_type, "GetToolType");
    m_interface->AddCommandVoid(&mtsDallasChip1394::TriggerRead, this, "TriggerRead");
    m_interface->AddEventWrite(m_tool_type_event, "ToolType", ToolTypeUndefined);
}

void mtsDallasChip1394::CheckState(void)
{
    std::cerr << CMN_LOG_DETAILS
              << " --- nothing here?   Can we have outputs changed on us for no reason and we should emit event" << std::endl;
}

void mtsDallasChip1394::Configure(const osaDallasChip1394Configuration & config)
{
    // Store configuration
    m_configuration = config;
    m_tool_type = ToolTypeUndefined;
    m_waiting = false;
}

void mtsDallasChip1394::SetBoard(AmpIO * board)
{
    if (board == 0) {
        cmnThrow(this->Name() + ": invalid board pointer.");
    }
    m_board = board;
}

void mtsDallasChip1394::PollState(void)
{
    if (!m_waiting) {
        return;
    }
    TriggerRead();
}

const osaDallasChip1394Configuration & mtsDallasChip1394::Configuration(void) const
{
    return m_configuration;
}

const std::string & mtsDallasChip1394::Name(void) const
{
    return m_configuration.name;
}

const std::string & mtsDallasChip1394::ToolType(void) const
{
    return m_tool_type;
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
    m_tool_type = toolType.str();
    // send info
    m_interface->SendStatus(m_name + ": found tool type \"" + m_tool_type + "\"");
    m_tool_type_event(m_tool_type);
}

void mtsDallasChip1394::TriggerRead(void)
{
    uint32_t model;
    uint8_t version;
    std::string name;
    AmpIO::DallasStatus status
        = m_board->DallasReadTool(model, version, name);
    switch (status) {
    case AmpIO::DALLAS_OK:
        TriggerToolTypeEvent(model, version, name);
        m_waiting = false;
        break;
    case AmpIO::DALLAS_IO_ERROR:
        m_interface->SendError(m_name + ": tool read IO error");
        m_waiting = false;
        break;
    case AmpIO::DALLAS_WAIT:
        // check if we were not already waiting
        if (!m_waiting) {
            m_interface->SendStatus(m_name + ": requested tool read");
            m_waiting = true;
        }
        break;
    case AmpIO::DALLAS_NONE:
        m_interface->SendError(m_name + ": tool read not supported on this system (hardware or firmware too old)");
        m_waiting = false;
        break;
    case AmpIO::DALLAS_TIMEOUT:
        m_interface->SendError(m_name + ": tool read timeout");
        m_waiting = false;
        break;
    case AmpIO::DALLAS_DATA_ERROR:
        m_interface->SendError(m_name + ": tool read received some unexpected data");
        m_waiting = false;
        break;
    default:
        m_interface->SendError(m_name + ": tool read unknown error");
        m_waiting = false;
        break;
    }
}
