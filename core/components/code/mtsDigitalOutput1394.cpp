/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-11-06

  (C) Copyright 2014-2025 Johns Hopkins University (JHU), All Rights Reserved.

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
            digital_output_bits(0x0)
        {};
        uint32_t bit_mask;            // BitMask for this output. From DigitalOutput Stream.
        uint32_t digital_output_bits; // BitMask for this output. From DigitalOutput Stream.
    };
}

using namespace sawRobotIO1394;

mtsDigitalOutput1394::mtsDigitalOutput1394(const cmnGenericObject & owner):
    m_owner_services(owner.Services()),
    m_data(0),
    m_value(false)
{
    m_data = new mtsDigitalOutput1394Data;
}


mtsDigitalOutput1394::~mtsDigitalOutput1394()
{
    if (m_data) {
        delete m_data;
    }
}


void mtsDigitalOutput1394::SetupStateTable(mtsStateTable & stateTable)
{
    stateTable.AddData(m_value, m_configuration.name + "Value");
}


void mtsDigitalOutput1394::SetupProvidedInterface(mtsInterfaceProvided * interfaceProvided, mtsStateTable & stateTable)
{
    interfaceProvided->AddCommandReadState(stateTable, this->m_value, "GetValue");
    interfaceProvided->AddCommandWrite(&mtsDigitalOutput1394::SetValue, this,
                                       "SetValue");
    if (m_configuration.is_PWM) {
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
    m_configuration = config;
    m_data->bit_mask = 0x1 << m_configuration.bit_id;
    // Set the value
    m_value = false;
}


void mtsDigitalOutput1394::SetBoard(AmpIO * board)
{
    if (board == 0) {
        cmnThrow(this->Name() + ": invalid board pointer.");
    }
    m_board = board;
    m_board->WriteDoutControl(m_configuration.bit_id,
                              m_board->GetDoutCounts(m_configuration.high_duration),
                              m_board->GetDoutCounts(m_configuration.low_duration));
}


void mtsDigitalOutput1394::PollState(void)
{
    // Get the new value
    m_data->digital_output_bits = m_board->GetDigitalOutput();

    // Use masked bit
    m_value = (m_data->digital_output_bits & m_data->bit_mask);
}


const osaDigitalOutput1394Configuration & mtsDigitalOutput1394::Configuration(void) const
{
    return m_configuration;
}


const std::string & mtsDigitalOutput1394::Name(void) const
{
    return m_configuration.name;
}


const bool & mtsDigitalOutput1394::Value(void) const
{
    return m_value;
}


void mtsDigitalOutput1394::SetValue(const bool & newValue)
{
    // read from the boards
    m_data->digital_output_bits = m_board->GetDigitalOutput();
    if (newValue) {
        m_data->digital_output_bits |= m_data->bit_mask;
    } else {
        m_data->digital_output_bits &= ~(m_data->bit_mask);
    }
    m_board->WriteDigitalOutput(0x0f, m_data->digital_output_bits);
}


void mtsDigitalOutput1394::SetPWMDutyCycle(const double & dutyCycle)
{
    if ((dutyCycle > 0.0) && (dutyCycle < 1.0)) {
        m_board->WritePWM(m_configuration.bit_id, m_configuration.PWM_frequency, dutyCycle);
    } else {
        m_board->WriteDoutControl(m_configuration.bit_id, 0, 0);
    }
}
