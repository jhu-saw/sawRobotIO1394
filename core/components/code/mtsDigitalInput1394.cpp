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

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsStateTable.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawRobotIO1394/mtsDigitalInput1394.h>

#include "AmpIO.h"

namespace sawRobotIO1394 {
    class mtsDigitalInput1394Data {
    public:
        mtsDigitalInput1394Data():
            digital_input_bits(0x0)
        {};
        uint32_t bit_mask;           // BitMask for this input. From DigitalInput Stream.
        uint32_t digital_input_bits; // BitMask for this input. From DigitalInput Stream.
    };
}

using namespace sawRobotIO1394;

mtsDigitalInput1394::mtsDigitalInput1394(const cmnGenericObject & owner,
                                         const osaDigitalInput1394Configuration & config):
    m_owner_services(owner.Services())
{
    m_data = new mtsDigitalInput1394Data;

    // predefined payloads
    m_event_payloads.pressed.SetType(prmEventButton::PRESSED);
    m_event_payloads.pressed.SetValid(true);
    m_event_payloads.released.SetType(prmEventButton::RELEASED);
    m_event_payloads.released.SetValid(true);
    m_event_payloads.clicked.SetType(prmEventButton::CLICKED);
    m_event_payloads.clicked.SetValid(true);
    Configure(config);
}

mtsDigitalInput1394::~mtsDigitalInput1394()
{
    if (m_data) {
        delete m_data;
    }
}

void mtsDigitalInput1394::SetupStateTable(mtsStateTable & stateTable)
{
    stateTable.AddData(m_value, m_configuration.name + "Value");
    m_state_table = &stateTable;
}

void mtsDigitalInput1394::SetupProvidedInterface(mtsInterfaceProvided * prov, mtsStateTable & stateTable)
{
    prov->AddCommandReadState(stateTable, this->m_value, "GetButton");
    prov->AddEventWrite(this->m_button, "Button", prmEventButton());
}

void mtsDigitalInput1394::CheckState(void)
{
    // Send appropriate events if the value changed in the last update
    // Check if value has changed
    if (m_first_run || (m_value != m_previous_value)) {
        // Check if the value is equal to the value when the digital input is considered pressed
        if (m_value) {
            // Emit a press event if specified in config
            if (m_configuration.trigger_when_pressed) {
                if (m_state_table) {
                    m_event_payloads.pressed.SetTimestamp(m_state_table->GetTic());
                }
                m_button(m_event_payloads.pressed);
            }
        } else {
            // Emit a release event if specified in config
            if (m_configuration.trigger_when_released) {
                if (m_state_table) {
                    m_event_payloads.released.SetTimestamp(m_state_table->GetTic());
                }
                m_button(m_event_payloads.released);
            }
        }
    }
    // Disable first run
    if (m_first_run) {
        m_first_run = false;
    }
}

void mtsDigitalInput1394::Configure(const osaDigitalInput1394Configuration & config)
{
    // Store configuration
    m_configuration = config;
    m_data->bit_mask = 0x1 << m_configuration.bit_id;
    m_first_run = !m_configuration.skip_first_run;
}

void mtsDigitalInput1394::SetBoard(AmpIO * board)
{
    if (board == 0) {
        cmnThrow(this->Name() + ": invalid board pointer.");
    }
    m_board = board;
}

void mtsDigitalInput1394::PollState(void)
{
    // Store previous value
    m_previous_value = m_value;

    // Get the new value
    m_data->digital_input_bits =  m_board->GetDigitalInput();

    // If the masked bit is low, set the value to the pressed value
    bool value = ((m_data->digital_input_bits & m_data->bit_mask)
                  ? (!m_configuration.pressed_value) : (m_configuration.pressed_value));

    // No debounce needed
    if (m_first_run || (m_configuration.debounce_threshold == 0.0)) {
        m_value = value;
        return;
    }

    // Debounce - start if we find one new different value
    if (m_debounce_counter < 0.0) {
        if (value != m_previous_value) {
            m_debounce_counter = 0.0;
            m_transition_value = value;
        }
    // count consecutive equal values
    } else {
        if (m_debounce_counter < m_configuration.debounce_threshold) {
            if (value == m_transition_value) {
                m_debounce_counter += m_board->GetTimestamp() * m_board->GetFPGAClockPeriod();
            } else {
                // click if button is now changed back and counter is short enough
                if ((m_configuration.debounce_threshold_click != m_configuration.debounce_threshold) // click is activated
                    && !value // input is "changed back"
                    && (m_debounce_counter >  m_configuration.debounce_threshold_click) // pressed long enough
                    ) {
                    if (m_state_table) {
                        m_event_payloads.clicked.SetTimestamp(m_state_table->GetTic());
                    }
                    m_button(m_event_payloads.clicked);
                }
                m_debounce_counter = -1.0;
            }
        } else {
            m_value = value;
            m_debounce_counter = -1.0;
        }
    }
}

const osaDigitalInput1394Configuration & mtsDigitalInput1394::Configuration(void) const
{
    return m_configuration;
}

const std::string & mtsDigitalInput1394::Name(void) const
{
    return m_configuration.name;
}

const bool & mtsDigitalInput1394::Value(void) const
{
    return m_value;
}

const bool & mtsDigitalInput1394::PreviousValue(void) const
{
    return m_previous_value;
}
