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

#ifndef _mtsDigitalOutput1394_h
#define _mtsDigitalOutput1394_h

#include <cisstParameterTypes/prmEventButton.h>
#include <sawRobotIO1394/osaConfiguration1394.h>

#include <sawRobotIO1394/sawRobotIO1394ForwardDeclarations.h>

// Always include last
#include <sawRobotIO1394/sawRobotIO1394Export.h>

namespace sawRobotIO1394 {

    class mtsDigitalOutput1394Data;

    class CISST_EXPORT mtsDigitalOutput1394 {
    public:
        /*! Pointer on existing services.  This allows to use the class
          name and level of detail of another class, e.g. the class that
          owns this map.  To set the "Owner", use the method SetOwner
          after the cmnNamedMap is constructed. */
        const cmnClassServicesBase * m_owner_services;

        /*! Method used to emulate the cmnGenericObject interface used by
          CMN_LOG_CLASS macros. */
        //@{
        inline const cmnClassServicesBase * Services(void) const {
            return this->m_owner_services;
        }

        inline cmnLogger::StreamBufType * GetLogMultiplexer(void) const {
            return cmnLogger::GetMultiplexer();
        }
        //@}

        mtsDigitalOutput1394(const cmnGenericObject & owner);
        ~mtsDigitalOutput1394();

        void SetupStateTable(mtsStateTable & stateTable);
        void SetupProvidedInterface(mtsInterfaceProvided * interfaceProvided, mtsStateTable & stateTable);

        /*! Check state and trigger events as needed. */
        void CheckState(void);

        void Configure(const osaDigitalOutput1394Configuration & config);
        void SetBoard(AmpIO * board);

        void PollState(void);

        const osaDigitalOutput1394Configuration & Configuration(void) const;
        const std::string & Name(void) const;
        const bool & Value(void) const;
        void SetValue(const bool & newValue);
        void SetPWMDutyCycle(const double & dutyCycle);


    protected:
    // mtsFunctionWrite m_button;    // The event function for button, will return prmEventButton
        AmpIO * m_board;             // Board Assignment
        mtsDigitalOutput1394Data * m_data; // Internal data using AmpIO types
        osaDigitalOutput1394Configuration m_configuration;
        // State data
        bool m_value;                // Current read value
    };

} // namespace sawRobotIO1394

#endif // _mtsDigitalOutput1394_h
