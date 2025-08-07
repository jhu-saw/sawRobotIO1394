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

#ifndef _mtsDallasChip1394_h
#define _mtsDallasChip1394_h

#include <cisstMultiTask/mtsForwardDeclarations.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <sawRobotIO1394/osaConfiguration1394.h>

#include <sawRobotIO1394/sawRobotIO1394ForwardDeclarations.h>

// Always include last
#include <sawRobotIO1394/sawRobotIO1394Export.h>

namespace sawRobotIO1394 {

    class CISST_EXPORT mtsDallasChip1394 {
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

        mtsDallasChip1394(const cmnGenericObject & owner,
                             const osaDallasChip1394Configuration & config);
        ~mtsDallasChip1394();

        const std::string ToolTypeUndefined = "UNDEFINED";
        const std::string ToolTypeError = "ERROR";

        void SetupStateTable(mtsStateTable & stateTable);
        void SetupProvidedInterface(mtsInterfaceProvided * interfaceProvided, mtsStateTable & stateTable);

        /*! Check state and trigger events as needed. */
        void CheckState(void);

        void Configure(const osaDallasChip1394Configuration & config);
        void SetBoard(AmpIO * board);

        void PollState(void);

        const osaDallasChip1394Configuration & Configuration(void) const;
        const std::string & Name(void) const;
        const std::string & ToolType(void) const;
        void TriggerRead(void);


    protected:
        void TriggerToolTypeEvent(const unsigned int & model,
                                  const unsigned int & version,
                                  const std::string & name);
        mtsInterfaceProvided * m_interface = nullptr;
        mtsFunctionWrite m_tool_type_event;
        AmpIO * m_board = nullptr;
        osaDallasChip1394Configuration m_configuration;
        std::string m_name;
        std::string m_tool_type = ToolTypeUndefined;
        bool m_waiting = false;
    };

} // namespace sawRobotIO1394

#endif // _mtsDallasChip1394_h
