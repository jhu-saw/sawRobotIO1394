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

#ifndef _mtsDallasChip1394_h
#define _mtsDallasChip1394_h

#include <cisstParameterTypes/prmEventButton.h>
#include <sawRobotIO1394/osaConfiguration1394.h>

#include <sawRobotIO1394/sawRobotIO1394ForwardDeclarations.h>

// Always include last
#include <sawRobotIO1394/sawRobotIO1394Export.h>

namespace sawRobotIO1394 {

    class mtsDallasChip1394Data;

    class CISST_EXPORT mtsDallasChip1394 {
    public:
        /*! Pointer on existing services.  This allows to use the class
          name and level of detail of another class, e.g. the class that
          owns this map.  To set the "Owner", use the method SetOwner
          after the cmnNamedMap is constructed. */
        const cmnClassServicesBase * OwnerServices;

        /*! Method used to emulate the cmnGenericObject interface used by
          CMN_LOG_CLASS macros. */
        //@{
        inline const cmnClassServicesBase * Services(void) const {
            return this->OwnerServices;
        }

        inline cmnLogger::StreamBufType * GetLogMultiplexer(void) const {
            return cmnLogger::GetMultiplexer();
        }
        //@}

        mtsDallasChip1394(const cmnGenericObject & owner,
                             const osaDallasChip1394Configuration & config);
        ~mtsDallasChip1394();

        const std::string UndefinedToolType = "UNDEFINED";

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
        mtsFunctionWrite ToolTypeEvent;    // The event function for button, will return prmEventButton
        AmpIO * mBoard;              // Board Assignment
        mtsDallasChip1394Data * mData; // Internal data using AmpIO types
        osaDallasChip1394Configuration mConfiguration;
        std::string mName;
        mtsStdString mToolType = UndefinedToolType;
        int mStatus = 0; // 0: nothing to do, 1: sent ctrl, waiting for status, 2: reading data
    };

} // namespace sawRobotIO1394

#endif // _mtsDallasChip1394_h
