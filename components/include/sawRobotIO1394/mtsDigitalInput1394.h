/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Zihan Chen, Peter Kazanzides, Jonathan Bohren
  Created on: 2011-06-10

  (C) Copyright 2011-2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsDigitalInput1394_h
#define _mtsDigitalInput1394_h

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawRobotIO1394/osaDigitalInput1394.h>

namespace sawRobotIO1394 {

    class mtsDigitalInput1394: public osaDigitalInput1394 {
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

        mtsDigitalInput1394(const cmnGenericObject & owner,
                            const osaDigitalInput1394Configuration & config);

        void SetupStateTable(mtsStateTable & stateTable);
        void SetupProvidedInterface(mtsInterfaceProvided * interfaceProvided, mtsStateTable & stateTable);

        /*! Check state and trigger events as needed. */
        void CheckState(void);

    protected:
        mtsFunctionWrite Button;    // The event function for button, will return prmEventButton
    };

} // namespace sawRobotIO1394

#endif // _mtsDigitalInput1394_h
/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides, Jonathan Bohren
  Created on: 2011-06-10

  (C) Copyright 2011-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _osaDigitalInput1394_h
#define _osaDigitalInput1394_h

#include <sawRobotIO1394/osaConfiguration1394.h>
#include "AmpIO.h"
#include <sawRobotIO1394/sawRobotIO1394Export.h>

namespace sawRobotIO1394 {

    class CISST_EXPORT osaDigitalInput1394 {
    public:
        osaDigitalInput1394(const osaDigitalInput1394Configuration & config);

        void Configure(const osaDigitalInput1394Configuration & config);
        void SetBoard(AmpIO * board);

        void PollState(void);

        const osaDigitalInput1394Configuration & Configuration(void) const;

        const std::string & Name(void) const;
        const bool & Value(void) const;
        const bool & PreviousValue(void) const;

    protected:
        AmpIO * mBoard;              // Board Assignment

        osaDigitalInput1394Configuration mConfiguration;
        std::string mName;
        int mBitID;                  // Board assigned bitID for this Digital Input
        AmpIO_UInt32 mBitMask;       // BitMask for this input. From DigitalInput Stream.
        bool mPressedValue;          // Boolean Flag for Active High(true)/Active Low(false)
        bool mTriggerPress;          // Boolean Flag for Press Trigger Setting
        bool mTriggerRelease;        // Boolean Flag for Release Trigger Setting
        double mDebounceThreshold;   // 0, no debounce required otherwise time in seconds

        // ZC: hysteresis
        double mTestConfidence;
        double mTestLow;
        double mTestHigh;
        double mTestWeight;

        // State data
        AmpIO_UInt32 mDigitalInputBits; // BitMask for this input. From DigitalInput Stream.
        bool mValue;                    // Current read value
        bool mTransitionValue;          // For debouncing
        bool mPreviousValue;            // Saved value from the previous read
        double mDebounceCounter;        // time in seconds with constant value
    };

}

#endif // _osaDigitalInput1394_h
