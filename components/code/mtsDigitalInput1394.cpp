/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2011-06-10

  (C) Copyright 2011-2023 Johns Hopkins University (JHU), All Rights Reserved.

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
            DigitalInputBits(0x0)
        {};
        uint32_t BitMask;       // BitMask for this input. From DigitalInput Stream.
        uint32_t DigitalInputBits; // BitMask for this input. From DigitalInput Stream.
    };
}

using namespace sawRobotIO1394;

mtsDigitalInput1394::mtsDigitalInput1394(const cmnGenericObject & owner,
                                         const osaDigitalInput1394Configuration & config):
    OwnerServices(owner.Services())
{
    mData = new mtsDigitalInput1394Data;

    // predefined payloads
    mEventPayloads.Pressed.SetType(prmEventButton::PRESSED);
    mEventPayloads.Pressed.SetValid(true);
    mEventPayloads.Released.SetType(prmEventButton::RELEASED);
    mEventPayloads.Released.SetValid(true);
    mEventPayloads.Clicked.SetType(prmEventButton::CLICKED);
    mEventPayloads.Clicked.SetValid(true);
    Configure(config);
}

mtsDigitalInput1394::~mtsDigitalInput1394()
{
    if (mData) {
        delete mData;
    }
}

void mtsDigitalInput1394::SetupStateTable(mtsStateTable & stateTable)
{
    stateTable.AddData(mValue, mName + "Value");
    mStateTable = &stateTable;
}

void mtsDigitalInput1394::SetupProvidedInterface(mtsInterfaceProvided * prov, mtsStateTable & stateTable)
{
    prov->AddCommandReadState(stateTable, this->mValue, "GetButton");
    prov->AddEventWrite(this->Button, "Button", prmEventButton());
}

void mtsDigitalInput1394::CheckState(void)
{
    // Send appropriate events if the value changed in the last update
    // Check if value has changed
    if (mFirstRun || (mValue != mPreviousValue)) {
        // Check if the value is equal to the value when the digital input is considered pressed
        if (mValue) {
            // Emit a press event if specified in config
            if (mTriggerPress) {
                if (mStateTable) {
                    mEventPayloads.Pressed.SetTimestamp(mStateTable->GetTic());
                }
                Button(mEventPayloads.Pressed);
            }
        } else {
            // Emit a release event if specified in config
            if (mTriggerRelease) {
                if (mStateTable) {
                    mEventPayloads.Released.SetTimestamp(mStateTable->GetTic());
                }
                Button(mEventPayloads.Released);
            }
        }
    }
    // Disable first run
    if (mFirstRun) {
        mFirstRun = false;
    }
}

void mtsDigitalInput1394::Configure(const osaDigitalInput1394Configuration & config)
{
    // Store configuration
    mConfiguration = config;
    mName = config.Name;
    mBitID = config.BitID;
    mData->BitMask = 0x1 << mBitID;
    mPressedValue = config.PressedValue;
    mTriggerPress = config.TriggerWhenPressed;
    mTriggerRelease = config.TriggerWhenReleased;
    mFirstRun = !config.SkipFirstRun;
    mDebounceThreshold = config.DebounceThreshold;
    mDebounceThresholdClick = config.DebounceThresholdClick;
}

void mtsDigitalInput1394::SetBoard(AmpIO * board)
{
    if (board == 0) {
        cmnThrow(this->Name() + ": invalid board pointer.");
    }
    mBoard = board;
}

void mtsDigitalInput1394::PollState(void)
{
    // Store previous value
    mPreviousValue = mValue;

    // Get the new value
    mData->DigitalInputBits =  mBoard->GetDigitalInput();

    // If the masked bit is low, set the value to the pressed value
    bool value = ((mData->DigitalInputBits & mData->BitMask)
                  ? (!mPressedValue) : (mPressedValue));

    // No debounce needed
    if (mFirstRun || (mDebounceThreshold == 0.0)) {
        mValue = value;
        return;
    }

    // Debounce - start if we find one new different value
    if (mDebounceCounter < 0.0) {
        if (value != mPreviousValue) {
            mDebounceCounter = 0.0;
            mTransitionValue = value;
        }
    // count consecutive equal values
    } else {
        if (mDebounceCounter < mDebounceThreshold) {
            if (value == mTransitionValue) {
                mDebounceCounter += mBoard->GetTimestamp() * mBoard->GetFPGAClockPeriod();
            } else {
                // click if button is now changed back and counter is short enough
                if ((mDebounceThresholdClick != mDebounceThreshold) // click is activated
                    && !value // input is "changed back"
                    && (mDebounceCounter >  mDebounceThresholdClick) // pressed long enough
                    ) {
                    if (mStateTable) {
                        mEventPayloads.Clicked.SetTimestamp(mStateTable->GetTic());
                    }
                    Button(mEventPayloads.Clicked);
                }
                mDebounceCounter = -1.0;
            }
        } else {
            mValue = value;
            mDebounceCounter = -1.0;
        }
    }
}

const osaDigitalInput1394Configuration & mtsDigitalInput1394::Configuration(void) const
{
    return mConfiguration;
}

const std::string & mtsDigitalInput1394::Name(void) const
{
    return mName;
}

const bool & mtsDigitalInput1394::Value(void) const
{
    return mValue;
}

const bool & mtsDigitalInput1394::PreviousValue(void) const
{
    return mPreviousValue;
}
