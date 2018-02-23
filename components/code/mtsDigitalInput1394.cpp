/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2011-06-10

  (C) Copyright 2011-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsStateTable.h>
#include <cisstParameterTypes/prmEventButton.h>

#include "AmpIO.h"

#include <sawRobotIO1394/mtsDigitalInput1394.h>

using namespace sawRobotIO1394;


mtsDigitalInput1394::mtsDigitalInput1394(const cmnGenericObject & owner,
                                         const osaDigitalInput1394Configuration & config):
    OwnerServices(owner.Services()),
    mDigitalInputBits(0x0),
    mValue(false),
    mPreviousValue(false),
    mDebounceCounter(-1)
{
    Configure(config);
}

void mtsDigitalInput1394::SetupStateTable(mtsStateTable & stateTable)
{
    stateTable.AddData(mValue, mName + "Value");
}

void mtsDigitalInput1394::SetupProvidedInterface(mtsInterfaceProvided * prov, mtsStateTable & stateTable)
{
    prov->AddCommandReadState(stateTable, this->mValue, "GetButton");
    prov->AddEventWrite(this->Button, "Button", prmEventButton());
}

void mtsDigitalInput1394::CheckState(void)
{
    static const prmEventButton
        pressed = prmEventButton::PRESSED,
        released = prmEventButton::RELEASED;

    // Send appropriate events if the value changed in the last update

    // Check if value has changed
    if (mValue != mPreviousValue) {
        // Check if the value is equal to the value when the digital input is considered pressed
        if (mValue) {
            // Emit a press event
            if (mTriggerPress) {
                Button(pressed);
            }
        } else {
            // Emit a release event
            if (mTriggerRelease) {
                Button(released);
            }
        }
    }
}

void mtsDigitalInput1394::Configure(const osaDigitalInput1394Configuration & config)
{
    // Store configuration
    mConfiguration = config;
    mName = config.Name;
    mBitID = config.BitID;
    mBitMask = 0x1 << mBitID;
    mPressedValue = config.PressedValue;
    mTriggerPress = config.TriggerWhenPressed;
    mTriggerRelease = config.TriggerWhenReleased;
    mDebounceThreshold = config.DebounceThreshold;

    // Set the value to un-pressed
    mValue = !mPressedValue;
    mPreviousValue = mValue;

    // ZC: HEAD
    mTestConfidence = 0;
    mTestLow = 0.2;
    mTestHigh = 0.8;
    mTestWeight = 0.98;
}

void mtsDigitalInput1394::SetBoard(AmpIO * board)
{
    if (board == 0) {
        cmnThrow(osaRuntimeError1394(this->Name() + ": invalid board pointer."));
    }
    mBoard = board;
}

void mtsDigitalInput1394::PollState(void)
{
    // Store previous value
    mPreviousValue = mValue;

    // Get the new value
    mDigitalInputBits =  mBoard->GetDigitalInput();

    // If the masked bit is low, set the value to the pressed value
    bool value = (mDigitalInputBits & mBitMask) ? (!mPressedValue) : (mPressedValue);

    // ZC: hack for quick testing
    if (Name() == "HEAD") {
        std::cerr << CMN_LOG_DETAILS << " this special case needs to be removed!" << std::endl;
        mTestConfidence = mTestWeight * mTestConfidence + (1 - mTestWeight) * value;
        if (mPreviousValue == true && mTestConfidence < mTestLow) mValue = false;
        else if (mPreviousValue == false && mTestConfidence > mTestHigh) mValue = true;
        return;
    }
    
    // No debounce needed
    if (mDebounceThreshold == 0.0) {
        mValue = value;
        return;
    }

    // Debounce - start if we find one new different value
    if (mDebounceCounter == -1.0) {
        if (value != mPreviousValue) {
            mDebounceCounter = 0.0;
            mTransitionValue = value;
        }
    // count consecutive equal values
    } else {
        if (mDebounceCounter < mDebounceThreshold) {
            if (value == mTransitionValue) {
                mDebounceCounter += mBoard->GetTimestamp() / (49.125 * 1000.0 * 1000.0); // clock is 49.125 MHz
            } else {
                mDebounceCounter = -1.0;
            }
        } else {
            mValue = value;
            mDebounceCounter = -1.0;
        }
    }
}

const osaDigitalInput1394Configuration & mtsDigitalInput1394::Configuration(void) const {
    return mConfiguration;
}

const std::string & mtsDigitalInput1394::Name(void) const {
    return mName;
}

const bool & mtsDigitalInput1394::Value(void) const {
    return mValue;
}

const bool & mtsDigitalInput1394::PreviousValue(void) const {
    return mPreviousValue;
}
