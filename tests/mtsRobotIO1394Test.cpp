/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Anton Deguet
  Created on: 2013-03-01

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "mtsRobotIO1394Test.h"
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <cisstVector/vctDynamicVectorTypes.h>

void mtsRobotIO1394Test::TestCreate(void) {
    mtsRobotIO1394 * robot = new mtsRobotIO1394("robot", 1.0 * cmn_ms, "fw:0");
    CPPUNIT_ASSERT(robot);
}

/*
void mtsRobotIO1394Test::TestConfigure(void) {
    std::stringstream errorStream;
    mtsRobotIO1394 * robot = new mtsRobotIO1394("robot", 1.0 * cmn_ms, 0, errorStream);
    CPPUNIT_ASSERT(robot);
    CPPUNIT_ASSERT(robot->configure());

}

void mtsRobotIO1394Test::TestEncoder(void) {
    mtsRobotIO1394 * robot = new mtsRobotIO1394("robot", 1.0 * cmn_ms, 0, errorStream);
    CPPUNIT_ASSERT(robot);
    CPPUNIT_ASSERT(robot->configure());
    robotCount = robot.RobotList.size();
    int i = 0;
    int j = 0;
    int jointCount = 0;
    for(i = 0; i < robotCount; i++) {
        CPPUNIT_ASSERT(robot.RobotList[i]->GetData());
        CPPUNIT_ASSERT(robot.RobotList[i]->GetNumberOfJoints(jointCount));
        vctDoubleVec tempJointVector1;
        vctLongVec tempJointVector2;
        tempJointVector1.SetSize(jointCount);
        tempJointVector2.SetSize(jointCount);
        CPPUNIT_ASSERT(robot.RobotList[i]->EncoderSIToRaw(robot.RobotList[i].encPosRaw,tempJointVector1));

        CPPUNIT_ASSERT(robot.RobotList[i]->EncoderRawToSI(tempJointVector1,tempJointVector2));


        for(j = 0; j < jointCount; j++) {
            CPPUNIT_ASSERT_DOUBLE_EQUAL(tempJointVector2[j],robot.RobotList[i].encPosRaw[j],MINIMUM_THRESHOLD_LONG);
        }
   }
}

void mtsRobotIO1394::RobotInternal::DriveAmpsToBitsToFbAmps(const vctDoubleVec &fromData, vctDoubleVec &toData, vctBoolVec &result) const
{
    toData.SetAll(0.0);
    result.SetAll(false);
    vctLongVec tempData;
    tempData.SetSize(fromData.size());
    tempData.SetAll(0L);

    DriveAmpsToBits(fromData,tempData);
    DriveBitsToFbAmps(tempData,toData);

    for(size_t index=0; index<JointList.size();index++){
        if(fabs(toData[index] - fromData[index]) < MINIMUM_THRESHOLD_DOUBLE){
            result[index]=true;
        }
    }
}

void mtsRobotIO1394::RobotInternal::DriveNmToAmpsToNm(const vctDoubleVec &fromData, vctDoubleVec &toData, vctBoolVec &result) const
{
    toData.SetAll(0.0);
    result.SetAll(false);
    vctDoubleVec tempData;
    tempData.SetSize(fromData.size());
    tempData.SetAll(0.0);

    DriveNmToAmps(fromData,tempData);
    DriveAmpsToNm(tempData,toData);

    for(size_t index=0;index<JointList.size();index++){
        if(fabs(toData[index] - fromData[index]) < MINIMUM_THRESHOLD_DOUBLE){
            result[index]=true;
        }
    }
}

void mtsRobotIO1394::RobotInternal::AnalogInBitsToPosSI(const vctLongVec &fromData, vctDoubleVec &toData) const
{
    toData.SetAll(0.0);
    vctDoubleVec tempData;
    tempData.SetSize(fromData.size());
    tempData.SetAll(0.0);

    AnalogInBitsToVolts(fromData,tempData);
    AnalogInVoltsToPosSI(tempData,toData);
}
*/
