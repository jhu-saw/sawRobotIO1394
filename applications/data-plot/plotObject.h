/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-01-09

  (C) Copyright 2014-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#ifndef _plotObject_h
#define _plotObject_h

// system
#include <iostream>

// cisst/saw
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobot1394.h>

// Qt
#include <QFrame>
#include <QVBoxLayout>
#include <cisstVector/vctPlot2DOpenGLQtWidget.h>

using namespace sawRobotIO1394;

class plotObject: public QObject
{
    Q_OBJECT;
public:
    plotObject(mtsRobotIO1394 * port,
               mtsRobot1394 * robot,
               int actuatorIndex);

private slots:
    void timerEvent(QTimerEvent * CMN_UNUSED(event));

protected:
    QFrame * mFrame;
    QVBoxLayout * mLayout;
    vctPlot2DOpenGLQtWidget * mPlot;
    vctPlot2DBase::Scale * mVelocityScale;
    vctPlot2DBase::Signal * mZeroVelocity;
    vctPlot2DBase::Signal * mEncoderDtSignal;
    vctPlot2DBase::Signal * mEncoderDxSignal;
    vctPlot2DBase::Signal * mEncoderDxFilteredSignal;
    vctPlot2DBase::Signal * mEncoderSoftware;
    vctPlot2DBase::Signal * mPotDxSignal;

    mtsRobotIO1394 * mPort;
    mtsRobot1394 * mRobot;
    int mActuatorIndex;

    double mElapsedTime;
    vctDoubleVec mPreviousEncoderPosition;
    vctDoubleVec mEncoderDx;
    vctDoubleVec mPreviousPotPosition;
    vctDoubleVec mPotDx;

    size_t mFilterSize;
    vctDoubleVec mSavitzkyGolayCoeff;
    vctDoubleVec mHistory;
    vctDoubleVec mFilterElementwiseProduct;
};

#endif // _plotObject_h
