/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-01-09

  (C) Copyright 2014-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#include "plotObject.h"
#include <cisstNumerical/nmrSavitzkyGolay.h>

plotObject::plotObject(mtsRobotIO1394 * port,
                       mtsRobot1394 * robot,
                       int actuatorIndex):
    mPort(port),
    mRobot(robot),
    mActuatorIndex(actuatorIndex),
    mElapsedTime(0.0),
    mFilterSize(6)
{
    const Eigen::VectorXd savitzky_golay_coeffs = nmrSavitzkyGolay(2, // 4 degrees polynomial
                                          0, // no derivative
                                          mFilterSize - 1, // nb left samples
                                          0 // nb right samples
                                          );

    mHistory.resize(mFilterSize);
    mHistory.fill(0.0);

    mFrame = new QFrame();
    mLayout = new QVBoxLayout();

    mPlot = new vctPlot2DOpenGLQtWidget();
    mLayout->addWidget(mPlot);

    mVelocityScale = mPlot->AddScale("encoder-velocities");

    mEncoderDtSignal = mVelocityScale->AddSignal("encoder-dt");
    mEncoderDtSignal->SetColor(Eigen::Vector3d(1.0, 0.0, 0.0));
    std::cout << "red:   encoder-dt" << std::endl;

    mEncoderDxSignal = mVelocityScale->AddSignal("encoder-dx");
    mEncoderDxSignal->SetColor(Eigen::Vector3d(0.0, 1.0, 0.0));
    std::cout << "green: encoder-dx" << std::endl;

    mEncoderDxFilteredSignal = mVelocityScale->AddSignal("encoder-dx-filtered");
    mEncoderDxFilteredSignal->SetColor(Eigen::Vector3d(7.0, 7.0, 0.0));
    std::cout << "yellow: encoder-dx-filtered" << std::endl;

    mPotDxSignal = mVelocityScale->AddSignal("pot-dx");
    mPotDxSignal->SetColor(Eigen::Vector3d(1.0, 1.0, 1.0));
    std::cout << "white: pot-dx" << std::endl;

    mZeroVelocity = mVelocityScale->AddSignal("zero");
    mZeroVelocity->SetColor(Eigen::Vector3d(0.2, 0.2, 0.2));
    std::cout << "gray:  zero" << std::endl;

    mFrame->setLayout(mLayout);
    mFrame->resize(1200, 600);
    mFrame->show();

    mPreviousEncoderPosition = mRobot->ActuatorJointState().Position();
    mPreviousPotPosition = mRobot->PotentiometerPosition();

    startTimer(0); // in ms, 0 is as fast as possible
}

void plotObject::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    mPort->Read();
    // get time and plot 0 value
    mElapsedTime += mRobot->ActuatorTimestamp()[mActuatorIndex];
    mZeroVelocity->AppendPoint(Eigen::Vector2d(mElapsedTime, 0.005)); // slight offset to avoid overlap

    // encoder dt
    mEncoderDtSignal->AppendPoint(Eigen::Vector2d(mElapsedTime,
                                       mRobot->ActuatorJointState().Velocity()[mActuatorIndex]));
    // encoder velocity dx / dt
    mEncoderDx = mRobot->ActuatorJointState().Position();
    mEncoderDx -= mPreviousEncoderPosition;
    mEncoderDx /= mRobot->ActuatorTimestamp().array();
    mEncoderDxSignal->AppendPoint((Eigen::Vector2d(mElapsedTime,
                                        mEncoderDx[mActuatorIndex])));

    // filtered dx / dt
    // update history
    for (size_t index = 1; index < mFilterSize; ++index) {
        mHistory[index - 1] = mHistory[index];
    }
    mHistory[mFilterSize - 1] = mEncoderDx[mActuatorIndex];
    // apply filter
    double filtered_value = mSavitzkyGolayCoeff.cwiseProduct(mHistory).sum();
    mEncoderDxFilteredSignal->AppendPoint(Eigen::Vector2d(mElapsedTime, filtered_value));

    // pot velocity dx / dt
    mPotDx = mRobot->PotentiometerPosition();
    mPotDx -= mPreviousPotPosition;
    mPotDx /= mRobot->ActuatorTimestamp().array();
    mPotDxSignal->AppendPoint((Eigen::Vector2d(mElapsedTime,
                                    mPotDx[mActuatorIndex])));

    // update plot
    mPlot->update();

    // save previous state
    mPreviousEncoderPosition = mRobot->ActuatorJointState().Position();
    mPreviousPotPosition = mRobot->PotentiometerPosition();
}
