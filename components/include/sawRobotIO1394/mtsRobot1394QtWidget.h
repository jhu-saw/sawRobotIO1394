/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen
  Created on: 2012-07-20

  (C) Copyright 2012-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsRobot1394QtWidget_h
#define _mtsRobot1394QtWidget_h

#include <cisstOSAbstraction/osaTimeServer.h>
#include <cisstVector/vctQtWidgetDynamicVector.h>
#include <cisstMultiTask/mtsQtWidgetIntervalStatistics.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstParameterTypes/prmStateJoint.h>

#include <QWidget>
#include <QCheckBox>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <sawRobotIO1394/sawRobotIO1394ExportQt.h>

/*!
  \todo maybe rename this class to mtsRobotIO1394{Robot,DigitalInputs,Log}QtWidget and create using mtsRobotIO1394FactoryQtWidget
  \todo cisst Qt convention is now to start with the Qt prefix, i.e. mtsQtWidgetRobotIO1394 ...
  */
class CISST_EXPORT mtsRobot1394QtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsRobot1394QtWidget(const std::string & componentName,
                         unsigned int numberOfActuators,
                         unsigned int numberOfBrakes,
                         double periodInSeconds = 50.0 * cmn_ms);
    mtsRobot1394QtWidget(const mtsComponentConstructorNameAndUInt &arg);
    inline ~mtsRobot1394QtWidget(void) {}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

protected:
    void Init(void);
    virtual void closeEvent(QCloseEvent *event);

private slots:
    void SlotSafetyRelay(bool toggle);
    void SlotPowerEnable(bool toggle);
    void SlotEnableAll(bool toggle);
    void SlotEnableDirectControl(bool toggle);
    void SlotActuatorAmpEnable(void);
    void SlotResetCurrentAll(void);
    void SlotActuatorCurrentValueChanged(void);
    void SlotSliderActuatorCurrentValueChanged(void);
    void SlotBrakeAmpEnable(void);
    void SlotResetEncodersAll(void);
    void SlotBiasEncodersAll(void);
    void SlotUsePotsForSafetyCheck(bool status);
    void SlotWatchdogPeriod(void);
    void SlotBrakeEngage(void);
    void SlotBrakeRelease(void);

    void timerEvent(QTimerEvent * event);

private:
    void SetupCisstInterface(void);
    void setupUi(void);

    // gui update
    void UpdateCurrentDisplay(void);
    void UpdateRobotInfo(void);

protected:
    bool DirectControl;
    int TimerPeriodInMilliseconds;
    double WatchdogPeriodInSeconds;
    size_t WatchdogCounter;

    struct RobotStruct {
        mtsFunctionRead GetSerialNumber;
        mtsFunctionRead period_statistics;
        mtsFunctionRead IsValid;

        mtsFunctionWrite WriteSafetyRelay;
        mtsFunctionRead GetSafetyRelay;
        mtsFunctionRead GetSafetyRelayStatus;

        mtsFunctionWrite SetWatchdogPeriod;
        mtsFunctionRead WatchdogPeriod;

        mtsFunctionVoid PowerOnSequence;
        mtsFunctionWrite PowerOffSequence;
        mtsFunctionRead GetPowerEnable;
        mtsFunctionRead GetFullyPowered;

        mtsFunctionRead measured_js;
        mtsFunctionRead GetAnalogInputVolts;
        mtsFunctionRead GetAnalogInputPosSI;
        mtsFunctionRead GetActuatorRequestedCurrent;
        mtsFunctionRead GetActuatorFeedbackCurrent;
        mtsFunctionRead GetActuatorCurrentMax;
        mtsFunctionRead configuration_js;

        mtsFunctionRead GetActuatorAmpTemperature;

        mtsFunctionWrite SetBrakeAmpEnable;
        mtsFunctionRead GetBrakeAmpEnable;
        mtsFunctionRead GetBrakeAmpStatus;
        mtsFunctionRead GetBrakeRequestedCurrent;
        mtsFunctionRead GetBrakeFeedbackCurrent;
        mtsFunctionRead GetBrakeAmpTemperature;
        mtsFunctionVoid BrakeEngage;
        mtsFunctionVoid BrakeRelease;
        mtsFunctionWrite SetActuatorCurrent;
        mtsFunctionWrite SetEncoderPosition;

        mtsFunctionWrite BiasEncoder;
        mtsFunctionWrite UsePotsForSafetyCheck;
    } Robot;

    struct ActuatorStruct {
        mtsFunctionWrite WritePowerEnable;
        mtsFunctionWrite SetAmpEnable;
        mtsFunctionRead GetAmpEnable;
        mtsFunctionRead GetAmpStatus;
        mtsFunctionRead measured_js;
    } Actuators;


private:
    mtsIntervalStatistics IntervalStatistics;

    int SerialNumber;
    size_t NumberOfActuators;
    size_t NumberOfBrakes;

    vctDoubleVec UnitFactor;
    prmStateJoint StateJoint, ActuatorStateJoint;
    vctDoubleVec PotentiometersVolts;
    prmStateJoint PotentiometersPosition;
    vctDoubleVec ActuatorFeedbackCurrent;
    vctDoubleVec ActuatorRequestedCurrent;
    vctDoubleVec ActuatorAmpTemperature;
    vctBoolVec ActuatorAmpEnable;
    vctBoolVec ActuatorAmpStatus;
    vctDoubleVec BrakeFeedbackCurrent;
    vctDoubleVec BrakeRequestedCurrent;
    vctDoubleVec BrakeAmpTemperature;
    vctBoolVec BrakeAmpEnable;
    vctBoolVec BrakeAmpStatus;

    bool FullyPowered;
    bool PowerEnable;
    bool SafetyRelay, SafetyRelayStatus;

    // Interface
    double DummyValueWhenNotConnected;
    vctDynamicVector<bool> LastEnableState;
    double StartTime;

    // GUI: Commands
    QCheckBox * QCBSafetyRelay;
    QCheckBox * QCBPowerEnable;
    QCheckBox * QCBEnableAll;
    QPushButton * QPBResetCurrentAll;

    // GUI: Feedbacks
    QPushButton * QPBResetEncAll;
    QPushButton * QPBBiasEncAll;
    QCheckBox * QCBUsePotsForSafetyCheck;
    QDoubleSpinBox * QSBWatchdogPeriod;
    QCheckBox * QCBEnableDirectControl;
    QLabel * QLSerialNumber;

    // GUI: timing
    mtsQtWidgetIntervalStatistics * QMIntervalStatistics;

    vctQtWidgetDynamicVectorBoolWrite * QVWActuatorCurrentEnableEach;
    vctQtWidgetDynamicVectorDoubleWrite * QVWActuatorCurrentSpinBox;
    vctQtWidgetDynamicVectorDoubleWrite * QVWActuatorCurrentSlider;
    vctQtWidgetDynamicVectorDoubleRead * QVRActuatorCurrentFeedback;
    vctQtWidgetDynamicVectorDoubleRead * QVRJointPosition;
    vctQtWidgetDynamicVectorDoubleRead * QVRActuatorPosition;
    vctQtWidgetDynamicVectorDoubleRead * QVRActuatorVelocity;
    vctQtWidgetDynamicVectorDoubleRead * QVRPotVolts;
    vctQtWidgetDynamicVectorDoubleRead * QVRPotPosition;
    vctQtWidgetDynamicVectorDoubleRead * QVRActuatorAmpTemperature;

    // Brakes
    QPushButton * QPBBrakeRelease;
    QPushButton * QPBBrakeEngage;
    vctQtWidgetDynamicVectorBoolWrite * QVWBrakeCurrentEnableEach;
    vctQtWidgetDynamicVectorDoubleRead * QVRBrakeCurrentCommand;
    vctQtWidgetDynamicVectorDoubleRead * QVRBrakeCurrentFeedback;
    vctQtWidgetDynamicVectorDoubleRead * QVRBrakeAmpTemperature;

    QLabel * QLAmpStatus;
    QLabel * QLFullyPowered;
    QLabel * QLSafetyRelayStatus;
    QLabel * QLWatchdog;
    QLabel * QLWatchdogLastTimeout;

    void FullyPoweredEventHandler(const bool & status);
    void WatchdogTimeoutStatusEventHandler(const bool & status);
    void UsePotsForSafetyCheckEventHandler(const bool & status);

    // signal and slot used by mts event handlers, this this component
    // doesn't have a thread (i.e. mtsTask), events are handled in the
    // thread that sends the event.  We then use Qt emit/slot to maintain
    // thread safety
signals:
    void SignalFullyPowered(bool status);
    void SignalWatchdogTimeoutStatus(bool status);
    void SignalUsePotsForSafetyCheck(bool status);
protected slots:
    void SlotFullyPoweredEvent(bool status);
    void SlotWatchdogTimeoutStatusEvent(bool status);
    void SlotUsePotsForSafetyCheckEvent(bool status);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsRobot1394QtWidget);

#endif // _mtsRobot1394QtWidget_h
