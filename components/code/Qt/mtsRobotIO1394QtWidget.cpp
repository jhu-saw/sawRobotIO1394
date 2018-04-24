/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-16

  (C) Copyright 2013-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// Qt include
#include <QString>
#include <QMessageBox>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QTime>

// project include
#include <sawRobotIO1394/mtsRobotIO1394QtWidget.h>

#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmJointType.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsRobotIO1394QtWidget, mtsComponent, mtsComponentConstructorNameAndUInt)


mtsRobotIO1394QtWidget::mtsRobotIO1394QtWidget(const std::string & componentName,
                                               unsigned int numberOfActuators,
                                               unsigned int numberOfBrakes,
                                               double periodInSeconds):
    mtsComponent(componentName),
    DirectControl(false),
    TimerPeriodInMilliseconds(periodInSeconds * 1000), // Qt timers are in milliseconds
    SerialNumber(0),
    NumberOfActuators(numberOfActuators),
    NumberOfBrakes(numberOfBrakes)
{
    WatchdogPeriodInSeconds = 300.0 * cmn_ms;
    Init();
}

mtsRobotIO1394QtWidget::mtsRobotIO1394QtWidget(const mtsComponentConstructorNameAndUInt &arg):
    mtsComponent(arg.Name),
    SerialNumber(0),
    NumberOfActuators(arg.Arg)
{
    Init();
}

void mtsRobotIO1394QtWidget::Init(void)
{
    DummyValueWhenNotConnected = 0;
    LastEnableState.SetSize(NumberOfActuators);
    LastEnableState.SetAll(false);

    UnitFactor.SetSize(NumberOfActuators);
    JointPosition.SetSize(NumberOfActuators);
    ActuatorPosition.SetSize(NumberOfActuators);
    ActuatorPositionGet.Position().SetSize(NumberOfActuators);
    ActuatorVelocity.SetSize(NumberOfActuators);
    PotentiometersVolts.SetSize(NumberOfActuators);
    PotentiometersPosition.SetSize(NumberOfActuators);
    ActuatorFeedbackCurrent.SetSize(NumberOfActuators);
    ActuatorFeedbackCurrent.Zeros();
    ActuatorRequestedCurrent.SetSize(NumberOfActuators);
    ActuatorRequestedCurrent.Zeros();
    ActuatorAmpTemperature.SetSize(NumberOfActuators);

    StartTime = osaGetTime();

    SetupCisstInterface();
    setupUi();

    startTimer(TimerPeriodInMilliseconds); // ms
}

void mtsRobotIO1394QtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsRobotIO1394QtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
    vctDoubleVec actuatorCurrentMax(this->NumberOfActuators);
    mtsExecutionResult result = Robot.GetActuatorCurrentMax(actuatorCurrentMax);
    if (!result) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: Robot interface isn't connected properly, unable to get actuator current max.  Function call returned: "
                                 << result << std::endl;
    } else {
        // convert to mA
        actuatorCurrentMax.Multiply(1000.0);
        QVWActuatorCurrentSpinBox->SetRange(-actuatorCurrentMax, actuatorCurrentMax);
        QVWActuatorCurrentSlider->SetRange(-actuatorCurrentMax, actuatorCurrentMax);
    }

    prmJointTypeVec jointType;
    result = Robot.GetJointType(jointType);
    if (!result) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: Robot interface isn't connected properly, unable to get joint type.  Function call returned: "
                                 << result << std::endl;
        UnitFactor.SetAll(0.0);
    } else {
        // set unitFactor
        for (size_t i = 0; i < this->NumberOfActuators; i++ ) {
            switch (jointType[i]) {
            case PRM_JOINT_REVOLUTE:
                UnitFactor[i] = cmn180_PI;
                break;
            case PRM_JOINT_PRISMATIC:
                UnitFactor[i] = 1.0 / cmn_mm; // convert internal values to mm
                break;
            case PRM_JOINT_INACTIVE:
            case PRM_JOINT_UNDEFINED:
                break;
            default:
                cmnThrow("mtsRobotIO1394QtWidget: unknown joint type");
                break;
            }
        }
    }
    // get serial number
    result = Robot.GetSerialNumber(SerialNumber);
    if (!result) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: Robot interface isn't connected properly, unable to get serial number.  Function call returned: "
                                 << result << std::endl;
    }
    QLSerialNumber->setText(QString::number(SerialNumber));

    if (!parent()) {
        show();
    }
}

void mtsRobotIO1394QtWidget::Cleanup(void)
{
    this->hide();
    Robot.DisablePower();
    Actuators.DisableBoardsPower();
}

void mtsRobotIO1394QtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsRobotIO1394QtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsRobotIO1394QtWidget::SlotEnableAmps(bool toggle)
{
    // send to controller first
    if (toggle) {
        Actuators.EnableBoardsPower();
    } else {
        Actuators.DisableBoardsPower();
    }
    // update GUI, make sure no signal is generated
    if (!toggle) {
        QCBEnableAll->blockSignals(true);
        {
            QCBEnableAll->setChecked(false);
        }
        QCBEnableAll->blockSignals(false);
    }
    // set all current to 0
    SlotResetCurrentAll();
}

void mtsRobotIO1394QtWidget::SlotEnableAll(bool toggle)
{
    // send to controller first
    if (toggle) {
        Robot.EnablePower();
    } else {
        Robot.DisablePower();
    }
    // update GUI, make sure no signal is generated
    QCBEnableAmps->blockSignals(true);
    {
        QCBEnableAmps->setChecked(toggle);
    } QCBEnableAmps->blockSignals(false);
    if (NumberOfActuators != 0) {
        vctBoolVec allEnable(NumberOfActuators, toggle);
        QVWActuatorCurrentEnableEach->SetValue(allEnable);
    }
    if (NumberOfBrakes != 0) {
        vctBoolVec allEnable(NumberOfBrakes, toggle);
        QVWBrakeCurrentEnableEach->SetValue(allEnable);
    }

    // set all current to 0
    SlotResetCurrentAll();
}

void mtsRobotIO1394QtWidget::SlotEnableDirectControl(bool toggle)
{
    if (toggle) {
        int answer = QMessageBox::warning(this, tr("mtsRobotIO1394QtWidget"),
                                          tr("In direct control mode you can potentially harm your robot.\nAre you sure you want to continue?"),
                                          QMessageBox::No | QMessageBox::Yes);
        if (answer == QMessageBox::No) {
            toggle = false;
        }
    }
    QCBEnableDirectControl->setChecked(toggle);
    DirectControl = toggle;
    // update widgets
    QCBEnableAll->setEnabled(toggle);
    QCBEnableAmps->setEnabled(toggle);
    QSBWatchdogPeriod->setEnabled(toggle);
    QPBResetEncAll->setEnabled(toggle);
    QPBBiasEncAll->setEnabled(toggle);
    QCBUsePotsForSafetyCheck->setEnabled(toggle);
    QVWActuatorCurrentEnableEach->setEnabled(toggle);
    QVWActuatorCurrentSpinBox->setEnabled(toggle);
    QVWActuatorCurrentSlider->setEnabled(toggle);
    QPBResetCurrentAll->setEnabled(toggle);
    if (NumberOfBrakes != 0) {
        QPBBrakeRelease->setEnabled(toggle);
        QPBBrakeEngage->setEnabled(toggle);
        QVWBrakeCurrentEnableEach->setEnabled(toggle);
    }

    // set all current to 0
    SlotResetCurrentAll();
}

void mtsRobotIO1394QtWidget::SlotResetCurrentAll(void)
{
    // send to controller first
    vctDoubleVec cmdCurA(NumberOfActuators);
    cmdCurA.SetAll(0.0);
    Robot.SetActuatorCurrent(cmdCurA);
    // update GUI
    QVWActuatorCurrentSpinBox->SetValue(cmdCurA);
    QVWActuatorCurrentSlider->SetValue(cmdCurA);
}

void mtsRobotIO1394QtWidget::SlotActuatorAmpEnable(void)
{
    ActuatorAmpEnable.SetSize(NumberOfActuators);
    QVWActuatorCurrentEnableEach->GetValue(ActuatorAmpEnable);
    Actuators.SetAmpEnable(ActuatorAmpEnable);
}

void mtsRobotIO1394QtWidget::SlotBrakeAmpEnable(void)
{
    BrakeAmpEnable.SetSize(NumberOfBrakes);
    QVWBrakeCurrentEnableEach->GetValue(BrakeAmpEnable);
    Robot.SetBrakeAmpEnable(BrakeAmpEnable);
}

void mtsRobotIO1394QtWidget::SlotActuatorCurrentValueChanged()
{
    vctDoubleVec cmdCurmA(NumberOfActuators);
    vctDoubleVec cmdCurA(NumberOfActuators);
    // get value from GUI
    QVWActuatorCurrentSpinBox->GetValue(cmdCurmA);
    QVWActuatorCurrentSlider->SetValue(cmdCurmA);
    // convert to amps and apply
    cmdCurA = cmdCurmA.Divide(1000.0);
    Robot.SetActuatorCurrent(cmdCurA);
}

void mtsRobotIO1394QtWidget::SlotSliderActuatorCurrentValueChanged()
{
    vctDoubleVec cmdCurmA(NumberOfActuators);
    vctDoubleVec cmdCurA(NumberOfActuators);
    // get value from GUI
    QVWActuatorCurrentSlider->GetValue(cmdCurmA);
    QVWActuatorCurrentSpinBox->SetValue(cmdCurmA);
    // convert to amps and apply
    cmdCurA = cmdCurmA.Divide(1000.0);
    Robot.SetActuatorCurrent(cmdCurA);
}

void mtsRobotIO1394QtWidget::SlotResetEncodersAll()
{
    vctDoubleVec newEncoderValues(NumberOfActuators);
    newEncoderValues.SetAll(0.0);
    mtsExecutionResult result = Robot.SetEncoderPosition(newEncoderValues);
    if (!result.IsOK()) {
        CMN_LOG_CLASS_RUN_WARNING << "SlotResetEncodersAll: command failed \"" << result << "\"" << std::endl;
     }
}

void mtsRobotIO1394QtWidget::SlotBiasEncodersAll()
{
    mtsExecutionResult result = Robot.BiasEncoder(1000);
    if (!result.IsOK()) {
        CMN_LOG_CLASS_RUN_WARNING << "SlotBiasEncodersAll: command failed \"" << result << "\"" << std::endl;
    }
}

void mtsRobotIO1394QtWidget::SlotWatchdogPeriod(double period_ms)
{
    if (period_ms == 0.0) {
        QMessageBox message;
        message.setText("Setting the watchdog period to 0 disables the watchdog!");
        message.exec();
    }
    WatchdogPeriodInSeconds = period_ms * cmn_ms;
    mtsExecutionResult result = Robot.SetWatchdogPeriod(WatchdogPeriodInSeconds);
    if(!result.IsOK()){
        CMN_LOG_CLASS_RUN_WARNING << "SlotWatchdogPeriod: command failed \""
                                  << result << "\"" << std::endl;
    }
}

void mtsRobotIO1394QtWidget::SlotBrakeEngage(void)
{
    Robot.BrakeEngage();
}

void mtsRobotIO1394QtWidget::SlotBrakeRelease(void)
{
    Robot.BrakeRelease();
}

void mtsRobotIO1394QtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    ProcessQueuedEvents();

    // make sure we should update the display
    if (!this->isHidden()) {

        bool flag;
        Robot.IsValid(flag);
        if (flag) {
            Robot.GetPeriodStatistics(IntervalStatistics);
            Robot.GetPowerStatus(PowerStatus);
            Robot.GetSafetyRelay(SafetyRelay);
            if (NumberOfActuators != 0) {
                Actuators.GetAmpEnable(ActuatorAmpEnable);
                Actuators.GetAmpStatus(ActuatorAmpStatus);
                Robot.GetPosition(JointPosition); // vct
                JointPosition.ElementwiseMultiply(UnitFactor); // to degrees or mm
                Actuators.GetPositionActuator(ActuatorPositionGet); // prm
                ActuatorPosition.Assign(ActuatorPositionGet.Position()); // vct
                ActuatorPosition.ElementwiseMultiply(UnitFactor); // to degrees or mm
                Robot.GetVelocity(ActuatorVelocity);
                ActuatorVelocity.ElementwiseMultiply(UnitFactor); // to degrees or mm
                Robot.GetAnalogInputVolts(PotentiometersVolts);
                Robot.GetAnalogInputPosSI(PotentiometersPosition);
                PotentiometersPosition.ElementwiseMultiply(UnitFactor); // to degrees or mm
                Robot.GetActuatorFeedbackCurrent(ActuatorFeedbackCurrent);
                ActuatorFeedbackCurrent.Multiply(1000.0); // to mA
                Robot.GetActuatorAmpTemperature(ActuatorAmpTemperature);
            }
            if (NumberOfBrakes != 0) {
                Robot.GetBrakeAmpEnable(BrakeAmpEnable);
                Robot.GetBrakeAmpStatus(BrakeAmpStatus);
                Robot.GetBrakeRequestedCurrent(BrakeRequestedCurrent);
                BrakeRequestedCurrent.Multiply(1000.0); // to mA
                Robot.GetBrakeFeedbackCurrent(BrakeFeedbackCurrent);
                BrakeFeedbackCurrent.Multiply(1000.0); // to mA
                Robot.GetBrakeAmpTemperature(BrakeAmpTemperature);
            }
        } else {
            JointPosition.SetAll(DummyValueWhenNotConnected);
            ActuatorPosition.SetAll(DummyValueWhenNotConnected);
            ActuatorVelocity.SetAll(DummyValueWhenNotConnected);
            PotentiometersVolts.SetAll(DummyValueWhenNotConnected);
            PotentiometersPosition.SetAll(DummyValueWhenNotConnected);
            ActuatorFeedbackCurrent.SetAll(DummyValueWhenNotConnected);
            ActuatorAmpTemperature.SetAll(DummyValueWhenNotConnected);
        }

        DummyValueWhenNotConnected += 0.1;

        // display requested current when we are not trying to set it using GUI
        if (flag && !DirectControl) {
            Robot.GetActuatorRequestedCurrent(ActuatorRequestedCurrent);
            ActuatorRequestedCurrent.Multiply(1000.0); // got A, need mA for display
            QVWActuatorCurrentSpinBox->SetValue(ActuatorRequestedCurrent);
            QVWActuatorCurrentSlider->SetValue(ActuatorRequestedCurrent);
        }

        QMIntervalStatistics->SetValue(IntervalStatistics);
        if (NumberOfActuators != 0) {
            QVRJointPosition->SetValue(JointPosition);
            QVRActuatorPosition->SetValue(ActuatorPosition);
            QVRActuatorVelocity->SetValue(ActuatorVelocity);
            QVRPotVolts->SetValue(PotentiometersVolts);
            QVRPotPosition->SetValue(PotentiometersPosition);
            QVRActuatorCurrentFeedback->SetValue(ActuatorFeedbackCurrent);
            QVRActuatorAmpTemperature->SetValue(ActuatorAmpTemperature);
        }
        if (NumberOfBrakes != 0) {
            QVRBrakeCurrentCommand->SetValue(BrakeRequestedCurrent);
            QVRBrakeCurrentFeedback->SetValue(BrakeFeedbackCurrent);
            QVRBrakeAmpTemperature->SetValue(BrakeAmpTemperature);
        }

        UpdateRobotInfo();
    }

    // refresh watchdog period
    double watchdogPeriodInSeconds;
    Robot.WatchdogPeriod(watchdogPeriodInSeconds);
    QSBWatchdogPeriod->setValue(cmnInternalTo_ms(watchdogPeriodInSeconds));
}

////------------ Private Methods ----------------
void mtsRobotIO1394QtWidget::SetupCisstInterface(void)
{
    // Required Interface
    mtsInterfaceRequired * robotInterface = AddInterfaceRequired("Robot");
    if (robotInterface) {
        robotInterface->AddFunction("GetSerialNumber", Robot.GetSerialNumber);
        robotInterface->AddFunction("GetPeriodStatistics", Robot.GetPeriodStatistics);
        robotInterface->AddFunction("IsValid", Robot.IsValid);
        robotInterface->AddFunction("GetWatchdogPeriod", Robot.WatchdogPeriod);
        robotInterface->AddFunction("EnablePower", Robot.EnablePower);
        robotInterface->AddFunction("DisablePower", Robot.DisablePower);

        robotInterface->AddFunction("GetPosition", Robot.GetPosition);
        robotInterface->AddFunction("GetVelocity", Robot.GetVelocity);
        robotInterface->AddFunction("GetAnalogInputVolts", Robot.GetAnalogInputVolts);
        robotInterface->AddFunction("GetAnalogInputPosSI", Robot.GetAnalogInputPosSI);
        robotInterface->AddFunction("GetActuatorRequestedCurrent", Robot.GetActuatorRequestedCurrent);
        robotInterface->AddFunction("GetActuatorFeedbackCurrent", Robot.GetActuatorFeedbackCurrent);
        robotInterface->AddFunction("GetActuatorCurrentMax", Robot.GetActuatorCurrentMax);
        robotInterface->AddFunction("GetActuatorAmpTemperature", Robot.GetActuatorAmpTemperature);
        robotInterface->AddFunction("GetJointType", Robot.GetJointType);
        robotInterface->AddFunction("GetPowerStatus", Robot.GetPowerStatus);
        robotInterface->AddFunction("GetSafetyRelay", Robot.GetSafetyRelay);

        robotInterface->AddFunction("SetBrakeAmpEnable", Robot.SetBrakeAmpEnable);
        robotInterface->AddFunction("GetBrakeAmpStatus", Robot.GetBrakeAmpStatus);
        robotInterface->AddFunction("GetBrakeRequestedCurrent", Robot.GetBrakeRequestedCurrent);
        robotInterface->AddFunction("GetBrakeFeedbackCurrent", Robot.GetBrakeFeedbackCurrent);
        robotInterface->AddFunction("GetBrakeAmpTemperature", Robot.GetBrakeAmpTemperature);
        robotInterface->AddFunction("BrakeRelease", Robot.BrakeRelease);
        robotInterface->AddFunction("BrakeEngage", Robot.BrakeEngage);

        robotInterface->AddFunction("SetActuatorCurrent", Robot.SetActuatorCurrent);
        robotInterface->AddFunction("SetEncoderPosition", Robot.SetEncoderPosition);
        robotInterface->AddFunction("SetWatchdogPeriod", Robot.SetWatchdogPeriod);

        robotInterface->AddFunction("BiasEncoder", Robot.BiasEncoder);
        robotInterface->AddFunction("UsePotsForSafetyCheck", Robot.UsePotsForSafetyCheck);

        // make sure the events are queued
        robotInterface->AddEventHandlerWrite(&mtsRobotIO1394QtWidget::PowerStatusEventHandler,
                                             this, "PowerStatus");
        robotInterface->AddEventHandlerWrite(&mtsRobotIO1394QtWidget::WatchdogStatusEventHandler,
                                             this, "WatchdogStatus");
        robotInterface->AddEventHandlerWrite(&mtsRobotIO1394QtWidget::UsePotsForSafetyCheckEventHandler,
                                             this, "UsePotsForSafetyCheck");
    }

    mtsInterfaceRequired * actuatorInterface = AddInterfaceRequired("RobotActuators");
    if (actuatorInterface) {
        actuatorInterface->AddFunction("GetPositionActuator", Actuators.GetPositionActuator);

        actuatorInterface->AddFunction("EnableBoardsPower", Actuators.EnableBoardsPower);
        actuatorInterface->AddFunction("DisableBoardsPower", Actuators.DisableBoardsPower);
        actuatorInterface->AddFunction("SetAmpEnable", Actuators.SetAmpEnable);
        actuatorInterface->AddFunction("GetAmpEnable", Actuators.GetAmpEnable);
        actuatorInterface->AddFunction("GetAmpStatus", Actuators.GetAmpStatus);
    }
}



void mtsRobotIO1394QtWidget::setupUi(void)
{
    QFont font;
    font.setBold(true);

    // Power commands
    QVBoxLayout * powerLayout = new QVBoxLayout;
    powerLayout->setContentsMargins(2, 2, 2, 2);
    QFrame * powerFrame = new QFrame;
    QLabel * powerTitle = new QLabel("Power");
    powerTitle->setFont(font);
    powerTitle->setAlignment(Qt::AlignCenter);
    powerLayout->addWidget(powerTitle);
    QCBEnableAll = new QCheckBox("Enable all");
    powerLayout->addWidget(QCBEnableAll);
    QCBEnableAmps = new QCheckBox("Enable boards");
    powerLayout->addWidget(QCBEnableAmps);
    powerLayout->addSpacing(5);
    QLAmpStatus = new QLabel("Actuators ON");
    QLAmpStatus->setAlignment(Qt::AlignCenter);
    powerLayout->addWidget(QLAmpStatus);
    QLPowerStatus = new QLabel("Boards ON");
    QLPowerStatus->setAlignment(Qt::AlignCenter);
    powerLayout->addWidget(QLPowerStatus);
    powerLayout->addStretch();
    powerFrame->setLayout(powerLayout);
    powerFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // watchdog commands
    QVBoxLayout * watchdogLayout = new QVBoxLayout;
    watchdogLayout->setContentsMargins(2, 2, 2, 2);
    QFrame * watchdogFrame = new QFrame;
    QLabel * watchdogTitle = new QLabel("Watchdog");
    watchdogTitle->setFont(font);
    watchdogTitle->setAlignment(Qt::AlignCenter);
    watchdogLayout->addWidget(watchdogTitle);
    QHBoxLayout * watchdogSetLayout = new QHBoxLayout;
    {
        QLabel * wdogLabel = new QLabel("Period (ms)");
        QSBWatchdogPeriod = new QDoubleSpinBox;
        QSBWatchdogPeriod->setMaximum(340.0); // max wdog_period = 340 ms
        QSBWatchdogPeriod->setMinimum(0.0);
        QSBWatchdogPeriod->setSingleStep(0.05);
        QSBWatchdogPeriod->setValue(cmnInternalTo_ms(WatchdogPeriodInSeconds));
        watchdogSetLayout->addWidget(wdogLabel);
        watchdogSetLayout->addWidget(QSBWatchdogPeriod);
    }
    watchdogLayout->addLayout(watchdogSetLayout);
    QLSafetyRelay = new QLabel("Safety relay ON");
    QLSafetyRelay->setAlignment(Qt::AlignCenter);
    watchdogLayout->addWidget(QLSafetyRelay);
    QLWatchdog = new QLabel("Timeout FALSE");
    QLWatchdog->setAlignment(Qt::AlignCenter);
    QLWatchdog->setStyleSheet("QLabel { background-color: green }");
    watchdogLayout->addWidget(QLWatchdog);
    QLWatchdogLastTimeout = new QLabel("Last timeout: n/a");
    QLWatchdogLastTimeout->setAlignment(Qt::AlignCenter);
    watchdogLayout->addWidget(QLWatchdogLastTimeout);
    watchdogLayout->addStretch();
    watchdogFrame->setLayout(watchdogLayout);
    watchdogFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Encoder commands
    QVBoxLayout * encoderLayout = new QVBoxLayout;
    encoderLayout->setContentsMargins(2, 2, 2, 2);
    QFrame * encoderFrame = new QFrame;
    QLabel * encoderTitle = new QLabel("Encoders");
    encoderTitle->setFont(font);
    encoderTitle->setAlignment(Qt::AlignCenter);
    encoderLayout->addWidget(encoderTitle);
    QPBResetEncAll = new QPushButton("Reset all");
    encoderLayout->addWidget(QPBResetEncAll);
    QPBBiasEncAll = new QPushButton("Bias from potentiometers");
    encoderLayout->addWidget(QPBBiasEncAll);
    QCBUsePotsForSafetyCheck = new QCheckBox("Use pot/encoder check");
    encoderLayout->addWidget(QCBUsePotsForSafetyCheck);
    encoderLayout->addStretch();
    encoderFrame->setLayout(encoderLayout);
    encoderFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Current comands
    QVBoxLayout * currentLayout = new QVBoxLayout;
    currentLayout->setContentsMargins(2, 2, 2, 2);
    QFrame * currentFrame = new QFrame;
    QLabel * currentTitle = new QLabel("Current");
    currentTitle->setFont(font);
    currentTitle->setAlignment(Qt::AlignCenter);
    currentLayout->addWidget(currentTitle);
    QCBEnableDirectControl = new QCheckBox("Direct control");
    currentLayout->addWidget(QCBEnableDirectControl);
    currentLayout->addStretch();
    QLabel * serialTitle = new QLabel("Serial Number");
    serialTitle->setFont(font);
    serialTitle->setAlignment(Qt::AlignCenter);
    currentLayout->addWidget(serialTitle);
    QLSerialNumber = new QLabel("-----");
    QLSerialNumber->setAlignment(Qt::AlignCenter);
    currentLayout->addWidget(QLSerialNumber);
    currentLayout->addStretch();
    currentFrame->setLayout(currentLayout);
    currentFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Timing
    QVBoxLayout * timingLayout = new QVBoxLayout;
    timingLayout->setContentsMargins(2, 2, 2, 2);
    QFrame * timingFrame = new QFrame;
    QLabel * timingTitle = new QLabel("Timing");
    timingTitle->setFont(font);
    timingTitle->setAlignment(Qt::AlignCenter);
    timingLayout->addWidget(timingTitle);
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    timingLayout->addWidget(QMIntervalStatistics);
    timingLayout->addStretch();
    timingFrame->setLayout(timingLayout);
    timingFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Commands layout
    QHBoxLayout * commandLayout = new QHBoxLayout;
    commandLayout->setContentsMargins(2, 2, 2, 2);
    commandLayout->addWidget(powerFrame);
    commandLayout->addWidget(watchdogFrame);
    commandLayout->addWidget(encoderFrame);
    commandLayout->addWidget(currentFrame);
    commandLayout->addWidget(timingFrame);

    // Feedbacks Label
    QGridLayout * gridLayout = new QGridLayout;
    gridLayout->setContentsMargins(2, 2, 2, 2);
    gridLayout->setSpacing(1);
    int row = 0;

    if (NumberOfActuators != 0) {

        vctBoolVec defaultEnable(NumberOfActuators, false);
        vctDoubleVec defaultCurrent(NumberOfActuators, 0.0);

        gridLayout->addWidget(new QLabel("Actuator power"), row, 0);
        QVWActuatorCurrentEnableEach = new vctQtWidgetDynamicVectorBoolWrite();
        QVWActuatorCurrentEnableEach->SetValue(defaultEnable);
        gridLayout->addWidget(QVWActuatorCurrentEnableEach, row, 1);
        row++;

        gridLayout->addWidget(new QLabel("Desired current (mA)"), row, 0);
        QVWActuatorCurrentSpinBox = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
        QVWActuatorCurrentSpinBox->SetValue(defaultCurrent);
        gridLayout->addWidget(QVWActuatorCurrentSpinBox, row, 1);
        row++;

        QPBResetCurrentAll = new QPushButton("Reset current");
        gridLayout->addWidget(QPBResetCurrentAll, row, 0);
        QVWActuatorCurrentSlider = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SLIDER_WIDGET);
        QVWActuatorCurrentSlider->SetValue(defaultCurrent);
        gridLayout->addWidget(QVWActuatorCurrentSlider, row, 1);
        row++;

        gridLayout->addWidget(new QLabel("Current feedback (mA)"), row, 0);
        QVRActuatorCurrentFeedback = new vctQtWidgetDynamicVectorDoubleRead();
        gridLayout->addWidget(QVRActuatorCurrentFeedback, row, 1);
        row++;

        gridLayout->addWidget(new QLabel("Joint position (deg)"), row, 0);
        QVRJointPosition = new vctQtWidgetDynamicVectorDoubleRead();
        gridLayout->addWidget(QVRJointPosition, row, 1);
        row++;

        gridLayout->addWidget(new QLabel("Actuator position (deg)"), row, 0);
        QVRActuatorPosition = new vctQtWidgetDynamicVectorDoubleRead();
        gridLayout->addWidget(QVRActuatorPosition, row, 1);
        row++;

        gridLayout->addWidget(new QLabel("Actuator velocity (deg/s)"), row, 0);
        QVRActuatorVelocity = new vctQtWidgetDynamicVectorDoubleRead();
        gridLayout->addWidget(QVRActuatorVelocity, row, 1);
        row++;

        gridLayout->addWidget(new QLabel("Analog inputs (V)"), row, 0);
        QVRPotVolts = new vctQtWidgetDynamicVectorDoubleRead();
        gridLayout->addWidget(QVRPotVolts, row, 1);
        row++;

        gridLayout->addWidget(new QLabel("Potentiometers (deg)"), row, 0);
        QVRPotPosition = new vctQtWidgetDynamicVectorDoubleRead();
        gridLayout->addWidget(QVRPotPosition, row, 1);
        row++;

        gridLayout->addWidget(new QLabel("Amp temperature (C)"), row, 0);
        QVRActuatorAmpTemperature = new vctQtWidgetDynamicVectorDoubleRead();
        gridLayout->addWidget(QVRActuatorAmpTemperature, row, 1);
        row++;
    }

    if (NumberOfBrakes != 0) {

        vctBoolVec defaultEnable(NumberOfBrakes, false);

        gridLayout->addWidget(new QLabel("Brakes"), row, 0);
        QHBoxLayout * brakeButtonsLayout = new QHBoxLayout();
        QPBBrakeRelease =  new QPushButton("Release");
        brakeButtonsLayout->addWidget(QPBBrakeRelease);
        QPBBrakeEngage =  new QPushButton("Engage");
        brakeButtonsLayout->addWidget(QPBBrakeEngage);
        gridLayout->addLayout(brakeButtonsLayout, row, 1);
        row++;

        gridLayout->addWidget(new QLabel("Brake power"), row, 0);
        QVWBrakeCurrentEnableEach = new vctQtWidgetDynamicVectorBoolWrite();
        QVWBrakeCurrentEnableEach->SetValue(defaultEnable);
        gridLayout->addWidget(QVWBrakeCurrentEnableEach, row, 1);
        row++;

        gridLayout->addWidget(new QLabel("Current desired (mA)"), row, 0);
        QVRBrakeCurrentCommand = new vctQtWidgetDynamicVectorDoubleRead();
        gridLayout->addWidget(QVRBrakeCurrentCommand, row, 1);
        row++;

        gridLayout->addWidget(new QLabel("Current feedback (mA)"), row, 0);
        QVRBrakeCurrentFeedback = new vctQtWidgetDynamicVectorDoubleRead();
        gridLayout->addWidget(QVRBrakeCurrentFeedback, row, 1);
        row++;

        gridLayout->addWidget(new QLabel("Amp temperature (C)"), row, 0);
        QVRBrakeAmpTemperature = new vctQtWidgetDynamicVectorDoubleRead();
        gridLayout->addWidget(QVRBrakeAmpTemperature, row, 1);
        row++;
    }

    // main layout
    QVBoxLayout * mainLayout = new QVBoxLayout;
    mainLayout->setContentsMargins(2, 2, 2, 2);
    mainLayout->addLayout(commandLayout);
    mainLayout->addLayout(gridLayout);
    mainLayout->addStretch();

    setLayout(mainLayout);

    setWindowTitle(QString(this->GetName().c_str()));
    resize(sizeHint());

    // connect signals & slots
    connect(QCBEnableAmps, SIGNAL(toggled(bool)),
            this, SLOT(SlotEnableAmps(bool)));
    connect(QCBEnableAll, SIGNAL(toggled(bool)),
            this, SLOT(SlotEnableAll(bool)));
    connect(QCBEnableDirectControl, SIGNAL(toggled(bool)),
            this, SLOT(SlotEnableDirectControl(bool)));
    connect(QPBResetCurrentAll, SIGNAL(clicked()),
            this, SLOT(SlotResetCurrentAll()));
    connect(QPBResetEncAll, SIGNAL(clicked()),
            this, SLOT(SlotResetEncodersAll()));
    connect(QPBBiasEncAll, SIGNAL(clicked()),
            this, SLOT(SlotBiasEncodersAll()));
    connect(QCBUsePotsForSafetyCheck, SIGNAL(toggled(bool)),
            this, SLOT(SlotUsePotsForSafetyCheck(bool)));
    connect(QSBWatchdogPeriod, SIGNAL(valueChanged(double)),
            this, SLOT(SlotWatchdogPeriod(double)));
    connect(QVWActuatorCurrentEnableEach, SIGNAL(valueChanged()),
            this, SLOT(SlotActuatorAmpEnable()));
    connect(QVWActuatorCurrentSpinBox, SIGNAL(valueChanged()),
            this, SLOT(SlotActuatorCurrentValueChanged()));
    connect(QVWActuatorCurrentSlider, SIGNAL(valueChanged()),
            this, SLOT(SlotSliderActuatorCurrentValueChanged()));

    if (NumberOfBrakes != 0) {
        connect(QPBBrakeRelease, SIGNAL(clicked()),
                this, SLOT(SlotBrakeRelease()));
        connect(QPBBrakeEngage, SIGNAL(clicked()),
                this, SLOT(SlotBrakeEngage()));
        connect(QVWBrakeCurrentEnableEach, SIGNAL(valueChanged()),
                this, SLOT(SlotBrakeAmpEnable()));
    }

    // connect cisstMultiTask events
    connect(this, SIGNAL(SignalPowerStatus(bool)),
            this, SLOT(SlotPowerStatusEvent(bool)));
    connect(this, SIGNAL(SignalWatchdogStatus(bool)),
            this, SLOT(SlotWatchdogStatusEvent(bool)));
    connect(this, SIGNAL(SignalUsePotsForSafetyCheck(bool)),
            this, SLOT(SlotUsePotsForSafetyCheckEvent(bool)));

    // set initial value
    QCBEnableAmps->setChecked(false);
    QCBEnableAll->setChecked(false);
    QCBEnableDirectControl->setChecked(DirectControl);
    SlotEnableDirectControl(DirectControl);
}


void mtsRobotIO1394QtWidget::UpdateRobotInfo(void)
{
    // actuator amplifier status
    bool ampStatusGood = ActuatorAmpStatus.All();
    if (NumberOfActuators != 0) {
        QVWActuatorCurrentEnableEach->SetValue(ActuatorAmpStatus);
    }
    if (ampStatusGood) {
        QLAmpStatus->setText("Actuators ON");
        QLAmpStatus->setStyleSheet("QLabel { background-color: green }");
    } else {
        QLAmpStatus->setText("Actuators OFF");
        QLAmpStatus->setStyleSheet("QLabel { background-color: red }");
    }

    // brake amplifier status
    if (NumberOfBrakes != 0) {
        QVWBrakeCurrentEnableEach->SetValue(BrakeAmpStatus);
    }

    // power status
    if (PowerStatus) {
        QLPowerStatus->setText("Boards ON");
        QLPowerStatus->setStyleSheet("QLabel { background-color: green }");
    } else {
        QLPowerStatus->setText("Boards OFF");
        QLPowerStatus->setStyleSheet("QLabel { background-color: red }");
    }

    // update check box to enable/disable based on current state
    QCBEnableAmps->blockSignals(true); {
        QCBEnableAmps->setChecked(ampStatusGood);
    } QCBEnableAmps->blockSignals(false);
    QCBEnableAll->blockSignals(true); {
        bool status = ampStatusGood;
        if (NumberOfActuators != 0) {
            status = status && ActuatorAmpEnable.All();
        }
        if (NumberOfBrakes != 0) {
            status = status && BrakeAmpEnable.All();
        }
        QCBEnableAll->setChecked(status);
    } QCBEnableAll->blockSignals(false);

    // safety Relay
    if (SafetyRelay) {
        QLSafetyRelay->setText("Safety Relay ON");
        QLSafetyRelay->setStyleSheet("QLabel { background-color: green }");
    } else {
        QLSafetyRelay->setText("Safety Relay OFF");
        QLSafetyRelay->setStyleSheet("QLabel { background-color: red }");
    }
}

void mtsRobotIO1394QtWidget::PowerStatusEventHandler(const bool & status)
{
    emit SignalPowerStatus(status);
}

void mtsRobotIO1394QtWidget::SlotPowerStatusEvent(bool status)
{
    if (status == false) {
        QCBEnableAll->setChecked(false);
    }
}

void mtsRobotIO1394QtWidget::WatchdogStatusEventHandler(const bool & status)
{
    emit SignalWatchdogStatus(status);
}

void mtsRobotIO1394QtWidget::SlotWatchdogStatusEvent(bool status)
{
    if (status) {
        QLWatchdog->setText("Timeout TRUE");
        QLWatchdog->setStyleSheet("QLabel { background-color: red }");
        QLWatchdogLastTimeout->setText(QString("Last timeout: " + QTime::currentTime().toString("hh:mm:ss")));
    } else {
        QLWatchdog->setText("Timeout FALSE");
        QLWatchdog->setStyleSheet("QLabel { background-color: green }");
    }
}

void mtsRobotIO1394QtWidget::UsePotsForSafetyCheckEventHandler(const bool & status)
{
    emit SignalUsePotsForSafetyCheck(status);
}

void mtsRobotIO1394QtWidget::SlotUsePotsForSafetyCheckEvent(bool status)
{
    QCBUsePotsForSafetyCheck->setChecked(status);
}

void mtsRobotIO1394QtWidget::SlotUsePotsForSafetyCheck(bool checked)
{
    mtsExecutionResult result = Robot.UsePotsForSafetyCheck(checked);
    if (!result.IsOK()) {
        CMN_LOG_CLASS_RUN_WARNING << "SlotUsePotsForSafetyCheck: command failed \"" << result << "\"" << std::endl;
    }
}
