/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-16

  (C) Copyright 2013-2024 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QSignalMapper>

// project include
#include <sawRobotIO1394/mtsRobot1394QtWidget.h>

#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstVector/vctPlot2DOpenGLQtWidget.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsRobot1394QtWidget, mtsComponent, mtsComponentConstructorNameAndUInt)


mtsRobot1394QtWidget::mtsRobot1394QtWidget(const std::string & componentName,
                                           unsigned int numberOfActuators,
                                           unsigned int numberOfBrakes,
                                           double periodInSeconds):
    mtsComponent(componentName),
    DirectControl(false),
    PlotMode(false),
    TimerPeriodInMilliseconds(periodInSeconds * 1000), // Qt timers are in milliseconds
    SerialNumber(""),
    NumberOfActuators(numberOfActuators),
    NumberOfBrakes(numberOfBrakes)
{
    WatchdogPeriodInSeconds = 300.0 * cmn_ms;
    WatchdogCounter = 0;
    Init();
}

mtsRobot1394QtWidget::mtsRobot1394QtWidget(const mtsComponentConstructorNameAndUInt &arg):
    mtsComponent(arg.Name),
    SerialNumber(""),
    NumberOfActuators(arg.Arg)
{
    Init();
}

void mtsRobot1394QtWidget::Init(void)
{
    DummyValueWhenNotConnected = 0;
    LastEnableState.SetSize(NumberOfActuators);
    LastEnableState.SetAll(false);

    UnitFactor.SetSize(NumberOfActuators);
    PotentiometersVolts.SetSize(NumberOfActuators);
    PotentiometersPosition.SetSize(NumberOfActuators);
    ActuatorFeedbackCurrent.SetSize(NumberOfActuators);
    ActuatorFeedbackCurrent.Zeros();
    ActuatorRequestedCurrent.SetSize(NumberOfActuators);
    ActuatorRequestedCurrent.Zeros();
    ActuatorAmpTemperature.SetSize(NumberOfActuators);

    StartTime = osaGetTime();

    PlotIndex = 0;

    SetupCisstInterface();
    setupUi();

    startTimer(TimerPeriodInMilliseconds); // ms
}

void mtsRobot1394QtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsRobot1394QtWidget::Startup(void)
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

    prmConfigurationJoint jointConfig;
    result = Robot.configuration_js(jointConfig);
    if (!result) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: Robot interface isn't connected properly, unable to get joint configuration.  Function call returned: "
                                 << result << std::endl;
        UnitFactor.SetAll(0.0);
    } else {
        // set unitFactor
        for (size_t i = 0; i < this->NumberOfActuators; i++ ) {
            switch (jointConfig.Type().at(i)) {
            case CMN_JOINT_REVOLUTE:
                UnitFactor[i] = cmn180_PI;
                break;
            case CMN_JOINT_PRISMATIC:
                UnitFactor[i] = 1.0 / cmn_mm; // convert internal values to mm
                break;
            case CMN_JOINT_INACTIVE:
            case CMN_JOINT_UNDEFINED:
                break;
            default:
                cmnThrow("mtsRobot1394QtWidget: unknown joint type");
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
    QLSerialNumber->setText(QString(SerialNumber.c_str()));

    if (!parent()) {
        show();
    }
}

void mtsRobot1394QtWidget::Cleanup(void)
{
    this->hide();
    Robot.PowerOffSequence(true);
}

void mtsRobot1394QtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsRobot1394QtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsRobot1394QtWidget::SlotSafetyRelay(bool toggle)
{
    Robot.WriteSafetyRelay(toggle);
}

void mtsRobot1394QtWidget::SlotPowerEnable(bool toggle)
{
    // send to controller first
    Robot.WritePowerEnable(toggle);

    // set all current to 0
    SlotResetCurrentAll();
}

void mtsRobot1394QtWidget::SlotEnableAll(bool toggle)
{
    // send to controller first
    if (toggle) {
        Robot.PowerOnSequence();
    } else {
        Robot.PowerOffSequence(true);
    }

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

void mtsRobot1394QtWidget::SlotEnableDirectControl(bool toggle)
{
    if (toggle) {
        int answer = QMessageBox::warning(this, tr("mtsRobot1394QtWidget"),
                                          tr("In direct control mode you can potentially harm your robot.\nAre you sure you want to continue?"),
                                          QMessageBox::No | QMessageBox::Yes);
        if (answer == QMessageBox::No) {
            toggle = false;
        }
    }
    QCBEnableDirectControl->setChecked(toggle);
    DirectControl = toggle;
    // update widgets
    QCBSafetyRelay->setEnabled(toggle);
    QCBEnableAll->setEnabled(toggle);
    QCBPowerEnable->setEnabled(toggle);
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

void mtsRobot1394QtWidget::SlotEnablePlotMode(bool toggle)
{
    PlotMode = toggle;
    if (PlotMode) {
        QWPlot->show();
        QWText->hide();
        QPlot->resize(QPlot->sizeHint());
        delete MainLayout->takeAt(MainLayout->count() - 1);
    } else {
        QWPlot->hide();
        QWText->show();
        // add stretch at end of main layout
        MainLayout->addStretch();
    }
}

void mtsRobot1394QtWidget::SlotResetCurrentAll(void)
{
    // send to controller first
    vctDoubleVec cmdCurA(NumberOfActuators);
    cmdCurA.SetAll(0.0);
    Robot.SetActuatorCurrent(cmdCurA);
    // update GUI
    QVWActuatorCurrentSpinBox->SetValue(cmdCurA);
    QVWActuatorCurrentSlider->SetValue(cmdCurA);
}

void mtsRobot1394QtWidget::SlotActuatorAmpEnable(void)
{
    ActuatorAmpEnable.SetSize(NumberOfActuators);
    QVWActuatorCurrentEnableEach->GetValue(ActuatorAmpEnable);
    Robot.SetActuatorAmpEnable(ActuatorAmpEnable);
}

void mtsRobot1394QtWidget::SlotBrakeAmpEnable(void)
{
    BrakeAmpEnable.SetSize(NumberOfBrakes);
    QVWBrakeCurrentEnableEach->GetValue(BrakeAmpEnable);
    Robot.SetBrakeAmpEnable(BrakeAmpEnable);
}

void mtsRobot1394QtWidget::SlotActuatorCurrentValueChanged()
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

void mtsRobot1394QtWidget::SlotSliderActuatorCurrentValueChanged()
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

void mtsRobot1394QtWidget::SlotResetEncodersAll()
{
    int answer = QMessageBox::warning(this, tr("mtsRobot1394QtWidget"),
                                      tr("Don't reset the encoders if there is an active controller (e.g. PID).\nAre you sure you want to continue?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::No) {
        return;
    }

    vctDoubleVec newEncoderValues(NumberOfActuators);
    newEncoderValues.SetAll(0.0);
    mtsExecutionResult result = Robot.SetEncoderPosition(newEncoderValues);
    if (!result.IsOK()) {
        CMN_LOG_CLASS_RUN_WARNING << "SlotResetEncodersAll: command failed \"" << result << "\"" << std::endl;
     }
}

void mtsRobot1394QtWidget::SlotBiasEncodersAll()
{
    int answer = QMessageBox::warning(this, tr("mtsRobot1394QtWidget"),
                                      tr("Don't reset the encoders if there is an active controller (e.g. PID).\nAre you sure you want to continue?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::No) {
        return;
    }

    mtsExecutionResult result = Robot.BiasEncoder(1000);
    if (!result.IsOK()) {
        CMN_LOG_CLASS_RUN_WARNING << "SlotBiasEncodersAll: command failed \"" << result << "\"" << std::endl;
    }
}

void mtsRobot1394QtWidget::SlotWatchdogPeriod(void)
{
    double period_ms = QSBWatchdogPeriod->value();
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

void mtsRobot1394QtWidget::SlotBrakeEngage(void)
{
    Robot.BrakeEngage();
}

void mtsRobot1394QtWidget::SlotBrakeRelease(void)
{
    Robot.BrakeRelease();
}

void mtsRobot1394QtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    ProcessQueuedEvents();

    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    bool isValid;
    Robot.IsValid(isValid);
    if (isValid) {
        Robot.period_statistics(IntervalStatistics);
        Robot.GetSafetyRelay(SafetyRelay);
        Robot.GetFullyPowered(FullyPowered);
        Robot.GetPowerEnable(PowerEnable);
        Robot.GetSafetyRelayStatus(SafetyRelayStatus);
        if (NumberOfActuators != 0) {
            Robot.GetActuatorAmpEnable(ActuatorAmpEnable);
            Robot.GetActuatorAmpStatus(ActuatorAmpStatus);
            Robot.measured_js(ActuatorStateJoint);
            ActuatorStateJoint.Position().ElementwiseMultiply(UnitFactor); // to degrees or mm
            ActuatorStateJoint.Velocity().ElementwiseMultiply(UnitFactor); // to degrees or mm
            Robot.GetAnalogInputVolts(PotentiometersVolts);
            Robot.pot_measured_js(PotentiometersPosition);
            PotentiometersPosition.Position().ElementwiseMultiply(UnitFactor); // to degrees or mm
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
        ActuatorStateJoint.Position().SetAll(DummyValueWhenNotConnected);
        ActuatorStateJoint.Velocity().SetAll(DummyValueWhenNotConnected);
        PotentiometersVolts.SetAll(DummyValueWhenNotConnected);
        PotentiometersPosition.Position().SetAll(DummyValueWhenNotConnected);
        ActuatorFeedbackCurrent.SetAll(DummyValueWhenNotConnected);
        ActuatorAmpTemperature.SetAll(DummyValueWhenNotConnected);
    }

    DummyValueWhenNotConnected += 0.1;

    // display requested current when we are not trying to set it using GUI
    if (isValid && !DirectControl) {
        Robot.GetActuatorRequestedCurrent(ActuatorRequestedCurrent);
        ActuatorRequestedCurrent.Multiply(1000.0); // got A, need mA for display
        QVWActuatorCurrentSpinBox->SetValue(ActuatorRequestedCurrent);
        QVWActuatorCurrentSlider->SetValue(ActuatorRequestedCurrent);
    }

    QMIntervalStatistics->SetValue(IntervalStatistics);
    if (PlotMode) {
        Robot.measured_js(ActuatorStateJoint);
        PlotSignals[0]->AppendPoint(vct2(ActuatorStateJoint.Timestamp(),
                                         ActuatorStateJoint.Position()[PlotIndex] * UnitFactor[PlotIndex]));
        Robot.pot_measured_js(PotentiometersPosition);
        PlotSignals[1]->AppendPoint(vct2(PotentiometersPosition.Timestamp(),
                                         PotentiometersPosition.Position()[PlotIndex] * UnitFactor[PlotIndex]));
        QPlot->update();
    } else {
        if (NumberOfActuators != 0) {
            QVRActuatorPosition->SetValue(ActuatorStateJoint.Position());
            QVRActuatorVelocity->SetValue(ActuatorStateJoint.Velocity());
            QVRPotVolts->SetValue(PotentiometersVolts);
            QVRPotPosition->SetValue(PotentiometersPosition.Position());
            QVRActuatorCurrentFeedback->SetValue(ActuatorFeedbackCurrent);
            QVRActuatorAmpTemperature->SetValue(ActuatorAmpTemperature);
        }
        if (NumberOfBrakes != 0) {
            QVRBrakeCurrentCommand->SetValue(BrakeRequestedCurrent);
            QVRBrakeCurrentFeedback->SetValue(BrakeFeedbackCurrent);
            QVRBrakeAmpTemperature->SetValue(BrakeAmpTemperature);
        }
    }

    // refresh watchdog period if needed
    double watchdogPeriodInSeconds;
    Robot.WatchdogPeriod(watchdogPeriodInSeconds);
    if (watchdogPeriodInSeconds != WatchdogPeriodInSeconds) {
        WatchdogPeriodInSeconds = watchdogPeriodInSeconds;
        QSBWatchdogPeriod->setValue(cmnInternalTo_ms(watchdogPeriodInSeconds));
    }

    UpdateRobotInfo();
}

////------------ Private Methods ----------------
void mtsRobot1394QtWidget::SetupCisstInterface(void)
{
    // Required Interface
    mtsInterfaceRequired * robotInterface = AddInterfaceRequired("Robot");
    if (robotInterface) {
        robotInterface->AddFunction("GetSerialNumber", Robot.GetSerialNumber);
        robotInterface->AddFunction("period_statistics", Robot.period_statistics);
        robotInterface->AddFunction("IsValid", Robot.IsValid);

        robotInterface->AddFunction("WriteSafetyRelay", Robot.WriteSafetyRelay);
        robotInterface->AddFunction("GetSafetyRelay", Robot.GetSafetyRelay);
        robotInterface->AddFunction("GetSafetyRelayStatus", Robot.GetSafetyRelayStatus);

        robotInterface->AddFunction("GetWatchdogPeriod", Robot.WatchdogPeriod);
        robotInterface->AddFunction("SetWatchdogPeriod", Robot.SetWatchdogPeriod);

        robotInterface->AddFunction("PowerOnSequence", Robot.PowerOnSequence);
        robotInterface->AddFunction("PowerOffSequence", Robot.PowerOffSequence);
        robotInterface->AddFunction("GetPowerEnable", Robot.GetPowerEnable);
        robotInterface->AddFunction("GetFullyPowered", Robot.GetFullyPowered);

        robotInterface->AddFunction("measured_js", Robot.measured_js);
        robotInterface->AddFunction("GetAnalogInputVolts", Robot.GetAnalogInputVolts);
        robotInterface->AddFunction("pot/measured_js", Robot.pot_measured_js);
        robotInterface->AddFunction("GetActuatorRequestedCurrent", Robot.GetActuatorRequestedCurrent);
        robotInterface->AddFunction("GetActuatorFeedbackCurrent", Robot.GetActuatorFeedbackCurrent);
        robotInterface->AddFunction("GetActuatorCurrentMax", Robot.GetActuatorCurrentMax);
        robotInterface->AddFunction("GetActuatorAmpTemperature", Robot.GetActuatorAmpTemperature);
        robotInterface->AddFunction("configuration_js", Robot.configuration_js);

        robotInterface->AddFunction("SetBrakeAmpEnable", Robot.SetBrakeAmpEnable);
        robotInterface->AddFunction("GetBrakeAmpStatus", Robot.GetBrakeAmpStatus);
        robotInterface->AddFunction("GetBrakeRequestedCurrent", Robot.GetBrakeRequestedCurrent);
        robotInterface->AddFunction("GetBrakeFeedbackCurrent", Robot.GetBrakeFeedbackCurrent);
        robotInterface->AddFunction("GetBrakeAmpTemperature", Robot.GetBrakeAmpTemperature);
        robotInterface->AddFunction("BrakeRelease", Robot.BrakeRelease);
        robotInterface->AddFunction("BrakeEngage", Robot.BrakeEngage);

        robotInterface->AddFunction("SetActuatorCurrent", Robot.SetActuatorCurrent);
        robotInterface->AddFunction("SetEncoderPosition", Robot.SetEncoderPosition);

        robotInterface->AddFunction("BiasEncoder", Robot.BiasEncoder);
        robotInterface->AddFunction("UsePotsForSafetyCheck", Robot.UsePotsForSafetyCheck);

        // make sure the events are queued
        robotInterface->AddEventHandlerWrite(&mtsRobot1394QtWidget::FullyPoweredEventHandler,
                                             this, "FullyPowered");
        robotInterface->AddEventHandlerWrite(&mtsRobot1394QtWidget::WatchdogTimeoutStatusEventHandler,
                                             this, "WatchdogTimeoutStatus");
        robotInterface->AddEventHandlerWrite(&mtsRobot1394QtWidget::UsePotsForSafetyCheckEventHandler,
                                             this, "UsePotsForSafetyCheck");

        robotInterface->AddFunction("WritePowerEnable", Robot.WritePowerEnable);
        robotInterface->AddFunction("SetActuatorAmpEnable", Robot.SetActuatorAmpEnable);
        robotInterface->AddFunction("GetActuatorAmpEnable", Robot.GetActuatorAmpEnable);
        robotInterface->AddFunction("GetActuatorAmpStatus", Robot.GetActuatorAmpStatus);
    }
}



void mtsRobot1394QtWidget::setupUi(void)
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
    QCBPowerEnable = new QCheckBox("Enable power");
    powerLayout->addWidget(QCBPowerEnable);
    QLAmpStatus = new QLabel("Actuators on");
    QLAmpStatus->setAlignment(Qt::AlignCenter);
    powerLayout->addWidget(QLAmpStatus);
    QLFullyPowered = new QLabel("Power on");
    QLFullyPowered->setAlignment(Qt::AlignCenter);
    powerLayout->addWidget(QLFullyPowered);
    powerLayout->addStretch();
    powerFrame->setLayout(powerLayout);
    powerFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Safety relays
    QVBoxLayout * relayLayout = new QVBoxLayout;
    relayLayout->setContentsMargins(2, 2, 2, 2);
    QFrame * relayFrame = new QFrame;
    QLabel * relayTitle = new QLabel("Relays");
    relayTitle->setFont(font);
    relayTitle->setAlignment(Qt::AlignCenter);
    relayLayout->addWidget(relayTitle);
    QCBSafetyRelay = new QCheckBox("Close");
    relayLayout->addWidget(QCBSafetyRelay);
    QLSafetyRelayStatus = new QLabel();
    QLSafetyRelayStatus->setAlignment(Qt::AlignCenter);
    relayLayout->addWidget(QLSafetyRelayStatus);
    relayLayout->addStretch();
    relayFrame->setLayout(relayLayout);
    relayFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

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
    QLWatchdog = new QLabel("Timed out");
    QLWatchdog->setAlignment(Qt::AlignCenter);
    QLWatchdog->setStyleSheet("QLabel { background-color: rgb(255, 100, 100) }");
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

    // Service commands
    QVBoxLayout * ServiceLayout = new QVBoxLayout;
    ServiceLayout->setContentsMargins(2, 2, 2, 2);
    QFrame * serviceFrame = new QFrame;
    QLabel * ServiceTitle = new QLabel("Service");
    ServiceTitle->setFont(font);
    ServiceTitle->setAlignment(Qt::AlignCenter);
    ServiceLayout->addWidget(ServiceTitle);
    QCBEnableDirectControl = new QCheckBox("Direct control");
    ServiceLayout->addWidget(QCBEnableDirectControl);
    QCBEnablePlotMode = new QCheckBox("Plot position");
    ServiceLayout->addWidget(QCBEnablePlotMode);
    ServiceLayout->addStretch();
    QLabel * serialTitle = new QLabel("Serial number");
    serialTitle->setFont(font);
    serialTitle->setAlignment(Qt::AlignCenter);
    ServiceLayout->addWidget(serialTitle);
    QLSerialNumber = new QLabel("-----");
    QLSerialNumber->setAlignment(Qt::AlignCenter);
    ServiceLayout->addWidget(QLSerialNumber);
    ServiceLayout->addStretch();
    serviceFrame->setLayout(ServiceLayout);
    serviceFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Timing
    QVBoxLayout * timingLayout = new QVBoxLayout;
    timingLayout->setContentsMargins(2, 2, 2, 2);
    QFrame * timingFrame = new QFrame;
    QLabel * timingTitle = new QLabel("Timing");
    timingTitle->setFont(font);
    timingTitle->setAlignment(Qt::AlignCenter);
    timingLayout->addWidget(timingTitle);
    QMIntervalStatistics = new mtsIntervalStatisticsQtWidget();
    timingLayout->addWidget(QMIntervalStatistics);
    timingLayout->addStretch();
    timingFrame->setLayout(timingLayout);
    timingFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    // Commands layout
    QWidget * commandWidget = new QWidget();
    QHBoxLayout * commandLayout = new QHBoxLayout(commandWidget);
    commandWidget->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed));
    commandLayout->setContentsMargins(2, 2, 2, 2);
    commandLayout->addWidget(powerFrame, 1);
    commandLayout->addWidget(relayFrame, 1);
    commandLayout->addWidget(watchdogFrame, 1);
    commandLayout->addWidget(encoderFrame, 1);
    commandLayout->addWidget(serviceFrame, 1);
    commandLayout->addWidget(timingFrame);

    // Feedbacks Label
    QWText = new QWidget();
    QGridLayout * textLayout = new QGridLayout(QWText);
    textLayout->setContentsMargins(2, 2, 2, 2);
    textLayout->setSpacing(1);
    int row = 0;

    if (NumberOfActuators != 0) {

        vctBoolVec defaultEnable(NumberOfActuators, false);
        vctDoubleVec defaultCurrent(NumberOfActuators, 0.0);

        textLayout->addWidget(new QLabel("Actuator power"), row, 0);
        QVWActuatorCurrentEnableEach = new vctQtWidgetDynamicVectorBoolWrite();
        QVWActuatorCurrentEnableEach->SetValue(defaultEnable);
        textLayout->addWidget(QVWActuatorCurrentEnableEach, row, 1);
        row++;

        textLayout->addWidget(new QLabel("Desired current (mA)"), row, 0);
        QVWActuatorCurrentSpinBox = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
        QVWActuatorCurrentSpinBox->SetValue(defaultCurrent);
        textLayout->addWidget(QVWActuatorCurrentSpinBox, row, 1);
        row++;

        QPBResetCurrentAll = new QPushButton("Reset current");
        textLayout->addWidget(QPBResetCurrentAll, row, 0);
        QVWActuatorCurrentSlider = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SLIDER_WIDGET);
        QVWActuatorCurrentSlider->SetValue(defaultCurrent);
        textLayout->addWidget(QVWActuatorCurrentSlider, row, 1);
        row++;

        textLayout->addWidget(new QLabel("Current feedback (mA)"), row, 0);
        QVRActuatorCurrentFeedback = new vctQtWidgetDynamicVectorDoubleRead();
        textLayout->addWidget(QVRActuatorCurrentFeedback, row, 1);
        row++;

        textLayout->addWidget(new QLabel("Actuator position (deg|mm)"), row, 0);
        QVRActuatorPosition = new vctQtWidgetDynamicVectorDoubleRead();
        textLayout->addWidget(QVRActuatorPosition, row, 1);
        row++;

        textLayout->addWidget(new QLabel("Actuator velocity (deg/s)"), row, 0);
        QVRActuatorVelocity = new vctQtWidgetDynamicVectorDoubleRead();
        textLayout->addWidget(QVRActuatorVelocity, row, 1);
        row++;

        textLayout->addWidget(new QLabel("Analog inputs (V)"), row, 0);
        QVRPotVolts = new vctQtWidgetDynamicVectorDoubleRead();
        textLayout->addWidget(QVRPotVolts, row, 1);
        row++;

        textLayout->addWidget(new QLabel("Potentiometer (deg|mm)"), row, 0);
        QVRPotPosition = new vctQtWidgetDynamicVectorDoubleRead();
        textLayout->addWidget(QVRPotPosition, row, 1);
        row++;

        textLayout->addWidget(new QLabel("Amp temperature (C)"), row, 0);
        QVRActuatorAmpTemperature = new vctQtWidgetDynamicVectorDoubleRead();
        textLayout->addWidget(QVRActuatorAmpTemperature, row, 1);
        row++;
    }

    if (NumberOfBrakes != 0) {

        vctBoolVec defaultEnable(NumberOfBrakes, false);

        textLayout->addWidget(new QLabel("Brakes"), row, 0);
        QHBoxLayout * brakeButtonsLayout = new QHBoxLayout();
        QPBBrakeRelease =  new QPushButton("Release");
        brakeButtonsLayout->addWidget(QPBBrakeRelease);
        QPBBrakeEngage =  new QPushButton("Engage");
        brakeButtonsLayout->addWidget(QPBBrakeEngage);
        textLayout->addLayout(brakeButtonsLayout, row, 1);
        row++;

        textLayout->addWidget(new QLabel("Brake power"), row, 0);
        QVWBrakeCurrentEnableEach = new vctQtWidgetDynamicVectorBoolWrite();
        QVWBrakeCurrentEnableEach->SetValue(defaultEnable);
        textLayout->addWidget(QVWBrakeCurrentEnableEach, row, 1);
        row++;

        textLayout->addWidget(new QLabel("Current desired (mA)"), row, 0);
        QVRBrakeCurrentCommand = new vctQtWidgetDynamicVectorDoubleRead();
        textLayout->addWidget(QVRBrakeCurrentCommand, row, 1);
        row++;

        textLayout->addWidget(new QLabel("Current feedback (mA)"), row, 0);
        QVRBrakeCurrentFeedback = new vctQtWidgetDynamicVectorDoubleRead();
        textLayout->addWidget(QVRBrakeCurrentFeedback, row, 1);
        row++;

        textLayout->addWidget(new QLabel("Amp temperature (C)"), row, 0);
        QVRBrakeAmpTemperature = new vctQtWidgetDynamicVectorDoubleRead();
        textLayout->addWidget(QVRBrakeAmpTemperature, row, 1);
        row++;
    }

    // Plot area
    QWPlot = new QWidget();
    QWPlot->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
    QHBoxLayout * plotLayout = new QHBoxLayout(QWPlot);
    plotLayout->setContentsMargins(0, 0, 0, 0);

    // left side, selector and legend
    QVBoxLayout * plotLeftLayout = new QVBoxLayout;
    plotLeftLayout->setContentsMargins(2, 2, 2, 2);
    plotLayout->addLayout(plotLeftLayout);

    // axis selection to plot
    QSBPlotIndex = new QSpinBox();
    QSBPlotIndex->setRange(0, (NumberOfActuators > 0) ? (NumberOfActuators - 1) : 0);
    plotLeftLayout->addWidget(QSBPlotIndex);
    connect(QSBPlotIndex, SIGNAL(valueChanged(int)), this, SLOT(SlotPlotIndex(int)));

    // constants
    const QColor baseColor = palette().color(QPalette::Base);
    const vct3 _colors[2] = {vct3(1.0, 0.0, 0.0),
                             vct3(0.0, 1.0, 0.0)};
    const std::string _signals[2] = {"Actuator position", "Potentiometer"};

    // mapping
    PlotSignalMapper = new QSignalMapper();

    // legends
    QLabel * label;
    QPalette palette;

    for (size_t signal = 0;
         signal < 2;
         ++signal) {
        QHBoxLayout * signalLayout = new QHBoxLayout();
        signalLayout->setContentsMargins(0, 0, 0, 0);
        plotLeftLayout->addLayout(signalLayout);
        // checkbox
        PlotCheckBoxes[signal] = new QCheckBox("");
        PlotCheckBoxes[signal]->setChecked(true);
        signalLayout->addWidget(PlotCheckBoxes[signal]);
        PlotSignalMapper->setMapping(PlotCheckBoxes[signal], signal);
        connect(PlotCheckBoxes[signal], SIGNAL(released()), PlotSignalMapper, SLOT(map()));
        // label
        label = new QLabel(_signals[signal].c_str());
        label->setAutoFillBackground(true);
        palette.setColor(QPalette::WindowText, QColor(_colors[signal].X() * 255,
                                                      _colors[signal].Y() * 255,
                                                      _colors[signal].Z() * 255));
        label->setPalette(palette);
        signalLayout->addWidget(label);
    }

    connect(PlotSignalMapper, SIGNAL(mapped(int)),
            this, SLOT(SlotPlotVisibleSignal(int)));

    plotLeftLayout->addStretch();

    // plot area
    QPlot = new vctPlot2DOpenGLQtWidget();
    QPlot->SetBackgroundColor(vct3(baseColor.redF(), baseColor.greenF(), baseColor.blueF()));
    QPlot->resize(QPlot->sizeHint());
    QPlot->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
    plotLayout->addWidget(QPlot);

    // create scale and signals
    PlotScale = QPlot->AddScale("Position");
    QPlot->SetDisplayYRangeScale(PlotScale);
    for (size_t signal = 0;
         signal < 2;
         ++signal) {
        PlotSignals[signal] = PlotScale->AddSignal(_signals[signal]);
        PlotSignals[signal]->SetColor(_colors[signal]);
        PlotVisibleSignals[signal] = true;
    }

    // main layout
    MainLayout = new QVBoxLayout;
    MainLayout->setContentsMargins(2, 2, 2, 2);
    MainLayout->addWidget(commandWidget);
    MainLayout->addWidget(QWText);
    MainLayout->addWidget(QWPlot);

    setLayout(MainLayout);

    setWindowTitle(QString(this->GetName().c_str()));
    resize(sizeHint());

    // connect signals & slots
    connect(QCBSafetyRelay, SIGNAL(toggled(bool)),
            this, SLOT(SlotSafetyRelay(bool)));
    connect(QCBPowerEnable, SIGNAL(toggled(bool)),
            this, SLOT(SlotPowerEnable(bool)));
    connect(QCBEnableAll, SIGNAL(toggled(bool)),
            this, SLOT(SlotEnableAll(bool)));
    connect(QCBEnableDirectControl, SIGNAL(toggled(bool)),
            this, SLOT(SlotEnableDirectControl(bool)));
    connect(QCBEnablePlotMode, SIGNAL(toggled(bool)),
            this, SLOT(SlotEnablePlotMode(bool)));
    connect(QPBResetCurrentAll, SIGNAL(clicked()),
            this, SLOT(SlotResetCurrentAll()));
    connect(QPBResetEncAll, SIGNAL(clicked()),
            this, SLOT(SlotResetEncodersAll()));
    connect(QPBBiasEncAll, SIGNAL(clicked()),
            this, SLOT(SlotBiasEncodersAll()));
    connect(QCBUsePotsForSafetyCheck, SIGNAL(toggled(bool)),
            this, SLOT(SlotUsePotsForSafetyCheck(bool)));
    connect(QSBWatchdogPeriod, SIGNAL(editingFinished()),
            this, SLOT(SlotWatchdogPeriod()));
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
    connect(this, SIGNAL(SignalFullyPowered(bool)),
            this, SLOT(SlotFullyPoweredEvent(bool)));
    connect(this, SIGNAL(SignalWatchdogTimeoutStatus(bool)),
            this, SLOT(SlotWatchdogTimeoutStatusEvent(bool)));
    connect(this, SIGNAL(SignalUsePotsForSafetyCheck(bool)),
            this, SLOT(SlotUsePotsForSafetyCheckEvent(bool)));

    // set initial value
    QCBSafetyRelay->setChecked(false);
    QCBPowerEnable->setChecked(false);
    QCBEnableAll->setChecked(false);
    QCBEnableDirectControl->setChecked(DirectControl);
    SlotEnableDirectControl(DirectControl);
    QCBEnablePlotMode->setChecked(PlotMode);
    SlotEnablePlotMode(PlotMode);
}


void mtsRobot1394QtWidget::UpdateRobotInfo(void)
{
    // safety relay
    QCBSafetyRelay->blockSignals(true); {
        QCBSafetyRelay->setChecked(SafetyRelay);
    } QCBSafetyRelay->blockSignals(false);

    // actuator amplifier status
    bool ampStatusGood = ActuatorAmpStatus.All();
    if (NumberOfActuators != 0) {
        QVWActuatorCurrentEnableEach->SetValue(ActuatorAmpStatus);
    }
    if (ampStatusGood) {
        QLAmpStatus->setText("Actuators on");
        QLAmpStatus->setStyleSheet("QLabel { background-color: rgb(50, 255, 50) }");
    } else {
        QLAmpStatus->setText("Actuators off");
        QLAmpStatus->setStyleSheet("QLabel { background-color: rgb(255, 100, 100) }");
    }

    // brake amplifier status
    if (NumberOfBrakes != 0) {
        QVWBrakeCurrentEnableEach->SetValue(BrakeAmpStatus);
    }

    // power status
    if (FullyPowered) {
        QLFullyPowered->setText("Power on");
        QLFullyPowered->setStyleSheet("QLabel { background-color: rgb(50, 255, 50) }");
    } else {
        QLFullyPowered->setText("Power off");
        QLFullyPowered->setStyleSheet("QLabel { background-color: rgb(255, 100, 100) }");
    }

    // update check box to enable/disable based on current state
    QCBPowerEnable->blockSignals(true); {
        QCBPowerEnable->setChecked(PowerEnable);
    } QCBPowerEnable->blockSignals(false);

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
    if (SafetyRelayStatus) {
        QLSafetyRelayStatus->setText("Closed");
        QLSafetyRelayStatus->setStyleSheet("QLabel { background-color: rgb(50, 255, 50) }");
    } else {
        QLSafetyRelayStatus->setText("Open");
        QLSafetyRelayStatus->setStyleSheet("QLabel { background-color: rgb(255, 100, 100) }");
    }
}

void mtsRobot1394QtWidget::FullyPoweredEventHandler(const bool & status)
{
    emit SignalFullyPowered(status);
}

void mtsRobot1394QtWidget::SlotFullyPoweredEvent(bool status)
{
    if (status == false) {
        QCBEnableAll->setChecked(false);
    }
}

void mtsRobot1394QtWidget::WatchdogTimeoutStatusEventHandler(const bool & status)
{
    emit SignalWatchdogTimeoutStatus(status);
}

void mtsRobot1394QtWidget::SlotWatchdogTimeoutStatusEvent(bool status)
{
    WatchdogCounter++;
    QString counter = QString::number(WatchdogCounter);
    if (status) {
        QLWatchdog->setText("Timed out [" + counter + "]");
        QLWatchdog->setStyleSheet("QLabel { background-color: rgb(255, 100, 100) }");
        QLWatchdogLastTimeout->setText(QString("Last timeout: " + QTime::currentTime().toString("hh:mm:ss")));
    } else {
        QLWatchdog->setText("Not timed out [" + counter + "]");
        QLWatchdog->setStyleSheet("QLabel { background-color: rgb(50, 255, 50) }");
    }
}

void mtsRobot1394QtWidget::UsePotsForSafetyCheckEventHandler(const bool & status)
{
    emit SignalUsePotsForSafetyCheck(status);
}

void mtsRobot1394QtWidget::SlotUsePotsForSafetyCheckEvent(bool status)
{
    QCBUsePotsForSafetyCheck->setChecked(status);
}

void mtsRobot1394QtWidget::SlotUsePotsForSafetyCheck(bool checked)
{
    mtsExecutionResult result = Robot.UsePotsForSafetyCheck(checked);
    if (!result.IsOK()) {
        CMN_LOG_CLASS_RUN_WARNING << "SlotUsePotsForSafetyCheck: command failed \"" << result << "\"" << std::endl;
    }
}

void mtsRobot1394QtWidget::SlotPlotVisibleSignal(int index)
{
    const bool checked = PlotCheckBoxes[index]->isChecked();
    PlotSignals[index]->SetVisible(checked);
    PlotVisibleSignals[index] = checked;
}

void mtsRobot1394QtWidget::SlotPlotIndex(int index)
{
    PlotIndex = index;
    QPlot->SetContinuousExpandYResetSlot();
}
