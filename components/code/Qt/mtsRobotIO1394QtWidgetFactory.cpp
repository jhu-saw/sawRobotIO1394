/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Kwang Young Lee
  Created on: 2013-04-11

  (C) Copyright 2013-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// project include
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsManagerLocal.h>

#include <sawRobotIO1394/mtsRobotIO1394QtWidgetFactory.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsRobotIO1394QtWidgetFactory, mtsComponent, std::string);


mtsRobotIO1394QtWidgetFactory::mtsRobotIO1394QtWidgetFactory(const std::string &name):
    mtsComponent(name),
    mButtonsWidget(0),
    BuildWidgetsCalled(false)
{
    // This function will make the required interface to be connected with
    // the provided interface of mtsRobotIO1394 named Configure with predefined function names.
    RobotConfigureInterface = AddInterfaceRequired("RobotConfiguration");
    if (RobotConfigureInterface) {
        RobotConfigureInterface->AddFunction("GetRobotNames", Configuration.GetRobotNames);
        RobotConfigureInterface->AddFunction("GetNumActuators", Configuration.GetNumbersOfActuators);
        RobotConfigureInterface->AddFunction("GetNumBrakes", Configuration.GetNumbersOfBrakes);
        RobotConfigureInterface->AddFunction("GetNumRobots", Configuration.GetNumberOfRobots);
        RobotConfigureInterface->AddFunction("GetNumDigitalInputs", Configuration.GetNumberOfDigitalInputs);
        RobotConfigureInterface->AddFunction("GetDigitalInputNames", Configuration.GetDigitalInputNames);
        RobotConfigureInterface->AddFunction("GetName", Configuration.GetName);
    }
}

void mtsRobotIO1394QtWidgetFactory::Configure(const std::string &) {
    this->BuildWidgets();
}

void mtsRobotIO1394QtWidgetFactory::Startup(void) {
    this->BuildWidgets();
}

const mtsRobotIO1394QtWidgetFactory::WidgetListType &
mtsRobotIO1394QtWidgetFactory::Widgets(void) const
{
    return this->mWidgets;
}

void mtsRobotIO1394QtWidgetFactory::BuildWidgets(void)
{
    if (BuildWidgetsCalled) {
        return;
    }
    BuildWidgetsCalled = true;

    if (!RobotConfigureInterface->GetConnectedInterface()) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: unable to connect to configuration interface"
                                 << std::endl;
        return;
    }

    std::string suffix = " IO";
    std::string actuatorInterfaceSuffix = "Actuators";
    std::string newComponentName;
    std::string newInterfaceActuatorName;

    std::string tmpRobotName;

    // robots, one component per robot with 2 interfaces to be connected
    Configuration.GetName(NameOfRobotIO1394);
    Configuration.GetNumberOfRobots(NumberOfRobots);

    if (NumberOfRobots > 0) {
        RobotNames.resize(NumberOfRobots);
        NumberOfActuatorsPerRobot.resize(NumberOfRobots);
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "BuildWidgets: no robot found" << std::endl;
    }

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    Configuration.GetRobotNames(RobotNames);
    Configuration.GetNumbersOfActuators(NumberOfActuatorsPerRobot);
    Configuration.GetNumbersOfBrakes(NumberOfBrakesPerRobot);

    for (int i = 0; i < NumberOfRobots; ++i) {
        tmpRobotName = RobotNames[i];
        newComponentName = tmpRobotName.append(suffix);
        tmpRobotName = RobotNames[i];
        newInterfaceActuatorName = tmpRobotName.append(actuatorInterfaceSuffix);
        tmpRobotName = RobotNames[i];

        mtsRobot1394QtWidget * robotWidget =
                new mtsRobot1394QtWidget(newComponentName, NumberOfActuatorsPerRobot[i], NumberOfBrakesPerRobot[i]);
        mWidgets.push_back(robotWidget);
        robotWidget->Configure();
        componentManager->AddComponent(robotWidget);
        componentManager->Connect(newComponentName, "Robot", NameOfRobotIO1394, tmpRobotName);
        componentManager->Connect(newComponentName, "RobotActuators", NameOfRobotIO1394, newInterfaceActuatorName);

        robotWidget->Create();
    }

    // digital inputs, one component with multiple required interfaces
    Configuration.GetNumberOfDigitalInputs(NumberOfDigitalInputs);

    if (NumberOfDigitalInputs > 0) {
        DigitalInputNames.resize(NumberOfDigitalInputs);
        Configuration.GetDigitalInputNames(DigitalInputNames);

        mButtonsWidget = new prmEventButtonQtWidgetComponent("Digital inputs");
        // adapt width to number of IOs
        if (NumberOfDigitalInputs > 32) {
            mButtonsWidget->SetNumberOfColumns(6);
        } else {
            mButtonsWidget->SetNumberOfColumns(4);
        }

        // add all the interfaces
        for (int i = 0; i < NumberOfDigitalInputs; ++i) {
            mButtonsWidget->AddEventButton(DigitalInputNames[i]);
        }

        componentManager->AddComponent(mButtonsWidget);

        // connect all the interfaces
        for (int i = 0; i < NumberOfDigitalInputs; ++i) {
            componentManager->Connect(mButtonsWidget->GetName(), DigitalInputNames[i],
                                      NameOfRobotIO1394, DigitalInputNames[i]);
        }
    } else {
        CMN_LOG_CLASS_INIT_WARNING << "BuildWidgets: no digital input found" << std::endl;
    }
}
