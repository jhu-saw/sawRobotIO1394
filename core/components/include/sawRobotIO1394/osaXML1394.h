/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Jonathan Bohren
  Created on: 2013-06-29

  (C) Copyright 2013-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _osaXML1394_h
#define _osaXML1394_h

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnXMLPath.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <sawRobotIO1394/osaConfiguration1394.h>
#include <sawRobotIO1394/sawRobotIO1394Export.h>

namespace sawRobotIO1394 {

    template <typename _elementType>
    bool osaXML1394GetValue(cmnXMLPath & path, const std::string & context, const std::string & XPath,
                            _elementType & value, bool required = true) {
          bool found = path.GetXMLValue(context.c_str(), XPath.c_str(), value);
          if (required && !found) {
              CMN_LOG_INIT_ERROR << "osaXML1394GetValue: " << XPath << " in context " << context << " is required but not found" << std::endl;
              return false;
          }
          return true;
    }

    void CISST_EXPORT osaXML1394ConfigurePort(const std::string & filename,
                                              osaPort1394Configuration & config);

    bool CISST_EXPORT osaXML1394ConfigureRobot(cmnXMLPath & xmlConfig,
                                               const int robotIndex,
                                               osaRobot1394Configuration & robot);

    bool CISST_EXPORT osaXML1394ConfigureCoupling(cmnXMLPath & xmlConfig,
                                                  const int robotIndex,
                                                  osaRobot1394Configuration & robot);

    bool CISST_EXPORT osaXML1394ConfigureCouplingMatrix(cmnXMLPath & xmlConfig,
                                                        const char * couplingString,
                                                        int numRows,
                                                        int numCols,
                                                        vctDoubleMat & resultMatrix);

    bool CISST_EXPORT osaXML1394ConfigureDigitalInput(cmnXMLPath & xmlConfig,
                                                      const int inputIndex,
                                                      osaDigitalInput1394Configuration & digitalInput);

    bool CISST_EXPORT osaXML1394ConfigureDigitalOutput(cmnXMLPath & xmlConfig,
                                                       const int outputIndex,
                                                       osaDigitalOutput1394Configuration & digitalOutput);

    bool CISST_EXPORT osaXML1394ConfigureDallasChip(cmnXMLPath & xmlConfig,
                                                    const int dallasIndex,
                                                    osaDallasChip1394Configuration & dallasChip);

} // namespace sawRobotIO1394

#endif // _osaXML1394_h
