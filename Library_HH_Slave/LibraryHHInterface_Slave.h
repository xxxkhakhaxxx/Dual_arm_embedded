/**
 ********************************************************************************
 ** @file    LibraryHHInterface.h
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Oct 30, 2024 (created)
 ** @brief   Add all necessary header files here
 ********************************************************************************
 **/

#ifndef LIBRARYHHINTERFACE_SLAVE_H_
#define LIBRARYHHINTERFACE_SLAVE_H_


/********************************************************************************
 *									INCLUDES
 ********************************************************************************/
//  Basic C library
//#include "stdio.h"
//#include "stdlib.h"
//#include "stdint.h"
//#include "math.h"
#include "string.h"

#include "stm32f1xx_hal.h"

// Must add "Library_HH" as a source folder
#include "./Middleware/ApiMacro/ApiMacro.h"
#include "./Middleware/ApiProtocol/ApiProtocolMotorMG.h"
//#include "./Middleware/ApiController/ApiControllerPID.h"
//#include "./Middleware/ApiController/ApiControllerSMC.h"

#include "./Application/AppConfig/AppConfig.h"
#include "./Application/AppData/AppData.h"
#include "./Application/AppCommunicate/AppCommUART.h"
#include "./Application/AppCommunicate/AppCommCAN.h"
//#include "./Application/AppCommunicate/AppCommSPI.h"
#include "./Application/AppLoad/AppLoadLed.h"
#include "./Application/AppLoad/AppLoadMotor.h"
#include "./Application/AppPeriod/AppPeriodTask.h"


/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/




#endif /* LIBRARYHHINTERFACE_SLAVE_H_ */
