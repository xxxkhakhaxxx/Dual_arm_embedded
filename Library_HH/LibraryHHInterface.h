/**
 ********************************************************************************
 ** @file    LibraryHHInterface.h
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Oct 30, 2024 (created)
 ** @brief   Add all necessary header files here
 ********************************************************************************
 **/

#ifndef LIBRARYHHINTERFACE_H_
#define LIBRARYHHINTERFACE_H_


/********************************************************************************
 *									INCLUDES
 ********************************************************************************/
//  Basic C library
#include "stdio.h"
#include "stdlib.h"
#include "math.h"


//#include "./stm32f1xx_hal.h"  // it's the same with below code line whereas './' is source folder
#include "../../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"
#include "../../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio.h"
#include "../../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_can.h"
#include "../../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_spi.h"
#include "../../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h"

// Must add "Library_HH" as a source folder
#include "./Middleware/ApiMacro/ApiMacro.h"
#include "./Middleware/ApiProtocol/ApiProtocolMotorMG.h"
#include "./Middleware/ApiController/ApiControllerPID.h"
#include "./Middleware/ApiController/ApiControllerSMC.h"

#include "./Application/AppConfig/AppConfig.h"
#include "./Application/AppCommunicate/AppCommCAN.h"
#include "./Application/AppCommunicate/AppCommSPI.h"
#include "./Application/AppLoad/AppLoadLed.h"
#include "./Application/AppLoad/AppLoadMotor.h"
#include "./Application/AppData/AppData.h"

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/




#endif /* LIBRARYHHINTERFACE_H_ */
