/**
 ********************************************************************************
 ** @file    AppConfig.h
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Oct 30, 2024 (created)
 ** @brief   
 ********************************************************************************
 **/

#ifndef APPLICATION_APPCONFIG_APPCONFIG_H_
#define APPLICATION_APPCONFIG_APPCONFIG_H_


/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include <LibraryHHInterface_Master.h>

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/
#define PROJECT_MCU		STM32F4XX_FW_1_28_1

//#define TEST_UART_SEND				// Init-> Auto send and Receive: Used for checking which UART channel work with DMA and IT
//#define TEST_UART_CYCLE_NO_FEEDBACK	// Init-> Send init -> Wait Slave -> Cal Control -> Wait Slave->...
#define TEST_UART_CYCLE_MOTOR_DATA	// Init-> ... -> Request data -> wait Slave -> Read motor data -> ...
/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/


/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/



#endif /* APPLICATION_APPCONFIG_APPCONFIG_H_ */
