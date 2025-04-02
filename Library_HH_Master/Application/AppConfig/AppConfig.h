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
//#define TEST_UART_CYCLE_MOTOR_DATA	// Init-> ... -> Request data -> wait Slave -> Read motor data -> ...

//#define MASTER_NO_CONTROL				// Master will not send control command
//#define	MASTER_NO_GUI					// Master will not send data to GUI

#define	MASTER_CONTROL_POS
//#define MASTER_CONTROL_VEL
//#define MASTER_CONTROL_TOR

#define LEFT_ARM		(0)
#define RIGHT_ARM		(1)
#define DUAL_ARM		(2)		// Left arm and Right arm
#define JOINT_DIR_CCW		TRUE	// Same with motor
#define JOINT_DIR_CW		FALSE

#define PERIOD_CONTROL		(20)	// Update this base on your timer 2 (Unit: ms)
#define PERIOD_GUI_SEND		(20)	// Unit: ms
#define GUI_SEND_CNT_MAX	(PERIOD_GUI_SEND/PERIOD_CONTROL - 1)
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
