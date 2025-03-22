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
#include <LibraryHHInterface_Slave.h>

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/
#define PROJECT_MCU		STM32F1XX_FW_1_8_6

#define TEST_UART_SEND				// Init-> Auto send and Receive: Used for checking which UART channel work with DMA and IT
#define TEST_UART_CYCLE_NO_FEEDBACK	// Init-> Wait Master-> FeedBack Master-> Wait Master-> Control Motor-> Motor Feedback-> Feedback Master
#define TEST_UART_CYCLE_MOTOR_DATA	// Init-> ... -> wait Master -> Read motor data -> Cal speed + accel -> Feedback  motor data -> wait Master.


#define TOTAL_MOTOR_FOR_ONE_ARM	(3)
#define MOTOR_1		LINGKONG_MG5010E_i10
#define MOTOR_2		LINGKONG_MG4010E_i10
#define MOTOR_3		LINGKONG_MG4010E_i10

#if ((MOTOR_1==LINGKONG_MG5010E_i10) || (MOTOR_1==LINGKONG_MG4010E_i10))
#define MOTOR_1_GEARBOX		(10)
#endif

#if ((MOTOR_2==LINGKONG_MG5010E_i10) || (MOTOR_2==LINGKONG_MG4010E_i10))
#define MOTOR_2_GEARBOX		(10)
#endif
#if ((MOTOR_3==LINGKONG_MG5010E_i10) || (MOTOR_3==LINGKONG_MG4010E_i10))
#define MOTOR_3_GEARBOX		(10)
#endif


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
