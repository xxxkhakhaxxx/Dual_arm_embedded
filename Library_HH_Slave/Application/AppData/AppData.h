/**
 ********************************************************************************
 ** @file    AppData.h
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Oct 30, 2024 (created)
 ** @brief   
 ********************************************************************************
 **/

#ifndef APPLICATION_APPDATA_APPDATA_H_
#define APPLICATION_APPDATA_APPDATA_H_


/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include <LibraryHHInterface_Slave.h>

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/

/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef enum ENUM_SLAVE_STATE_LIST
{
	SLAVE_STATE_INIT = 0,
	SLAVE_STATE_WAIT_MASTER,
	SLAVE_STATE_CONTROL_MOTOR,
	SLAVE_STATE_MOTOR_FEEDBACK,
	SLAVE_STATE_FEEDBACK_MASTER,

	SLAVE_STATE_UART_TEST,

	SLAVE_STATE_MAX
} enSlaveStateList;

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL enSlaveStateList AppDataGet_SlaveState(void);
GLOBAL void AppDataSet_SlaveState(enSlaveStateList _state);

GLOBAL BOOL AppDataGet_IsMotorLowVoltage(U08 _u8MotorId);
GLOBAL BOOL AppDataGet_IsMotorHighTemp(U08 _u8MotorId);

GLOBAL BOOL AppDataGet_CanRxMsgFlag(void);
GLOBAL void AppDataSet_CanRxMsgFlag(BOOL _bFlag);

GLOBAL BOOL AppDataGet_SpiRxMsgFlag(void);
GLOBAL void AppDataSet_SpiRxMsgFlag(BOOL _bFlag);

GLOBAL BOOL AppDataGet_Uart1TxIsSendFlag(void);
GLOBAL void AppDataSet_Uart1TxIsSendFlag(BOOL _bFlag);
GLOBAL void AppDataSet_Uart1TxError(void);

GLOBAL BOOL AppDataGet_Uart1RxIsReceiveFlag(void);
GLOBAL void AppDataSet_Uart1RxIsSendFlag(BOOL _bFlag);
GLOBAL void AppDataSet_Uart1RxError(void);
#endif /* APPLICATION_APPDATA_APPDATA_H_ */
