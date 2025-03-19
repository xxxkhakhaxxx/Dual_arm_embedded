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

/************ CAN BUS RX MANAGE FUNCTION  ************/
GLOBAL BOOL AppDataGet_CanRxMsgFlag(void);
GLOBAL void AppDataSet_CanRxMsgFlag(BOOL _bFlag);

/************ UART TX MANAGE FUNCTION  ************/
GLOBAL BOOL AppDataGet_UartTxWaitFlag(U08 _node);
GLOBAL void AppDataSet_UartTxWaitFlag(U08 _node, BOOL _flag);	// Set Tx waiting flag
GLOBAL void AppDataSet_UartTxMsgCnt(U08 _node);
GLOBAL void AppDataSet_UartTxErrCnt(U08 _node);
/************ UART RX MANAGE FUNCTION  ************/
GLOBAL BOOL AppDataGet_UartRxWaitFlag(U08 _node);
GLOBAL void AppDataSet_UartRxWaitFlag(U08 _node, BOOL _flag);	// Set Rx waiting flag
GLOBAL void AppDataSet_UartRxMsgCnt(U08 _node);
GLOBAL void AppDataSet_UartRxErrCnt(U08 _node);
GLOBAL BOOL AppDataGet_UartRxNewFlag(U08 _node);
GLOBAL void AppDataSet_UartRxNewFlag(U08 _node, BOOL _flag);

#endif /* APPLICATION_APPDATA_APPDATA_H_ */
