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
#include <LibraryHHInterface_Master.h>

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/

/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef enum ENUM_MASTER_STATE_LIST
{
	MASTER_STATE_INIT = 0,
	MASTER_STATE_WAIT_GUI_CMD,
	MASTER_STATE_TRAJECTORY_PLANNING,
	MASTER_STATE_CAL_ERROR,
	MASTER_STATE_CAL_CONTROL,
	MASTER_STATE_WAIT_SLAVE,
	MASTER_STATE_DATA_ACQUISITION,

	MASTER_STATE_UART_TEST,

	MASTER_STATE_MAX
} enMasterStateList;

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/

/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL enMasterStateList AppDataGet_MasterState(void);
GLOBAL void AppDataSet_MasterState(enMasterStateList _state);

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
