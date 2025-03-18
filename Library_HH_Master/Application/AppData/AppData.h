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


GLOBAL BOOL AppDataGet_IsMotorLowVoltage(U08 _u8MotorId);
GLOBAL BOOL AppDataGet_IsMotorHighTemp(U08 _u8MotorId);

GLOBAL BOOL AppDataGet_CanRxMsgFlag(void);
GLOBAL void AppDataSet_CanRxMsgFlag(BOOL _bFlag);

GLOBAL BOOL AppDataGet_SpiRxMsgFlag(void);
GLOBAL void AppDataSet_SpiRxMsgFlag(BOOL _bFlag);

GLOBAL BOOL AppDataGet_Uart1TxIsSendFlag(void);
GLOBAL void AppDataSet_Uart1TxIsSendFlag(BOOL _bFlag);
GLOBAL void AppDataSet_Uart1TxError(void);

#endif /* APPLICATION_APPDATA_APPDATA_H_ */
