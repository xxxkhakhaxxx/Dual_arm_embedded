/**
 ********************************************************************************
 ** @file    AppData.c
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Oct 30, 2024 (created)
 ** @brief   
 ********************************************************************************
 **/

/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include "AppData.h"

/********************************************************************************
 * EXTERN VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE MACROS AND DEFINES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef union
{
	U16 allState[MASTER_STATE_MAX];
	struct
	{
		U16 Init;
		U16 WaitGuiCmd;
		U16 TrajectoryPlanning;
		U16 CalError;
		U16 CalControl;
		U16 WaitSlave;
		U16 DataAcquisition;

		U16 UartTest;
	} Cnt;
} uniMasterState;

/********************************************************************************
 * PRIVATE VARIABLES
 ********************************************************************************/
PRIVATE enMasterStateList enMasterState = MASTER_STATE_INIT;
PRIVATE uniMasterState MasterState = {0, };


/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/

/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/

/********************************************************************************
 * GLOBAL FUNCTION IMPLEMENTATION
 ********************************************************************************/
GLOBAL enMasterStateList AppDataGet_MasterState(void)
{
	return enMasterState;
}

GLOBAL void AppDataSet_MasterState(enMasterStateList _state)
{
	if ((_state != enMasterState) && (_state < MASTER_STATE_MAX))
	{
		enMasterState = _state;				// Change state
		if (U32_MAX > MasterState.allState[_state])
		{
			MasterState.allState[_state]++;	// Count current state
		}
	}

	return;
}

/** ###################### **/
GLOBAL BOOL AppDataGet_IsMotorLowVoltage(U08 _u8MotorId)
{	// 1 is TRUE, 0 is FALSE
	return GETBIT(strRobotArmMotorRx[_u8MotorId].State.u8Error,0);	// 1st bit
}

GLOBAL BOOL AppDataGet_IsMotorHighTemp(U08 _u8MotorId)
{	// 1 is TRUE, 0 is FALSE
	return GETBIT(strRobotArmMotorRx[_u8MotorId].State.u8Error,3);	// 4th bit
}

PRIVATE BOOL bCanRxMsgFlag = FALSE;			// Any data in CAN Rx or not
GLOBAL BOOL AppDataGet_CanRxMsgFlag(void)
{
	return bCanRxMsgFlag;
}
GLOBAL void AppDataSet_CanRxMsgFlag(BOOL _bFlag)
{
	if (_bFlag != bCanRxMsgFlag)
	{
		bCanRxMsgFlag = _bFlag;
	}

	return;
}

PRIVATE BOOL bSpiRxMsgFlag = FALSE;			// Any data in CAN Rx or not
GLOBAL BOOL AppDataGet_SpiRxMsgFlag(void)
{
	return bSpiRxMsgFlag;
}
GLOBAL void AppDataSet_SpiRxMsgFlag(BOOL _bFlag)
{
	if (_bFlag != bSpiRxMsgFlag)
	{
		bSpiRxMsgFlag = _bFlag;
	}

	return;
}

PRIVATE U32 u32Uart1TxIsSendCnt = 0;
PRIVATE BOOL u8Uart1TxIsSendFlag = TRUE;	// End sending Tx
PRIVATE U32 u32Uart1TxError = 0;
GLOBAL BOOL AppDataGet_Uart1TxIsSendFlag(void)
{
	return u8Uart1TxIsSendFlag;
}
GLOBAL void AppDataSet_Uart1TxIsSendFlag(BOOL _bFlag)
{
	if (_bFlag != u8Uart1TxIsSendFlag)
	{
		u8Uart1TxIsSendFlag = _bFlag;
		if (TRUE == _bFlag)
		{
			u32Uart1TxIsSendCnt++;	// Count when finish send
		}
	}

	return;
}

GLOBAL void AppDataSet_Uart1TxError(void)
{
	if (U32_MAX > u32Uart1TxError)
	{
		u32Uart1TxError++;	// Count when error
	}
	return;
}

