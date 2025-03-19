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
typedef union UNION_MASTER_STATE
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

typedef struct STRUCT_COMM_MANAGER
{
	struct
	{
		BOOL IsWait;
		U32  SendMsgCnt;
		U32  ErrCnt;
	} Tx;

	struct
	{
		BOOL IsWait;
		BOOL IsNew;
		U32  RecvMsgCnt;
		U32  ErrCnt;
	} Rx;
} strCommManager;

/********************************************************************************
 * PRIVATE VARIABLES
 ********************************************************************************/
PRIVATE enMasterStateList enMasterState = MASTER_STATE_INIT;
PRIVATE uniMasterState MasterState = {0, };

PRIVATE strCommManager myUart[UART_NODE_MAX] = {0, };

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

/************ UART TX MANAGE FUNCTION  ************/
GLOBAL BOOL AppDataGet_UartTxWaitFlag(U08 _node)
{
	BOOL waitFlag = FALSE;
	if (_node < UART_NODE_MAX)
	{
		waitFlag = myUart[_node].Tx.IsWait;
	}

	return waitFlag;
}
GLOBAL void AppDataSet_UartTxWaitFlag(U08 _node, BOOL _flag)
{
	if (_node < UART_NODE_MAX)
	{
		if (_flag != myUart[_node].Tx.IsWait)
		{
			myUart[_node].Tx.IsWait = _flag;
		}
	}

	return;
}
GLOBAL void AppDataSet_UartTxMsgCnt(U08 _node)
{
	if (_node < UART_NODE_MAX)
	{
		if (U32_MAX >  myUart[_node].Tx.SendMsgCnt)
		{
			myUart[_node].Tx.SendMsgCnt++;
		}
	}

	return;
}
GLOBAL void AppDataSet_UartTxErrCnt(U08 _node)
{
	if (_node < UART_NODE_MAX)
	{
		if (U32_MAX >  myUart[_node].Tx.ErrCnt)
		{
			myUart[_node].Tx.ErrCnt++;
		}
	}

	return;
}
/************ UART RX MANAGE FUNCTION  ************/
GLOBAL BOOL AppDataGet_UartRxWaitFlag(U08 _node)
{
	BOOL waitFlag = TRUE;	// For safety: default TRUE for not starting Rx DMA twice
	if (_node < UART_NODE_MAX)
	{
		waitFlag = myUart[_node].Rx.IsWait;
	}

	return waitFlag;
}
GLOBAL void AppDataSet_UartRxWaitFlag(U08 _node, BOOL _flag)
{
	if (_node < UART_NODE_MAX)
	{
		if (_flag != myUart[_node].Rx.IsWait)
		{
			myUart[_node].Rx.IsWait = _flag;
		}
	}

	return;
}
GLOBAL void AppDataSet_UartRxMsgCnt(U08 _node)
{
	if (_node < UART_NODE_MAX)
	{
		if (U32_MAX >  myUart[_node].Rx.RecvMsgCnt)
		{
			myUart[_node].Rx.RecvMsgCnt++;
		}
	}

	return;
}
GLOBAL void AppDataSet_UartRxErrCnt(U08 _node)
{
	if (_node < UART_NODE_MAX)
	{
		if (U32_MAX >  myUart[_node].Rx.ErrCnt)
		{
			myUart[_node].Rx.ErrCnt++;
		}
	}

	return;
}
GLOBAL BOOL AppDataGet_UartRxNewFlag(U08 _node)
{
	BOOL newFlag = TRUE;
	if (_node < UART_NODE_MAX)
	{
		newFlag = myUart[_node].Rx.IsNew;
	}

	return newFlag;
}
GLOBAL void AppDataSet_UartRxNewFlag(U08 _node, BOOL _flag)
{
	if (_node < UART_NODE_MAX)
	{
		if (_flag != myUart[_node].Rx.IsNew)
		{
			myUart[_node].Rx.IsNew = _flag;
		}
	}

	return;
}



