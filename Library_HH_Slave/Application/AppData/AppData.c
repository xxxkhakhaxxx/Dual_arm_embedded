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
typedef union UNION_SLAVE_STATE
{
	U16 allState[SLAVE_STATE_MAX];
	struct
	{
		U16 Init;
		U16 WaitMaster;
		U16 ControlMotor;
		U16 MotorFeedback;
		U16 FeedbackMaster;

		U16 UartTest;
	} Cnt;
} uniSlaveState;

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
PRIVATE uniSlaveState SlaveState = {0, };

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
PRIVATE enSlaveStateList enSlaveState = SLAVE_STATE_INIT;
GLOBAL enSlaveStateList AppDataGet_SlaveState(void)
{
	return enSlaveState;
}

GLOBAL void AppDataSet_SlaveState(enSlaveStateList _state)
{
	if ((_state != enSlaveState) && (_state < SLAVE_STATE_MAX))
	{
		enSlaveState = _state;				// Change state
		if (U16_MAX > SlaveState.allState[_state])
		{
			SlaveState.allState[_state]++;	// Count current state
		}
	}

	return;
}

PRIVATE enRobotMode u8RobotMode = ROBOT_MODE_READ_ONLY;
GLOBAL U08  AppDataGet_RobotMode(void)
{
	return u8RobotMode;
}
GLOBAL void AppDataSet_RobotMode(enRobotMode _mode)
{
	if ((_mode < ROBOT_MODE_MAX) && (_mode != u8RobotMode))
	{
		u8RobotMode = _mode;
	}

	return;
}



/** ###################### **/
GLOBAL BOOL AppDataGet_IsMotorLowVoltage(U08 _motorId)
{	// 1 is TRUE, 0 is FALSE
	return GETBIT(strRobotArmMotorRx[_motorId].State.u8Error,0);	// 1st bit
}

GLOBAL BOOL AppDataGet_IsMotorHighTemp(U08 _motorId)
{	// 1 is TRUE, 0 is FALSE
	return GETBIT(strRobotArmMotorRx[_motorId].State.u8Error,3);	// 4th bit
}

/************ CAN BUS TX MANAGE FUNCTION  ************/
PRIVATE enCanNode u8CanNode = CAN_NODE_MOTOR_1;
GLOBAL U08  AppDataGet_CanComm(void)
{
	return u8CanNode;
}
GLOBAL void AppDataSet_CanComm(U08 _canNode)
{
	if ((_canNode < CAN_NODE_MAX) && (_canNode != u8CanNode))
	{
		u8CanNode = _canNode;
	}

	return;
}


/************ CAN BUS RX MANAGE FUNCTION  ************/
PRIVATE BOOL bCanRxNewFlag = FALSE;			// If new Msg in Can Rx buffer
GLOBAL BOOL AppDataGet_CanRxNewFlag(void)
{
	return bCanRxNewFlag;
}
GLOBAL void AppDataSet_CanRxNewFlag(BOOL _bFlag)
{
	if (_bFlag != bCanRxNewFlag)
	{
		bCanRxNewFlag = _bFlag;
	}
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
		if (U16_MAX >  myUart[_node].Tx.SendMsgCnt)
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
		if (U16_MAX >  myUart[_node].Tx.ErrCnt)
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
		if (U16_MAX >  myUart[_node].Rx.RecvMsgCnt)
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
		if (U16_MAX >  myUart[_node].Rx.ErrCnt)
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



