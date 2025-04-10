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
GLOBAL strRobotDataCommand  myRobotCommand[DUAL_ARM]  = {0, };
GLOBAL strRobotDataFeedback myRobotFeedback[DUAL_ARM] = {0, };


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
		if (U16_MAX > MasterState.allState[_state])
		{
			MasterState.allState[_state]++;	// Count current state
		}
	}

	return;
}

/**
  * @brief  Sets the state of an LED (ON/OFF)
  * @param  pin: LED GPIO pin (e.g., LED4_GREEN_PIN)
  * @param  state: TRUE (ON) or FALSE (OFF)
  * @retval None
  */
GLOBAL void AppDataSet_LedState(uint16_t _ledName, BOOL _ledState)
{
	if (_ledState != HAL_GPIO_ReadPin(LED_PORT, _ledName))
	{
		HAL_GPIO_WritePin(LED_PORT, _ledName, (_ledState ? GPIO_PIN_SET : GPIO_PIN_RESET));

	}

	return;
}

/************ BUTTON MANAGE FUNCTION  ************/
PRIVATE volatile BOOL _userButtonEvent = FALSE;
PRIVATE U08 btnPrevState = BUTTON_RELEASED; // Default to not pressed (pull-up)
PRIVATE U32 lastChangeTime = 0;
GLOBAL BOOL AppDataGet_UserButtonEvent(void)
{
	if (TRUE == _userButtonEvent)
	{
		_userButtonEvent = FALSE;
		return TRUE;
	}
	return FALSE;
}

GLOBAL void AppDataCheck_UserButtonState(void)
{
	BOOL btnCurrState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET ? BUTTON_RELEASED : BUTTON_PRESSED;
	U32 currentTime = HAL_GetTick();

	if (btnCurrState != btnPrevState)
	{
		if (currentTime - lastChangeTime >= BUTTON_DEBOUNCE_DELAY_MS)
		{
			// Stable state change detected
			if (
			(BUTTON_RELEASED == btnPrevState) && \
			(BUTTON_PRESSED == btnCurrState))
			{
				// Pressed down (not setting event yet)
			}
			else if (
			(BUTTON_PRESSED == btnPrevState) && \
			(BUTTON_RELEASED == btnCurrState))
			{
				// Released - set event
				_userButtonEvent = TRUE;
			}
			btnPrevState = btnCurrState;
		}
	}
	else
	{
		lastChangeTime = currentTime;
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

		switch (_node)
		{
		case UART_NODE_SLAVE_1:		AppDataSet_LedState(LED_4_GREEN, TRUE);		break;
		case UART_NODE_SLAVE_2:		AppDataSet_LedState(LED_6_BLUE, TRUE);		break;
		case UART_NODE_GUI:			AppDataSet_LedState(LED_5_RED, FALSE);		break;
		default:	break;
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



