/**
 ********************************************************************************
 ** @file    AppCommUART.c
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Mar 14, 2025 (created)
 ** @brief   
 ********************************************************************************
 **/

/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include "AppCommUART.h"

/********************************************************************************
 * EXTERN VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE MACROS AND DEFINES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE TYPEDEFS AND ENUMS
 ********************************************************************************/


/********************************************************************************
 * PRIVATE VARIABLES
 ********************************************************************************/
PRIVATE UART_HandleTypeDef* strUartSlaveLeft;
PRIVATE UART_HandleTypeDef* strUartSlaveRight;
PRIVATE UART_HandleTypeDef* strUartGUI;

/* Rx buffer - Just for receive */
PRIVATE U08 arrUartSlaveLeftRxBuffer[UART_BUFFER_SIZE] = {0, };
PRIVATE U08 arrUartSlaveRightRxBuffer[UART_BUFFER_SIZE] = {0, };
PRIVATE U08 arrUartGUIRxBuffer[UART_BUFFER_SIZE] = {0, };
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
/* Saved Rx buffer - Use in app */
GLOBAL U08 RxDataSlaveLeft[UART_BUFFER_SIZE] = {0, };
GLOBAL U08 RxDataSlaveRight[UART_BUFFER_SIZE] = {0, };
GLOBAL U08 RxDataGUI[UART_BUFFER_SIZE] = {0, };

GLOBAL U08 TxDataSlaveLeft[UART_BUFFER_SIZE] = {0, };
GLOBAL U08 TxDataSlaveRight[UART_BUFFER_SIZE] = {0, };
GLOBAL U08 TxDataGUI[UART_BUFFER_SIZE] = {0, };

/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/


/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/


/********************************************************************************
 * GLOBAL FUNCTION IMPLEMENTATION
 ********************************************************************************/
GLOBAL void AppCommUART_UserSetup(UART_HandleTypeDef* huart, enUartNode _node)
{
	switch (_node)
	{
	case UART_NODE_SLAVE_1:
		strUartSlaveLeft = huart;
		break;
	case UART_NODE_SLAVE_2:
		strUartSlaveRight = huart;
		break;
	case UART_NODE_GUI:
		strUartGUI = huart;
		break;
	default:
		// Not support
		break;
	}

	return;
}

/**
===============================================================================
                  ##### HUNG HOANG UART PROCESS SUMMARY #####
===============================================================================
  PROCESS:
      (Step 1) User call the "AppCommUART_SendMsg" anywhere in the APP process
      (Step 2) Function "AppCommUART_SendMsg" auto:
                   + Check if the Tx-wait flag is TRUE -> return
                   + If FALSE, package the new message and send
                   + Set the Tx-wait flag
                   + Wait (Step 3)
      (Step 3) Function "HAL_UART_TxCpltCallback" auto:
                   + Called when finished Tx transfer
                   + Reset the Tx-wait flag
                   + Increase the Tx-counter (for debug)
                   + Call (Step 4)
      (Step 4) Function "AppCommUart_RecvMsgStart" auto:
                   + Check if the Rx-wait flag is TRUE -> return
                   + If FALSE, start waiting Rx message
                   + Set the Rx-wait flag
                   + Wait (Step 5)
      (Step 5) Function "HAL_UARTEx_RxEventCallback" auto:
                   + Called when received Rx
                   + Copy Rx data from UART buffer to User buffer
                   + Increase the Rx-counter (for debug)
                   + Reset the Rx-wait flag
                   + Set the Rx-new flag for user know
      ( User ) Can do below things:
                   + Check the Tx-wait for knowing Tx is send or not
                   + Check the Rx-wait for knowing Rx is received or not
                   + Check the Rx-new for handling the Rx data
                   + Reset the Rx-new (MUST DO)
                   + Debug Tx-counter for total Tx message
                   + Debug Rx-counter for total Rx message
                   + Don't use SET/RESEET function of Rx/Tx-wait outside this file
 */


/************ UART TX MANAGE FUNCTION  ************/
/* Callback function when finished sending uart */
GLOBAL void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		AppDataSet_UartTxWaitFlag(UART_NODE_GUI, FALSE);	// Msg is send
		AppDataSet_UartTxMsgCnt(UART_NODE_GUI);			// Increase counter
		AppCommUart_RecvMsgStart(UART_NODE_GUI);		// Looking for Rx msg
	}
	else if (huart->Instance == USART3)
	{
		AppDataSet_UartTxWaitFlag(UART_NODE_SLAVE_2, FALSE);	// Msg is send
		AppDataSet_UartTxMsgCnt(UART_NODE_SLAVE_2);			// Increase counter
		AppCommUart_RecvMsgStart(UART_NODE_SLAVE_2);		// Looking for Rx msg
	}
	else if (huart->Instance == USART6)
	{
		AppDataSet_UartTxWaitFlag(UART_NODE_SLAVE_1, FALSE);	// Msg is send
		AppDataSet_UartTxMsgCnt(UART_NODE_SLAVE_1);			// Increase counter
		AppCommUart_RecvMsgStart(UART_NODE_SLAVE_1);		// Looking for Rx msg
	}
	else
	{
		// Do nothing
	}

	return;
}

/* Sending function */
/* TODO: (DeepSeek check)
 * 		1/ Error Handling: Consider adding a return value to AppCommUART_SendMsg to indicate success or failure
 * 		2/ Default Case in _txMsgId Switch: The default case in the _txMsgId switch does nothing and returns.
 * 		                                    This is fine, but you might want to log an error or handle it differently.
 */
GLOBAL void AppCommUART_SendMsg(enUartNode _node, enUartTxMsg _txMsgId)
{
// 1. Safety check
	if (TRUE == AppDataGet_UartTxWaitFlag(_node))
	{
		// There's Message wating to be send. Therefore, this function call end here.
		return;
	}


// 2. Initialize variables
	UART_HandleTypeDef* uartGoal;
	U08* sourceTxData;
	U16 sizeSend = 0;	// if not set value, TxErrCnt++


// 3. Get goal and data to be transfered
	switch (_node)
	{
	case UART_NODE_SLAVE_1:
		uartGoal = strUartSlaveLeft;
		sourceTxData = TxDataSlaveLeft;
		break;
	case UART_NODE_SLAVE_2:
		uartGoal = strUartSlaveRight;
		sourceTxData = TxDataSlaveRight;
		break;
	case UART_NODE_GUI:
		uartGoal = strUartGUI;
		sourceTxData = TxDataGUI;
		break;
	default:
		// Do nothing and return
		return;
	}


// 4. Set data to be transfer
#ifdef TEST_MASTER_UART
	// Test data
	sizeSend = 11;

	sourceTxData[0]  = UART_TX_MSG_TEST;	// '@'
	sourceTxData[1]  = 0x31;				// '1'
	sourceTxData[2]  = 0x32;				// '2'
	sourceTxData[3]  = 0x33;				// '3'
	sourceTxData[4]  = 0x34;				// '4'
	sourceTxData[5]  = 0x35;				// '5'
	sourceTxData[6]  = 0x36;				// '6'
	sourceTxData[7]  = 0x37;				// '7'
	sourceTxData[8]  = 0x38;				// '8'
	sourceTxData[9]  = 0x39;				// '9'
	sourceTxData[10] = 0x30;				// '0'
#else
	// Set data to be transfer
	switch (_txMsgId)
	{
	case UART_TX_MSG_INIT:
		sourceTxData[0] = UART_TX_MSG_INIT;
		sizeSend = 1;
		break;
	default:
		// Do nothing and return
		return;
	}
#endif


// 5. Send the message
	if (HAL_OK != HAL_UART_Transmit_DMA(uartGoal, sourceTxData, sizeSend))
	{
		AppDataSet_UartTxErrCnt(_node);
	}


// 6. Set sending flag
	AppDataSet_UartTxWaitFlag(_node, TRUE);

// TODO: 7. Handle return value


	return;
}

/************ UART RX MANAGE FUNCTION  ************/
GLOBAL void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
/* Save Rx data from UART buffer to User buffer
 * Rx counter++
 * Rx-wait flag RESET
 * Rx-new flag SET
 */
	if (huart->Instance == USART2)
	{
		memcpy(RxDataGUI, arrUartGUIRxBuffer, Size);
		AppDataSet_UartRxMsgCnt(UART_NODE_GUI);
		AppDataSet_UartRxWaitFlag(UART_NODE_GUI, FALSE);
		AppDataSet_UartRxNewFlag(UART_NODE_GUI, TRUE);
	}
	else if (huart->Instance == USART3)
	{
		memcpy(RxDataSlaveRight, arrUartSlaveRightRxBuffer, Size);
		AppDataSet_UartRxMsgCnt(UART_NODE_SLAVE_2);
		AppDataSet_UartRxWaitFlag(UART_NODE_SLAVE_2, FALSE);
		AppDataSet_UartRxNewFlag(UART_NODE_SLAVE_2, TRUE);
	}
	else if (huart->Instance == USART6)
	{
		memcpy(RxDataSlaveLeft, arrUartSlaveLeftRxBuffer, Size);
		AppDataSet_UartRxMsgCnt(UART_NODE_SLAVE_1);
		AppDataSet_UartRxWaitFlag(UART_NODE_SLAVE_1, FALSE);
		AppDataSet_UartRxNewFlag(UART_NODE_SLAVE_1, TRUE);
	}
	else
	{
		// Do nothing
	}

	return;
}
/* Function initialize UART DMA Rx IDLE for corresponding node */
GLOBAL void AppCommUart_RecvMsgStart(enUartNode _node)
{
	if (TRUE == AppDataGet_UartRxWaitFlag(_node))	// Check for duplicate DMA start or not
	{
		// The DMA Rx is already started
		return;
	}

	// Start Rx DMA
	switch (_node)
	{
	case UART_NODE_SLAVE_1:
		HAL_UARTEx_ReceiveToIdle_DMA(strUartSlaveLeft, arrUartSlaveLeftRxBuffer, UART_BUFFER_SIZE);
		break;
	case UART_NODE_SLAVE_2:
		HAL_UARTEx_ReceiveToIdle_DMA(strUartSlaveRight, arrUartSlaveRightRxBuffer, UART_BUFFER_SIZE);
		break;
	case UART_NODE_GUI:
		HAL_UARTEx_ReceiveToIdle_DMA(strUartGUI, arrUartGUIRxBuffer, UART_BUFFER_SIZE);
		break;
	default:
		break;
	}

	// Set Rx waiting flag
	AppDataSet_UartRxWaitFlag(_node, TRUE);		// After start Rx DMA
	return;
}

/************ UART ERROR MANAGE FUNCTION  ************/
GLOBAL void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	U32 error = HAL_UART_ERROR_NONE;

	// Get error
	error = HAL_UART_GetError(huart);

	// Set Error count
	switch (error)
	{

	case HAL_UART_ERROR_PE:   /*!< Parity error        */
	case HAL_UART_ERROR_NE:   /*!< Noise error         */
	case HAL_UART_ERROR_FE:   /*!< Frame error         */
	case HAL_UART_ERROR_ORE:  /*!< Overrun error       */
		if (huart->Instance == USART2)		AppDataSet_UartRxErrCnt(UART_NODE_GUI);
		else if (huart->Instance == USART3)	AppDataSet_UartRxErrCnt(UART_NODE_SLAVE_2);
		else if (huart->Instance == USART6)	AppDataSet_UartRxErrCnt(UART_NODE_SLAVE_1);
		else
		{
			// Do nothing
		}
		break;
	case HAL_UART_ERROR_DMA:  /*!< DMA transfer error  */
		// Could be DMA Tx or DMA Rx
		// Need to implement more
		break;
	case HAL_UART_ERROR_NONE: /*!< No error            */
	default:
		// Do nothing
		break;
	}


	return;
}
