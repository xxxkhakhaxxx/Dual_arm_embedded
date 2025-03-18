/**
 ********************************************************************************
 ** @file    AppCommUART.c
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Mar 16, 2025 (created)
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
PRIVATE UART_HandleTypeDef* strUartMaster;

/********************************************************************************
 * PRIVATE TYPEDEFS AND ENUMS
 ********************************************************************************/


/********************************************************************************
 * PRIVATE VARIABLES
 ********************************************************************************/
PRIVATE U08 arrUartRxMsgData[UART_RX_BUFFER_SIZE] = {0, };

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE void AppCommUart_SetupRxHandler(enUartNode _node);

/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
PRIVATE void AppCommUart_SetupRxHandler(enUartNode _node)
{
	switch (_node)
	{
	case UART_NODE_MASTER:
		// Slave use DMA normal mode for receiving data
		// Save data to "arrUartRxMsgData" with maximum 256 bytes or when Rx line is IDLE (<256 bytes)
		HAL_UARTEx_ReceiveToIdle_DMA(strUartMaster, arrUartRxMsgData, UART_RX_BUFFER_SIZE);
		break;
	default:
		break;
	}
	return;
}

/********************************************************************************
 * GLOBAL FUNCTION IMPLEMENTATION
 ********************************************************************************/
GLOBAL void AppCommUART_UserSetup(UART_HandleTypeDef* huart, enUartNode _node)
{
	switch (_node)
	{
	case UART_NODE_MASTER:
		strUartMaster = huart;
		AppCommUart_SetupRxHandler(UART_NODE_MASTER);	// Enable DMA Rx 1st time
		break;
	default:
		// Not support
		break;
	}

	return;
}


GLOBAL void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	AppCommUart_SetupRxHandler(UART_NODE_MASTER);	// Re-enable DMA Rx after receiving data (For DMA normal mode only)

	return;
}


GLOBAL void AppCommUART_SendMsg(enUartNode _node, enUartTxMsg _txMsgId)
{
#ifdef TEST_SLAVE_UART
	// Make test data
	U08 uartTxMsgDataTest[UART_TX_TEST_BUFFER_SIZE] = {0, };
	U08 idx;

	uartTxMsgDataTest[0] = _txMsgId;
	for (idx = 1 ; idx < UART_TX_TEST_BUFFER_SIZE ; idx++)
	{
		uartTxMsgDataTest[idx] = idx;
	}

	switch (_node)
	{
	case UART_NODE_MASTER:
		HAL_UART_Transmit_DMA(strUartMaster, uartTxMsgDataTest, UART_TX_TEST_BUFFER_SIZE);
		break;
	default:
		// Not support
		break;
	}
#endif

	return;
}

