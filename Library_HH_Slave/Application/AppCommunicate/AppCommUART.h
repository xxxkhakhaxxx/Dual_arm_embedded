/**
 ********************************************************************************
 ** @file    AppCommUART.h
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Mar 16, 2025 (created)
 ** @brief   [UART GENERAL CONFIG]
 **            - Speed: 115200 Bits/s
 **            - Words length: 8 Bits
 **            - Parity: none
 **            - Stop bit: 1 Bit
 **
 **          [SLAVE UART CHANNEL]
 **            - UART 2 = MASTER:
 **                + Pins: Tx-PA2, Rx-PA3, DMA normal mode, 1 byte
 **                + DMA : Normal mode, 1 byte
 **                        DMA Tx Channel 1 Stream 7
 **                        DMA Rx Channel 1 Stream 6
 **                + IT  : Disable
 ********************************************************************************
 **/

#ifndef APPLICATION_APPCOMMUNICATE_APPCOMMUART_H_
#define APPLICATION_APPCOMMUNICATE_APPCOMMUART_H_


/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include <LibraryHHInterface_Slave.h>

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/
#define UART_RX_BUFFER_SIZE	(256)

#ifdef TEST_SLAVE_UART
#define UART_TX_TEST_BUFFER_SIZE (50)
#endif

/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef enum ENUM_UART_NODE
{
	UART_NODE_MASTER = 0
} enUartNode;

typedef enum ENUM_UART_TX_MSG
{
	UART_TX_MSG_SLAVE_SET_POSITION_FEEDBACK = 0,
	UART_TX_MSG_SLAVE_SET_VELOCITY_FEEDBACK,
	UART_TX_MSG_SLAVE_SET_TORQUE_FEEDBACK,
} enUartTxMsg;

typedef enum ENUM_UART_RX_MSG
{
	UART_RX_MSG_SLAVE_SET_POSITION = 0,
	UART_RX_MSG_SLAVE_SET_VELOCITY,
	UART_RX_MSG_SLAVE_SET_TORQUE,

	UART_RX_MSG_TEST = 0xFF
} enUartRxMsg;


/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL void AppCommUART_UserSetup(UART_HandleTypeDef* huart, enUartNode _node);		// Save huart structure pointer generated from IDE to library's private pointer
GLOBAL void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);	// UART Rx callback function with unknown data length

GLOBAL void AppCommUART_SendMsg(enUartNode _node, enUartTxMsg _txMsgId);


#endif /* APPLICATION_APPCOMMUNICATE_APPCOMMUART_H_ */
