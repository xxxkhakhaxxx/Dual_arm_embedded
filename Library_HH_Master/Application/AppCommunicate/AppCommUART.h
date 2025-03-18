/**
 ********************************************************************************
 ** @file    AppCommUART.h
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Mar 14, 2025 (created)
 ** @brief   [UART GENERAL CONFIG]
 **            - Speed: 115200 Bits/s
 **            - Words length: 8 Bits
 **            - Parity: none
 **            - Stop bit: 1 Bit
 **
 **          [MASTER UART CHANNEL]
 **            - UART 6 = SLAVE 1:
 **                + Pins: Tx-PC6 , Rx-PC7
 **                + DMA : Normal mode, 1 byte
 **                        DMA Tx Channel 2 Stream 6
 **                        DMA Rx Channel 2 Stream 1
 **                + IT  : Disable
 **            - UART 2 = GUI :
 **                + Pins: Tx-PD5 , Rx-PD6
 **                + DMA : Normal mode, 1 byte
 **                        DMA Tx Channel 1 Stream 6
 **                        DMA Rx Channel 1 Stream 5
 **                + IT  : Disable
 **            - UART 3 = SLAVE 2:
 **                + Pins: Tx-PB10, Rx-PB11
 **                + DMA : Normal mode, 1 byte
 **                        DMA Tx Channel 1 Stream 3
 **                        DMA Rx Channel 1 Stream 1
 **                + IT  : Disable
 **
 ********************************************************************************
 **/

#ifndef APPLICATION_APPCOMMUNICATE_APPCOMMUART_H_
#define APPLICATION_APPCOMMUNICATE_APPCOMMUART_H_


/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include <LibraryHHInterface_Master.h>

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/
#ifdef TEST_MASTER_UART
#define UART_TX_TEST_BUFFER_SIZE (10)
#endif

typedef enum ENUM_UART_NODE
{
	UART_NODE_SLAVE_1 = 0,
	UART_NODE_SLAVE_2 = 1,
	UART_NODE_GUI = 3
} enUartNode;

typedef enum ENUM_UART_TX_MSG
{
	UART_TX_MSG_SLAVE_SET_POSITION = 0,
	UART_TX_MSG_SLAVE_SET_VELOCITY,
	UART_TX_MSG_SLAVE_SET_TORQUE,
	UART_TX_MSG_GUI_DATA_FEEDBACK,

	UART_TX_MSG_TEST = 0xFF
} enUartTxMsg;

typedef enum ENUM_UART_RX_MSG
{
	UART_RX_MSG_SLAVE_SET_POSITION_FEEDBACK = 0,
	UART_RX_MSG_SLAVE_SET_VELOCITY_FEEDBACK,
	UART_RX_MSG_SLAVE_SET_TORQUE_FEEDBACK,
	UART_RX_MSG_GUI_DATA_SET_TRAJECTORY
} enUartRxMsg;
/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/


/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/

/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL void AppCommUART_UserSetup(UART_HandleTypeDef* huart, enUartNode _node);		// Save huart structure pointer generated from IDE to library's private pointer
GLOBAL void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);						// Function callback when finished Tx

GLOBAL void AppCommUART_SendMsg(enUartNode _node, enUartTxMsg _txMsgId);

#endif /* APPLICATION_APPCOMMUNICATE_APPCOMMUART_H_ */
