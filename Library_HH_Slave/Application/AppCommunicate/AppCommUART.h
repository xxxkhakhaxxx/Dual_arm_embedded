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
#define UART_BUFFER_SIZE	(256)


/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef enum ENUM_UART_NODE
{
	UART_NODE_MASTER = 0,

	UART_NODE_MAX
} enUartNode;

typedef enum ENUM_UART_MSG
{
	UART_MSG_INIT = 0x00,
	UART_MSG_MOTOR_DATA,
	UART_MSG_MOTOR_CONTROL_POS,
//	UART_MSG_MOTOR_CONTROL_VEL,
	UART_MSG_MOTOR_CONTROL_TOR
} enUartMsg;

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
extern U08 RxDataMaster[UART_BUFFER_SIZE];

/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL void AppCommUART_UserSetup(UART_HandleTypeDef* huart, enUartNode _node);		// Save huart structure pointer generated from IDE to library's private pointer

GLOBAL void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);						// Function callback when finished Tx
GLOBAL void AppCommUART_SendMsg(enUartNode _node, enUartMsg _txMsgId);				// Function send msg header + payloads

GLOBAL void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);	// Function call back when detect IDLE on Rx
GLOBAL void AppCommUart_RecvMsgStart(enUartNode _node);								// Start receive Rx message on DMA
GLOBAL void AppCommUart_RecvMasterMsg(enUartMsg _rxMsgId);							// Function handle Master msg payloads

GLOBAL void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif /* APPLICATION_APPCOMMUNICATE_APPCOMMUART_H_ */
