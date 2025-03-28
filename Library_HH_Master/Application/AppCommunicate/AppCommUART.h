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
#define UART_BUFFER_SIZE	(256)


/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef enum ENUM_UART_NODE
{
	UART_NODE_SLAVE_1 = 0x00,
	UART_NODE_SLAVE_2 = 0x01,
	UART_NODE_GUI     = 0x02,

	UART_NODE_MAX
} enUartNode;

typedef enum ENUM_UART_MSG
{
	UART_MSG_INIT = 0x00,
	UART_MSG_MOTOR_DATA,
	UART_MSG_MOTOR_CONTROL_POS,
//	UART_MSG_MOTOR_CONTROL_VEL,
	UART_MSG_MOTOR_CONTROL_TOR,

	UART_MSG_GUI_DATA_1,			// Pos-Vel-Accel
	UART_MSG_GUI_DATA_2				// Ref-pos-Tor-Sliding
} enUartMsg;

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
extern U08 RxDataSlaveLeft[UART_BUFFER_SIZE];
extern U08 RxDataSlaveRight[UART_BUFFER_SIZE];
extern U08 RxDataGUI[UART_BUFFER_SIZE];

/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL void AppCommUART_UserSetup(UART_HandleTypeDef* huart, enUartNode _node);		// Save huart structure pointer generated from IDE to library's private pointer

GLOBAL void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);						// Function callback when finished Tx
GLOBAL void AppCommUART_SendMsg(enUartNode _node, enUartMsg _txMsgId);				// Function send msg header + payload

GLOBAL void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);	// Function call back when detect IDLE on Rx
GLOBAL void AppCommUart_RecvMsgStart(enUartNode _node);								// Start receive Rx message on DMA

GLOBAL void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif /* APPLICATION_APPCOMMUNICATE_APPCOMMUART_H_ */
