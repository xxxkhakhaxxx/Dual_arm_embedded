/**
 ********************************************************************************
 ** @file    AppData.h
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Oct 30, 2024 (created)
 ** @brief   
 ********************************************************************************
 **/

#ifndef APPLICATION_APPDATA_APPDATA_H_
#define APPLICATION_APPDATA_APPDATA_H_


/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include <LibraryHHInterface_Master.h>

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/
#define LED_4_GREEN		GPIO_PIN_12
#define LED_3_ORANGE	GPIO_PIN_13
#define LED_5_RED		GPIO_PIN_14
#define LED_6_BLUE		GPIO_PIN_15
#define LED_PORT		GPIOD

#define MSG_INIT_LENGTH	(5)
#define MSG_INIT_BYTE_1	(0xFF)
#define MSG_INIT_BYTE_2	(0x01)
#define MSG_INIT_BYTE_3	(0xFF)
#define MSG_INIT_BYTE_4	(0x02)

#define MSG_DATA_RESPOND_LENGTH	(37)
#define MSG_DATA_REQUEST_LENGTH	(5)
#define MSG_DATA_BYTE_1	(0xFF)
#define MSG_DATA_BYTE_2	(0x03)
#define MSG_DATA_BYTE_3	(0xFF)
#define MSG_DATA_BYTE_4	(0x04)

#define MSG_CONTROL_POS	(0xF5)
#define MSG_CONTROL_VEL	(0xF6)
#define MSG_CONTROL_TOR	(0xF7)


/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef enum ENUM_MASTER_STATE_LIST
{
	MASTER_STATE_INIT = 0,		// Wait 2 SLAVEs init
	MASTER_STATE_WAIT_NEW_SEQUENCE,
	MASTER_STATE_WAIT_SLAVE_FEEDBACK,

	MASTER_STATE_CAL_CONTROL,
	MASTER_STATE_SEND_GUI,

	MASTER_STATE_MAX
} enMasterStateList;

typedef enum ENUM_ROBOT_MODE
{
	ROBOT_MODE_INIT = 0,
	ROBOT_MODE_READ_ONLY,
	ROBOT_MODE_POSITION,
	ROBOT_MODE_VELOCITY,
	ROBOT_MODE_TORQUE,

	ROBOT_MODE_MAX
} enRobotMode;


/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/

/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL enMasterStateList AppDataGet_MasterState(void);
GLOBAL void AppDataSet_MasterState(enMasterStateList _state);
GLOBAL void AppDataSet_LedState(uint16_t pin_name, BOOL _state);

/************ UART TX MANAGE FUNCTION  ************/
GLOBAL BOOL AppDataGet_UartTxWaitFlag(U08 _node);
GLOBAL void AppDataSet_UartTxWaitFlag(U08 _node, BOOL _flag);	// Set Tx waiting flag
GLOBAL void AppDataSet_UartTxMsgCnt(U08 _node);
GLOBAL void AppDataSet_UartTxErrCnt(U08 _node);
/************ UART RX MANAGE FUNCTION  ************/
GLOBAL BOOL AppDataGet_UartRxWaitFlag(U08 _node);
GLOBAL void AppDataSet_UartRxWaitFlag(U08 _node, BOOL _flag);	// Set Rx waiting flag
GLOBAL void AppDataSet_UartRxMsgCnt(U08 _node);
GLOBAL void AppDataSet_UartRxErrCnt(U08 _node);
GLOBAL BOOL AppDataGet_UartRxNewFlag(U08 _node);
GLOBAL void AppDataSet_UartRxNewFlag(U08 _node, BOOL _flag);

#endif /* APPLICATION_APPDATA_APPDATA_H_ */
