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
#include <LibraryHHInterface_Slave.h>

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/
#define MSG_INIT_LENGTH	(3)				// 3 header + 0 payloads
#define MSG_INIT_BYTE_0	(0xA1)
#define MSG_INIT_BYTE_1	(0x01)
#define MSG_INIT_BYTE_2	(0x01)

#define MSG_DATA_REQUEST_LENGTH	(3)		// 3 header + 0 payloads
#define MSG_DATA_REQUEST_BYTE_0	(0xB2)
#define MSG_DATA_REQUEST_BYTE_1	(0x02)
#define MSG_DATA_REQUEST_BYTE_2	(0x02)

#define MSG_DATA_RESPOND_LENGTH	(39)	// 3 header + 4*9 payloads
#define MSG_DATA_RESPOND_BYTE_0	(0xB2)
#define MSG_DATA_RESPOND_BYTE_1	(0x02)
#define MSG_DATA_RESPOND_BYTE_2	(0x02)

#define MSG_CONTROL_POS_LENGTH	(24)	// 3 header + 3*(4+2+1) payloads
#define MSG_CONTROL_POS_BYTE_0	(0xC3)
#define MSG_CONTROL_POS_BYTE_1	(0x03)
#define MSG_CONTROL_POS_BYTE_2	(0x03)

#define MSG_CONTROL_VEL_LENGTH	(15)	// 3 header + 3*4 payloads
#define MSG_CONTROL_VEL_BYTE_0	(0xD4)
#define MSG_CONTROL_VEL_BYTE_1	(0x04)
#define MSG_CONTROL_VEL_BYTE_2	(0x04)

#define MSG_CONTROL_TOR_LENGTH	(15)	// 3 header + 3*4 payloads
#define MSG_CONTROL_TOR_BYTE_0	(0xE5)
#define MSG_CONTROL_TOR_BYTE_1	(0x05)
#define MSG_CONTROL_TOR_BYTE_2	(0x05)

/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef enum ENUM_SLAVE_STATE_LIST
{
	SLAVE_STATE_INIT = 0,
//	SLAVE_STATE_WAIT_MASTER_INIT,
	SLAVE_STATE_WAIT_MASTER_REQUEST,

	SLAVE_STATE_WAIT_MOTOR_FEEDBACK,
	SLAVE_STATE_SEND_MOTOR_SEQUENCE,

	SLAVE_STATE_MAX
} enSlaveStateList;

typedef enum ENUM_ROBOT_MODE
{
	ROBOT_MODE_INIT = 0,
	ROBOT_MODE_READ_ONLY,
	ROBOT_MODE_POSITION,
//	ROBOT_MODE_VELOCITY,
	ROBOT_MODE_TORQUE,

	ROBOT_MODE_MAX
} enRobotMode;


/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL enSlaveStateList AppDataGet_SlaveState(void);
GLOBAL void AppDataSet_SlaveState(enSlaveStateList _state);

GLOBAL U08  AppDataGet_RobotMode(void);
GLOBAL void AppDataSet_RobotMode(enRobotMode _mode);

GLOBAL BOOL AppDataGet_IsMotorLowVoltage(U08 _motorId);
GLOBAL BOOL AppDataGet_IsMotorHighTemp(U08 _motorId);

/************ CAN BUS TX MANAGE FUNCTION  ************/
GLOBAL U08  AppDataGet_CanComm(void);
GLOBAL void AppDataSet_CanComm(U08 _canNode);

/************ CAN BUS RX MANAGE FUNCTION  ************/
GLOBAL BOOL AppDataGet_CanRxNewFlag(void);
GLOBAL void AppDataSet_CanRxNewFlag(BOOL _bFlag);

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
