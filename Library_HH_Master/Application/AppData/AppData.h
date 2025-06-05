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

#define BUTTON_DEBOUNCE_DELAY_MS	(50)
#define BUTTON_PRESSED				(FALSE)
#define BUTTON_RELEASED				(TRUE)

// UART frame: 2 msg bytes + 1 length byte + 1 checksum byte (optional) + payload (optional)
#define MSG_INIT_BYTE_0	(0xA1)
#define MSG_INIT_BYTE_1	(0x01)
#define MSG_INIT_LENGTH	(3)				// 3 header + 0 checksum + 0 payloads

#define MSG_DATA_REQUEST_BYTE_0	(0xB2)
#define MSG_DATA_REQUEST_BYTE_1	(0x02)
#define MSG_DATA_REQUEST_LENGTH	(3)		// 3 header + 0 checksum + 0 payloads

#define MSG_DATA_RESPOND_BYTE_0	(0xB2)
#define MSG_DATA_RESPOND_BYTE_1	(0x02)
#define MSG_DATA_RESPOND_LENGTH	(40)	// 3 header + 1 checksum + 3*(4+4+4) payloads

#define MSG_DATA_RESPOND_F_BYTE_0 (0xB3) // Check 1st order filter effect
#define MSG_DATA_RESPOND_F_BYTE_1 (0x02)
#define MSG_DATA_RESPOND_F_LENGTH (64)	// 3 header + 1 checksum + 3*(4+4+4+4+4) payloads: pos-vel-velf-accel-accelf

#define MSG_CONTROL_POS_BYTE_0	(0xC3)
#define MSG_CONTROL_POS_BYTE_1	(0x03)
#define MSG_CONTROL_POS_LENGTH	(25)	// 3 header + 1 checksum + 3*(4+2+1) payloads
/*
#define MSG_CONTROL_VEL_BYTE_0	(0xD4)
#define MSG_CONTROL_VEL_BYTE_1	(0x04)*/
#define MSG_CONTROL_VEL_LENGTH	()	// 3 header + 3*4 payloads

#define MSG_CONTROL_TOR_BYTE_0	(0xE5)
#define MSG_CONTROL_TOR_BYTE_1	(0x05)
#define MSG_CONTROL_TOR_LENGTH	(16)	// 3 header + 1 checksum + 3*(4) payloads

#define MSG_GUI_DATA_1_SING_BYTE_0	(0xF6)
#define MSG_GUI_DATA_1_SING_BYTE_1	(0x06)
#define MSG_GUI_DATA_1_SING_LENGTH	(40)	// 3 header + 1 checksum + 3*(4+4+4) payloads

#define MSG_GUI_DATA_1_DUAL_BYTE_0	(0xF7)
#define MSG_GUI_DATA_1_DUAL_BYTE_1	(0x07)
#define MSG_GUI_DATA_1_DUAL_LENGTH	(76)	// 3 header + 1 checksum + 2*3*(4+4+4) payloads

#define MSG_GUI_DATA_2_SING_BYTE_0	(0xF8)
#define MSG_GUI_DATA_2_SING_BYTE_1	(0x08)
#define MSG_GUI_DATA_2_SING_LENGTH	(40)	// 3 header + 1 checksum + 3*(4+4+4) payloads

#define MSG_GUI_DATA_2_DUAL_BYTE_0	(0xF9)
#define MSG_GUI_DATA_2_DUAL_BYTE_1	(0x09)
#define MSG_GUI_DATA_2_DUAL_LENGTH	(76)	// 3 header + 1 checksum + 2*3*(4+4+4) payloads

#define MSG_GUI_DATA_3_DUAL_BYTE_0	(0xFA)  // Filter data
#define MSG_GUI_DATA_3_DUAL_BYTE_1	(0x10)
#define MSG_GUI_DATA_3_DUAL_LENGTH	(124)	// 3 header + 1 checksum + 2*3*(4+4+4+4+4) payloads

#define MSG_GUI_DATA_4_SING_BYTE_0	(0xFB)  // Single SMC data
#define MSG_GUI_DATA_4_SING_BYTE_1	(0x11)
#define MSG_GUI_DATA_4_SING_LENGTH	(64)	// 3 header + 1 checksum + 3*(4+4+4+4+4) payloads

#define MSG_GUI_DATA_4_DUAL_BYTE_0	(0xFC)  // Single SMC data
#define MSG_GUI_DATA_4_DUAL_BYTE_1	(0x12)
#define MSG_GUI_DATA_4_DUAL_LENGTH	(124)	// 3 header + 1 checksum + 2*3*(4+4+4+4+4) payloads


#define CURR_POS_J11	((float)myRobotFeedback[LEFT_ARM].Joint[0].Position)
#define CURR_POS_J21	((float)myRobotFeedback[LEFT_ARM].Joint[1].Position)
#define CURR_POS_J31	((float)myRobotFeedback[LEFT_ARM].Joint[2].Position)
#define CURR_VEL_J11	((float)myRobotFeedback[LEFT_ARM].Joint[0].Speed)
#define CURR_VEL_J21	((float)myRobotFeedback[LEFT_ARM].Joint[1].Speed)
#define CURR_VEL_J31	((float)myRobotFeedback[LEFT_ARM].Joint[2].Speed)
#define CURR_ACCEL_J11	((float)myRobotFeedback[LEFT_ARM].Joint[0].Accel)
#define CURR_ACCEL_J21	((float)myRobotFeedback[LEFT_ARM].Joint[1].Accel)
#define CURR_ACCEL_J31	((float)myRobotFeedback[LEFT_ARM].Joint[2].Accel)

#define CURR_POS_J12	((float)myRobotFeedback[RIGHT_ARM].Joint[0].Position)
#define CURR_POS_J22	((float)myRobotFeedback[RIGHT_ARM].Joint[1].Position)
#define CURR_POS_J32	((float)myRobotFeedback[RIGHT_ARM].Joint[2].Position)
#define CURR_VEL_J12	((float)myRobotFeedback[RIGHT_ARM].Joint[0].Speed)
#define CURR_VEL_J22	((float)myRobotFeedback[RIGHT_ARM].Joint[1].Speed)
#define CURR_VEL_J32	((float)myRobotFeedback[RIGHT_ARM].Joint[2].Speed)
#define CURR_ACCEL_J12	((float)myRobotFeedback[RIGHT_ARM].Joint[0].Accel)
#define CURR_ACCEL_J22	((float)myRobotFeedback[RIGHT_ARM].Joint[1].Accel)
#define CURR_ACCEL_J32	((float)myRobotFeedback[RIGHT_ARM].Joint[2].Accel)

/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef struct
{
	struct
	{
		float Angle;	// deg   per bit: range -360.0f ~ +360.0f
		U16 Speed;		// deg/s per bit
		U08 Direction;	// CC or CCW
	} JointPos[3];

/*	struct
	{
		I32 Speed;
	} JointVel[3];*/

	struct
	{
		float Tor;		// [Nm]
	} JointTor[3];

} strRobotDataCommand;

typedef struct
{
	struct
	{
		float Position;	// deg     per bit
		float Speed;	// deg/s   per bit
		float Accel;	// deg/s^2 per bit

		float Speedf;	// [Deg/s]
		float Accelf;	// [Deg/s2]
	} Joint[3];
} strRobotDataFeedback;


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
	ROBOT_MODE_READ_DATA,
	ROBOT_MODE_POSITION,
	ROBOT_MODE_VELOCITY,
	ROBOT_MODE_TORQUE,

	ROBOT_MODE_MAX
} enRobotMode;

typedef enum ENUM_BTN_CTRL_SEQUENCE
{
	BTN_CTRL_INIT = 0,
	BTN_CTRL_TO_HOME,
	BTN_CTRL_TO_PLANNING_INIT,
	BTN_CTRL_PLANNING,

	BTN_CTRL_TEST_POS_SEQUENCE,
	BTN_CTRL_TEST_TOR_SEQUENCE,
	BTN_CTRL_IDLE
} enBtnCtrlSequence;

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
GLOBAL extern strRobotDataCommand  myRobotCommand[DUAL_ARM];		// Trajectory Planning data to be sent to Slave
GLOBAL extern strRobotDataFeedback myRobotFeedback[DUAL_ARM];		// Motors' data are received from Slave


/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL enMasterStateList AppDataGet_MasterState(void);
GLOBAL void AppDataSet_MasterState(enMasterStateList _state);
GLOBAL void AppDataSet_LedState(uint16_t pin_name, BOOL _state);
GLOBAL BOOL AppDataGet_UserButtonEvent(void);
GLOBAL void AppDataCheck_UserButtonState(void);

GLOBAL BOOL AppDataGet_TPCalculated(void);
GLOBAL void AppDataSet_TPCalculated(BOOL _flag);

/************ UART TX MANAGE FUNCTION  ************/
GLOBAL BOOL AppDataGet_UartTxWaitFlag(U08 _node);				// if you don't want to use this flag, you should handle Tx success or not
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
