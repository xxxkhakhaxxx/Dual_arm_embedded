/**
 ********************************************************************************
 ** @file    AppCommCAN.h
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Oct 30, 2024 (created)
 ** @brief   
 ********************************************************************************
 **/

#ifndef APPLICATION_APPCOMMUNICATE_APPCOMMCAN_H_
#define APPLICATION_APPCOMMUNICATE_APPCOMMCAN_H_


/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include <LibraryHHInterface_Slave.h>

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/
#define MOTOR_NUMBER	(3)


/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef struct	STRUCT_MOTOR_DATA
{
	U32 prevTime;		// Previous time - ms
	U32 currTime;		// Current time - ms
	float currPosition;		// Current position
	float prevPosition;		// Previous position
	float currSpeed;		// Current speed
	float prevSpeed;		// Previous speed
	float currAccel;		// Current acceleration
} strMotorData;

typedef struct STRUCT_MOTOR_COMMAND
{
	struct
	{
		U32 Angle;
		U16 Speed;
		U08 Direction;
	} Pos;	// Single loop angle control 2

/*	struct
	{
		I32 Speed;
	} Vel;*/

	struct
	{
		I16 CurrentTor;
	} Tor;
} strMotorCmd;

typedef enum ENUM_CAN_NODE
{
	CAN_NODE_MOTOR_1 = 0,
	CAN_NODE_MOTOR_2,
	CAN_NODE_MOTOR_3,

	CAN_NODE_MAX
} enCanNode;

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
//GLOBAL enMotorCommSequence MotorCommCmd = MOTOR_COMM_ON;
GLOBAL extern strMotorData myMotorToMaster[MOTOR_NUMBER];	// Motor's data using for communication with Master
GLOBAL extern strMotorCmd  myMasterToMotor[MOTOR_NUMBER];	// Data from Master using for control Motor
/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL void AppCommCAN_UserSetup(CAN_HandleTypeDef *hcan);
GLOBAL void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);		// Stm32 function handling Rx interrupt

GLOBAL void AppCommCAN_SendMotorMsg(U08 _u8MotorMsgId, U08 _u8MsgDataCmd);
GLOBAL void AppCommCAN_RecvMotorMsg(void);


#endif /* APPLICATION_APPCOMMUNICATE_APPCOMMCAN_H_ */
