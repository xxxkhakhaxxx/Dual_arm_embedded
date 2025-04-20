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
/****************** Torque Current calculate ******************
 * I16 range: -2048 ~ +2048 == -33A ~ +33A
 * - FOR MG4010-i10V3:
 *      + max Curr = max Power / Voltage
 *                 = 140W / 24V
 *                 =   35/6 (A)
 *      + max Torque = 4.5 Nm
 *      --> I16 range for MG4010-i10V3: -362  ~ +362  (bit)
 *                                    = -35/6 ~ +35/6 (A)
 *                                    = -4.5  ~ +4.5  (Nm)
 *      --> Torque constant MG4010-i10V3 = 0.771 (Nm/A) (after gearbox)
 *                                       = 0.077 (Nm/A) (before gearbox)
 * - FOR MG5010-i10V3:
 *      + max Curr = max Power / Voltage
 *                 = 160 / 24V
 *                 =  20/3 (A)
 *      + max Torque = 7.0 Nm
 *      --> I16 range for MG5010-i10V3: -413  ~ +413  (bit)
 *                                    = -20/3 ~ +20/3 (A)
 *                                    = -7.0  ~ +7.0  (Nm)
 *      --> Torque constant MG5010-i10V3 = 1.05  (Nm/A) (after gearbox)
 *                                       = 0.105 (Nm/A) (before gearbox)
 **************************************************************/


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
