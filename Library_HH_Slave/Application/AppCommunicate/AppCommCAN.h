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
typedef struct
{
	uint32_t prevTime;		// Previous time - ms
	uint32_t currTime;		// Current time - ms
	float currPosition;		// Current position
	float prevPosition;		// Previous position
	float currSpeed;		// Current speed
	float prevSpeed;		// Previous speed
	float currAccel;		// Current acceleration
} strMotorData;

/*typedef enum ENUM_MOTOR_COMM_SEQUENCE
{
	MOTOR_COMM_ON = 0,
	MOTOR_COMM_READ_PID_PARAMS,
	MOTOR_COMM_READ_ACCEL,
	MOTOR_COMM_READ_ENCODER_INIT,
	MOTOR_COMM_READ_ANGLE_MULTI,
	MOTOR_COMM_READ_ANGLE_SINGLE,
	MOTOR_COMM_READ_STATE_ERROR,
	MOTOR_COMM_READ_STATE_MECHANICAL,
	MOTOR_COMM_READ_STATE_ELECTRICAL

} enMotorCommSequence;*/

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
GLOBAL extern strMotorData myMotor[MOTOR_NUMBER];	// Motor's data using for communication with Master
/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL void AppCommCAN_UserSetup(CAN_HandleTypeDef *hcan);
GLOBAL void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);		// Stm32 function handling Rx interrupt

GLOBAL void AppCommCAN_SendMotorMessage(U08 _u8MotorMsgId, U08 _u8MsgDataCmd);
GLOBAL void AppCommCAN_GetMotorMessage(void);


#endif /* APPLICATION_APPCOMMUNICATE_APPCOMMCAN_H_ */
