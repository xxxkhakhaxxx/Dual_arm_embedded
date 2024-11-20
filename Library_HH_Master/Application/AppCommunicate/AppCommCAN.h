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
#include <LibraryHHInterface_Master.h>

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/


/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef enum ENUM_MOTOR_COMM_SEQUENCE
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

} enMotorCommSequence;

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
//GLOBAL enMotorCommSequence MotorCommCmd = MOTOR_COMM_ON;

/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL void AppCommCAN_UserSetup();
GLOBAL void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);		// Stm32 function handling Rx interrupt

GLOBAL void AppCommCAN_SendMotorMessage(U08 _u8MotorMsgId, U08 _u8MsgDataCmd);
GLOBAL void AppCommCAN_GetMotorMessage();


#endif /* APPLICATION_APPCOMMUNICATE_APPCOMMCAN_H_ */
