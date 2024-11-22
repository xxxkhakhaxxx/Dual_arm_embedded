/**
 ********************************************************************************
 ** @file    AppCommSPI.c
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Oct 30, 2024 (created)
 ** @brief   
 ********************************************************************************
 **/

/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include "AppCommSPI.h"

/********************************************************************************
 * EXTERN VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE MACROS AND DEFINES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef struct
{
	I32 q1;
	I32 q2;
	I32 q3;
} strKinematicsData;	// Unit: 0.01 degree/bit


/********************************************************************************
 * PRIVATE VARIABLES
 ********************************************************************************/
PRIVATE SPI_HandleTypeDef* strSpiUse;

PRIVATE strKinematicsData strRobotAngleFkData = {0, };
PRIVATE strKinematicsData strRobotAngleDirectData = { 9000 , -9000 , 0 };
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE void AppComSPI_SetSlaveComm(enSlaveId _SlaveId);


/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
PRIVATE void AppComSPI_SetSlaveComm(enSlaveId _SlaveId)
{
	switch (_SlaveId)
	{
	case SLAVE_1_ID:

		break;
	case SLAVE_2_ID:

		break;
	default:
		break;
	}

	return;
}

/********************************************************************************
 * GLOBAL FUNCTION IMPLEMENTATION
 ********************************************************************************/
GLOBAL void AppCommSPI_UserSetup(SPI_HandleTypeDef* hspi)
{
	strSpiUse = hspi;

	return;
}

GLOBAL void AppCommSPI_SendSlaveMessage(enSlaveId _SlaveId, enSlaveMsgId _SlaveMsgId)
{
	// Enable slave
	AppComSPI_SetSlaveComm(_SlaveId);

	// Get data and send
	switch (_SlaveMsgId)
	{
	case SLAVE_MSG_ANGLE_KINEMATICS:
		/* Change unit from 1°/bit to 0.01°/bit */
//		strRobotAngleFkData[_SlaveId].q1 = (I32)(strRobotJoint.Joint_1 * 100);
//		strRobotAngleFkData[_SlaveId].q2 = (I32)(strRobotJoint.Joint_2 * 100);
//		strRobotAngleFkData[_SlaveId].q2 = (I32)(strRobotJoint.Joint_3 * 100);
		strRobotAngleFkData.q1 = (I32)(strRobotJoint.Joint_1 * 100);
		strRobotAngleFkData.q2 = (I32)(strRobotJoint.Joint_2 * 100);
		strRobotAngleFkData.q2 = (I32)(strRobotJoint.Joint_3 * 100);

		HAL_SPI_Transmit_DMA(strSpiUse, (U08*)&strRobotAngleFkData, sizeof(strKinematicsData));
		break;
	case SLAVE_MSG_ANGLE_DIRECT:
		HAL_SPI_Transmit_DMA(strSpiUse, (U08*)&strRobotAngleDirectData, sizeof(strKinematicsData));
		break;
	case SLAVE_MSG_FORCE:
	default:
		break;
	}

	return;
}

GLOBAL void AppCommSPI_GetSlaveMessage(void)
{
	return;
}
