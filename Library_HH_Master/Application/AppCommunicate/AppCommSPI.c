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
#define SPI_RX_BUFFER_SIZE	(20)	// Unit: byte


/********************************************************************************
 * PRIVATE TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef struct
{
	I32 q1;
	I32 q2;
	I32 q3;
} strKinematicsData;	// Unit: 0.01 degree/bit

typedef struct
{
	U16 q1;
	U16 q2;
	U16 q3;
} strSlaveDataPosition;
/********************************************************************************
 * PRIVATE VARIABLES
 ********************************************************************************/
PRIVATE SPI_HandleTypeDef* strSpiUse;
PRIVATE U08 arrSpiRxBuffer[SPI_RX_BUFFER_SIZE] = {0, };		// Data to be store data receive from spi with DMA


/* Kinematics Control variable*/
PRIVATE strKinematicsData strRobotAngleFkData = {0, };						// Set value somewhere else, Ex) GUI comm
PRIVATE strKinematicsData strRobotAngleDirectData = { 9000 , -9000 , 0 };	// Set value here
PRIVATE U08 u8SpiSendDirectControl = 0x00;

/* Slave data saved variables */
PRIVATE strSlaveDataPosition strSlavePos = {0 , };

/* Debug variable */

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE void AppCommSPI_SetupRxBuffer(void);
PRIVATE void AppCommSPI_SetSlaveComm(enSlaveId _SlaveId);


/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
PRIVATE void AppCommSPI_SetupRxBuffer(void)
{
	HAL_SPI_Receive_DMA(strSpiUse, arrSpiRxBuffer, SPI_RX_BUFFER_SIZE);		// Rx is using circle mode

	return;
}
PRIVATE void AppCommSPI_SetSlaveComm(enSlaveId _SlaveId)
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

	AppCommSPI_SetupRxBuffer();

	return;
}

GLOBAL void AppCommSPI_SendSlaveMessage(enSlaveId _SlaveId, enMasterSendMsgId _SlaveMsgId)
{
	// Enable slave
	AppCommSPI_SetSlaveComm(_SlaveId);

	// Get data and send
	switch (_SlaveMsgId)
	{
	case MASTER_MSG_ANGLE_KINEMATICS:
		/* Change unit from 1°/bit to 0.01°/bit */
//		strRobotAngleFkData[_SlaveId].q1 = (I32)(strRobotJoint.Joint_1 * 100);
//		strRobotAngleFkData[_SlaveId].q2 = (I32)(strRobotJoint.Joint_2 * 100);
//		strRobotAngleFkData[_SlaveId].q2 = (I32)(strRobotJoint.Joint_3 * 100);
		strRobotAngleFkData.q1 = (I32)(strRobotJoint.Joint_1 * 100);
		strRobotAngleFkData.q2 = (I32)(strRobotJoint.Joint_2 * 100);
		strRobotAngleFkData.q2 = (I32)(strRobotJoint.Joint_3 * 100);

		HAL_SPI_Transmit_DMA(strSpiUse, (U08*)&strRobotAngleFkData, sizeof(strKinematicsData));
		break;
	case MASTER_MSG_ANGLE_DIRECT:
		if (0x00 != u8SpiSendDirectControl)
		{
			HAL_SPI_Transmit_DMA(strSpiUse, (U08*)&strRobotAngleDirectData, sizeof(strKinematicsData));
		}
		if (0x01 == u8SpiSendDirectControl)
		{	// Reset, send 1 time only
			u8SpiSendDirectControl = 0x00;
		}
		else
		{
			// Not reset, re-send multiple-time
		}
		break;
	case MASTER_MSG_FORCE:
	default:
		break;
	}

	return;
}

GLOBAL void AppCommSPI_GetSlaveMessage(void)
{
	/* Step to process receive message
	 *  1. Check Message ID
	 *  2. Save data to corresponding variable base on Message ID
	 *  3. Reset buffer data
	 */
	switch (arrSpiRxBuffer[0])
	{
	case SLAVE_MSG_POSITION:
		memcpy(&strSlavePos, &arrSpiRxBuffer[1], sizeof(strSlavePos));
		break;
	case SLAVE_MSG_POSITION_SPEED:

		break;
	case SLAVE_MSG_POSITION_SPEED_TORQUE:

		break;
	default:
		// Do nothing
		break;
	}

	// Reset buffer data
	memset(arrSpiRxBuffer, 0x00, sizeof(arrSpiRxBuffer));

	return;
}

GLOBAL void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == strSpiUse->Instance)		// Check if it's the spi we're using
	{
		AppDataSet_SpiRxMsgFlag(TRUE);
	}

	return;
}
