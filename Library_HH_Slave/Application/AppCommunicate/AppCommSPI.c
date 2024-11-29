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
#define SPI_BUFFER_SIZE	(20)	// Unit: byte

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
PRIVATE U08 arrSpiRxBuffer[SPI_BUFFER_SIZE] = {0, };		// Data to be store data receive from spi with DMA
PRIVATE U08 arrSpiTxBuffer[SPI_BUFFER_SIZE] = {0, };		// Data to be store data send to spi via DMA

/* Value from Master */
PRIVATE strKinematicsData strRobotAngleData = {0, };

/* Debug variable */
volatile static U32 debug_cnt_SPI_Error = 0;
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE void AppCommSPI_SetupRxBuffer(void);

/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
PRIVATE void AppCommSPI_SetupRxBuffer(void)
{
	HAL_SPI_Receive_IT(strSpiUse, arrSpiRxBuffer, SPI_BUFFER_SIZE);		// Rx is using circle mode

	return;
}

/********************************************************************************
 * GLOBAL FUNCTION IMPLEMENTATION
 ********************************************************************************/
GLOBAL void AppCommSPI_UserSetup(SPI_HandleTypeDef* hspi)
{
	strSpiUse = hspi;

//	AppCommSPI_SetupRxBuffer();

	return;
}

/************************************************************
 ** @brief   User sending function
 ************************************************************/
static U16 test1 = 0x0103;
static U16 test2 = 0x0107;
static U16 test3 = 0x010F;

GLOBAL void AppCommSPI_SendMasterMessage(enSlaveSendMsgId _TxMsgId)
{
	// Set message Id to buffer
	arrSpiTxBuffer[0] = _TxMsgId;

	// Set data to buffer
	switch (_TxMsgId)
	{
	case SLAVE_MSG_POSITION:

//		memcpy(&arrSpiTxBuffer[1], &strRobotArmMotorRx[MOTOR_1_ID].Data.u16Encoder14Bit, sizeof(U16));
//		memcpy(&arrSpiTxBuffer[3], &strRobotArmMotorRx[MOTOR_2_ID].Data.u16Encoder14Bit, sizeof(U16));
//		memcpy(&arrSpiTxBuffer[5], &strRobotArmMotorRx[MOTOR_3_ID].Data.u16Encoder14Bit, sizeof(U16));
//		HAL_SPI_Transmit_DMA(strSpiUse, arrSpiTxBuffer, SPI_BUFFER_SIZE);

		memcpy(&arrSpiTxBuffer[1], &test1, sizeof(U16));
		memcpy(&arrSpiTxBuffer[3], &test2, sizeof(U16));
		memcpy(&arrSpiTxBuffer[5], &test3, sizeof(U16));

		break;
	case SLAVE_MSG_POSITION_SPEED:

		break;
	case SLAVE_MSG_POSITION_SPEED_TORQUE:

		break;
	default:
		// Do nothing
		break;
	}
	// Send data in full-duplex mode
//	HAL_SPI_Transmit_DMA(strSpiUse, arrSpiTxBuffer, SPI_BUFFER_SIZE);
	if (HAL_OK != HAL_SPI_TransmitReceive_DMA(strSpiUse, arrSpiTxBuffer, arrSpiRxBuffer, SPI_BUFFER_SIZE))
	{
		debug_cnt_SPI_Error++;
	}
	return;
}

/************************************************************
 ** @brief   User receive function
 ************************************************************/
GLOBAL void AppCommSPI_GetMasterMessage(void)
{
	/* Step to process receive message
	 *  1. Check Message ID
	 *  2. Save data to corresponding variable base on Message ID
	 *  3. Reset buffer data
	 */
	switch (arrSpiRxBuffer[0])
	{
	case MASTER_MSG_ANGLE_KINEMATICS:
	case MASTER_MSG_ANGLE_DIRECT:
		memcpy(&strRobotAngleData, &arrSpiRxBuffer[1], sizeof(strKinematicsData));
		break;

	case MASTER_MSG_FORCE:

		break;
	default:
		// Do nothing
		break;
	}

	// Reset buffer data
	memset(arrSpiRxBuffer, 0x00, SPI_BUFFER_SIZE);

	// Re-init Rx handler
//	HAL_SPI_Receive_IT(strSpiUse, arrSpiRxBuffer, SPI_BUFFER_SIZE);
	return;
}

/************************************************************
 ** @brief   Interrupt function when finished receiving data
 ************************************************************/
GLOBAL void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == strSpiUse->Instance)		// Check if it's the spi we're using
	{
		AppDataSet_SpiRxMsgFlag(TRUE);
	}

	return;
}

/************************************************************
 ** @brief   Interrupt function when finished sending data
 ************************************************************/
GLOBAL void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == strSpiUse->Instance)		// Check if it's the spi we're using
	{
		memset(arrSpiTxBuffer, 0x00, SPI_BUFFER_SIZE);	// Clear Tx buffer
	}
	return;
}
