/**
 ********************************************************************************
 ** @file    AppCommUART.c
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Mar 14, 2025 (created)
 ** @brief   
 ********************************************************************************
 **/

/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include "AppCommUART.h"

/********************************************************************************
 * EXTERN VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE MACROS AND DEFINES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE TYPEDEFS AND ENUMS
 ********************************************************************************/


/********************************************************************************
 * PRIVATE VARIABLES
 ********************************************************************************/
PRIVATE UART_HandleTypeDef* strUartSlaveLeft;
PRIVATE UART_HandleTypeDef* strUartSlaveRight;
PRIVATE UART_HandleTypeDef* strUartGUI;

/* Rx buffer - Just for receive */
PRIVATE U08 arrUartSlaveLeftRxBuffer[UART_BUFFER_SIZE] = {0, };
PRIVATE U08 arrUartSlaveRightRxBuffer[UART_BUFFER_SIZE] = {0, };
PRIVATE U08 arrUartGUIRxBuffer[UART_BUFFER_SIZE] = {0, };
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
/* Saved Rx buffer - Use in app */
GLOBAL U08 RxDataSlaveLeft[UART_BUFFER_SIZE] = {0, };
GLOBAL U08 RxDataSlaveRight[UART_BUFFER_SIZE] = {0, };
GLOBAL U08 RxDataGUI[UART_BUFFER_SIZE] = {0, };

GLOBAL U08 TxDataSlaveLeft[UART_BUFFER_SIZE] = {0, };
GLOBAL U08 TxDataSlaveRight[UART_BUFFER_SIZE] = {0, };
GLOBAL U08 TxDataGUI[UART_BUFFER_SIZE] = {0, };

/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/


/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/


/********************************************************************************
 * GLOBAL FUNCTION IMPLEMENTATION
 ********************************************************************************/
GLOBAL void AppCommUART_UserSetup(UART_HandleTypeDef* huart, enUartNode _node)
{
	switch (_node)
	{
	case UART_NODE_SLAVE_1:
		strUartSlaveLeft = huart;
		break;
	case UART_NODE_SLAVE_2:
		strUartSlaveRight = huart;
		break;
	case UART_NODE_GUI:
		strUartGUI = huart;
		break;
	default:
		// Not support
		break;
	}

	return;
}

/**
===============================================================================
                  ##### HUNG HOANG UART PROCESS SUMMARY #####
===============================================================================
  PROCESS:
      (Step 1) User call the "AppCommUART_SendMsg" anywhere in the APP process
      (Step 2) Function "AppCommUART_SendMsg" auto:
                   + Check if the Tx-wait flag is TRUE -> return
                   + If FALSE, package the new message and send
                   + Set the Tx-wait flag
                   + Wait (Step 3)
      (Step 3) Function "HAL_UART_TxCpltCallback" auto:
                   + Called when finished Tx transfer
                   + Reset the Tx-wait flag
                   + Increase the Tx-counter (for debug)
                   + Call (Step 4)
      (Step 4) Function "AppCommUart_RecvMsgStart" auto:
                   + Check if the Rx-wait flag is TRUE -> return
                   + If FALSE, start waiting Rx message
                   + Set the Rx-wait flag
                   + Wait (Step 5)
      (Step 5) Function "HAL_UARTEx_RxEventCallback" auto:
                   + Called when received Rx
                   + Copy Rx data from UART buffer to User buffer
                   + Increase the Rx-counter (for debug)
                   + Reset the Rx-wait flag
                   + Set the Rx-new flag for user know
      ( User ) Can do below things:
                   + Check the Tx-wait for knowing Tx is send or not
                   + Check the Rx-wait for knowing Rx is received or not
                   + Check the Rx-new for handling the Rx data
                   + Reset the Rx-new (MUST DO)
                   + Debug Tx-counter for total Tx message
                   + Debug Rx-counter for total Rx message
                   + Don't use SET/RESEET function of Rx/Tx-wait outside this file
 */


/************ UART TX MANAGE FUNCTION  ************/
/* Callback function when finished sending uart */
GLOBAL void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		AppDataSet_UartTxWaitFlag(UART_NODE_GUI, FALSE);	// Msg is send
		AppDataSet_UartTxMsgCnt(UART_NODE_GUI);			// Increase counter
		AppCommUart_RecvMsgStart(UART_NODE_GUI);		// Looking for Rx msg
	}
	else if (huart->Instance == USART3)
	{
		AppDataSet_UartTxWaitFlag(UART_NODE_SLAVE_2, FALSE);	// Msg is send
		AppDataSet_UartTxMsgCnt(UART_NODE_SLAVE_2);			// Increase counter
		AppCommUart_RecvMsgStart(UART_NODE_SLAVE_2);		// Looking for Rx msg
	}
	else if (huart->Instance == USART6)
	{
		AppDataSet_UartTxWaitFlag(UART_NODE_SLAVE_1, FALSE);	// Msg is send
		AppDataSet_UartTxMsgCnt(UART_NODE_SLAVE_1);			// Increase counter
		AppCommUart_RecvMsgStart(UART_NODE_SLAVE_1);		// Looking for Rx msg
	}
	else
	{

		// Do nothing
	}

	return;
}

/* Sending function */
/* TODO: (DeepSeek check)
 * 		1/ Error Handling: Consider adding a return value to AppCommUART_SendMsg to indicate success or failure
 * 		2/ Default Case in _txMsgId Switch: The default case in the _txMsgId switch does nothing and returns.
 * 		                                    This is fine, but you might want to log an error or handle it differently.
 */
GLOBAL void AppCommUART_SendMsg(enUartNode _node, enUartMsg _txMsgId)
{
	// 1. Safety check
	if (TRUE == AppDataGet_UartTxWaitFlag(_node))
	{
		return;
	}


	// 2. Initialize variables
	static UART_HandleTypeDef* uartGoal;
	static U08* sourceTxData;
	U16 sizeSend = 0;	// if not set value, TxErrCnt++
	U08 checksum = 0x00;
	U08 _arm = LEFT_ARM;


	// 3. Get UART target and data
	switch (_node)
	{
#if SLAVE_1_ENA
	case UART_NODE_SLAVE_1:
		_arm = LEFT_ARM;
		uartGoal = strUartSlaveLeft;
		sourceTxData = TxDataSlaveLeft;
		break;
#endif
#if SLAVE_2_ENA
	case UART_NODE_SLAVE_2:
		_arm = RIGHT_ARM;
		uartGoal = strUartSlaveRight;
		sourceTxData = TxDataSlaveRight;
		break;
#endif
	case UART_NODE_GUI:
		uartGoal = strUartGUI;
		sourceTxData = TxDataGUI;
		break;
	default:
		// Do nothing and return
		return;
	}


	// 4. Prepare data
	switch (_txMsgId)
	{
	/* --------- Message for SLAVE --------- */
	case UART_MSG_INIT:
		sourceTxData[0] = MSG_INIT_BYTE_0;
		sourceTxData[1] = MSG_INIT_BYTE_1;
		sourceTxData[2] = MSG_INIT_LENGTH;
		sizeSend = MSG_INIT_LENGTH;
		break;

	case UART_MSG_MOTOR_DATA:
		sourceTxData[0] = MSG_DATA_REQUEST_BYTE_0;
		sourceTxData[1] = MSG_DATA_REQUEST_BYTE_1;
		sourceTxData[2] = MSG_DATA_REQUEST_LENGTH;
		sizeSend = MSG_DATA_REQUEST_LENGTH;
		break;

	case UART_MSG_MOTOR_CONTROL_POS:
		// Payload: q1-dq1-dir1 - q2-dq2-dir2 - q3-dq3-dir3
		sourceTxData[0] = MSG_CONTROL_POS_BYTE_0;
		sourceTxData[1] = MSG_CONTROL_POS_BYTE_1;
		sourceTxData[2] = MSG_CONTROL_POS_LENGTH;
		sizeSend = MSG_CONTROL_POS_LENGTH;

		memcpy(&sourceTxData[4],  &myRobotCommand[_arm].JointPos[0].Angle,     sizeof(float));
		memcpy(&sourceTxData[8],  &myRobotCommand[_arm].JointPos[0].Speed,     sizeof(U16));
		memcpy(&sourceTxData[10], &myRobotCommand[_arm].JointPos[0].Direction, sizeof(U08));
		memcpy(&sourceTxData[11], &myRobotCommand[_arm].JointPos[1].Angle,     sizeof(float));
		memcpy(&sourceTxData[15], &myRobotCommand[_arm].JointPos[1].Speed,     sizeof(U16));
		memcpy(&sourceTxData[17], &myRobotCommand[_arm].JointPos[1].Direction, sizeof(U08));
		memcpy(&sourceTxData[18], &myRobotCommand[_arm].JointPos[2].Angle,     sizeof(float));
		memcpy(&sourceTxData[22], &myRobotCommand[_arm].JointPos[2].Speed,     sizeof(U16));
		memcpy(&sourceTxData[24], &myRobotCommand[_arm].JointPos[2].Direction, sizeof(U08));

		 // Calculate checksum (payload only: byte 4-24)
		for (int i = 4; i < MSG_CONTROL_POS_LENGTH; i++)
		{
			checksum ^= sourceTxData[i];  // XOR checksum
		}
		sourceTxData[3] = checksum;
		break;;

	case UART_MSG_MOTOR_CONTROL_TOR:
		sourceTxData[0] = MSG_CONTROL_TOR_BYTE_0;
		sourceTxData[1] = MSG_CONTROL_TOR_BYTE_1;
		sourceTxData[2] = MSG_CONTROL_TOR_LENGTH;
		sizeSend = MSG_CONTROL_TOR_LENGTH;

		memcpy(&sourceTxData[4],  &myRobotCommand[_arm].JointTor[0].CurrentTor, sizeof(float));
		memcpy(&sourceTxData[8],  &myRobotCommand[_arm].JointTor[1].CurrentTor, sizeof(float));
		memcpy(&sourceTxData[12], &myRobotCommand[_arm].JointTor[2].CurrentTor, sizeof(float));

		 // Calculate checksum (payload only: byte 4-24)
		for (int i = 4; i < MSG_CONTROL_TOR_LENGTH; i++)
		{
			checksum ^= sourceTxData[i];  // XOR checksum
		}
		sourceTxData[3] = checksum;
		break;

	/* --------- Message for GUI --------- */
	case UART_MSG_GUI_DATA_1_LEFT:
		sourceTxData[0] = MSG_GUI_DATA_1_SING_BYTE_0;
		sourceTxData[1] = MSG_GUI_DATA_1_SING_BYTE_1;
		sourceTxData[2] = MSG_GUI_DATA_1_SING_LENGTH;
		sizeSend = MSG_GUI_DATA_1_SING_LENGTH;

		// Cast sourceTxData to 32-bit pointer for direct access
		U32* txData32Left = (U32*)sourceTxData;

		txData32Left[1] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Position; // sourceTxData[4-7]
		txData32Left[2] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Speed;    // sourceTxData[8-11]
		txData32Left[3] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Accel;    // sourceTxData[12-15]
		txData32Left[4] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Position; // sourceTxData[16-19]
		txData32Left[5] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Speed;    // sourceTxData[20-23]
		txData32Left[6] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Accel;    // sourceTxData[24-27]
		txData32Left[7] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Position; // sourceTxData[28-31]
		txData32Left[8] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Speed;    // sourceTxData[32-35]
		txData32Left[9] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Accel;    // sourceTxData[36-39]

		 // Calculate checksum (payload only: byte 4-39)
		for (int i = 4; i < MSG_GUI_DATA_1_SING_LENGTH; i++)
		{
			checksum ^= sourceTxData[i];  // XOR checksum
		}
		sourceTxData[3] = checksum;
		break;

	case UART_MSG_GUI_DATA_1_RIGHT:
		sourceTxData[0] = MSG_GUI_DATA_1_SING_BYTE_0;
		sourceTxData[1] = MSG_GUI_DATA_1_SING_BYTE_1;
		sourceTxData[2] = MSG_GUI_DATA_1_SING_LENGTH;
		sizeSend = MSG_GUI_DATA_1_SING_LENGTH;

		// Cast sourceTxData to 32-bit pointer for direct access
		U32* txData32Right = (U32*)sourceTxData;

		txData32Right[1] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Position; // sourceTxData[4-7]
		txData32Right[2] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Speed;    // sourceTxData[8-11]
		txData32Right[3] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Accel;    // sourceTxData[12-15]
		txData32Right[4] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Position; // sourceTxData[16-19]
		txData32Right[5] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Speed;    // sourceTxData[20-23]
		txData32Right[6] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Accel;    // sourceTxData[24-27]
		txData32Right[7] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Position; // sourceTxData[28-31]
		txData32Right[8] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Speed;    // sourceTxData[32-35]
		txData32Right[9] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Accel;    // sourceTxData[36-39]

		 // Calculate checksum (payload only: byte 4-39)
		for (int i = 4; i < MSG_GUI_DATA_1_SING_LENGTH; i++)
		{
			checksum ^= sourceTxData[i];  // XOR checksum
		}
		sourceTxData[3] = checksum;
		break;

	case UART_MSG_GUI_DATA_1_DUAL:
		sourceTxData[0] = MSG_GUI_DATA_1_DUAL_BYTE_0;
		sourceTxData[1] = MSG_GUI_DATA_1_DUAL_BYTE_1;
		sourceTxData[2] = MSG_GUI_DATA_1_DUAL_LENGTH;
		sizeSend = MSG_GUI_DATA_1_DUAL_LENGTH;

		// Cast sourceTxData to 32-bit pointer for direct access
		U32* txData32Dual = (U32*)sourceTxData;

#ifndef MASTER_GUI_TP_CHECK
		txData32Dual[1] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Position; // sourceTxData[4-7]
		txData32Dual[2] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Speed;    // sourceTxData[8-11]
		txData32Dual[3] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Accel;    // sourceTxData[12-15]
		txData32Dual[4] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Position; // sourceTxData[16-19]
		txData32Dual[5] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Speed;    // sourceTxData[20-23]
		txData32Dual[6] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Accel;    // sourceTxData[24-27]
		txData32Dual[7] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Position; // sourceTxData[28-31]
		txData32Dual[8] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Speed;    // sourceTxData[32-35]
		txData32Dual[9] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Accel;    // sourceTxData[36-39]

		txData32Dual[10] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Position; // sourceTxData[40-43]
		txData32Dual[11] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Speed;    // sourceTxData[44-47]
		txData32Dual[12] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Accel;    // sourceTxData[48-51]
		txData32Dual[13] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Position; // sourceTxData[52-55]
		txData32Dual[14] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Speed;    // sourceTxData[56-59]
		txData32Dual[15] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Accel;    // sourceTxData[60-63]
		txData32Dual[16] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Position; // sourceTxData[64-67]
		txData32Dual[17] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Speed;    // sourceTxData[68-71]
		txData32Dual[18] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Accel;    // sourceTxData[72-75]
#else
		float Pos, Vel, Accel;
		Pos   = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[0].currPos);
		Vel   = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[0].currVel);
		Accel = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[0].currAccel);
		txData32Dual[1] = *(U32*)&Pos;
		txData32Dual[2] = *(U32*)&Vel;
		txData32Dual[3] = *(U32*)&Accel;
		Pos   = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[1].currPos);
		Vel   = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[1].currVel);
		Accel = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[1].currAccel);
		txData32Dual[4] = *(U32*)&Pos;
		txData32Dual[5] = *(U32*)&Vel;
		txData32Dual[6] = *(U32*)&Accel;
		Pos   = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[2].currPos);
		Vel   = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[2].currVel);
		Accel = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[2].currAccel);
		txData32Dual[7] = *(U32*)&Pos;
		txData32Dual[8] = *(U32*)&Vel;
		txData32Dual[9] = *(U32*)&Accel;

		Pos   = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[0].currPos);
		Vel   = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[0].currVel);
		Accel = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[0].currAccel);
		txData32Dual[10] = *(U32*)&Pos;
		txData32Dual[11] = *(U32*)&Vel;
		txData32Dual[12] = *(U32*)&Accel;
		Pos   = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[1].currPos);
		Vel   = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[1].currVel);
		Accel = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[1].currAccel);
		txData32Dual[13] = *(U32*)&Pos;
		txData32Dual[14] = *(U32*)&Vel;
		txData32Dual[15] = *(U32*)&Accel;
		Pos   = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[2].currPos);
		Vel   = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[2].currVel);
		Accel = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[2].currAccel);
		txData32Dual[16] = *(U32*)&Pos;
		txData32Dual[17] = *(U32*)&Vel;
		txData32Dual[18] = *(U32*)&Accel;
#endif /* MASTER_GUI_TP_CHECK */

		 // Calculate checksum (payload only: byte 4-39)
		for (int i = 4; i < MSG_GUI_DATA_1_DUAL_LENGTH; i++)
		{
			checksum ^= sourceTxData[i];  // XOR checksum
		}
		sourceTxData[3] = checksum;
		break;

	case UART_MSG_GUI_DATA_2_LEFT:
	case UART_MSG_GUI_DATA_2_RIGHT:
	case UART_MSG_GUI_DATA_2_DUAL:
		// TODO Create GUI DATA 2 send message
		break;

//	case UART_MSG_MOTOR_CONTROL_VEL:
	default:
		// Do nothing and return
		return;
	}



	// 5. Send the message
	if (HAL_OK != HAL_UART_Transmit_DMA(uartGoal, sourceTxData, sizeSend))
	{
		AppDataSet_UartTxErrCnt(_node);

	}


	// 6. Set sending flag
	AppDataSet_UartTxWaitFlag(_node, TRUE);

	// TODO: 7. Handle return value


	return;
}

/************ UART RX MANAGE FUNCTION  ************/
GLOBAL void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
/* Save Rx data from UART buffer to User buffer
 * Rx counter++
 * Rx-wait flag RESET
 * Rx-new flag SET
 */
	if (huart->Instance == USART2)
	{
		memcpy(RxDataGUI, arrUartGUIRxBuffer, Size);
		AppDataSet_UartRxMsgCnt(UART_NODE_GUI);
		AppDataSet_UartRxWaitFlag(UART_NODE_GUI, FALSE);
		AppDataSet_UartRxNewFlag(UART_NODE_GUI, TRUE);
	}
	else if (huart->Instance == USART3)
	{
		memcpy(RxDataSlaveRight, arrUartSlaveRightRxBuffer, Size);
		AppDataSet_UartRxMsgCnt(UART_NODE_SLAVE_2);
		AppDataSet_UartRxWaitFlag(UART_NODE_SLAVE_2, FALSE);
		AppDataSet_UartRxNewFlag(UART_NODE_SLAVE_2, TRUE);
	}
	else if (huart->Instance == USART6)
	{
		memcpy(RxDataSlaveLeft, arrUartSlaveLeftRxBuffer, Size);
		AppDataSet_UartRxMsgCnt(UART_NODE_SLAVE_1);
		AppDataSet_UartRxWaitFlag(UART_NODE_SLAVE_1, FALSE);
		AppDataSet_UartRxNewFlag(UART_NODE_SLAVE_1, TRUE);
	}
	else
	{
		// Do nothing
	}

	return;
}
/* Function initialize UART DMA Rx IDLE for corresponding node */
GLOBAL void AppCommUart_RecvMsgStart(enUartNode _node)
{
// 1. Safety check
	if (TRUE == AppDataGet_UartRxWaitFlag(_node))	// Check for duplicate DMA start or not
	{
		// The DMA Rx is already started
		return;
	}

// 2. Start Rx DMA
	switch (_node)
	{
	case UART_NODE_SLAVE_1:
		HAL_UARTEx_ReceiveToIdle_DMA(strUartSlaveLeft, arrUartSlaveLeftRxBuffer, UART_BUFFER_SIZE);
		break;
	case UART_NODE_SLAVE_2:
		HAL_UARTEx_ReceiveToIdle_DMA(strUartSlaveRight, arrUartSlaveRightRxBuffer, UART_BUFFER_SIZE);
		break;
	case UART_NODE_GUI:
		HAL_UARTEx_ReceiveToIdle_DMA(strUartGUI, arrUartGUIRxBuffer, UART_BUFFER_SIZE);
		break;
	default:
		break;
	}

// 3. Set Rx-wait flag
	AppDataSet_UartRxWaitFlag(_node, TRUE);		// After start Rx DMA
	return;
}

/************ UART ERROR MANAGE FUNCTION  ************/
GLOBAL void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	U32 error = HAL_UART_ERROR_NONE;

	// Get error
	error = HAL_UART_GetError(huart);

	// Set Error count
	switch (error)
	{

	case HAL_UART_ERROR_PE:   /*!< Parity error        */
	case HAL_UART_ERROR_NE:   /*!< Noise error         */
	case HAL_UART_ERROR_FE:   /*!< Frame error         */
	case HAL_UART_ERROR_ORE:  /*!< Overrun error       */
		if (huart->Instance == USART2)		AppDataSet_UartRxErrCnt(UART_NODE_GUI);
		else if (huart->Instance == USART3)	AppDataSet_UartRxErrCnt(UART_NODE_SLAVE_2);
		else if (huart->Instance == USART6)	AppDataSet_UartRxErrCnt(UART_NODE_SLAVE_1);
		else
		{
			// Do nothing
		}
		break;
	case HAL_UART_ERROR_DMA:  /*!< DMA transfer error  */
		// Could be DMA Tx or DMA Rx
		// Need to implement more
		break;
	case HAL_UART_ERROR_NONE: /*!< No error            */
	default:
		// Do nothing
		break;
	}


	return;
}
