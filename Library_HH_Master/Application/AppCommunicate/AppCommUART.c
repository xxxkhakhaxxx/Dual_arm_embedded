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

GLOBAL U08 TxDataSlaveLeft[UART_BUFFER_SIZE] __attribute__((aligned(4)))= {0, };
GLOBAL U08 TxDataSlaveRight[UART_BUFFER_SIZE] __attribute__((aligned(4)))= {0, };
GLOBAL U08 TxDataGUI[UART_BUFFER_SIZE] __attribute__((aligned(4)))= {0, };

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
	static U08* sourceTxData08;		// Pointer for casting U08 variable
	static U32* sourceTxData32;		// Pointer for casting float varialbe: be carefull when using, ensure 4-byte aligned
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
		sourceTxData08 = (U08*)TxDataSlaveLeft;
		sourceTxData32 = (U32*)TxDataSlaveLeft;
		break;
#endif
#if SLAVE_2_ENA
	case UART_NODE_SLAVE_2:
		_arm = RIGHT_ARM;
		uartGoal = strUartSlaveRight;
		sourceTxData08 = (U08*)TxDataSlaveRight;
		sourceTxData32 = (U32*)TxDataSlaveRight;
		break;
#endif
	case UART_NODE_GUI:
		uartGoal = strUartGUI;
		sourceTxData08 = (U08*)TxDataGUI;
		sourceTxData32 = (U32*)TxDataGUI;
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
		sourceTxData08[0] = MSG_INIT_BYTE_0;
		sourceTxData08[1] = MSG_INIT_BYTE_1;
		sourceTxData08[2] = MSG_INIT_LENGTH;
		sizeSend = MSG_INIT_LENGTH;
		break;

	case UART_MSG_MOTOR_DATA:
		sourceTxData08[0] = MSG_DATA_REQUEST_BYTE_0;
		sourceTxData08[1] = MSG_DATA_REQUEST_BYTE_1;
		sourceTxData08[2] = MSG_DATA_REQUEST_LENGTH;
		sizeSend = MSG_DATA_REQUEST_LENGTH;
		break;

	case UART_MSG_MOTOR_CONTROL_POS:
		// Payload: q1-dq1-dir1 - q2-dq2-dir2 - q3-dq3-dir3
		sourceTxData08[0] = MSG_CONTROL_POS_BYTE_0;
		sourceTxData08[1] = MSG_CONTROL_POS_BYTE_1;
		sourceTxData08[2] = MSG_CONTROL_POS_LENGTH;
		sizeSend = MSG_CONTROL_POS_LENGTH;

		memcpy(&sourceTxData08[4],  &myRobotCommand[_arm].JointPos[0].Angle,     sizeof(float));
		memcpy(&sourceTxData08[8],  &myRobotCommand[_arm].JointPos[0].Speed,     sizeof(U16));
		memcpy(&sourceTxData08[10], &myRobotCommand[_arm].JointPos[0].Direction, sizeof(U08));
		memcpy(&sourceTxData08[11], &myRobotCommand[_arm].JointPos[1].Angle,     sizeof(float));
		memcpy(&sourceTxData08[15], &myRobotCommand[_arm].JointPos[1].Speed,     sizeof(U16));
		memcpy(&sourceTxData08[17], &myRobotCommand[_arm].JointPos[1].Direction, sizeof(U08));
		memcpy(&sourceTxData08[18], &myRobotCommand[_arm].JointPos[2].Angle,     sizeof(float));
		memcpy(&sourceTxData08[22], &myRobotCommand[_arm].JointPos[2].Speed,     sizeof(U16));
		memcpy(&sourceTxData08[24], &myRobotCommand[_arm].JointPos[2].Direction, sizeof(U08));

		 // Calculate checksum (payload only: byte 4-24)
		for (int i = 4; i < MSG_CONTROL_POS_LENGTH; i++)
		{
			checksum ^= sourceTxData08[i];  // XOR checksum
		}
		sourceTxData08[3] = checksum;
		break;;

	case UART_MSG_MOTOR_CONTROL_TOR:
		sourceTxData08[0] = MSG_CONTROL_TOR_BYTE_0;
		sourceTxData08[1] = MSG_CONTROL_TOR_BYTE_1;
		sourceTxData08[2] = MSG_CONTROL_TOR_LENGTH;
		sizeSend = MSG_CONTROL_TOR_LENGTH;

		memcpy(&sourceTxData08[4],  &myRobotCommand[_arm].JointTor[0].Tor, sizeof(float));
		memcpy(&sourceTxData08[8],  &myRobotCommand[_arm].JointTor[1].Tor, sizeof(float));
		memcpy(&sourceTxData08[12], &myRobotCommand[_arm].JointTor[2].Tor, sizeof(float));

		 // Calculate checksum (payload only: byte 4-24)
		for (int i = 4; i < MSG_CONTROL_TOR_LENGTH; i++)
		{
			checksum ^= sourceTxData08[i];  // XOR checksum
		}
		sourceTxData08[3] = checksum;
		break;

	/* --------- Message for GUI --------- */
	case UART_MSG_GUI_DATA_1_LEFT:
		sourceTxData08[0] = MSG_GUI_DATA_1_SING_BYTE_0;
		sourceTxData08[1] = MSG_GUI_DATA_1_SING_BYTE_1;
		sourceTxData08[2] = MSG_GUI_DATA_1_SING_LENGTH;
		sizeSend = MSG_GUI_DATA_1_SING_LENGTH;

		sourceTxData32[1] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Position; // sourceTxData08[4-7]
		sourceTxData32[2] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Speed;    // sourceTxData08[8-11]
		sourceTxData32[3] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Accel;    // sourceTxData08[12-15]
		sourceTxData32[4] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Position; // sourceTxData08[16-19]
		sourceTxData32[5] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Speed;    // sourceTxData08[20-23]
		sourceTxData32[6] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Accel;    // sourceTxData08[24-27]
		sourceTxData32[7] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Position; // sourceTxData08[28-31]
		sourceTxData32[8] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Speed;    // sourceTxData08[32-35]
		sourceTxData32[9] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Accel;    // sourceTxData08[36-39]

		 // Calculate checksum (payload only: byte 4-39)
		for (int i = 4; i < MSG_GUI_DATA_1_SING_LENGTH; i++)
		{
			checksum ^= sourceTxData08[i];  // XOR checksum
		}
		sourceTxData08[3] = checksum;
		break;

	case UART_MSG_GUI_DATA_1_RIGHT:
		sourceTxData08[0] = MSG_GUI_DATA_1_SING_BYTE_0;
		sourceTxData08[1] = MSG_GUI_DATA_1_SING_BYTE_1;
		sourceTxData08[2] = MSG_GUI_DATA_1_SING_LENGTH;
		sizeSend = MSG_GUI_DATA_1_SING_LENGTH;

		sourceTxData32[1] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Position; // sourceTxData08[4-7]
		sourceTxData32[2] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Speed;    // sourceTxData08[8-11]
		sourceTxData32[3] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Accel;    // sourceTxData08[12-15]
		sourceTxData32[4] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Position; // sourceTxData08[16-19]
		sourceTxData32[5] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Speed;    // sourceTxData08[20-23]
		sourceTxData32[6] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Accel;    // sourceTxData08[24-27]
		sourceTxData32[7] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Position; // sourceTxData08[28-31]
		sourceTxData32[8] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Speed;    // sourceTxData08[32-35]
		sourceTxData32[9] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Accel;    // sourceTxData08[36-39]

		 // Calculate checksum (payload only: byte 4-39)
		for (int i = 4; i < MSG_GUI_DATA_1_SING_LENGTH; i++)
		{
			checksum ^= sourceTxData08[i];  // XOR checksum
		}
		sourceTxData08[3] = checksum;
		break;

	case UART_MSG_GUI_DATA_1_DUAL:
#ifndef TEST_MOTOR_FILTER
		sourceTxData08[0] = MSG_GUI_DATA_1_DUAL_BYTE_0;
		sourceTxData08[1] = MSG_GUI_DATA_1_DUAL_BYTE_1;
		sourceTxData08[2] = MSG_GUI_DATA_1_DUAL_LENGTH;
		sizeSend = MSG_GUI_DATA_1_DUAL_LENGTH;

#ifndef MASTER_GUI_TP_CHECK
		sourceTxData32[1] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Position; // sourceTxData08[4-7]
		sourceTxData32[2] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Speed;    // sourceTxData08[8-11]
		sourceTxData32[3] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Accel;    // sourceTxData08[12-15]
		sourceTxData32[4] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Position; // sourceTxData08[16-19]
		sourceTxData32[5] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Speed;    // sourceTxData08[20-23]
		sourceTxData32[6] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Accel;    // sourceTxData08[24-27]
		sourceTxData32[7] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Position; // sourceTxData08[28-31]
		sourceTxData32[8] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Speed;    // sourceTxData08[32-35]
		sourceTxData32[9] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Accel;    // sourceTxData08[36-39]

		sourceTxData32[10] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Position; // sourceTxData08[40-43]
		sourceTxData32[11] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Speed;    // sourceTxData08[44-47]
		sourceTxData32[12] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Accel;    // sourceTxData08[48-51]
		sourceTxData32[13] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Position; // sourceTxData08[52-55]
		sourceTxData32[14] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Speed;    // sourceTxData08[56-59]
		sourceTxData32[15] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Accel;    // sourceTxData08[60-63]
		sourceTxData32[16] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Position; // sourceTxData08[64-67]
		sourceTxData32[17] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Speed;    // sourceTxData08[68-71]
		sourceTxData32[18] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Accel;    // sourceTxData08[72-75]
#else
		float Pos, Vel, Accel;
		Pos   = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[0].currPos);
		Vel   = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[0].currVel);
		Accel = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[0].currAccel);
		sourceTxData32[1] = *(U32*)&Pos;
		sourceTxData32[2] = *(U32*)&Vel;
		sourceTxData32[3] = *(U32*)&Accel;
		Pos   = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[1].currPos);
		Vel   = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[1].currVel);
		Accel = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[1].currAccel);
		sourceTxData32[4] = *(U32*)&Pos;
		sourceTxData32[5] = *(U32*)&Vel;
		sourceTxData32[6] = *(U32*)&Accel;
		Pos   = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[2].currPos);
		Vel   = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[2].currVel);
		Accel = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[2].currAccel);
		sourceTxData32[7] = *(U32*)&Pos;
		sourceTxData32[8] = *(U32*)&Vel;
		sourceTxData32[9] = *(U32*)&Accel;

		Pos   = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[0].currPos);
		Vel   = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[0].currVel);
		Accel = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[0].currAccel);
		sourceTxData32[10] = *(U32*)&Pos;
		sourceTxData32[11] = *(U32*)&Vel;
		sourceTxData32[12] = *(U32*)&Accel;
		Pos   = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[1].currPos);
		Vel   = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[1].currVel);
		Accel = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[1].currAccel);
		sourceTxData32[13] = *(U32*)&Pos;
		sourceTxData32[14] = *(U32*)&Vel;
		sourceTxData32[15] = *(U32*)&Accel;
		Pos   = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[2].currPos);
		Vel   = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[2].currVel);
		Accel = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[2].currAccel);
		sourceTxData32[16] = *(U32*)&Pos;
		sourceTxData32[17] = *(U32*)&Vel;
		sourceTxData32[18] = *(U32*)&Accel;
#endif /* MASTER_GUI_TP_CHECK */

		 // Calculate checksum (payload only: byte 4-39)
		for (int i = 4; i < MSG_GUI_DATA_1_DUAL_LENGTH; i++)
		{
			checksum ^= sourceTxData08[i];  // XOR checksum
		}
		sourceTxData08[3] = checksum;

#else	/* TEST_MOTOR_FILTER */
		sourceTxData08[0] = MSG_GUI_DATA_3_DUAL_BYTE_0;
		sourceTxData08[1] = MSG_GUI_DATA_3_DUAL_BYTE_1;
		sourceTxData08[2] = MSG_GUI_DATA_3_DUAL_LENGTH;
		sizeSend = MSG_GUI_DATA_3_DUAL_LENGTH;

		sourceTxData32[1] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Position;
		sourceTxData32[2] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Speed;
		sourceTxData32[3] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Accel;
		sourceTxData32[4] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Speedf;
		sourceTxData32[5] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Accelf;

		sourceTxData32[6] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Position;
		sourceTxData32[7] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Speed;
		sourceTxData32[8] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Accel;
		sourceTxData32[9] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Speedf;
		sourceTxData32[10]= *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Accelf;

		sourceTxData32[11] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Position;
		sourceTxData32[12] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Speed;
		sourceTxData32[13] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Accel;
		sourceTxData32[14] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Speedf;
		sourceTxData32[15] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Accelf;

		sourceTxData32[16] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Position;
		sourceTxData32[17] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Speed;
		sourceTxData32[18] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Accel;
		sourceTxData32[19] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Speedf;
		sourceTxData32[20] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Accelf;

		sourceTxData32[21] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Position;
		sourceTxData32[22] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Speed;
		sourceTxData32[23] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Accel;
		sourceTxData32[24] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Speedf;
		sourceTxData32[25] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Accelf;

		sourceTxData32[26] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Position;
		sourceTxData32[27] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Speed;
		sourceTxData32[28] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Accel;
		sourceTxData32[29] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Speedf;
		sourceTxData32[30] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Accelf;

		 // Calculate checksum (payload only: byte 4-39)
		for (int i = 4; i < MSG_GUI_DATA_3_DUAL_LENGTH; i++)
		{
			checksum ^= sourceTxData08[i];  // XOR checksum
		}
		sourceTxData08[3] = checksum;
#endif
		break;

	case UART_MSG_GUI_DATA_2_LEFT:
		sourceTxData08[0] = MSG_GUI_DATA_2_SING_BYTE_0;
		sourceTxData08[1] = MSG_GUI_DATA_2_SING_BYTE_1;
		sourceTxData08[2] = MSG_GUI_DATA_2_SING_LENGTH;
		sizeSend = MSG_GUI_DATA_2_SING_LENGTH;

		sourceTxData32[1] = *(U32*)&myRobotTrajectory[LEFT_ARM].Joint[0].currPos; // sourceTxData08[4-7]
		sourceTxData32[2] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Position;  // sourceTxData08[8-11]
		sourceTxData32[3] = *(U32*)&myRobotCommand[LEFT_ARM].JointTor[0].Tor;     // sourceTxData08[12-15]
		sourceTxData32[4] = *(U32*)&myRobotTrajectory[LEFT_ARM].Joint[1].currPos; // sourceTxData08[16-19]
		sourceTxData32[5] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Position;  // sourceTxData08[20-23]
		sourceTxData32[6] = *(U32*)&myRobotCommand[LEFT_ARM].JointTor[1].Tor;     // sourceTxData08[24-27]
		sourceTxData32[7] = *(U32*)&myRobotTrajectory[LEFT_ARM].Joint[2].currPos; // sourceTxData08[28-31]
		sourceTxData32[8] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Position;  // sourceTxData08[32-35]
		sourceTxData32[9] = *(U32*)&myRobotCommand[LEFT_ARM].JointTor[2].Tor;     // sourceTxData08[36-39]

		 // Calculate checksum (payload only: byte 4-39)
		for (int i = 4; i < MSG_GUI_DATA_2_SING_LENGTH; i++)
		{
			checksum ^= sourceTxData08[i];  // XOR checksum
		}
		sourceTxData08[3] = checksum;
		break;

	case UART_MSG_GUI_DATA_2_RIGHT:
		sourceTxData08[0] = MSG_GUI_DATA_2_SING_BYTE_0;
		sourceTxData08[1] = MSG_GUI_DATA_2_SING_BYTE_1;
		sourceTxData08[2] = MSG_GUI_DATA_2_SING_LENGTH;
		sizeSend = MSG_GUI_DATA_2_SING_LENGTH;

		sourceTxData32[1] = *(U32*)&myRobotTrajectory[RIGHT_ARM].Joint[0].currPos; // sourceTxData08[4-7]
		sourceTxData32[2] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Position;  // sourceTxData08[8-11]
		sourceTxData32[3] = *(U32*)&myRobotCommand[RIGHT_ARM].JointTor[0].Tor;     // sourceTxData08[12-15]
		sourceTxData32[4] = *(U32*)&myRobotTrajectory[RIGHT_ARM].Joint[1].currPos; // sourceTxData08[16-19]
		sourceTxData32[5] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Position;  // sourceTxData08[20-23]
		sourceTxData32[6] = *(U32*)&myRobotCommand[RIGHT_ARM].JointTor[1].Tor;     // sourceTxData08[24-27]
		sourceTxData32[7] = *(U32*)&myRobotTrajectory[RIGHT_ARM].Joint[2].currPos; // sourceTxData08[28-31]
		sourceTxData32[8] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Position;  // sourceTxData08[32-35]
		sourceTxData32[9] = *(U32*)&myRobotCommand[RIGHT_ARM].JointTor[2].Tor;     // sourceTxData08[36-39]

		 // Calculate checksum (payload only: byte 4-39)
		for (int i = 4; i < MSG_GUI_DATA_2_SING_LENGTH; i++)
		{
			checksum ^= sourceTxData08[i];  // XOR checksum
		}
		sourceTxData08[3] = checksum;
		break;

	case UART_MSG_GUI_DATA_2_DUAL:
		sourceTxData08[0] = MSG_GUI_DATA_2_DUAL_BYTE_0;
		sourceTxData08[1] = MSG_GUI_DATA_2_DUAL_BYTE_1;
		sourceTxData08[2] = MSG_GUI_DATA_2_DUAL_LENGTH;
		sizeSend = MSG_GUI_DATA_2_DUAL_LENGTH;

		sourceTxData32[1] = *(U32*)&myRobotTrajectory[LEFT_ARM].Joint[0].currPos;
		sourceTxData32[2] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Position;
		sourceTxData32[3] = *(U32*)&myRobotCommand[LEFT_ARM].JointTor[0].Tor;
		sourceTxData32[4] = *(U32*)&myRobotTrajectory[LEFT_ARM].Joint[1].currPos;
		sourceTxData32[5] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Position;
		sourceTxData32[6] = *(U32*)&myRobotCommand[LEFT_ARM].JointTor[1].Tor;
		sourceTxData32[7] = *(U32*)&myRobotTrajectory[LEFT_ARM].Joint[2].currPos;
		sourceTxData32[8] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Position;
		sourceTxData32[9] = *(U32*)&myRobotCommand[LEFT_ARM].JointTor[2].Tor;
		sourceTxData32[10] = *(U32*)&myRobotTrajectory[RIGHT_ARM].Joint[0].currPos;
		sourceTxData32[11] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Position;
		sourceTxData32[12] = *(U32*)&myRobotCommand[RIGHT_ARM].JointTor[0].Tor;
		sourceTxData32[13] = *(U32*)&myRobotTrajectory[RIGHT_ARM].Joint[1].currPos;
		sourceTxData32[14] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Position;
		sourceTxData32[15] = *(U32*)&myRobotCommand[RIGHT_ARM].JointTor[1].Tor;
		sourceTxData32[16] = *(U32*)&myRobotTrajectory[RIGHT_ARM].Joint[2].currPos;
		sourceTxData32[17] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Position;
		sourceTxData32[18] = *(U32*)&myRobotCommand[RIGHT_ARM].JointTor[2].Tor;

		 // Calculate checksum (payload only: byte 4-39)
		for (int i = 4; i < MSG_GUI_DATA_2_DUAL_LENGTH; i++)
		{
			checksum ^= sourceTxData08[i];  // XOR checksum
		}
		sourceTxData08[3] = checksum;
		break;


	case UART_MSG_GUI_DATA_4_LEFT:
		sourceTxData08[0] = MSG_GUI_DATA_4_SING_BYTE_0;
		sourceTxData08[1] = MSG_GUI_DATA_4_SING_BYTE_1;
		sourceTxData08[2] = MSG_GUI_DATA_4_SING_LENGTH;
		sizeSend = MSG_GUI_DATA_4_SING_LENGTH;

		sourceTxData32[1]  = *(U32*)&myRobotTrajectory[LEFT_ARM].Joint[0].currPos; // sourceTxData08[4-7]
		sourceTxData32[2]  = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Position;  // sourceTxData08[8-11]
		sourceTxData32[3]  = *(U32*)&myRobotCommand[LEFT_ARM].JointTor[0].Tor;     // sourceTxData08[12-15]
		sourceTxData32[4]  = *(U32*)&S_single[LEFT_ARM][0];
		sourceTxData32[5]  = *(U32*)&myRobotTrajectory[LEFT_ARM].Joint[1].currPos; // sourceTxData08[20-23]
		sourceTxData32[6]  = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Position;  // sourceTxData08[24-27]
		sourceTxData32[7]  = *(U32*)&myRobotCommand[LEFT_ARM].JointTor[1].Tor;     // sourceTxData08[28-31]
		sourceTxData32[8]  = *(U32*)&S_single[LEFT_ARM][1];
		sourceTxData32[9]  = *(U32*)&myRobotTrajectory[LEFT_ARM].Joint[2].currPos; // sourceTxData08[36-39]
		sourceTxData32[10] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Position;  // sourceTxData08[40-43]
		sourceTxData32[11] = *(U32*)&myRobotCommand[LEFT_ARM].JointTor[2].Tor;     // sourceTxData08[44-47]
		sourceTxData32[12] = *(U32*)&S_single[LEFT_ARM][2];

		// Calculate checksum
		for (int i = 4; i < MSG_GUI_DATA_4_SING_LENGTH; i++)
		{
			checksum ^= sourceTxData08[i];  // XOR checksum
		}
		sourceTxData08[3] = checksum;
		break;

	case UART_MSG_GUI_DATA_4_RIGHT:
		sourceTxData08[0] = MSG_GUI_DATA_4_SING_BYTE_0;
		sourceTxData08[1] = MSG_GUI_DATA_4_SING_BYTE_1;
		sourceTxData08[2] = MSG_GUI_DATA_4_SING_LENGTH;
		sizeSend = MSG_GUI_DATA_4_SING_LENGTH;

		sourceTxData32[1]  = *(U32*)&myRobotTrajectory[RIGHT_ARM].Joint[0].currPos; // sourceTxData08[4-7]
		sourceTxData32[2]  = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Position;  // sourceTxData08[8-11]
		sourceTxData32[3]  = *(U32*)&myRobotCommand[RIGHT_ARM].JointTor[0].Tor;     // sourceTxData08[12-15]
		sourceTxData32[4]  = *(U32*)&S_single[RIGHT_ARM][0];
		sourceTxData32[5]  = *(U32*)&myRobotTrajectory[RIGHT_ARM].Joint[1].currPos; // sourceTxData08[20-23]
		sourceTxData32[6]  = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Position;  // sourceTxData08[24-27]
		sourceTxData32[7]  = *(U32*)&myRobotCommand[RIGHT_ARM].JointTor[1].Tor;     // sourceTxData08[28-31]
		sourceTxData32[8]  = *(U32*)&S_single[RIGHT_ARM][1];
		sourceTxData32[9]  = *(U32*)&myRobotTrajectory[RIGHT_ARM].Joint[2].currPos; // sourceTxData08[36-39]
		sourceTxData32[10] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Position;  // sourceTxData08[40-43]
		sourceTxData32[11] = *(U32*)&myRobotCommand[RIGHT_ARM].JointTor[2].Tor;     // sourceTxData08[44-47]
		sourceTxData32[12] = *(U32*)&S_single[RIGHT_ARM][2];

		// Calculate checksum
		for (int i = 4; i < MSG_GUI_DATA_4_SING_LENGTH; i++)
		{
			checksum ^= sourceTxData08[i];  // XOR checksum
		}
		sourceTxData08[3] = checksum;
		break;

	case UART_MSG_GUI_DATA_4_DUAL:
		sourceTxData08[0] = MSG_GUI_DATA_4_DUAL_BYTE_0;
		sourceTxData08[1] = MSG_GUI_DATA_4_DUAL_BYTE_1;
		sourceTxData08[2] = MSG_GUI_DATA_4_DUAL_LENGTH;
		sizeSend = MSG_GUI_DATA_4_DUAL_LENGTH;

		sourceTxData32[1]  = *(U32*)&myRobotTrajectory[LEFT_ARM].Joint[0].currPos; // sourceTxData08[4-7]
		sourceTxData32[2]  = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[0].Position;  // sourceTxData08[8-11]
		sourceTxData32[3]  = *(U32*)&myRobotCommand[LEFT_ARM].JointTor[0].Tor;     // sourceTxData08[12-15]
		sourceTxData32[4]  = *(U32*)&S_dual[0];
		sourceTxData32[5]  = *(U32*)&myRobotTrajectory[LEFT_ARM].Joint[1].currPos; // sourceTxData08[20-23]
		sourceTxData32[6]  = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[1].Position;  // sourceTxData08[24-27]
		sourceTxData32[7]  = *(U32*)&myRobotCommand[LEFT_ARM].JointTor[1].Tor;     // sourceTxData08[28-31]
		sourceTxData32[8]  = *(U32*)&S_dual[1];
		sourceTxData32[9]  = *(U32*)&myRobotTrajectory[LEFT_ARM].Joint[2].currPos; // sourceTxData08[36-39]
		sourceTxData32[10] = *(U32*)&myRobotFeedback[LEFT_ARM].Joint[2].Position;  // sourceTxData08[40-43]
		sourceTxData32[11] = *(U32*)&myRobotCommand[LEFT_ARM].JointTor[2].Tor;     // sourceTxData08[44-47]
		sourceTxData32[12] = *(U32*)&S_dual[2];
		sourceTxData32[13] = *(U32*)&myRobotTrajectory[RIGHT_ARM].Joint[0].currPos; // sourceTxData08[52-55]
		sourceTxData32[14] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[0].Position;  // sourceTxData08[56-59]
		sourceTxData32[15] = *(U32*)&myRobotCommand[RIGHT_ARM].JointTor[0].Tor;     // sourceTxData08[60-63]
		sourceTxData32[16] = *(U32*)&S_dual[3];
		sourceTxData32[17] = *(U32*)&myRobotTrajectory[RIGHT_ARM].Joint[1].currPos; // sourceTxData08[68-71]
		sourceTxData32[18] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[1].Position;  // sourceTxData08[72-75]
		sourceTxData32[19] = *(U32*)&myRobotCommand[RIGHT_ARM].JointTor[1].Tor;     // sourceTxData08[76-79]
		sourceTxData32[20] = *(U32*)&S_dual[4];
		sourceTxData32[21] = *(U32*)&myRobotTrajectory[RIGHT_ARM].Joint[2].currPos; // sourceTxData08[84-87]
		sourceTxData32[22] = *(U32*)&myRobotFeedback[RIGHT_ARM].Joint[2].Position;  // sourceTxData08[88-91]
		sourceTxData32[23] = *(U32*)&myRobotCommand[RIGHT_ARM].JointTor[2].Tor;     // sourceTxData08[92-95]
		sourceTxData32[24] = *(U32*)&S_dual[5];											// sourceTxData08[96-99]

		// Calculate checksum
		for (int i = 4; i < MSG_GUI_DATA_4_DUAL_LENGTH; i++)
		{
			checksum ^= sourceTxData08[i];  // XOR checksum
		}
		sourceTxData08[3] = checksum;
		break;

//	case UART_MSG_MOTOR_CONTROL_VEL:
	default:
		// Do nothing and return
		return;
	}



	// 5. Send the message
	if (HAL_OK != HAL_UART_Transmit_DMA(uartGoal, sourceTxData08, sizeSend))
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

GLOBAL BOOL AppCommUart_RecvSlaveMsg(enUartNode _slaveNode)
{
	// 1. Initialize variables
	if ((UART_NODE_SLAVE_1 != _slaveNode) && (UART_NODE_SLAVE_2 != _slaveNode))
	{
		return FALSE;
	}

	// 2. Initialize variables
	static U08* sourceRxData;
	U08 checksum = 0x00;
	U08 _arm = LEFT_ARM;
	BOOL result = FALSE;

	switch (_slaveNode)
	{
#if SLAVE_1_ENA
	case UART_NODE_SLAVE_1:
		_arm = LEFT_ARM;
		sourceRxData = RxDataSlaveLeft;
		break;
#endif
#if SLAVE_2_ENA
	case UART_NODE_SLAVE_2:
		_arm = RIGHT_ARM;
		sourceRxData = RxDataSlaveRight;
		break;
#endif
	case UART_NODE_GUI:
	default:
		// Do nothing and return
		return FALSE;
	}


	if (
	(MSG_DATA_RESPOND_BYTE_0 == sourceRxData[0]) && \
	(MSG_DATA_RESPOND_BYTE_1 == sourceRxData[1]) && \
	(MSG_DATA_RESPOND_LENGTH == sourceRxData[2])
	)
	{
		for (int i = 4; i < MSG_DATA_RESPOND_LENGTH; i++)
		{
			checksum ^= sourceRxData[i];  // XOR checksum
		}

		if (checksum != sourceRxData[3])
		{
			// Wrong checksum
			result = FALSE;
		}
		else
		{
			memcpy(&myRobotFeedback[_arm].Joint[0].Position, &sourceRxData[4],  sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[0].Speed,    &sourceRxData[8],  sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[0].Accel,    &sourceRxData[12], sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[1].Position, &sourceRxData[16], sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[1].Speed,    &sourceRxData[20], sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[1].Accel,    &sourceRxData[24], sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[2].Position, &sourceRxData[28], sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[2].Speed,    &sourceRxData[32], sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[2].Accel,    &sourceRxData[36], sizeof(float));
			result = TRUE;
		}
	}
	else if (
	(MSG_DATA_RESPOND_F_BYTE_0 == sourceRxData[0]) && \
	(MSG_DATA_RESPOND_F_BYTE_1 == sourceRxData[1]) && \
	(MSG_DATA_RESPOND_F_LENGTH == sourceRxData[2])
	)
	{
		for (int i = 4; i < MSG_DATA_RESPOND_F_LENGTH; i++)
		{
			checksum ^= sourceRxData[i];  // XOR checksum
		}

		if (checksum != sourceRxData[3])
		{
			// Wrong checksum
			result = FALSE;
		}
		else
		{
			memcpy(&myRobotFeedback[_arm].Joint[0].Position, &sourceRxData[4],  sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[0].Speed,    &sourceRxData[8],  sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[0].Accel,    &sourceRxData[12], sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[0].Speedf,   &sourceRxData[16], sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[0].Accelf,   &sourceRxData[20], sizeof(float));

			memcpy(&myRobotFeedback[_arm].Joint[1].Position, &sourceRxData[24], sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[1].Speed,    &sourceRxData[28], sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[1].Accel,    &sourceRxData[32], sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[1].Speedf,   &sourceRxData[36], sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[1].Accelf,   &sourceRxData[40], sizeof(float));

			memcpy(&myRobotFeedback[_arm].Joint[2].Position, &sourceRxData[44], sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[2].Speed,    &sourceRxData[48], sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[2].Accel,    &sourceRxData[52], sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[2].Speedf,   &sourceRxData[56], sizeof(float));
			memcpy(&myRobotFeedback[_arm].Joint[2].Accelf,   &sourceRxData[60], sizeof(float));
			result = TRUE;
		}
	}
	else
	{
		result = FALSE;
	}

	return result;
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
