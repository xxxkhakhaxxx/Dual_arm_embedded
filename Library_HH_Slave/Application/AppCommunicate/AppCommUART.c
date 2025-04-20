/**
 ********************************************************************************
 ** @file    AppCommUART.c
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Mar 16, 2025 (created)
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
PRIVATE UART_HandleTypeDef* strUartMaster;

/* Rx buffer - Just for receive */
PRIVATE U08 arrUartMasterRxBuffer[UART_BUFFER_SIZE] = {0, };

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
/* Saved Rx buffer - Use in app */
GLOBAL U08 RxDataMaster[UART_BUFFER_SIZE] = {0, };

GLOBAL U08 TxDataMaster[UART_BUFFER_SIZE] = {0, };
/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE U32 _PosControl_ConvertKineUnit2MotorUnit(float _angleRxFromMaster, I32 _offset, I08 _dir);


/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
PRIVATE U32 _PosControl_ConvertKineUnit2MotorUnit(float _angleRxFromMaster, I32 _offset, I08 _dir)
{	/************************ CALCULATE OFFSET ************************
	 *   Joint  |  kine_value  |  motor_value   |   offset_value
	 * -----------------------------------------------------------
	 *    q11   |    252.73    |   36000 & 0   ->      +10727
	 *    q11   |      0       |       10727   ->      +10727
	 *    q11   |     10       |       11727   ->      +10727
	 * -----------------------------------------------------------
	 *    q21   |      0       |       30217   ->      +30217
	 *    q21   |     10       |       29217   ->      +30217
	 *    q31   |     57.83    |   36000 & 0   ->      +30217
	 ******************************************************************/

	// Scale, offset, and handle wrapping
	I32 _angleTxToMotor = (I32)roundf(_angleRxFromMaster * 100.0f * _dir) + _offset;

	// Wrap negative values and clamp to [0, 36000)
	_angleTxToMotor = (_angleTxToMotor % 36000 + 36000) % 36000;
	return (U32)_angleTxToMotor;
}


/********************************************************************************
 * GLOBAL FUNCTION IMPLEMENTATION
 ********************************************************************************/
GLOBAL void AppCommUART_UserSetup(UART_HandleTypeDef* huart, enUartNode _node)
{
	switch (_node)
	{
	case UART_NODE_MASTER:
		strUartMaster = huart;
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
		AppDataSet_UartTxWaitFlag(UART_NODE_MASTER, FALSE);	// Msg is send
		AppDataSet_UartTxMsgCnt(UART_NODE_MASTER);			// Increase counter
		AppCommUart_RecvMsgStart(UART_NODE_MASTER);		// Looking for Rx msg
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
GLOBAL void AppCommUART_SendMsg(enUartNode _node, enUartMsg _txMsgId)	// TODO: return send result
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
	float j_kinematics;	// Calculate J_kinematics for each motor and copy to UART buffer


	// 3. Get UART target and data
	switch (_node)
	{
	case UART_NODE_MASTER:
		uartGoal = strUartMaster;
		sourceTxData = TxDataMaster;
		break;
	default:
		// Do nothing and return
		return;
	}


	// 4. Prepare data
	switch (_txMsgId)
	{
	case UART_MSG_INIT:
		sourceTxData[0] = MSG_INIT_BYTE_0;
		sourceTxData[1] = MSG_INIT_BYTE_1;
		sourceTxData[2] = MSG_INIT_LENGTH;
		sizeSend = MSG_INIT_LENGTH;
		break;

	case UART_MSG_MOTOR_DATA:
		sourceTxData[0] = MSG_DATA_RESPOND_BYTE_0;
		sourceTxData[1] = MSG_DATA_RESPOND_BYTE_1;
		sourceTxData[2] = MSG_DATA_RESPOND_LENGTH;

		// Equation: J_real = J_kine*J_dir + J_offset
		// Equation: J_kine = (J_real - J_offset)*J_dir
		j_kinematics = (myMotorToMaster[0].currPosition + J1_OFFSET_REAL2KINE) * J1_DIR_REAL2KINE;
		memcpy(&sourceTxData[3] ,  &j_kinematics,                   sizeof(float));
		memcpy(&sourceTxData[7] ,  &myMotorToMaster[0].currSpeed,   sizeof(float));
		memcpy(&sourceTxData[11],  &myMotorToMaster[0].currAccel,   sizeof(float));

		j_kinematics = (myMotorToMaster[1].currPosition + J2_OFFSET_REAL2KINE) * J2_DIR_REAL2KINE;
		memcpy(&sourceTxData[15], &j_kinematics,                    sizeof(float));
		memcpy(&sourceTxData[19], &myMotorToMaster[1].currSpeed,    sizeof(float));
		memcpy(&sourceTxData[23], &myMotorToMaster[1].currAccel,    sizeof(float));

		j_kinematics = (myMotorToMaster[2].currPosition + J3_OFFSET_REAL2KINE) * J3_DIR_REAL2KINE;
		memcpy(&sourceTxData[27], &j_kinematics,                    sizeof(float));
		memcpy(&sourceTxData[31], &myMotorToMaster[2].currSpeed,    sizeof(float));
		memcpy(&sourceTxData[35], &myMotorToMaster[2].currAccel,    sizeof(float));
		sizeSend = MSG_DATA_RESPOND_LENGTH;
		break;

	case UART_MSG_MOTOR_CONTROL_POS:
	case UART_MSG_MOTOR_CONTROL_TOR:
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
		memcpy(RxDataMaster, arrUartMasterRxBuffer, Size);
		AppDataSet_UartRxMsgCnt(UART_NODE_MASTER);
		AppDataSet_UartRxWaitFlag(UART_NODE_MASTER, FALSE);
		AppDataSet_UartRxNewFlag(UART_NODE_MASTER, TRUE);
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
	case UART_NODE_MASTER:
		HAL_UARTEx_ReceiveToIdle_DMA(strUartMaster, arrUartMasterRxBuffer, UART_BUFFER_SIZE);
		break;
	default:
		break;
	}

// 3. Set Rx-wait flag
	AppDataSet_UartRxWaitFlag(_node, TRUE);		// After start Rx DMA
	return;
}

GLOBAL void AppCommUart_RecvMasterMsg(enUartMsg _rxMsgId)	// TODO: Return recv result
{
	// 1. Initialize variables
	float _angleRx;
	U32 _angleTx;
	U16 _speed;
	U08 _direction;	// 0 = CW, 1 = CCW
	U08 checksum = 0x00;

	// 2. Check checksum and save data
	switch (_rxMsgId)
	{
	case UART_MSG_MOTOR_CONTROL_POS:
		 // Check checksum correct or not(payload only: byte 4-24)
		for (int i = 4; i < MSG_CONTROL_POS_LENGTH; i++)
		{
			checksum ^= RxDataMaster[i];  // XOR checksum
		}
		if (checksum != RxDataMaster[3])
		{
			// Wrong checksum
			return;
		}

		// Payloads: q1-dq1-dir1 - q2-dq2-dir2 - q3-dq3-dir3
		memcpy(&_angleRx,   &RxDataMaster[4],  sizeof(float));
		memcpy(&_speed,     &RxDataMaster[8],  sizeof(U16));
		memcpy(&_direction, &RxDataMaster[10], sizeof(U08));
		_angleTx = _PosControl_ConvertKineUnit2MotorUnit(_angleRx, J1_OFFSET_KINE2REAL, J1_DIR_KINE2REAL);
		_direction = CONVERT_DIR_KINE2REAL(_direction, J1_DIR_KINE2REAL);
		ApiProtocolMotorMG_SetAngleSingle(MOTOR_1_ID, _angleTx, _speed, _direction);

		memcpy(&_angleRx,   &RxDataMaster[11], sizeof(float));
		memcpy(&_speed,     &RxDataMaster[15], sizeof(U16));
		memcpy(&_direction, &RxDataMaster[17], sizeof(U08));
		_angleTx = _PosControl_ConvertKineUnit2MotorUnit(_angleRx, J2_OFFSET_KINE2REAL, J2_DIR_KINE2REAL);
		_direction = CONVERT_DIR_KINE2REAL(_direction, J2_DIR_KINE2REAL);
		ApiProtocolMotorMG_SetAngleSingle(MOTOR_2_ID, _angleTx, _speed, _direction);

		memcpy(&_angleRx,   &RxDataMaster[18], sizeof(float));
		memcpy(&_speed,     &RxDataMaster[22], sizeof(U16));
		memcpy(&_direction, &RxDataMaster[24], sizeof(U08));
		_angleTx = _PosControl_ConvertKineUnit2MotorUnit(_angleRx, J3_OFFSET_KINE2REAL, J3_DIR_KINE2REAL);
		_direction = CONVERT_DIR_KINE2REAL(_direction, J3_DIR_KINE2REAL);
		ApiProtocolMotorMG_SetAngleSingle(MOTOR_3_ID, _angleTx, _speed, _direction);
		break;

	case UART_MSG_MOTOR_CONTROL_TOR:
		// TODO
		break;
	case UART_MSG_INIT:
	case UART_MSG_MOTOR_DATA:
	default:
		// No payload data to handle
		break;
	}


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
		if (huart->Instance == USART2)		AppDataSet_UartRxErrCnt(UART_NODE_MASTER);
		else
		{
			// Do nothing
		}
		break;
	case HAL_UART_ERROR_DMA:  /*!< DMA transfer error  */
		// Could be DMA Tx or DMA Rx
		// TODO: Need to implement more
		break;
	case HAL_UART_ERROR_NONE: /*!< No error            */
	default:
		// Do nothing
		break;
	}


	return;
}

