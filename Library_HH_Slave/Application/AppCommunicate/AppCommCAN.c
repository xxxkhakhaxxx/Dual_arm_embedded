/**
 ********************************************************************************
 ** @file    AppCommCAN.c
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Oct 30, 2024 (created)
 ** @brief   
 ********************************************************************************
 **/

/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include "AppCommCAN.h"

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
PRIVATE CAN_HandleTypeDef* strCanUse;

PRIVATE CAN_TxHeaderTypeDef strCanTxMsgId;		// Tx message ID - header
PRIVATE CAN_RxHeaderTypeDef strCanRxMsgId;		// Rx message ID - header

PRIVATE U08 arrCanTxMsgData[MOTOR_PROTOCOL_DATA_FRAME_LENGTH] = {0, };	// Tx data
PRIVATE U08 arrCanRxMsgData[MOTOR_PROTOCOL_DATA_FRAME_LENGTH] = {0, };	// Rx data

PRIVATE U32 u32CanTxMsgMailBox = 0;				// Tx header and data


/* Debug view variable */
volatile static U32 debug_cnt_Rx_Ok = 0;
volatile static U32 debug_cnt_Rx_Error = 0;
volatile static U32 debug_cnt_Tx_Ok = 0;
volatile static U32 debug_cnt_Tx_Error = 0;
volatile static U32 debug_cnt_Tx_ErrorParam = 0;
volatile static U32 debug_cnt_Tx_ErrorInit = 0;
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
GLOBAL strMotorData myMotorToMaster[MOTOR_NUMBER] = {0, };	// Motor's data using for communication with Master
GLOBAL strMotorCmd  myMasterToMotor[MOTOR_NUMBER] = {0, };	// Master's data using for controlling Motor

/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE void AppCommCAN_SetupFilter(void);
PRIVATE void AppCommCAN_SetupRxInterrupt(void);
PRIVATE void AppCommCAN_SetupTxFrame(void);
PRIVATE void AppCommCAN_ClearTxMailBox(void);

PRIVATE void AppCommCAN_UpdateMotorRecvData(U08 _motorId, U16 _newEncoderValue);
/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
PRIVATE void AppCommCAN_SetupFilter(void)
{
    CAN_FilterTypeDef canFilterIdConfig;					// User CAN filter ID - header filter

    // First filter for 0x140 to 0x160 (Single motor ID 1~32, 0 is for user)
    canFilterIdConfig.FilterBank = 0;
    canFilterIdConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterIdConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterIdConfig.FilterIdHigh = 0x140 << 5;			// 11-bit ID shifted to the correct position
    canFilterIdConfig.FilterIdLow = 0x0000;
    canFilterIdConfig.FilterMaskIdHigh = 0x7E0 << 5;		// Mask to match IDs from 0x140 to 0x160
    canFilterIdConfig.FilterMaskIdLow = 0x0000;
    canFilterIdConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canFilterIdConfig.FilterActivation = CAN_FILTER_ENABLE;
    canFilterIdConfig.SlaveStartFilterBank = 1;

    if (HAL_CAN_ConfigFilter(strCanUse, &canFilterIdConfig) != HAL_OK)
    {
        // Error_Handler();
    }

    // Second filter for 0x280 (Multi-motor)
    canFilterIdConfig.FilterBank = 1;
    canFilterIdConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterIdConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterIdConfig.FilterIdHigh = 0x280 << 5; // Exact ID
    canFilterIdConfig.FilterIdLow = 0x0000;
    canFilterIdConfig.FilterMaskIdHigh = 0x7FF << 5; // Full mask for exact match
    canFilterIdConfig.FilterMaskIdLow = 0x0000;
    canFilterIdConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canFilterIdConfig.FilterActivation = CAN_FILTER_ENABLE;
    canFilterIdConfig.SlaveStartFilterBank = 1;

    if (HAL_CAN_ConfigFilter(strCanUse, &canFilterIdConfig) != HAL_OK)
    {
        // Error_Handler();
    }
}


PRIVATE void AppCommCAN_SetupRxInterrupt(void)
{
	if (HAL_OK != HAL_CAN_ActivateNotification(strCanUse, CAN_IT_RX_FIFO0_MSG_PENDING))	// Active Rx for FIFO0
	{
//		Error_Handler();
	}
}

PRIVATE void AppCommCAN_SetupTxFrame(void)
{
	/* Not change value */
	strCanTxMsgId.IDE = CAN_ID_STD;		// Use CAN A - ID frame 11 bits
	strCanTxMsgId.RTR = CAN_RTR_DATA;	// Use Data frame
	strCanTxMsgId.DLC = 0x08;			// Data is 8 bytes

	/* Change value */
//	strCanTxMsgId.StdId = MOTOR_1_ID;	// Not here, set in send function
}

PRIVATE void AppCommCAN_ClearTxMailBox(void)
{
	uint32_t freeLevel = HAL_CAN_GetTxMailboxesFreeLevel(strCanUse);

	// Check if all mailboxes are full
	if (freeLevel == 0)
	{
		// Abort all pending transmissions
		HAL_CAN_AbortTxRequest(strCanUse, CAN_TX_MAILBOX0);
		HAL_CAN_AbortTxRequest(strCanUse, CAN_TX_MAILBOX1);
		HAL_CAN_AbortTxRequest(strCanUse, CAN_TX_MAILBOX2);

		// Clear data registers
		strCanUse->Instance->sTxMailBox[CAN_TX_MAILBOX0].TIR = 0;
		strCanUse->Instance->sTxMailBox[CAN_TX_MAILBOX0].TDTR = 0;
		strCanUse->Instance->sTxMailBox[CAN_TX_MAILBOX0].TDLR = 0;
		strCanUse->Instance->sTxMailBox[CAN_TX_MAILBOX0].TDHR = 0;
		strCanUse->Instance->sTxMailBox[CAN_TX_MAILBOX1].TIR = 0;
		strCanUse->Instance->sTxMailBox[CAN_TX_MAILBOX1].TDTR = 0;
		strCanUse->Instance->sTxMailBox[CAN_TX_MAILBOX1].TDLR = 0;
		strCanUse->Instance->sTxMailBox[CAN_TX_MAILBOX1].TDHR = 0;
		strCanUse->Instance->sTxMailBox[CAN_TX_MAILBOX2].TIR = 0;
		strCanUse->Instance->sTxMailBox[CAN_TX_MAILBOX2].TDTR = 0;
		strCanUse->Instance->sTxMailBox[CAN_TX_MAILBOX2].TDLR = 0;
		strCanUse->Instance->sTxMailBox[CAN_TX_MAILBOX2].TDHR = 0;

	}
}

PRIVATE void AppCommCAN_UpdateMotorRecvData(U08 _motorId, U16 _newEncoderValue)
{
	// 1. Convert encoder value (0-65535) to degrees (0-360)
	float newPosition = (float)_newEncoderValue * (360.0f / 65535.0f);

	// 2. Handle encoder overflow (wrapping from 65535→0)
	float positionDiff = newPosition - myMotorToMaster[_motorId].currPosition;
	if (positionDiff > 180.0f)
	{	// Check if position changed by more than 180° (adjust threshold if needed)
		newPosition -= 360.0f; // Negative wrap (e.g., 350° → 10° becomes 350° → -350°)
	}
	else if (positionDiff < -180.0f)
	{
		newPosition += 360.0f; // Positive wrap (e.g., 10° → 350° becomes 370° → 350°)
	}
	else
	{
		// Do nothing
	}

	// 3. Get current time and calculate delta
	U32 currentTime = HAL_GetTick();	// Unit: ms - timeout after ~49.7 days
	float diffTime;
	diffTime = (currentTime - myMotorToMaster[_motorId].currTime) / 1000.0f;	// Unit: seconds

	// 4. Save new previous values
	myMotorToMaster[_motorId].prevTime     = myMotorToMaster[_motorId].currTime;
	myMotorToMaster[_motorId].prevPosition = myMotorToMaster[_motorId].currPosition;
	myMotorToMaster[_motorId].prevSpeed    = myMotorToMaster[_motorId].currSpeed;

	// 5 Calculate current values
	myMotorToMaster[_motorId].currTime     = currentTime;
	myMotorToMaster[_motorId].currPosition = newPosition;
	if (diffTime > 0.0001f)
	{	// Protect against divide-by-zero on first run or rapid updates
		myMotorToMaster[_motorId].currSpeed    = (newPosition - myMotorToMaster[_motorId].prevPosition) / diffTime;
		myMotorToMaster[_motorId].currAccel    = (myMotorToMaster[_motorId].currSpeed - myMotorToMaster[_motorId].prevSpeed) / diffTime;
	}
	else
	{
		// If time difference is too small, keep previous values
		myMotorToMaster[_motorId].currSpeed = myMotorToMaster[_motorId].prevSpeed;
		myMotorToMaster[_motorId].currAccel = 0.0f;
	}

	return;
}



/********************************************************************************
 * GLOBAL FUNCTION IMPLEMENTATION
 ********************************************************************************/
/**
  * @brief  Cấu hình những thứ cần thiết để chạy CAN
  * @author Hung Hoang (hunghoang.1806@gmail.com)
  * @param  hcan: con trỏ tới biến CAN mà do CubeIDE tạo ra,
  *               nếu là STM32F1xx thì là "hcan" (do chỉ có 1 kênh CAN),
  *               nếu là STM32F4xx thì là "hcan1" hoặc "hcan2" (có 2 kênh CAN)
  * @proces 1/ Thiết lập CAN Filter
  *         2/ Thiết lập dùng ngắt
  *         3/ Thiết lập Tx frame
  *         4/ Sau khi chạy hàm này, user có thể start CAN ở hàm main bằng hàm "HAL_CAN_Start(&hcan?)"
  * @retval None
  */
GLOBAL void AppCommCAN_UserSetup(CAN_HandleTypeDef *hcan)
{
	strCanUse = hcan;				// Save this for private function can be use
	AppCommCAN_SetupFilter();
	AppCommCAN_SetupRxInterrupt();	// Enable interrupt
	AppCommCAN_SetupTxFrame();		// Setting for Tx message ID
}

GLOBAL void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)	// When Rx interrupt occur
{
	if (HAL_OK != HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &strCanRxMsgId, arrCanRxMsgData))	// Get MsgId and Data
	{
//		Error_Handler();
		debug_cnt_Rx_Error++;
	}
	else	// No error
	{
		AppDataSet_CanRxNewFlag(TRUE);
		debug_cnt_Rx_Ok++;
	}
}

GLOBAL void AppCommCAN_SendMotorMsg(U08 _u8MotorMsgId, U08 _u8MsgDataCmd)
{
	// Set Tx identifier
	strCanTxMsgId.StdId = (U32)(MOTOR_PROTOCOL_GET_HEADER(_u8MotorMsgId));

	// Set Tx data to buffer
	ApiProtocolMotorMG_TxHandler(_u8MotorMsgId, _u8MsgDataCmd, arrCanTxMsgData);

	// Send Tx data
	if (HAL_OK == HAL_CAN_AddTxMessage(strCanUse, &strCanTxMsgId, arrCanTxMsgData, &u32CanTxMsgMailBox))
	{
		debug_cnt_Tx_Ok++;
	}
	else	// HAL_ERROR
	{
		debug_cnt_Tx_Error++;
		if (HAL_CAN_ERROR_PARAM == (strCanUse->ErrorCode & HAL_CAN_ERROR_PARAM))
		{	// Tx Mailbox full
			debug_cnt_Tx_ErrorParam++;
			AppCommCAN_ClearTxMailBox();
		}
		else	// HAL_CAN_ERROR_NOT_INITIALIZED
		{
			debug_cnt_Tx_ErrorInit++;
		}
	}
}

GLOBAL void AppCommCAN_RecvMotorMsg(void)	// Process data in "strCanRxMsgId" and "arrCanRxMsgData"
{
	if ((MOTOR_HEADER_INIT >=strCanRxMsgId.StdId) \
		|| ((MOTOR_HEADER_MAX <= strCanRxMsgId.StdId) && (MOTOR_HEADER_MULTI != strCanRxMsgId.StdId)))
	{
		// Invalid, stop process
		return;
	}

	U08 _motorId = (U08)MOTOR_PROTOCOL_GET_ID(strCanRxMsgId.StdId);

	ApiProtocolMotorMG_RxHandler(_motorId, arrCanRxMsgData);	// Process CAN RX data

	if (MOTOR_CMD_READ_MECHANICAL_STATE == arrCanRxMsgData[0])	// Update only when it's respond of 0x9C command
	{	// Calculate pos-vel-accel
		AppCommCAN_UpdateMotorRecvData(_motorId, strRobotArmMotorRx[_motorId].Data.u16Encoder14Bit);
	}

	return;
}
