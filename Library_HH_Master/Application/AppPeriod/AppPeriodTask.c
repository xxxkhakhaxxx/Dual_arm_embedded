/**
 ********************************************************************************
 ** @file    AppPeriodTask.c
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Nov 7, 2024 (created)
 ** @brief   
 ********************************************************************************
 **/

/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include "AppPeriodTask.h"

/********************************************************************************
 * EXTERN VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE MACROS AND DEFINES
 ********************************************************************************/
#define TASK_TIMER_CNT_ELAPSED_10S	(10000)
#define TASK_TIMER_CNT_ELAPSED_1S	(1000)


/********************************************************************************
 * PRIVATE TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef enum ENUM_TASK_LIST
{
	TASK_NONE = 0,
	TASK_10MS_GUI_COMM,
	TASK_10MS_ROBOT_IK,
	TASK_10MS_ROBOT_CONTROLLER,
	TASK_10MS_SLAVE_1_COMM,	// Send and return
	TASK_10MS_SLAVE_2_COMM,	// Send and return

} enTaskList;

/********************************************************************************
 * PRIVATE VARIABLES
 ********************************************************************************/
PRIVATE enTaskList enTaskId = TASK_NONE;

volatile static BOOL bNewSequenceFlag = FALSE;

/* Debug variables */
volatile static U32 u32TaskTimerCnt_1ms = 0;
volatile static U32 debug_cnt_task_duplicate = 0;
volatile static U32 debug_cnt_task_override = 0;

PRIVATE U08 u8GuiSendCnt = 0;

PRIVATE BOOL _masterInitFlag = FALSE;
PRIVATE BOOL _slave1InitFlag = FALSE;
PRIVATE BOOL _slave2InitFlag = FALSE;
PRIVATE BOOL _slave1HandleFlag = FALSE;
PRIVATE BOOL _slave2HandleFlag = FALSE;
PRIVATE U08 _btnSequence = 0;

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/

/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE BOOL _CheckAllSlaveInit(void);
PRIVATE BOOL _CheckAllSlaveFeedback(void);

/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
PRIVATE BOOL _CheckAllSlaveInit(void)
{
#if defined(SLAVE_1_ENA) && !defined(SLAVE_2_ENA)
	if (TRUE == _slave1InitFlag)
	{
		return TRUE;
	}
#elif !defined(SLAVE_1_ENA) && defined(SLAVE_2_ENA)
	if (TRUE == _slave2InitFlag)
	{
		return TRUE;
	}
#elif defined(SLAVE_1_ENA) && defined(SLAVE_2_ENA)
	if ((TRUE == _slave1InitFlag) && (TRUE == _slave2InitFlag))
	{
		return TRUE;
	}
#else
	#error [USER] Enable SLAVE_1_ENA or SLAVE_2_ENA or both
#endif
	return FALSE;
}

PRIVATE BOOL _CheckAllSlaveFeedback(void)
{
#if defined(SLAVE_1_ENA) && !defined(SLAVE_2_ENA)
	if (TRUE == _slave1HandleFlag)
	{
		return TRUE;
	}
#elif !defined(SLAVE_1_ENA) && defined(SLAVE_2_ENA)
	if (TRUE == _slave2HandleFlag)
	{
		return TRUE;
	}
#elif defined(SLAVE_1_ENA) && defined(SLAVE_2_ENA)
	if ((TRUE == _slave1HandleFlag) && (TRUE == _slave2HandleFlag))
	{
		return TRUE;
	}
#else
	#error [USER] Enable SLAVE_1_ENA or SLAVE_2_ENA or both
#endif
	return FALSE;
}

/********************************************************************************
 * GLOBAL FUNCTION IMPLEMENTATION
 ********************************************************************************/
GLOBAL void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		AppDataCheck_UserButtonState();	// Check user btn every 20ms

		if (FALSE == bNewSequenceFlag)	// Cycle 20ms
		{
			bNewSequenceFlag = TRUE;	// Start new sequence every 20ms
			AppDataSet_LedState(LED_3_ORANGE, FALSE);
		}
		else	// All tasks take more 20ms to finished
		{
			if (U16_MAX > debug_cnt_task_override)
			{
				debug_cnt_task_override++;
				AppDataSet_LedState(LED_3_ORANGE, TRUE);
			}
		}

	}

	return;
}

GLOBAL void AppPeriodTask_TaskCall(void)	/* Performing the corresponding task */
{
	switch(enTaskId)
	{
	case TASK_10MS_ROBOT_IK:
//		_RobotCalculateIK();
		break;
	case TASK_10MS_SLAVE_1_COMM:

		break;
	default:
		// None task
		break;
	}

	enTaskId = TASK_NONE;	// Clear task flag

	return;
}


GLOBAL void AppPeriodTask_StateMachineProcess(void)
{
//	static BOOL _slave2HandleFlag = FALSE;
//	static enRobotMode _robotMode = ROBOT_MODE_READ_DATA;		// The mode that you want to use
	// Full sequence: Init ➩ wait Slave init ➩ Send init to confirm ➩ ...
	//            ... ➩ Wait 20ms Flag ➩ Request data ➩ Handle data feedback ➩ Calculate control ➩ Send control
	//                          ↖	⇦   ⇦   ⇦   ⇦   ⇦   ⇦   ⇦   ⇦   ⇦   ⇦   ⇦   ⇦   ⇦   ⇦  GUI feedback ↩

	switch (AppDataGet_MasterState())
	{
	case MASTER_STATE_INIT:
		if (FALSE == _masterInitFlag)
		{
#ifdef SLAVE_1_ENA
			AppCommUart_RecvMsgStart(UART_NODE_SLAVE_1);
#endif
#ifdef SLAVE_2_ENA
			AppCommUart_RecvMsgStart(UART_NODE_SLAVE_2);
#endif
			_masterInitFlag = TRUE;
			AppDataSet_LedState(LED_5_RED, TRUE);	// Init LED
		}
		else
		{
#ifdef SLAVE_1_ENA
			if (FALSE == _slave1InitFlag)
			{
				if (TRUE == AppDataGet_UartRxNewFlag(UART_NODE_SLAVE_1))
				{
					if (
						(MSG_INIT_BYTE_0 == RxDataSlaveLeft[0]) && \
						(MSG_INIT_BYTE_1 == RxDataSlaveLeft[1]) && \
						(MSG_INIT_LENGTH == RxDataSlaveLeft[2])
					)
					{	// Correct init format
						AppCommUART_SendMsg(UART_NODE_SLAVE_1, UART_MSG_INIT);
						_slave1InitFlag = TRUE;
					}
					else
					{
						// Wait for another Rx
						AppDataSet_UartRxErrCnt(UART_NODE_SLAVE_1);
						AppCommUart_RecvMsgStart(UART_NODE_SLAVE_1);
					}

					AppDataSet_UartRxMsgCnt(UART_NODE_SLAVE_1);
					AppDataSet_UartRxNewFlag(UART_NODE_SLAVE_1, FALSE);
				}
				else
				{
					// Wait for Slave 1 Tx
				}
			}
#endif
#ifdef SLAVE_2_ENA
			if (FALSE == _slave2InitFlag)
			{
				if (TRUE == AppDataGet_UartRxNewFlag(UART_NODE_SLAVE_2))
				{
					if (
						(MSG_INIT_BYTE_0 == RxDataSlaveRight[0]) && \
						(MSG_INIT_BYTE_1 == RxDataSlaveRight[1]) && \
						(MSG_INIT_LENGTH == RxDataSlaveRight[2])
					)
					{	// Correct init format
						AppCommUART_SendMsg(UART_NODE_SLAVE_2, UART_MSG_INIT);
						_slave2InitFlag = TRUE;
					}
					else
					{
						// Wait for another Rx
						AppDataSet_UartRxErrCnt(UART_NODE_SLAVE_2);
						AppCommUart_RecvMsgStart(UART_NODE_SLAVE_2);
					}

					AppDataSet_UartRxMsgCnt(UART_NODE_SLAVE_2);
					AppDataSet_UartRxNewFlag(UART_NODE_SLAVE_2, FALSE);
				}
				else
				{
					// Wait for Slave 2 Tx
				}
			}
#endif

			// Exit INIT STATE
			if (TRUE == _CheckAllSlaveInit())
			{
				HAL_Delay(2);	// Wait for the Master init cmd send - only for DMA
				bNewSequenceFlag = FALSE;
				AppDataSet_MasterState(MASTER_STATE_WAIT_NEW_SEQUENCE);
			}
		}
		break;

	case MASTER_STATE_WAIT_NEW_SEQUENCE:
		if (TRUE == bNewSequenceFlag)	// Timer 2
		{
#if defined(SLAVE_1_ENA)
			AppCommUART_SendMsg(UART_NODE_SLAVE_1, UART_MSG_MOTOR_DATA);
			_slave1HandleFlag = FALSE;
#endif
#if defined(SLAVE_2_ENA)
			AppCommUART_SendMsg(UART_NODE_SLAVE_2, UART_MSG_MOTOR_DATA);
			_slave2HandleFlag = FALSE;
#endif
			AppDataSet_MasterState(MASTER_STATE_WAIT_SLAVE_FEEDBACK);
		}
		else
		{
			// Wait timer 2 trigger for new sequence

			// If a Rx msg received during this state -> Something's abnormal on the system sequence -> need check if happen
			if (TRUE == AppDataGet_UartRxNewFlag(UART_NODE_SLAVE_1))
			{
				AppDataSet_UartRxErrCnt(UART_NODE_SLAVE_1);		// Receive un-support message ID in this Master state
				AppDataSet_UartRxMsgCnt(UART_NODE_SLAVE_1);
				AppDataSet_UartRxNewFlag(UART_NODE_SLAVE_1, FALSE);
			}

			if (TRUE == AppDataGet_UartRxNewFlag(UART_NODE_SLAVE_2))
			{
				AppDataSet_UartRxErrCnt(UART_NODE_SLAVE_2);		// Receive un-support message ID in this Master state
				AppDataSet_UartRxMsgCnt(UART_NODE_SLAVE_2);
				AppDataSet_UartRxNewFlag(UART_NODE_SLAVE_2, FALSE);
			}
		}
		break;

	case MASTER_STATE_WAIT_SLAVE_FEEDBACK:
#ifdef SLAVE_1_ENA
		if ((TRUE == AppDataGet_UartRxNewFlag(UART_NODE_SLAVE_1)) && (FALSE == _slave1HandleFlag))
		{
			if (
			(MSG_DATA_RESPOND_BYTE_0 == RxDataSlaveLeft[0]) && \
			(MSG_DATA_RESPOND_BYTE_1 == RxDataSlaveLeft[1]) && \
			(MSG_DATA_RESPOND_LENGTH == RxDataSlaveLeft[2])
			)
			{
				memcpy(&myRobotFeedback[LEFT_ARM].Joint[0].Position, &RxDataSlaveLeft[3],  sizeof(float));
				memcpy(&myRobotFeedback[LEFT_ARM].Joint[0].Speed,    &RxDataSlaveLeft[7],  sizeof(float));
				memcpy(&myRobotFeedback[LEFT_ARM].Joint[0].Accel,    &RxDataSlaveLeft[11], sizeof(float));
				memcpy(&myRobotFeedback[LEFT_ARM].Joint[1].Position, &RxDataSlaveLeft[15], sizeof(float));
				memcpy(&myRobotFeedback[LEFT_ARM].Joint[1].Speed,    &RxDataSlaveLeft[19], sizeof(float));
				memcpy(&myRobotFeedback[LEFT_ARM].Joint[1].Accel,    &RxDataSlaveLeft[23], sizeof(float));
				memcpy(&myRobotFeedback[LEFT_ARM].Joint[2].Position, &RxDataSlaveLeft[27], sizeof(float));
				memcpy(&myRobotFeedback[LEFT_ARM].Joint[2].Speed,    &RxDataSlaveLeft[31], sizeof(float));
				memcpy(&myRobotFeedback[LEFT_ARM].Joint[2].Accel,    &RxDataSlaveLeft[35], sizeof(float));
				_slave1HandleFlag = TRUE;
			}
			else
			{
				AppDataSet_UartRxErrCnt(UART_NODE_SLAVE_1);		// Receive un-support message ID in this Master state
				AppCommUart_RecvMsgStart(UART_NODE_SLAVE_1);	// Wait for another Master message
			}

			AppDataSet_UartRxMsgCnt(UART_NODE_SLAVE_1);
			AppDataSet_UartRxNewFlag(UART_NODE_SLAVE_1, FALSE);
		}
#endif
#ifdef SLAVE_2_ENA
		if ((TRUE == AppDataGet_UartRxNewFlag(UART_NODE_SLAVE_2)) && (FALSE == _slave2HandleFlag))
		{
			if (
			(MSG_DATA_RESPOND_BYTE_0 == RxDataSlaveRight[0]) && \
			(MSG_DATA_RESPOND_BYTE_1 == RxDataSlaveRight[1]) && \
			(MSG_DATA_RESPOND_LENGTH == RxDataSlaveRight[2])
			)
			{
				memcpy(&myRobotFeedback[RIGHT_ARM].Joint[0].Position, &RxDataSlaveRight[3],  sizeof(float));
				memcpy(&myRobotFeedback[RIGHT_ARM].Joint[0].Speed,    &RxDataSlaveRight[7],  sizeof(float));
				memcpy(&myRobotFeedback[RIGHT_ARM].Joint[0].Accel,    &RxDataSlaveRight[11], sizeof(float));
				memcpy(&myRobotFeedback[RIGHT_ARM].Joint[1].Position, &RxDataSlaveRight[15], sizeof(float));
				memcpy(&myRobotFeedback[RIGHT_ARM].Joint[1].Speed,    &RxDataSlaveRight[19], sizeof(float));
				memcpy(&myRobotFeedback[RIGHT_ARM].Joint[1].Accel,    &RxDataSlaveRight[23], sizeof(float));
				memcpy(&myRobotFeedback[RIGHT_ARM].Joint[2].Position, &RxDataSlaveRight[27], sizeof(float));
				memcpy(&myRobotFeedback[RIGHT_ARM].Joint[2].Speed,    &RxDataSlaveRight[31], sizeof(float));
				memcpy(&myRobotFeedback[RIGHT_ARM].Joint[2].Accel,    &RxDataSlaveRight[35], sizeof(float));
				_slave2HandleFlag = TRUE;
			}
			else
			{
				AppDataSet_UartRxErrCnt(UART_NODE_SLAVE_2);		// Receive un-support message ID in this Master state
				AppCommUart_RecvMsgStart(UART_NODE_SLAVE_2);	// Wait for another Master message
			}

			AppDataSet_UartRxMsgCnt(UART_NODE_SLAVE_2);
			AppDataSet_UartRxNewFlag(UART_NODE_SLAVE_2, FALSE);
		}
#endif

		// Exit WAIT SLAVE state
		if (TRUE == _CheckAllSlaveFeedback())
		{
			AppDataSet_MasterState(MASTER_STATE_CAL_CONTROL);
		}
		break;

	case MASTER_STATE_CAL_CONTROL:
#if defined (MASTER_NO_CONTROL)
		AppDataSet_MasterState(MASTER_STATE_SEND_GUI);
#else
	#if defined (MASTER_CONTROL_POS)
		if (TRUE == AppDataGet_UserButtonEvent())	// 1 time every btn press
		{
			/*AppControl_Pos_TestSquence(RIGHT_ARM, 10);	// Calculate position value to be sent
			 * AppCommUART_SendMsg(UART_NODE_SLAVE_1, UART_MSG_MOTOR_CONTROL_POS);
			AppCommUART_SendMsg(UART_NODE_SLAVE_2, UART_MSG_MOTOR_CONTROL_POS);	// Package and Send*/

			if (0 == _btnSequence)
			{
		#ifdef SLAVE_1_ENA
				AppControl_Pos_BackToHome(LEFT_ARM, HOME_SPEED);
				AppCommUART_SendMsg(UART_NODE_SLAVE_1, UART_MSG_MOTOR_CONTROL_POS);	// Package and Send
		#endif
		#ifdef SLAVE_2_ENA
				AppControl_Pos_BackToHome(RIGHT_ARM, HOME_SPEED);
				AppCommUART_SendMsg(UART_NODE_SLAVE_2, UART_MSG_MOTOR_CONTROL_POS);	// Package and Send
		#endif
				_btnSequence = 1;
			}
			else if (1 == _btnSequence)
			{
				_btnSequence = 2;
			}

			AppDataSet_MasterState(MASTER_STATE_SEND_GUI);
		}
		else if (2 == _btnSequence)
		{
		#ifdef SLAVE_1_ENA
			AppControl_TP_SineWave(LEFT_ARM, 0.02f);
			AppCommUART_SendMsg(UART_NODE_SLAVE_1, UART_MSG_MOTOR_CONTROL_POS);	// Package and Send
		#endif
		#ifdef SLAVE_2_ENA
			AppControl_TP_SineWave(RIGHT_ARM, 0.02f);
			AppCommUART_SendMsg(UART_NODE_SLAVE_2, UART_MSG_MOTOR_CONTROL_POS);	// Package and Send
		#endif
			AppDataSet_MasterState(MASTER_STATE_SEND_GUI);
		}
		else
		{
			AppDataSet_MasterState(MASTER_STATE_SEND_GUI);
		}

	#elif defined (MASTER_CONTROL_VEL)
		// TODO
	#elif defined (MASTER_CONTROL_TOR)
		// TODO
	#else	// No control
		AppDataSet_MasterState(MASTER_STATE_SEND_GUI);
	#endif
#endif
		break;

	case MASTER_STATE_SEND_GUI:
#if defined (MASTER_NO_GUI)
		// Do nothing
#else
	#if defined (MASTER_NO_CONTROL) || defined (MASTER_CONTROL_POS)
		if (u8GuiSendCnt < GUI_SEND_CNT_MAX)// 100ms
		{
			u8GuiSendCnt++;
		}
		else	// If PERIOD_CONTROL == PERIOD_GUI_SEND -> Send every PERIOD_CONTROL
		{
			u8GuiSendCnt = 0;
		#if defined(SLAVE_1_ENA) && defined(SLAVE_2_ENA)
			AppCommUART_SendMsg(UART_NODE_GUI, UART_MSG_GUI_DATA_1_DUAL);
		#elif defined(SLAVE_1_ENA)
			AppCommUART_SendMsg(UART_NODE_GUI, UART_MSG_GUI_DATA_1_LEFT);
		#elif defined(SLAVE_2_ENA)
			AppCommUART_SendMsg(UART_NODE_GUI, UART_MSG_GUI_DATA_1_RIGHT);
		#endif
		}
	#else
		#if defined(SLAVE_1_ENA) && defined(SLAVE_2_ENA)
			AppCommUART_SendMsg(UART_NODE_GUI, UART_MSG_GUI_DATA_2_DUAL);
		#elif defined(SLAVE_1_ENA)
			AppCommUART_SendMsg(UART_NODE_GUI, UART_MSG_GUI_DATA_2_LEFT);
		#elif defined(SLAVE_2_ENA)
			AppCommUART_SendMsg(UART_NODE_GUI, UART_MSG_GUI_DATA_2_RIGHT);
#endif
	#endif
#endif

		bNewSequenceFlag = FALSE;	// Sequence end
		AppDataSet_MasterState(MASTER_STATE_WAIT_NEW_SEQUENCE);
		break;

	default:
		// if any abnormal, reset variables and back to init state
		_masterInitFlag = FALSE;
		_slave1InitFlag = FALSE;
		_slave1HandleFlag = FALSE;
		_slave2InitFlag = FALSE;
		_slave2HandleFlag = FALSE;
		AppDataSet_MasterState(MASTER_STATE_INIT);
		break;
	}
	return;
}
