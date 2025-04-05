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

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
GLOBAL strRobotDataCommand  myRobotCommand[DUAL_ARM]  = {0, };
GLOBAL strRobotDataFeedback myRobotFeedback[DUAL_ARM] = {0, };

/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE void AppPeriodTask_SetTaskFlag(enTaskList _TaskName);
PRIVATE void AppPeriodTask_Scheduler(void);

/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
PRIVATE void AppPeriodTask_SetTaskFlag(enTaskList _TaskName)
{
	if (_TaskName != enTaskId)
	{
		if (TASK_NONE != enTaskId)
		{
			debug_cnt_task_override++;
		}
		enTaskId = _TaskName;
	}
	else
	{
		debug_cnt_task_duplicate++;
	}

	return;
}

PRIVATE void AppPeriodTask_Scheduler(void)
{
	/* At a time, only 1 task is set to do */


	/* 10ms Task */
	switch ((u32TaskTimerCnt_1ms)%10)
	{
//	case 1:
//		AppPeriodTask_SetTaskFlag(TASK_10MS_ROBOT_IK);
//		break;
	case 5:
//		AppPeriodTask_SetTaskFlag(TASK_10MS_SLAVE_1_COMM);
		break;
	default:
		break;
	}
}

/********************************************************************************
 * GLOBAL FUNCTION IMPLEMENTATION
 ********************************************************************************/
GLOBAL void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		/*
		u32TaskTimerCnt_1ms++;

		AppPeriodTask_Scheduler();		// Check time for which task to perform

		if (TASK_TIMER_CNT_ELAPSED_1S <= u32TaskTimerCnt_1ms)	// Cycle 1s
		{
			u32TaskTimerCnt_1ms = 0;
		}*/

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

static BOOL _masterInitFlag = FALSE;
static BOOL _slave1InitFlag = FALSE;
//	static BOOL _slave2InitFlag = FALSE;
static BOOL _slave1HandleFlag = FALSE;

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
			AppCommUart_RecvMsgStart(UART_NODE_SLAVE_1);
//			AppCommUart_RecvMsgStart(UART_NODE_SLAVE_2);
			_masterInitFlag = TRUE;
			AppDataSet_LedState(LED_5_RED, TRUE);	// Init LED
		}
		else
		{
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
					// Wait for Slave Tx
				}
			}

			if (TRUE == _slave1InitFlag) //&& (TRUE == _slave2InitFlag)	// SLAVEs are ready
			{	// Exit INIT STATE
				HAL_Delay(2);	// Wait for the Master init cmd send - only for DMA
				bNewSequenceFlag = FALSE;
				AppDataSet_MasterState(MASTER_STATE_WAIT_NEW_SEQUENCE);
			}
		}
		break;

	case MASTER_STATE_WAIT_NEW_SEQUENCE:
		if (TRUE == bNewSequenceFlag)	// Timer 2
		{
			AppCommUART_SendMsg(UART_NODE_SLAVE_1, UART_MSG_MOTOR_DATA);
//			AppCommUART_SendMsg(UART_NODE_SLAVE_2, UART_MSG_MOTOR_DATA);
			_slave1HandleFlag = FALSE;
//			_slave2HandleFlag = FALSE;
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
		}
		break;

	case MASTER_STATE_WAIT_SLAVE_FEEDBACK:
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

		if (TRUE == _slave1HandleFlag)
		{	// Exit WAIT SLAVE state
			AppDataSet_MasterState(MASTER_STATE_CAL_CONTROL);
		}
		break;

	case MASTER_STATE_CAL_CONTROL:
#if defined (MASTER_NO_CONTROL)
		AppDataSet_MasterState(MASTER_STATE_SEND_GUI);
#else
	#if defined (MASTER_CONTROL_POS)
		if (TRUE == AppDataGet_UserButtonEvent())
		{
//			AppPeriodTask_TrajectoryPlanning();
			AppControl_Pos_TestSquence();	// Calculate position value to be sent
			AppCommUART_SendMsg(UART_NODE_SLAVE_1, UART_MSG_MOTOR_CONTROL_POS);	// Package and Send
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
			AppCommUART_SendMsg(UART_NODE_GUI, UART_MSG_GUI_DATA_1);
		}
	#else
		AppCommUART_SendMsg(UART_NODE_GUI, UART_MSG_GUI_DATA_2);
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
		AppDataSet_MasterState(MASTER_STATE_INIT);
		break;
	}
	return;
}
