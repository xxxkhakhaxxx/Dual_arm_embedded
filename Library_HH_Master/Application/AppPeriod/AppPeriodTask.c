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

#define L1              ((float)253.0) /* Length of 1st link - mm */
#define L2              ((float)253.0) /* Length of 2nd link - mm */
#define L3              ((float)139.0) /* Length of 3rd link - mm */
#define D2R             (0.01745329F)

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

PRIVATE I32 Px = 0;
PRIVATE I32 Py = 0;
PRIVATE float Yaw = 0;
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
GLOBAL strRobot strRobotDualArm = {0, };
GLOBAL strRobotRxData myRobotRx[2] = {0, };
/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE void _RobotCalculateIK(void);

PRIVATE void AppPeriodTask_SetTaskFlag(enTaskList _TaskName);
PRIVATE void AppPeriodTask_Scheduler(void);

/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
/**
* Input:
*  - Px: unit mm, type I32, Example: Px = 1000(mm)
*  - Py: unit mm, type I32, Example: Py = 500(mm)
*  - Yaw: unit 0.01°, type float, Example: Yaw = 2.5°
* Output:
*  - Θ: unit 0.01°, type float pointer, pointer allocation include J1, J2, and J3
 */
PRIVATE void _RobotCalculateIK(void)
{
	float x3, y3, c2, s2, c1, s1;
	float _q1, _q2;

	/* Calculating x3 y3 */
	x3 = (float)((float)Px - L3*(float)cos(Yaw * D2R));
	y3 = (float)((float)Py - L3*(float)sin(Yaw * D2R));

	/* Calculating Θ_2 */
	c2 = (float)((x3*x3 + y3*y3 - L1*L1 - L2*L2) / (2.0*L1*L2));
	s2 = -sqrt(1.0 - c2*c2);
//	s2 = sqrt(1.0 - c2*c2);

	_q2 = atan2(s2,c2);

	/* Calculating Θ_1 */
	c1 = (x3*(L1 + L2*cos(_q2)) +  y3*L2*sin(_q2)) \
		  / ((L1 + L2*cos(_q2))*(L1 + L2*cos(_q2)) + (L2*sin(_q2))*(L2*sin(_q2)));

	s1 = (y3*(L1 + L2*cos(_q2)) -  x3*L2*sin(_q2)) \
		  / ((L1 + L2*cos(_q2))*(L1 + L2*cos(_q2)) + (L2*sin(_q2))*(L2*sin(_q2)));

	_q1 = atan2(s1,c1);

	/* Calculating Θ_3 */
	strRobotDualArm.q1 = RAD2DEG(_q1);
	strRobotDualArm.q2 = RAD2DEG(_q2);
	strRobotDualArm.q3 = RAD2DEG(Yaw - strRobotDualArm.q1 - strRobotDualArm.q2);
	return;
}

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
		AppPeriodTask_SetTaskFlag(TASK_10MS_SLAVE_1_COMM);
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
		if (FALSE == bNewSequenceFlag)	// Cycle 10ms
		{
			bNewSequenceFlag = TRUE;	// Start new sequence every 10ms
		}
		else	// All tasks take more 10ms to finished
		{
			if (U16_MAX > debug_cnt_task_override)
			{
				debug_cnt_task_override++;
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
		_RobotCalculateIK();
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
//	static enRobotMode _robotMode = ROBOT_MODE_READ_ONLY;		// The mode that you want to use


	switch (AppDataGet_MasterState())
	{
	case MASTER_STATE_INIT:
		if (FALSE == _masterInitFlag)
		{
			AppCommUart_RecvMsgStart(UART_NODE_SLAVE_1);
//			AppCommUart_RecvMsgStart(UART_NODE_SLAVE_2);
			_masterInitFlag = TRUE;
			AppDataSet_LedState(LED_5_RED, TRUE);
		}
		else
		{
			if (FALSE == _slave1InitFlag)
			{
				if (TRUE == AppDataGet_UartRxNewFlag(UART_NODE_SLAVE_1))
				{
					if (
						(UART_MSG_INIT == RxDataSlaveLeft[0]) && \
						(MSG_INIT_BYTE_1 == RxDataSlaveLeft[1]) && \
						(MSG_INIT_BYTE_2 == RxDataSlaveLeft[2]) && \
						(MSG_INIT_BYTE_3 == RxDataSlaveLeft[3]) && \
						(MSG_INIT_BYTE_4 == RxDataSlaveLeft[4])
					)
					{	// Correct init format
						AppCommUART_SendMsg(UART_NODE_SLAVE_1, UART_MSG_INIT);
						_slave1InitFlag = TRUE;
					}
					else
					{
						// Wait for another Rx
						AppDataSet_UartRxMsgCnt(UART_NODE_SLAVE_1);
						AppCommUart_RecvMsgStart(UART_NODE_SLAVE_1);
					}
					AppDataSet_UartRxNewFlag(UART_NODE_SLAVE_1, FALSE);
				}
				else
				{
					// Wait for Slave Tx
				}
			}

			if (TRUE == _slave1InitFlag) //&& (TRUE == _slave2InitFlag)	// SLAVEs are ready
			{	// Exit INIT STATE
				HAL_Delay(2);	// Wait for the Master init cmd send
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
			AppDataSet_MasterState(MASTER_STATE_WAIT_SLAVE_FEEDBACK);
		}
		else
		{
			// Wait timer 2 trigger for new sequence
		/*	AppDataSet_LedState(LED_6_BLUE, FALSE);
			AppDataSet_LedState(LED_4_GREEN, FALSE);
			AppDataSet_LedState(LED_3_ORANGE, FALSE);*/
		}
		break;

	case MASTER_STATE_WAIT_SLAVE_FEEDBACK:
		if ((TRUE == AppDataGet_UartRxNewFlag(UART_NODE_SLAVE_1)) && (FALSE == _slave1HandleFlag))
		{
			switch (RxDataSlaveLeft[0])
			{
			case UART_MSG_MOTOR_DATA:
				memcpy(&myRobotRx[0].Joint[0].Position, &RxDataSlaveLeft[1],  sizeof(float));
				memcpy(&myRobotRx[0].Joint[0].Speed,    &RxDataSlaveLeft[5],  sizeof(float));
				memcpy(&myRobotRx[0].Joint[0].Accel,    &RxDataSlaveLeft[9],  sizeof(float));
				memcpy(&myRobotRx[0].Joint[1].Position, &RxDataSlaveLeft[13], sizeof(float));
				memcpy(&myRobotRx[0].Joint[1].Speed,    &RxDataSlaveLeft[17], sizeof(float));
				memcpy(&myRobotRx[0].Joint[1].Accel,    &RxDataSlaveLeft[21], sizeof(float));
				memcpy(&myRobotRx[0].Joint[2].Position, &RxDataSlaveLeft[25], sizeof(float));
				memcpy(&myRobotRx[0].Joint[2].Speed,    &RxDataSlaveLeft[29], sizeof(float));
				memcpy(&myRobotRx[0].Joint[2].Accel,    &RxDataSlaveLeft[33], sizeof(float));
				_slave1HandleFlag = TRUE;

				break;

			case UART_MSG_INIT:
			case UART_MSG_MOTOR_CONTROL:
			default:
				// Wait for another Rx
				AppDataSet_UartRxMsgCnt(UART_NODE_SLAVE_1);
				AppCommUart_RecvMsgStart(UART_NODE_SLAVE_1);
				break;;
			}
			AppDataSet_UartRxNewFlag(UART_NODE_SLAVE_1, FALSE);

		}

		if (TRUE == _slave1HandleFlag)
		{	// Exit WAIT SLAVE state
			AppDataSet_MasterState(MASTER_STATE_CAL_CONTROL);
		}
		AppDataSet_LedState(LED_4_GREEN, TRUE);
		break;

	case MASTER_STATE_CAL_CONTROL:
		AppDataSet_MasterState(MASTER_STATE_SEND_GUI);
		break;

	case MASTER_STATE_SEND_GUI:
		bNewSequenceFlag = FALSE;	// Sequence end
		AppDataSet_MasterState(MASTER_STATE_WAIT_NEW_SEQUENCE);
		break;

	default:
		// if any abnormal, back to init state
		AppDataSet_MasterState(MASTER_STATE_INIT);
		break;
	}
	return;
}
