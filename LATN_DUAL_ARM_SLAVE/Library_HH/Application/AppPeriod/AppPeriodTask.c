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
#define TASK_TIMER_CNT_ELAPSED	(10000)
#define TASK_100US				(1)		// Note: only for timer step 100us
#define TASK_1MS				(10)
#define TASK_10MS				(100)
#define TASK_100MS				(1000)

/********************************************************************************
 * PRIVATE TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef enum ENUM_TASK_LIST
{
	TASK_NONE = 0,
	TASK_1_1MS_MOTOR_INIT,
	TASK_2_1MS,
	TASK_3_10MS,
	TASK_4_10MS,
	TASK_5_100MS,
	TASK_6_100MS

} enTaskList;

/********************************************************************************
 * PRIVATE VARIABLES
 ********************************************************************************/
PRIVATE volatile U32 u32TaskTimerCnt_100us = 0;

PRIVATE enTaskList enTaskId = TASK_NONE;
PRIVATE BOOL u8MotorInitDone = FALSE;
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE void AppPeriodTask_SetTaskFlag(enTaskList _TaskName);
PRIVATE void AppPeriodTask_Scheduler(void);
PRIVATE void AppPeriodTask_MotorInit(void);

PRIVATE void AppPeriodTask_1ms_RobotKinematics();
PRIVATE void AppPeriodTask_1ms_MotorComm();
/* Kiểm tra quỹ đạo đặt -*/
/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
PRIVATE void AppPeriodTask_SetTaskFlag(enTaskList _TaskName)
{
	if (_TaskName != enTaskId)
	{
		enTaskId = _TaskName;
	}

	return;
}

PRIVATE void AppPeriodTask_Scheduler(void)
{

	/* 1ms Task */
	switch ((u32TaskTimerCnt_100us)%(TASK_1MS))
	{
	case 1:			// 0.1ms - 1.1ms - 2.1ms - ...
		AppPeriodTask_SetTaskFlag(TASK_1_1MS_MOTOR_INIT);
		break;
	case 6:			// 0.6ms - 1.6ms - 2.6ms - ...
		AppPeriodTask_SetTaskFlag(TASK_2_1MS);
		break;
	default:
		break;
	}

	/* 10ms Task */
	switch ((u32TaskTimerCnt_100us)%(TASK_10MS))
	{
	case 5:			// 0.5ms - 10.5ms - 20.5ms - ...
		break;
	case 33:		// 3.3ms - 13.3ms - 23.3ms - ...
		break;
	default:
		break;
	}

	/* 100ms Task */
	switch ((u32TaskTimerCnt_100us)%(TASK_100MS))
	{
	case 99:		// 9.9ms - 109.9ms - 209.9ms - ...
		break;
	case 456:		// 45.6ms - 145.6ms - 245.6ms - ...
		break;
	default:
		break;
	}
}

PRIVATE void AppPeriodTask_1ms_RobotKinematics()
{
	return;
}

PRIVATE void AppPeriodTask_1ms_MotorComm()
{
	return;
}
/********************************************************************************
 * GLOBAL FUNCTION IMPLEMENTATION
 ********************************************************************************/
GLOBAL void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)		// time step TIM 2 = 100us
	{
//		u32TimerCnt100us = __HAL_TIM_GET_COUNTER(htim);			// Directly read from register can take more time
		u32TaskTimerCnt_100us++;

		AppPeriodTask_Scheduler();		// Check time for which task to perform

		if (TASK_TIMER_CNT_ELAPSED <= u32TaskTimerCnt_100us)	// Cycle 1s
		{
			u32TaskTimerCnt_100us = 0;
		}
	}

}

GLOBAL void AppPeriodTask_TaskCall(void)	/* Performing the corresponding task */
{
	switch(enTaskId)
	{
	case TASK_1_1MS_MOTOR_INIT:
		AppPeriodTask_MotorInit();
		break;
	case TASK_2_1MS:

		break;
	default:
		// None task
		break;
	}
}

PRIVATE void AppPeriodTask_MotorInit(void)
{
	if (TRUE ==  u8MotorInitDone)
		return;

//	switch (_MotorInitId)
//	{
//	case MOTOR_1_ID:
//
//		break;
//	case MOTOR_2_ID:
//
//		break;
//	case MOTOR_3_ID:
//
//		break;
//	default:
//
//		break;
//	}
}
