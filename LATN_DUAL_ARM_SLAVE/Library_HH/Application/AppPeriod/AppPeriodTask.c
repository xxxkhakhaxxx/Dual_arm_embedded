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

/********************************************************************************
 * PRIVATE TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef enum ENUM_TASK_LIST
{
	TASK_NONE = 0,
	TASK_1_10MS_MOTOR_1_COMM,
	TASK_2_10MS_MOTOR_2_COMM,
	TASK_3_10MS_MOTOR_3_COMM,

} enTaskList;

/********************************************************************************
 * PRIVATE VARIABLES
 ********************************************************************************/
PRIVATE volatile U32 u32TaskTimerCnt_1ms = 0;

PRIVATE enTaskList enTaskId = TASK_NONE;
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE void AppPeriodTask_SetTaskFlag(enTaskList _TaskName);
PRIVATE void AppPeriodTask_Scheduler(void);

PRIVATE void AppPeriodTask_1ms_RobotKinematics();
PRIVATE void AppPeriodTask_1ms_Motor1Comm();
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
	/* At a time, only 1 task is set to do */


	/* 10ms Task */
	switch ((u32TaskTimerCnt_1ms)%10)
	{
	case 1:			// 1ms - 11ms - 21ms - ...
		AppPeriodTask_SetTaskFlag(TASK_1_10MS_MOTOR_1_COMM);
		break;
	case 2:			// 2ms - 12ms - 22ms - ...
		AppPeriodTask_SetTaskFlag(TASK_2_10MS_MOTOR_2_COMM);
		break;
	case 3:			// 3ms - 13ms - 23ms - ...
		AppPeriodTask_SetTaskFlag(TASK_3_10MS_MOTOR_3_COMM);
	default:
		break;
	}

	/* 100ms Task */
	switch ((u32TaskTimerCnt_1ms)%100)
	{
	case 5:			// 5ms - 105ms - 205ms - ...
		break;
	case 33:		// 33ms - 133ms - 233ms - ...
		break;
	default:
		break;
	}
}

PRIVATE void AppPeriodTask_1ms_RobotKinematics()
{
	return;
}

PRIVATE void AppPeriodTask_1ms_Motor1Comm()
{
	ApiProtocolMotorMG_TestComm();
	return;
}
/********************************************************************************
 * GLOBAL FUNCTION IMPLEMENTATION
 ********************************************************************************/
GLOBAL void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
//		u32TaskTimerCnt_1ms = __HAL_TIM_GET_COUNTER(htim);			// Directly read from register can take more time
		u32TaskTimerCnt_1ms++;

		AppPeriodTask_Scheduler();		// Check time for which task to perform

		if (TASK_TIMER_CNT_ELAPSED <= u32TaskTimerCnt_1ms)	// Cycle 1s
		{
			u32TaskTimerCnt_1ms = 0;
		}
	}

}

GLOBAL void AppPeriodTask_TaskCall(void)	/* Performing the corresponding task */
{
	switch(enTaskId)
	{
	case TASK_1_10MS_MOTOR_1_COMM:
		AppPeriodTask_1ms_Motor1Comm();
		break;
	case TASK_2_10MS_MOTOR_2_COMM:

		break;
	case TASK_3_10MS_MOTOR_3_COMM:
	default:
		// None task
		break;
	}

	enTaskId = TASK_NONE;	// Clear task flag
}

