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
	TASK_10MS_GUI_COMM,
	TASK_10MS_ROBOT_IK,
	TASK_10MS_ROBOT_CONTROLLER,
	TASK_10MS_SLAVE_1_COMM,	// Send and return
	TASK_10MS_SLAVE_2_COMM,	// Send and return

} enTaskList;

/********************************************************************************
 * PRIVATE VARIABLES
 ********************************************************************************/
PRIVATE volatile U32 u32TaskTimerCnt_1ms = 0;

PRIVATE enTaskList enTaskId = TASK_NONE;

volatile static U32 debug_cnt_task_duplicate = 0;
volatile static U32 debug_cnt_task_override = 0;
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE void AppPeriodTask_RobotCalIK(void);
PRIVATE void AppPeriodTask_SlaveCmdKinematics(void);

PRIVATE void AppPeriodTask_SetTaskFlag(enTaskList _TaskName);
PRIVATE void AppPeriodTask_Scheduler(void);

/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
PRIVATE void AppPeriodTask_RobotCalIK(void)
{

}

PRIVATE void AppPeriodTask_SlaveCmdKinematics(void)
{
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
//	case 5:
//		AppPeriodTask_SetTaskFlag(TASK_10MS_SLAVE_1_COMM);
//		break;
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
//		u32TaskTimerCnt_1ms = __HAL_TIM_GET_COUNTER(htim);			// Directly read from register can take more time
		u32TaskTimerCnt_1ms++;

		AppPeriodTask_Scheduler();		// Check time for which task to perform

		if (TASK_TIMER_CNT_ELAPSED <= u32TaskTimerCnt_1ms)	// Cycle 10s
		{
			u32TaskTimerCnt_1ms = 0;
		}
	}

}

GLOBAL void AppPeriodTask_TaskCall(void)	/* Performing the corresponding task */
{
	switch(enTaskId)
	{
	case TASK_10MS_ROBOT_IK:
		AppPeriodTask_RobotCalIK();
		break;
	case TASK_10MS_SLAVE_1_COMM:
		AppPeriodTask_SlaveCmdKinematics();
		break;
	default:
		// None task
		break;
	}

	enTaskId = TASK_NONE;	// Clear task flag
}

