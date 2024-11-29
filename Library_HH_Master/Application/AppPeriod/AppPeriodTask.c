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
GLOBAL strRobotJointInfor strRobotJoint = {0, };


/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE void AppPeriodTask_RobotCalIK(void);
PRIVATE void AppPeriodTask_SlaveCmdKinematics(void);
PRIVATE void AppPeriodTask_SlaveCmdDirectAngle(void);

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
PRIVATE void AppPeriodTask_RobotCalIK(void)
{
	float x3, y3, c2, s2, c1, s1;
	float q1, q2;

	/* Calculating x3 y3 */
	x3 = (float)((float)Px - L3*(float)cos(Yaw * D2R));
	y3 = (float)((float)Py - L3*(float)sin(Yaw * D2R));

	/* Calculating Θ_2 */
	c2 = (float)((x3*x3 + y3*y3 - L1*L1 - L2*L2) / (2.0*L1*L2));
	s2 = -sqrt(1.0 - c2*c2);
//	s2 = sqrt(1.0 - c2*c2);

	q2 = atan2(s2,c2);

	/* Calculating Θ_1 */
	c1 = (x3*(L1 + L2*cos(q2)) +  y3*L2*sin(q2)) \
		  / ((L1 + L2*cos(q2))*(L1 + L2*cos(q2)) + (L2*sin(q2))*(L2*sin(q2)));

	s1 = (y3*(L1 + L2*cos(q2)) -  x3*L2*sin(q2)) \
		  / ((L1 + L2*cos(q2))*(L1 + L2*cos(q2)) + (L2*sin(q2))*(L2*sin(q2)));

	q1 = atan2(s1,c1);

	/* Calculating Θ_3 */
	strRobotJoint.Joint_1 = RAD2DEG(q1);
	strRobotJoint.Joint_2 = RAD2DEG(q2);
	strRobotJoint.Joint_3 = RAD2DEG(Yaw - strRobotJoint.Joint_1 - strRobotJoint.Joint_2);
	return;
}

PRIVATE void AppPeriodTask_SlaveCmdKinematics(void)
{
	AppCommSPI_SendSlaveMessage(SLAVE_1_ID, MASTER_MSG_ANGLE_KINEMATICS);
	return;
}

PRIVATE void AppPeriodTask_SlaveCmdDirectAngle(void)
{
	AppCommSPI_SendSlaveMessage(SLAVE_1_ID, MASTER_MSG_ANGLE_DIRECT);
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
//		AppPeriodTask_SlaveCmdKinematics();
		AppPeriodTask_SlaveCmdDirectAngle();
		break;
	default:
		// None task
		break;
	}

	enTaskId = TASK_NONE;	// Clear task flag
}

