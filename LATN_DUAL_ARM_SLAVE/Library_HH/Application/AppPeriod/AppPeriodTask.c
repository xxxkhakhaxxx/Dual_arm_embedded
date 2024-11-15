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

PRIVATE void AppPeriodTask_1ms_RobotKinematics(void);
PRIVATE void AppPeriodTask_10ms_Motor1Comm(void);
/* Kiểm tra quỹ đạo đặt -*/
/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
volatile static U32 debug_cnt_task_duplicate = 0;
volatile static U32 debug_cnt_task_override = 0;
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

	if ((u32TaskTimerCnt_1ms%3000)==0)
	{
		AppDataSet_Flag3s(TRUE);
	}
}

PRIVATE void AppPeriodTask_1ms_RobotKinematics(void)
{
	return;
}

BOOL bDirection = 0;
PRIVATE void AppPeriodTask_10ms_Motor1Comm(void)
{
//	ApiProtocolMotorMG_TestComm();
	static BOOL _initFlag = FALSE;
	static U08 _initCmdSquence = MOTOR_CMD_SET_ON;
	static U08 _periodCmdSquence = MOTOR_CMD_READ_ERROR;


	// Cmd sequence when init - Read parameter
	if (FALSE == _initFlag)
	{
		// Send cmd
		AppCommCAN_SendMotorMessage(MOTOR_1_ID, _initCmdSquence);

		// Change cmd
		switch (_initCmdSquence)
		{
		case MOTOR_CMD_SET_ON:						_initCmdSquence = MOTOR_CMD_READ_PID;					break;
		case MOTOR_CMD_READ_PID:					_initCmdSquence = MOTOR_CMD_READ_ACCEL;					break;
		case MOTOR_CMD_READ_ACCEL:					_initCmdSquence = MOTOR_CMD_READ_ENCODER;				break;
		case MOTOR_CMD_READ_ENCODER:				_initCmdSquence = MOTOR_CMD_READ_POSITION_MULTILOOP;	break;
		case MOTOR_CMD_READ_POSITION_MULTILOOP:		_initCmdSquence = MOTOR_CMD_READ_POSITION_SINGLELOOP;	break;
		case MOTOR_CMD_READ_POSITION_SINGLELOOP:	_initCmdSquence = MOTOR_CMD_READ_ERROR;					break;
		case MOTOR_CMD_READ_ERROR:					_initCmdSquence = MOTOR_CMD_READ_MECHANICAL_STATE;		break;
		case MOTOR_CMD_READ_MECHANICAL_STATE:		_initCmdSquence = MOTOR_CMD_READ_ELECTRIC_STATE;		break;
		case MOTOR_CMD_READ_ELECTRIC_STATE:
			// End of init
			_initFlag = TRUE;
			break;
		default:
			// Do nothing
			break;
		}

		return;		// No control if not done init
	}

	// Before control, check error
	if ((TRUE == AppDataGet_IsMotorLowVoltage(MOTOR_1_ID)) || \
		(TRUE == AppDataGet_IsMotorHighTemp(MOTOR_1_ID)))
	{
		switch (_periodCmdSquence)
		{
		case MOTOR_CMD_SET_STOP:
			AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_SET_STOP); // Send stop again
			_periodCmdSquence = MOTOR_CMD_SET_OFF;
			break;
		case MOTOR_CMD_SET_OFF:
			AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_SET_OFF); // Off motor - continuously
			break;
		default:
			AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_SET_STOP);
			_periodCmdSquence = MOTOR_CMD_SET_STOP;
			break;
		}

		return;		// No control if error
	}

	/** Cmd sequence after init:
	 * 	1/ Control: torque / speed / position (multi/single/jog)
	 *  2/ Read error
	 *  3/ Read eletric state
	**/
	switch (_periodCmdSquence)
	{
	case MOTOR_CMD_READ_ERROR:
		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_READ_ERROR);
		_periodCmdSquence = MOTOR_CMD_READ_ELECTRIC_STATE;
		break;
	case MOTOR_CMD_READ_ELECTRIC_STATE:
		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_READ_ELECTRIC_STATE);

		// Choose which to control here
//		_periodCmdSquence = MOTOR_CMD_CONTROL_TORQUE;
//		_periodCmdSquence = MOTOR_CMD_CONTROL_SPEED;
//		_periodCmdSquence = MOTOR_CMD_CONTROL_POSITION_MULTILOOP_2;
//		_periodCmdSquence = MOTOR_CMD_CONTROL_POSITION_SINGLELOOP_2;
		_periodCmdSquence = MOTOR_CMD_CONTROL_POSITION_JOG_2;
		break;
	case MOTOR_CMD_CONTROL_TORQUE:
		ApiProtocolMotorMG_SetTorque(MOTOR_1_ID, 50);								// This param can be change somewhere else
		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_CONTROL_TORQUE);
		_periodCmdSquence = MOTOR_CMD_READ_ERROR;
		break;
	case MOTOR_CMD_CONTROL_SPEED:
		ApiProtocolMotorMG_SetSpeed(MOTOR_1_ID, 180);								// This param can be change somewhere else
		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_CONTROL_SPEED);
		_periodCmdSquence = MOTOR_CMD_READ_ERROR;
		break;
	case MOTOR_CMD_CONTROL_POSITION_MULTILOOP_2:
		ApiProtocolMotorMG_SetAngleMulti(MOTOR_1_ID, 36000, 180);					// This param can be change somewhere else
		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_CONTROL_POSITION_MULTILOOP_2);
		_periodCmdSquence = MOTOR_CMD_READ_ERROR;
		break;
	case MOTOR_CMD_CONTROL_POSITION_SINGLELOOP_2:
		ApiProtocolMotorMG_SetAngleSingle(MOTOR_1_ID, 36000, 180, MOTOR_MOVE_CW);	// This param can be change somewhere else
		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_CONTROL_POSITION_SINGLELOOP_2);
		_periodCmdSquence = MOTOR_CMD_READ_ERROR;
		break;
	case MOTOR_CMD_CONTROL_POSITION_JOG_2:
//		ApiProtocolMotorMG_SetAngleJog(MOTOR_1_ID, 36000, 180);						// This param can be change somewhere else

		if (TRUE == AppDataGet_Flag3s())
		{
			if (0 == bDirection)
			{
				ApiProtocolMotorMG_SetAngleJog(MOTOR_1_ID, 36000, 180);						// This param can be change somewhere else
				bDirection = 1;
			}
			else
			{
				ApiProtocolMotorMG_SetAngleJog(MOTOR_1_ID, -36000, 180);						// This param can be change somewhere else
				bDirection = 0;
			}

			AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_CONTROL_POSITION_JOG_2);
			AppDataSet_Flag3s(FALSE);
		}
		_periodCmdSquence = MOTOR_CMD_READ_ERROR;
		break;
	default:
		// Do nothing
		break;
	}


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
	case TASK_1_10MS_MOTOR_1_COMM:
		AppPeriodTask_10ms_Motor1Comm();
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

