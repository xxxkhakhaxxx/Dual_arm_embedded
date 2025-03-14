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
	TASK_10MS_MASTER_COMM_RX,
	TASK_10MS_MOTOR_COMM,
	TASK_10MS_MASTER_COMM_TX

} enTaskList;

/********************************************************************************
 * PRIVATE VARIABLES
 ********************************************************************************/
PRIVATE volatile U32 u32TaskTimerCnt_1ms = 0;

PRIVATE enTaskList enTaskId = TASK_NONE;

PRIVATE U32 debug_cnt_task_Motor = 0;
volatile static U32 debug_cnt_task_duplicate = 0;
volatile static U32 debug_cnt_task_override = 0;
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE void AppPeriodTask_10ms_MotorComm(void);
PRIVATE void AppPeriodTask_10ms_MasterCmdHandle(void);
PRIVATE void AppPeriodTask_10ms_MasterFeedback(void);

PRIVATE void AppPeriodTask_SetTaskFlag(enTaskList _TaskName);
PRIVATE void AppPeriodTask_Scheduler(void);
/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
PRIVATE void AppPeriodTask_10ms_MotorComm(void)
{
//	ApiProtocolMotorMG_TestComm();
	static BOOL _initFlag = FALSE;
	static U08 _motorId = MOTOR_1_ID;
	static U08 _cmdChange = FALSE;
	static U08 _cmdSend = MOTOR_CMD_SET_ON;

	debug_cnt_task_Motor++;


	// 1. Set cmd if start new sequence
	if (TRUE == _cmdChange)		// when motorId = motor1
	{
		_cmdChange = FALSE;

		if (FALSE == _initFlag)	// Initial process
		{
			// Change initial cmd
			switch (_cmdSend)
			{
			case MOTOR_CMD_SET_ON:						_cmdSend = MOTOR_CMD_READ_PID;					break;
			case MOTOR_CMD_READ_PID:					_cmdSend = MOTOR_CMD_READ_ACCEL;				break;
			case MOTOR_CMD_READ_ACCEL:					_cmdSend = MOTOR_CMD_READ_ENCODER;				break;
			case MOTOR_CMD_READ_ENCODER:				_cmdSend = MOTOR_CMD_READ_POSITION_MULTILOOP;	break;
			case MOTOR_CMD_READ_POSITION_MULTILOOP:		_cmdSend = MOTOR_CMD_READ_POSITION_SINGLELOOP;	break;
			case MOTOR_CMD_READ_POSITION_SINGLELOOP:	_cmdSend = MOTOR_CMD_READ_ERROR;				break;
			case MOTOR_CMD_READ_ERROR:					_cmdSend = MOTOR_CMD_READ_MECHANICAL_STATE;		break;
			case MOTOR_CMD_READ_MECHANICAL_STATE:		_cmdSend = MOTOR_CMD_READ_ELECTRIC_STATE;		break;
			case MOTOR_CMD_READ_ELECTRIC_STATE:
				_cmdSend = MOTOR_CMD_READ_ERROR;
				_initFlag = TRUE;
				break;	// End of initial
			default:									_cmdSend = MOTOR_CMD_SET_ON;					break;
			}
		}
		else	// Normal process
		{
			// Check safety
			if ((TRUE == AppDataGet_IsMotorLowVoltage(_motorId)) || \
				(TRUE == AppDataGet_IsMotorHighTemp(_motorId)))
			{
				switch (_cmdSend)
				{
				case MOTOR_CMD_SET_STOP:				_cmdSend = MOTOR_CMD_SET_OFF;					break;
				case MOTOR_CMD_SET_OFF:					_cmdSend = MOTOR_CMD_SET_OFF;					break;	// Keep off
				default:								_cmdSend = MOTOR_CMD_SET_STOP;					break;	// Change to send stop right away
				}
			}
			else	// Normal process sequence
			{
				switch (_cmdSend)
				{
				case MOTOR_CMD_READ_ERROR:				_cmdSend = MOTOR_CMD_READ_ELECTRIC_STATE;		break;
				case MOTOR_CMD_READ_ELECTRIC_STATE:		_cmdSend = MOTOR_CMD_READ_MECHANICAL_STATE;		break;
				case MOTOR_CMD_READ_MECHANICAL_STATE:	_cmdSend = MOTOR_CMD_READ_POSITION_MULTILOOP;	break;
				case MOTOR_CMD_READ_POSITION_MULTILOOP: _cmdSend = MOTOR_CMD_READ_ERROR;				break;
				default:								_cmdSend = MOTOR_CMD_READ_ERROR;				break;
				}
			}
		}
	}

	// 2. Send msg to motor
	AppCommCAN_SendMotorMessage(_motorId, _cmdSend);

	// 3. Update new MotorId for sending msg next time
	switch (_motorId)
	{
	case MOTOR_1_ID:	_motorId = MOTOR_2_ID;		break;
	case MOTOR_2_ID:	_motorId = MOTOR_3_ID;		break;
	case MOTOR_3_ID:
		_motorId = MOTOR_1_ID;
		_cmdChange = TRUE;
		break;
	default:			_motorId = MOTOR_1_ID;		break;
	}

	return;
}

PRIVATE void AppPeriodTask_10ms_MasterCmdHandle(void)
{
	return;
}

PRIVATE void AppPeriodTask_10ms_MasterFeedback(void)
{
	AppCommSPI_SendMasterMessage(SLAVE_MSG_POSITION);
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
	case 1:
//		AppPeriodTask_SetTaskFlag(TASK_10MS_MASTER_COMM_RX);
		break;
	case 3:			// 1ms - 11ms - 21ms - ...
	case 4:
	case 5:
//		AppPeriodTask_SetTaskFlag(TASK_10MS_MOTOR_COMM);
		break;
	case 7:
		AppPeriodTask_SetTaskFlag(TASK_10MS_MASTER_COMM_TX);
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
	case TASK_10MS_MOTOR_COMM:
		AppPeriodTask_10ms_MotorComm();
		break;
	case TASK_10MS_MASTER_COMM_TX:
		AppPeriodTask_10ms_MasterFeedback();
		break;
	case TASK_10MS_MASTER_COMM_RX:
		AppPeriodTask_10ms_MasterCmdHandle();
		break;
	default:
		// None task
		break;
	}

	enTaskId = TASK_NONE;	// Clear task flag
}

