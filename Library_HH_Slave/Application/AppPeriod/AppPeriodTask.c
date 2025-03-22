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
	TASK_10MS_MASTER_COMM_RX,
	TASK_10MS_MOTOR_COMM,
	TASK_10MS_MASTER_COMM_TX

} enTaskList;

/********************************************************************************
 * PRIVATE VARIABLES
 ********************************************************************************/
PRIVATE enTaskList enTaskId = TASK_NONE;

/* Debug variables */
volatile static U32 u32TaskTimerCnt_1ms = 0;
volatile static U32 debug_cnt_task_duplicate = 0;
volatile static U32 debug_cnt_task_override = 0;
PRIVATE U32 debug_cnt_task_Motor = 0;
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE void AppPeriodTask_10ms_MotorComm(void);

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
//		AppPeriodTask_SetTaskFlag(TASK_10MS_MASTER_COMM_TX);
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

		if (TASK_TIMER_CNT_ELAPSED_1S <= u32TaskTimerCnt_1ms)	// Cycle 10s
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

		break;
	case TASK_10MS_MASTER_COMM_RX:

		break;
	default:
		// None task
		break;
	}

	enTaskId = TASK_NONE;	// Clear task flag
}

GLOBAL void AppPeriodTask_StateMachineProcess(void)
{
	static BOOL isInit = FALSE;


	switch (AppDataGet_SlaveState())
	{
	case SLAVE_STATE_INIT:
#if defined(TEST_SLAVE_UART)
		AppDataSet_SlaveState(SLAVE_STATE_UART_TEST);		// Directly change to Uart test state
#elif defined(TEST_UART_CYCLE_NO_FEEDBACK)
		// Master send init request to Slave -> Wait Master
		// Change state
		AppCommUart_RecvMsgStart(UART_NODE_MASTER);
		AppDataSet_SlaveState(SLAVE_STATE_WAIT_MASTER);

#else
		// Master send init request to Slave -> Check motor and then respond
		// TODO: implement check motor
		// AppDataSet_SlaveState(SLAVE_STATE_INIT_MOTOR);
#endif
		break;

	case SLAVE_STATE_WAIT_MASTER:
#if defined(TEST_UART_CYCLE_NO_FEEDBACK)
		if (TRUE == AppDataGet_UartRxNewFlag(UART_NODE_MASTER))	// Received Rx from Master
		{
			switch (RxDataMaster[0])			// Check Rx msg ID
			{
			case UART_RX_MSG_INIT:
				if ((RxDataMaster[1] == 0xFE) && (RxDataMaster[2] == 0xFE) && (RxDataMaster[3] == 0xFE))
				{
					AppDataSet_SlaveState(SLAVE_STATE_FEEDBACK_MASTER);
				}
				break;

			case UART_RX_MSG_SLAVE_SET_POSITION:
				// TODO: Process Master Rx data
				AppDataSet_SlaveState(SLAVE_STATE_CONTROL_MOTOR);
				break;

			default:
				// Something wrong, back to init
				AppDataSet_SlaveState(SLAVE_STATE_INIT);
				isInit = FALSE;
				break;
			}

			AppDataSet_UartRxNewFlag(UART_NODE_MASTER, FALSE);
		}
#endif
		break;

	case SLAVE_STATE_CONTROL_MOTOR:
#if defined(TEST_UART_CYCLE_NO_FEEDBACK)
		// Do nothing, move to motor feedback
		AppDataSet_SlaveState(SLAVE_STATE_MOTOR_FEEDBACK);

#endif
		break;

	case SLAVE_STATE_MOTOR_FEEDBACK:
#if defined(TEST_UART_CYCLE_NO_FEEDBACK)
		// Do nothing, move to feedback master
		AppDataSet_SlaveState(SLAVE_STATE_FEEDBACK_MASTER);
#endif
		break;

	case SLAVE_STATE_FEEDBACK_MASTER:
#if defined(TEST_UART_CYCLE_NO_FEEDBACK)
		if (FALSE == isInit) // One time only
		{
			AppCommUART_SendMsg(UART_NODE_MASTER, UART_TX_MSG_INIT);
			AppDataSet_SlaveState(SLAVE_STATE_WAIT_MASTER);
			isInit = TRUE;
		}
		else
		{	// Check last Master command
			switch (RxDataMaster[0])
			{
			case UART_RX_MSG_SLAVE_SET_POSITION:
				AppCommUART_SendMsg(UART_NODE_MASTER, UART_TX_MSG_SLAVE_SET_POSITION_FEEDBACK);
				AppDataSet_SlaveState(SLAVE_STATE_WAIT_MASTER);
				break;
			case UART_RX_MSG_SLAVE_SET_VELOCITY:

				break;
			case UART_RX_MSG_SLAVE_SET_TORQUE:

				break;
			default:
				AppDataSet_SlaveState(SLAVE_STATE_INIT);
				isInit = FALSE;
				break;
			}
		}
#endif
//		AppCommUART_SendMsg(UART_NODE_MASTER, UART_RX_MSG_INIT);
		AppDataSet_SlaveState(SLAVE_STATE_WAIT_MASTER);
		break;

	case SLAVE_STATE_UART_TEST:
		if (FALSE == AppDataGet_UartTxWaitFlag(UART_NODE_MASTER))
		{
			AppCommUART_SendMsg(UART_NODE_MASTER, UART_TX_MSG_TEST);
		}
		break;

	default:
		// if any abnormal, back to init state
		AppDataSet_SlaveState(SLAVE_STATE_INIT);
		isInit = FALSE;
		break;
	}
	return;
}
