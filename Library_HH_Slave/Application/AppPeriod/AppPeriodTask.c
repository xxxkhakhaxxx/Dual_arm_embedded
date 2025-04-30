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
static BOOL _masterInitFlag = FALSE;
static BOOL _slaveInitFlag = FALSE;
static BOOL _sequenceEndFlag = FALSE;
static enCanNode _canNode = CAN_NODE_MOTOR_1;
static enRobotMode _robotMode = ROBOT_MODE_INIT;


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

//PRIVATE void AppPeriodTask_SetTaskFlag(enTaskList _TaskName);
PRIVATE void AppPeriodTask_Scheduler(void);

PRIVATE void AppPeriodTask_MotorComm(enCanNode _canNode, enRobotMode _mode);
PRIVATE BOOL _CheckJointLimit(void);
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
			// Change initial cmd: based on the previous cmd -> create a sequence
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
	AppCommCAN_SendMotorMsg(_motorId, _cmdSend);

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
/*
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
}*/

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


PRIVATE void AppPeriodTask_MotorComm(enCanNode _canNode, enRobotMode _mode)
{

	U08 _motorId;
	U08 _cmdSend;

// 1. Mapping the CAN NODE to MOTOR_ID
	switch (_canNode)
	{
	case CAN_NODE_MOTOR_1:	_motorId = MOTOR_1_ID;	break;
	case CAN_NODE_MOTOR_2:	_motorId = MOTOR_2_ID;	break;
	case CAN_NODE_MOTOR_3:	_motorId = MOTOR_3_ID;	break;
	default:	return;		// No NODE support, exit
	}

// 2. Mapping the ROBOT MODE to MG MOTOR CMD
	switch (_mode)
	{
	case ROBOT_MODE_INIT:		_cmdSend = MOTOR_CMD_SET_ON;						break;
	case ROBOT_MODE_READ_DATA:	_cmdSend = MOTOR_CMD_READ_MECHANICAL_STATE;			break;
	case ROBOT_MODE_POSITION:	_cmdSend = MOTOR_CMD_CONTROL_POSITION_SINGLELOOP_2;	break;
//	case ROBOT_MODE_VELOCITY:	_cmdSend = MOTOR_CMD_CONTROL_SPEED;					break;
	case ROBOT_MODE_TORQUE:		_cmdSend = MOTOR_CMD_CONTROL_TORQUE;				break;
	case ROBOT_MODE_ERROR_COMM:	_cmdSend = MOTOR_CMD_SET_STOP;						break;
	case ROBOT_MODE_ERROR_LIMIT:_cmdSend = MOTOR_CMD_SET_STOP;						break;
	default:	return;		// No CMD support, exit
	}

// 3. Send msg to motor
	AppCommCAN_SendMotorMsg(_motorId, _cmdSend);


	return;
}

PRIVATE BOOL _CheckJointLimit(void)
{
	BOOL isSafety = FALSE;

	if (
	(J1_KINE_LOW_LIMIT < myMotorToMaster[0].currPosKine) && (myMotorToMaster[0].currPosKine < J1_KINE_HIGH_LIMIT) && \
	(J2_KINE_LOW_LIMIT < myMotorToMaster[1].currPosKine) && (myMotorToMaster[1].currPosKine < J2_KINE_HIGH_LIMIT) && \
	(J3_KINE_LOW_LIMIT < myMotorToMaster[2].currPosKine) && (myMotorToMaster[2].currPosKine < J3_KINE_HIGH_LIMIT))
	{
		isSafety = TRUE;
	}

	return isSafety;
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

//		AppPeriodTask_Scheduler();		// Check time for which task to perform

		if (TASK_TIMER_CNT_ELAPSED_1S <= u32TaskTimerCnt_1ms)	// Cycle 10s
		{
			u32TaskTimerCnt_1ms = 0;
		}

		if (
		(TRUE == _slaveInitFlag) && \
		(SLAVE_STATE_WAIT_MASTER_REQUEST == AppDataGet_SlaveState())
		)
		{
			if (FALSE == _masterInitFlag)	// Waiting for Master init respond
			{
				if (u32TaskTimerCnt_1ms == 100)	// Every 100ms
				{
					AppDataSet_SlaveState(SLAVE_STATE_INIT);	// Re-send init to master: master takes ~2.1s to init
					u32TaskTimerCnt_1ms = 0;
				}
			}
			else // Master inited
			{
				if (u32TaskTimerCnt_1ms > 30)	// If wait Master request for more than 30ms -> Lost comm
				{
					u32TaskTimerCnt_1ms = 0;
					_robotMode = ROBOT_MODE_ERROR_COMM;		// Stop motor
					_canNode = CAN_NODE_MOTOR_1;
					_sequenceEndFlag = FALSE;	
					AppDataSet_SlaveState(SLAVE_STATE_SEND_MOTOR_SEQUENCE);	// Send motor state
				}
			}
		}
		else	// If not stuck in WAIT_MASTER_REQUEST, reset the count
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
	/*	Slave Sequence:
	 * 		1/ Init -> Send POWER ON to motors
	 * 		2/ Back to Init -> Send Master that Slave initialized (multiple) <-> Wait master request
	 * 		3/ Master request -> Send READ POSTIOTION to motors
	 * 		4/ Collected data -> Send to Master data
	 * 		5/ Master send control -> Send POSITION CONTROL to motors
	 */
	switch (AppDataGet_SlaveState())
	{
	case SLAVE_STATE_INIT:
		if (FALSE == _slaveInitFlag)
		{	// Set: Cmd -> Sequence -> Start -> State
			HAL_Delay(3000);
			_robotMode = ROBOT_MODE_INIT;							// Set init motor command
			_canNode = CAN_NODE_MOTOR_1;							// Set motor sequence: 1 -> 2 -> 3 -> end
			_sequenceEndFlag = FALSE;								// Start the sequence
			AppDataSet_SlaveState(SLAVE_STATE_SEND_MOTOR_SEQUENCE);	// Send motor state
		}
		else
		{
			AppCommUART_SendMsg(UART_NODE_MASTER, UART_MSG_INIT);	// Tell Master that Slave finished the init-process
			AppDataSet_SlaveState(SLAVE_STATE_WAIT_MASTER_REQUEST);	// Wait master state
		}
		break;

	case SLAVE_STATE_WAIT_MASTER_REQUEST:
		if (TRUE == AppDataGet_UartRxNewFlag(UART_NODE_MASTER))
		{
			// Check Rx header - first 3 bytes
			if (		// Master init message
			(MSG_INIT_BYTE_0 == RxDataMaster[0]) && \
			(MSG_INIT_BYTE_1 == RxDataMaster[1]) && \
			(MSG_INIT_LENGTH == RxDataMaster[2]))
			{
				AppCommUart_RecvMasterMsg(UART_MSG_INIT);				// Handle init msg
				_masterInitFlag = TRUE;									// Set Master done init
				AppCommUart_RecvMsgStart(UART_NODE_MASTER);				// Start waiting new request from master
				AppDataSet_SlaveState(SLAVE_STATE_WAIT_MASTER_REQUEST);	// Wait master state
			}
			else if (	// Master request data message
			(MSG_DATA_REQUEST_BYTE_0 == RxDataMaster[0]) && \
			(MSG_DATA_REQUEST_BYTE_1 == RxDataMaster[1]) && \
			(MSG_DATA_REQUEST_LENGTH == RxDataMaster[2]))
			{	// Set: Cmd -> Sequence -> Start -> State
				AppCommUart_RecvMasterMsg(UART_MSG_MOTOR_DATA);			// Handle data request msg
				_robotMode = ROBOT_MODE_READ_DATA;						// Set read motor command
				_canNode = CAN_NODE_MOTOR_1;							// Set motor sequence: 1 -> 2 -> 3 -> end
				_sequenceEndFlag = FALSE;								// Start the sequence
				AppDataSet_SlaveState(SLAVE_STATE_SEND_MOTOR_SEQUENCE);	// Send motor state
			}
			else if (	// Master control position message
			(MSG_CONTROL_POS_BYTE_0 == RxDataMaster[0]) && \
			(MSG_CONTROL_POS_BYTE_1 == RxDataMaster[1]) && \
			(MSG_CONTROL_POS_LENGTH == RxDataMaster[2]))
			{
				AppCommUart_RecvMasterMsg(UART_MSG_MOTOR_CONTROL_POS);
				_robotMode = ROBOT_MODE_POSITION;
				_canNode = CAN_NODE_MOTOR_1;
				_sequenceEndFlag = FALSE;								// Received request -> Start motor comm sequence_
				AppDataSet_SlaveState(SLAVE_STATE_SEND_MOTOR_SEQUENCE);	// Start control motors' position
			}
			else if (	// Master control torque message
			(MSG_CONTROL_TOR_BYTE_0 == RxDataMaster[0]) && \
			(MSG_CONTROL_TOR_BYTE_1 == RxDataMaster[1]) && \
			(MSG_CONTROL_TOR_LENGTH == RxDataMaster[2]))
			{
				AppCommUart_RecvMasterMsg(UART_MSG_MOTOR_CONTROL_TOR);
				_robotMode = ROBOT_MODE_TORQUE;
				_canNode = CAN_NODE_MOTOR_1;
				_sequenceEndFlag = FALSE;								// Received request -> Start motor comm sequence_
				AppDataSet_SlaveState(SLAVE_STATE_SEND_MOTOR_SEQUENCE);	// Start control motors' torque
			}
			else
			{
				AppDataSet_UartRxErrCnt(UART_NODE_MASTER);		// Receive un-support message ID in this Slave state
				AppCommUart_RecvMsgStart(UART_NODE_MASTER);		// Wait for another Master message
			}

			// Add Rx cnt and Reset Rx new flag
			AppDataSet_UartRxMsgCnt(UART_NODE_MASTER);
			AppDataSet_UartRxNewFlag(UART_NODE_MASTER, FALSE);
		}
		else
		{
			// Wait for UART Rx
		}
		break;

	case SLAVE_STATE_SEND_MOTOR_SEQUENCE:
#if defined (SLAVE_NO_MOTOR_COMM)
		// No motor comm
		AppDataSet_SlaveState(SLAVE_STATE_WAIT_MOTOR_FEEDBACK);
#else
		AppPeriodTask_MotorComm(_canNode, _robotMode);			// Send cmd to motor
		AppDataSet_SlaveState(SLAVE_STATE_WAIT_MOTOR_FEEDBACK);	// Wait motor state
#endif
		break;

	case SLAVE_STATE_WAIT_MOTOR_FEEDBACK:
#if defined (SLAVE_NO_MOTOR_COMM)
		HAL_Delay(1);	// Simulate 3 motors comm with 1ms delay

		// Finished motor comm
		_canNode = CAN_NODE_MOTOR_1;
		_sequenceEndFlag = TRUE;

		if (FALSE == _slaveInitFlag)
		{
			AppDataSet_SlaveState(SLAVE_STATE_INIT);	// Back to init state
			_slaveInitFlag = TRUE;
		}
		else	// Wait for master request
		{
			AppCommUART_SendMsg(UART_NODE_MASTER, UART_MSG_MOTOR_DATA);	// Empty data (no motor comm)
			AppDataSet_SlaveState(SLAVE_STATE_WAIT_MASTER_REQUEST);		// Wait for new request from Master
		}
#else
		if (TRUE == AppDataGet_CanRxNewFlag())
		{
			AppCommCAN_RecvMotorMsg();
			AppDataSet_CanRxNewFlag(FALSE);

			// Check to send next motor or stop
			switch (_canNode)
			{
			case CAN_NODE_MOTOR_1:	_canNode = CAN_NODE_MOTOR_2;	break;
			case CAN_NODE_MOTOR_2:	_canNode = CAN_NODE_MOTOR_3;	break;
			case CAN_NODE_MOTOR_3:
				_canNode = CAN_NODE_MOTOR_1;
				_sequenceEndFlag = TRUE;
				break;
			default:
				// Do nothing
				break;
			}

			// Change mode after CAN Rx handle
			if (TRUE == _sequenceEndFlag)
			{	// Finished motor comm
				if (FALSE == _slaveInitFlag)
				{
					AppDataSet_SlaveState(SLAVE_STATE_INIT);	// Back to init state
					_slaveInitFlag = TRUE;
				}
				else	// back to wait Master state
				{
					switch (_robotMode)
					{
					case ROBOT_MODE_ERROR_COMM:
					case ROBOT_MODE_ERROR_LIMIT:
						AppDataSet_SlaveState(SLAVE_STATE_ERROR);
						break;
					case ROBOT_MODE_READ_DATA:
						AppCommUART_SendMsg(UART_NODE_MASTER, UART_MSG_MOTOR_DATA);
						if (TRUE == _CheckJointLimit())
						{
							AppDataSet_SlaveState(SLAVE_STATE_WAIT_MASTER_REQUEST);		// Safety -> Wait for new request from Master
						}
						else // There's a joint excceed its safety limits
						{
							_robotMode = ROBOT_MODE_ERROR_LIMIT;	// Stop motor
							_canNode = CAN_NODE_MOTOR_1;
							_sequenceEndFlag = FALSE;
							AppDataSet_SlaveState(SLAVE_STATE_SEND_MOTOR_SEQUENCE);		// To send motor stop command
						}
						break;
					case ROBOT_MODE_POSITION:
					case ROBOT_MODE_TORQUE:
					default:
						AppCommUart_RecvMsgStart(UART_NODE_MASTER);
						AppDataSet_SlaveState(SLAVE_STATE_WAIT_MASTER_REQUEST);		// Wait for new request from Master
						break;
					}
				}
			}
			else
			{	// Continue motor comm
				AppDataSet_SlaveState(SLAVE_STATE_SEND_MOTOR_SEQUENCE);
			}
		}
		else
		{
			// Wait for CAN Rx
		}
#endif
		break;

	case SLAVE_STATE_ERROR:
		// TODO: implement Slave Error state
		break;

	default:
		// if any abnormal, reset variables and back to init state
		_slaveInitFlag = FALSE;
		_masterInitFlag = FALSE;
		_sequenceEndFlag = FALSE;
		_canNode = CAN_NODE_MOTOR_1;
		_robotMode = ROBOT_MODE_INIT;
		AppDataSet_SlaveState(SLAVE_STATE_INIT);
		break;
	}

	return;
}

