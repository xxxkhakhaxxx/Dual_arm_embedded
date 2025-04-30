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
PRIVATE enBtnCtrlSequence _btnSequence = BTN_CTRL_INIT;
PRIVATE BOOL _masterInitFlag = FALSE;
PRIVATE BOOL _slave1InitFlag = FALSE;
PRIVATE BOOL _slave2InitFlag = FALSE;
PRIVATE BOOL _slave1HandleFlag = FALSE;
PRIVATE BOOL _slave2HandleFlag = FALSE;
PRIVATE U08  _guiSendCnt = 0;
volatile static BOOL bNewSequenceFlag = FALSE;


/* Debug variables */
volatile static U32 u32TaskTimerCnt_1ms = 0;
volatile static U32 debug_cnt_task_duplicate = 0;
volatile static U32 debug_cnt_task_override = 0;


/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/

/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE BOOL _CheckAllSlaveInit(void);
PRIVATE BOOL _CheckAllSlaveFeedback(void);
PRIVATE void _MasterStateControl(void);
PRIVATE void _MasterStateGUIComm(void);


/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
PRIVATE BOOL _CheckAllSlaveInit(void)
{
#if defined(SLAVE_1_ENA) && !defined(SLAVE_2_ENA)
	if (TRUE == _slave1InitFlag)
	{
		return TRUE;
	}
#elif !defined(SLAVE_1_ENA) && defined(SLAVE_2_ENA)
	if (TRUE == _slave2InitFlag)
	{
		return TRUE;
	}
#elif defined(SLAVE_1_ENA) && defined(SLAVE_2_ENA)
	if ((TRUE == _slave1InitFlag) && (TRUE == _slave2InitFlag))
	{
		return TRUE;
	}
#else
	#error [USER] Enable SLAVE_1_ENA or SLAVE_2_ENA or both
#endif
	return FALSE;
}

PRIVATE BOOL _CheckAllSlaveFeedback(void)
{
#if defined(SLAVE_1_ENA) && !defined(SLAVE_2_ENA)
	if (TRUE == _slave1HandleFlag)
	{
		return TRUE;
	}
#elif !defined(SLAVE_1_ENA) && defined(SLAVE_2_ENA)
	if (TRUE == _slave2HandleFlag)
	{
		return TRUE;
	}
#elif defined(SLAVE_1_ENA) && defined(SLAVE_2_ENA)
	if ((TRUE == _slave1HandleFlag) && (TRUE == _slave2HandleFlag))
	{
		return TRUE;
	}
#else
	#error [USER] Enable SLAVE_1_ENA or SLAVE_2_ENA or both
#endif
	return FALSE;
}

PRIVATE void _MasterStateControl(void)
{
	static BOOL isMovingToHome = FALSE;		// Home position
	static BOOL isMovingToStart = FALSE;	// TP start position
	static BOOL isSendControl = FALSE;		// Allow send control signal or not

	// 1. Change cmd if button is pressed
	if (TRUE == AppDataGet_UserButtonEvent())
	{
		switch (_btnSequence)
		{
//		case BTN_CTRL_INIT:				_btnSequence = BTN_CTRL_TEST_TOR_SEQUENCE;	break;
//		case BTN_CTRL_INIT:				_btnSequence = BTN_CTRL_TEST_POS_SEQUENCE;	break;
		case BTN_CTRL_INIT:				_btnSequence = BTN_CTRL_TO_HOME;			break;

		case BTN_CTRL_TO_HOME:			_btnSequence = BTN_CTRL_TO_PLANNING_INIT;	break;
		case BTN_CTRL_TO_PLANNING_INIT:	_btnSequence = BTN_CTRL_PLANNING;			break;
		case BTN_CTRL_PLANNING:			_btnSequence = BTN_CTRL_IDLE;				break;

		case BTN_CTRL_TEST_POS_SEQUENCE:
			AppControl_Pos_TestSequence(10);
			AppCommUART_SendMsg(UART_NODE_SLAVE_2, UART_MSG_MOTOR_CONTROL_POS);
			break;
		case BTN_CTRL_TEST_TOR_SEQUENCE:
			AppControl_Tor_TestSequence(RIGHT_ARM, 2);
			AppCommUART_SendMsg(UART_NODE_SLAVE_2, UART_MSG_MOTOR_CONTROL_TOR);
			break;

		case BTN_CTRL_IDLE:
		default:
			// Do nothing
			break;
		}
	}

	// 2. Process based on the cmd
	switch (_btnSequence)
	{
	case BTN_CTRL_TO_HOME:
		if (FALSE == isMovingToHome)
		{
			AppControl_Pos_MoveToHome(LEFT_ARM, HOME_SPEED);
			AppControl_Pos_MoveToHome(RIGHT_ARM, HOME_SPEED);
			AppCommUART_SendMsg(UART_NODE_SLAVE_1, UART_MSG_MOTOR_CONTROL_POS);
			AppCommUART_SendMsg(UART_NODE_SLAVE_2, UART_MSG_MOTOR_CONTROL_POS);
			isMovingToHome = TRUE;
		}
		break;

	case BTN_CTRL_TO_PLANNING_INIT:
#if 0	// WORLD SPACE PLANNING
		// If not moving to start, successfully init the selected trajectory type
		if (
		(FALSE == isMovingToStart) && \
		(TRUE == AppControl_TP_InitWorldTrajectory(TP_TYPE_TASK_CIRCLE)) && \
		(TRUE == AppControl_TP_TaskTrajectoryUpdate(PERIOD_TRAJECTORY_PLANNING)))
		{
			AppControl_IK_World2EE(LEFT_ARM);
			AppControl_IK_World2EE(RIGHT_ARM);
			AppControl_IK_EE2Joints(LEFT_ARM);
			AppControl_IK_EE2Joints(RIGHT_ARM);
			isSendControl = TRUE;
		}
		else
		{
			isSendControl = FALSE;
		}

		if (TRUE == isSendControl)
		{
			// Control position of each joint to the Start of trajectory
			AppControl_Pos_MoveToTpStart(LEFT_ARM, TP_START_SPEED);
			AppControl_Pos_MoveToTpStart(RIGHT_ARM, TP_START_SPEED);
			AppCommUART_SendMsg(UART_NODE_SLAVE_1, UART_MSG_MOTOR_CONTROL_POS);	// Send cmd
			AppCommUART_SendMsg(UART_NODE_SLAVE_2, UART_MSG_MOTOR_CONTROL_POS);
			isMovingToStart = TRUE;

			// If using Tor, init the selected controller
	#if defined (MASTER_CONTROL_TOR)
			AppControl_Tor_ControllerInit(TOR_CTRL_PD);
	#endif
		}
#elif 0 // JOINT SPACE PLANNING - OLD

		if (TRUE == AppControl_TP_SineWaveJoint(LEFT_ARM, PERIOD_TRAJECTORY_PLANNING))
		{
			AppCommUART_SendMsg(UART_NODE_SLAVE_1, UART_MSG_MOTOR_CONTROL_POS);
		}
		if (TRUE == AppControl_TP_SineWaveJoint(RIGHT_ARM, PERIOD_TRAJECTORY_PLANNING))
		{
			AppCommUART_SendMsg(UART_NODE_SLAVE_2, UART_MSG_MOTOR_CONTROL_POS);
		}
#else	// JOINT SPACE PLANNING - NEW
		if (
		(FALSE == isMovingToStart) && \
		(TRUE == AppControl_TP_JointTrajectoryInit(TP_TYPE_JOINT_SINEWAVE)) && \
		(TRUE == AppControl_TP_JointTrajectoryUpdate(PERIOD_TRAJECTORY_PLANNING))
		)
		{
			isSendControl = TRUE;
		}
		else
		{
			isSendControl = FALSE;
		}

		if (TRUE == isSendControl)
		{
			AppControl_Pos_MoveToTpStart(LEFT_ARM, TP_START_SPEED);
			AppControl_Pos_MoveToTpStart(RIGHT_ARM, TP_START_SPEED);
			AppCommUART_SendMsg(UART_NODE_SLAVE_1, UART_MSG_MOTOR_CONTROL_POS);
			AppCommUART_SendMsg(UART_NODE_SLAVE_2, UART_MSG_MOTOR_CONTROL_POS);
			isMovingToStart = TRUE;

			// If using Tor, init the selected controller
	#if defined (MASTER_CONTROL_TOR)
			AppControl_Tor_ControllerInit(TOR_CTRL_PD);
	#endif
		}
#endif
		break;

	case BTN_CTRL_PLANNING:
#if 0	// WORLD SPACE PLANNING
		if (TRUE == AppControl_TP_TaskTrajectoryUpdate(PERIOD_TRAJECTORY_PLANNING))
		{
			AppControl_IK_World2EE(LEFT_ARM);
			AppControl_IK_World2EE(RIGHT_ARM);
			AppControl_IK_EE2Joints(LEFT_ARM);
			AppControl_IK_EE2Joints(RIGHT_ARM);

			isSendControl = TRUE;
		}
		else	// Finished
		{
			isSendControl = FALSE;
		}
#else	// JOINT SPACE PLANNING
		AppControl_TP_JointTrajectoryUpdate(PERIOD_TRAJECTORY_PLANNING);
		isSendControl = TRUE;

#endif
#if defined (MASTER_CONTROL_POS)
		if (TRUE == isSendControl)
		{
			AppControl_Pos_FollowTpPos(LEFT_ARM);
			AppControl_Pos_FollowTpPos(RIGHT_ARM);
			AppCommUART_SendMsg(UART_NODE_SLAVE_1, UART_MSG_MOTOR_CONTROL_POS);
			AppCommUART_SendMsg(UART_NODE_SLAVE_2, UART_MSG_MOTOR_CONTROL_POS);
		}
#endif

#if defined (MASTER_CONTROL_TOR)
		// It should still update the torque when finished TP
//		if (TRUE == AppControl_Tor_ControlUpdateJoint(RIGHT_ARM, 0))
		if ((TRUE == AppControl_Tor_ControlUpdateArm(RIGHT_ARM)) && (TRUE == AppControl_Tor_ControlUpdateArm(LEFT_ARM)))
		{
			AppCommUART_SendMsg(UART_NODE_SLAVE_1, UART_MSG_MOTOR_CONTROL_TOR);
			AppCommUART_SendMsg(UART_NODE_SLAVE_2, UART_MSG_MOTOR_CONTROL_TOR);
		}
#endif	// MASTER_CONTROL_POS

		break;

	case BTN_CTRL_TEST_POS_SEQUENCE:	break;
	case BTN_CTRL_TEST_TOR_SEQUENCE:	break;

	case BTN_CTRL_INIT:
	case BTN_CTRL_IDLE:
	default:
		// Do nothing
		break;
	}

	return;
}

PRIVATE void _MasterStateGUIComm(void)
{
	if (_guiSendCnt < GUI_SEND_CNT_MAX)
	{
		_guiSendCnt++;
	}
	else	// If PERIOD_CONTROL == PERIOD_GUI_SEND -> Send every PERIOD_CONTROL
	{
		_guiSendCnt = 0;
#if defined (MASTER_NO_CONTROL) || defined (MASTER_CONTROL_POS)
	#if defined(SLAVE_1_ENA) && defined(SLAVE_2_ENA)
		AppCommUART_SendMsg(UART_NODE_GUI, UART_MSG_GUI_DATA_1_DUAL);
	#elif defined(SLAVE_1_ENA)
		AppCommUART_SendMsg(UART_NODE_GUI, UART_MSG_GUI_DATA_1_LEFT);
	#elif defined(SLAVE_2_ENA)
		AppCommUART_SendMsg(UART_NODE_GUI, UART_MSG_GUI_DATA_1_RIGHT);
	#endif
#elif defined (MASTER_CONTROL_TOR)
	#if defined(SLAVE_1_ENA) && defined(SLAVE_2_ENA)
		AppCommUART_SendMsg(UART_NODE_GUI, UART_MSG_GUI_DATA_2_DUAL);
	#elif defined(SLAVE_1_ENA)
		AppCommUART_SendMsg(UART_NODE_GUI, UART_MSG_GUI_DATA_2_LEFT);
	#elif defined(SLAVE_2_ENA)
		AppCommUART_SendMsg(UART_NODE_GUI, UART_MSG_GUI_DATA_2_RIGHT);
	#endif
#endif
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
		AppDataCheck_UserButtonState();	// Check user btn every 5ms

		if (FALSE == bNewSequenceFlag)	// Cycle 5ms
		{
			bNewSequenceFlag = TRUE;	// Start new sequence every 5ms
			AppDataSet_LedState(LED_3_ORANGE, FALSE);
		}
		else	// All tasks take more 5ms to finished
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

GLOBAL void AppPeriodTask_StateMachineProcess(void)
{
	// Full sequence: Init ➩ wait Slave init ➩ Send init to confirm ➩ ...
	//            ... ➩ Wait 5ms Flag ➩ Request data ➩ Handle data feedback ➩ Calculate control ➩ Send control
	//                          ↖	⇦   ⇦   ⇦   ⇦   ⇦   ⇦   ⇦   ⇦   ⇦   ⇦   ⇦   ⇦   ⇦   ⇦  GUI feedback ↩

	switch (AppDataGet_MasterState())
	{
	case MASTER_STATE_INIT:
		if (FALSE == _masterInitFlag)
		{
			AppCommUart_RecvMsgStart(UART_NODE_SLAVE_1);
			AppCommUart_RecvMsgStart(UART_NODE_SLAVE_2);
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
					// Wait for Slave 1 Tx
				}
			}

			if (FALSE == _slave2InitFlag)
			{
				if (TRUE == AppDataGet_UartRxNewFlag(UART_NODE_SLAVE_2))
				{
					if (
						(MSG_INIT_BYTE_0 == RxDataSlaveRight[0]) && \
						(MSG_INIT_BYTE_1 == RxDataSlaveRight[1]) && \
						(MSG_INIT_LENGTH == RxDataSlaveRight[2])
					)
					{	// Correct init format
						AppCommUART_SendMsg(UART_NODE_SLAVE_2, UART_MSG_INIT);
						_slave2InitFlag = TRUE;
					}
					else
					{
						// Wait for another Rx
						AppDataSet_UartRxErrCnt(UART_NODE_SLAVE_2);
						AppCommUart_RecvMsgStart(UART_NODE_SLAVE_2);
					}

					AppDataSet_UartRxMsgCnt(UART_NODE_SLAVE_2);
					AppDataSet_UartRxNewFlag(UART_NODE_SLAVE_2, FALSE);
				}
				else
				{
					// Wait for Slave 2 Tx
				}
			}

			// Exit INIT STATE
			if (TRUE == _CheckAllSlaveInit())
			{
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
			AppCommUART_SendMsg(UART_NODE_SLAVE_2, UART_MSG_MOTOR_DATA);
			_slave1HandleFlag = FALSE;
			_slave2HandleFlag = FALSE;
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

			if (TRUE == AppDataGet_UartRxNewFlag(UART_NODE_SLAVE_2))
			{
				AppDataSet_UartRxErrCnt(UART_NODE_SLAVE_2);		// Receive un-support message ID in this Master state
				AppDataSet_UartRxMsgCnt(UART_NODE_SLAVE_2);
				AppDataSet_UartRxNewFlag(UART_NODE_SLAVE_2, FALSE);
			}
		}
		break;

	case MASTER_STATE_WAIT_SLAVE_FEEDBACK:
		if ((TRUE == AppDataGet_UartRxNewFlag(UART_NODE_SLAVE_1)) && (FALSE == _slave1HandleFlag))
		{
			if (TRUE == AppCommUart_RecvSlaveMsg(UART_NODE_SLAVE_1))
			{	// Received slave 1 data
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

		if ((TRUE == AppDataGet_UartRxNewFlag(UART_NODE_SLAVE_2)) && (FALSE == _slave2HandleFlag))
		{
			if (TRUE == AppCommUart_RecvSlaveMsg(UART_NODE_SLAVE_2))
			{
				_slave2HandleFlag = TRUE;
			}
			else
			{
				AppDataSet_UartRxErrCnt(UART_NODE_SLAVE_2);		// Receive un-support message ID in this Master state
				AppCommUart_RecvMsgStart(UART_NODE_SLAVE_2);	// Wait for another Master message
			}

			AppDataSet_UartRxMsgCnt(UART_NODE_SLAVE_2);
			AppDataSet_UartRxNewFlag(UART_NODE_SLAVE_2, FALSE);
		}

		// Exit WAIT SLAVE state
		if (TRUE == _CheckAllSlaveFeedback())
		{
			AppDataSet_MasterState(MASTER_STATE_CAL_CONTROL);
		}
		break;

	case MASTER_STATE_CAL_CONTROL:
#ifndef MASTER_NO_CONTROL
		_MasterStateControl();
#endif
		AppDataSet_MasterState(MASTER_STATE_SEND_GUI);
		break;

	case MASTER_STATE_SEND_GUI:
#ifndef MASTER_NO_GUI
		_MasterStateGUIComm();
#endif
		bNewSequenceFlag = FALSE;	// Sequence end
		AppDataSet_MasterState(MASTER_STATE_WAIT_NEW_SEQUENCE);
		break;

	default:
		// if any abnormal, reset variables and back to init state
		_btnSequence = BTN_CTRL_INIT;
		_masterInitFlag = FALSE;
		_slave1InitFlag = FALSE;
		_slave2InitFlag = FALSE;
		_slave1HandleFlag = FALSE;
		_slave2HandleFlag = FALSE;
		bNewSequenceFlag  = FALSE;
		_guiSendCnt = 0;
		AppDataSet_MasterState(MASTER_STATE_INIT);
		break;
	}
	return;
}
