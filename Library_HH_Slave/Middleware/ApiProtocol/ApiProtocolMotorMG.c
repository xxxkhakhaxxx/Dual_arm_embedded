/**
 ********************************************************************************
 ** @file    ApiProtocolMotorMG.c
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Oct 30, 2024 (created)
 ** @brief   
 ********************************************************************************
 **/

/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include "ApiProtocolMotorMG.h"

/********************************************************************************
 * EXTERN VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE MACROS AND DEFINES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE TYPEDEFS AND ENUMS
 ********************************************************************************/


/********************************************************************************
 * PRIVATE VARIABLES
 ********************************************************************************/


/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
GLOBAL strMotorMsgRx strRobotArmMotorRx[3] = {0, };
GLOBAL strMotorMsgTx strRobotArmMotorTx[3] = {0, };


/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
/*		Function for Rx		*/
PRIVATE void _RxFncHandlerSameMsg(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _RxFncHandlerState1(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _RxFncHandlerState2(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _RxFncHandlerState3(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _RxFncHandlerPID(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _RxFncHandlerAccel(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _RxFncHandlerEncoder(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _RxFncHandlerEncoderOffset(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _RxFncHandlerMultiAngle(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _RxFncHandlerSingleAngle(U08 _u8MotorId, U08* _b8Buffer);

/*		Function for Tx		*/
PRIVATE void _TxFncHandlerControlTorque(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _TxFncHandlerControlSpeed(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _TxFncHandlerControlPositionMulti1(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _TxFncHandlerControlPositionMulti2(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _TxFncHandlerControlPositionSingle1(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _TxFncHandlerControlPositionSingle2(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _TxFncHandlerControlPositionJog1(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _TxFncHandlerControlPositionJog2(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _TxFncHandlerWritePID(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _TxFncHandlerWriteAccel(U08 _u8MotorId, U08* _b8Buffer);
PRIVATE void _TxFncHandlerWriteEncoderOffset(U08 _u8MotorId, U08* _b8Buffer);

/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
/* Rx function */
PRIVATE void _RxFncHandlerSameMsg(U08 _u8MotorId, U08* _b8Buffer)
{
	switch(_b8Buffer[0])
	{
	case MOTOR_CMD_SET_OFF:		strRobotArmMotorRx[_u8MotorId].State.bOnOff = FALSE;	break;
	case MOTOR_CMD_SET_ON:		strRobotArmMotorRx[_u8MotorId].State.bOnOff = TRUE;	break;
	case MOTOR_CMD_SET_STOP:	strRobotArmMotorRx[_u8MotorId].State.bStop = TRUE;	break;
	default:
		break;
	}

	return;
}

PRIVATE void _RxFncHandlerState1(U08 _u8MotorId, U08* _b8Buffer)	// Temperature-Voltage-Error
{	// _b8Buffer[1] = *((U08*)(_b8Buffer+1))
	strRobotArmMotorRx[_u8MotorId].Data.i8Temp		= _b8Buffer[1];
	strRobotArmMotorRx[_u8MotorId].Data.u16Voltage	= (((U16)_b8Buffer[3])<<8) | ((U16)_b8Buffer[4]);
	strRobotArmMotorRx[_u8MotorId].State.u8Error	= _b8Buffer[7];

	return;
}

PRIVATE void _RxFncHandlerState2(U08 _u8MotorId, U08* _b8Buffer)	// Temperature-Torque-Speed-Encoder
{
	strRobotArmMotorRx[_u8MotorId].Data.i8Temp 				= _b8Buffer[1];
	strRobotArmMotorRx[_u8MotorId].Data.i16TorqueCurrent	= (((I16)_b8Buffer[3])<<8) | ((I16)_b8Buffer[2]);
	strRobotArmMotorRx[_u8MotorId].Data.i16Speed			= (((I16)_b8Buffer[5])<<8) | ((I16)_b8Buffer[4]);
	strRobotArmMotorRx[_u8MotorId].Data.u16Encoder14Bit		= (((U16)_b8Buffer[7])<<8) | ((U16)_b8Buffer[6]);

	return;
}


PRIVATE void _RxFncHandlerState3(U08 _u8MotorId, U08* _b8Buffer)	// Temperature-PhaseCurrent
{
	strRobotArmMotorRx[_u8MotorId].Data.i16CurrPhaseA	= (((I16)_b8Buffer[3])<<8) | ((I16)_b8Buffer[2]);
	strRobotArmMotorRx[_u8MotorId].Data.i16CurrPhaseB	= (((I16)_b8Buffer[5])<<8) | ((I16)_b8Buffer[4]);
	strRobotArmMotorRx[_u8MotorId].Data.i16CurrPhaseC	= (((I16)_b8Buffer[7])<<8) | ((I16)_b8Buffer[6]);

	return;
}

PRIVATE void _RxFncHandlerPID(U08 _u8MotorId, U08* _b8Buffer)
{
	strRobotArmMotorRx[_u8MotorId].Setting.u8AngleKp  = _b8Buffer[2];
	strRobotArmMotorRx[_u8MotorId].Setting.u8AngleKi  = _b8Buffer[3];
	strRobotArmMotorRx[_u8MotorId].Setting.u8SpeedKp  = _b8Buffer[4];
	strRobotArmMotorRx[_u8MotorId].Setting.u8SpeedKi  = _b8Buffer[5];
	strRobotArmMotorRx[_u8MotorId].Setting.u8TorqueKp = _b8Buffer[6];
	strRobotArmMotorRx[_u8MotorId].Setting.u8TorqueKi = _b8Buffer[7];

	return;
}

PRIVATE void _RxFncHandlerAccel(U08 _u8MotorId, U08* _b8Buffer)
{
	strRobotArmMotorRx[_u8MotorId].Setting.i32Accel =	(((I32)_b8Buffer[7])<<24) | (((I32)_b8Buffer[6])<<16) | \
														(((I32)_b8Buffer[5])<< 8) |  ((I32)_b8Buffer[4]);

	return;
}

PRIVATE void _RxFncHandlerEncoder(U08 _u8MotorId, U08* _b8Buffer)
{
	strRobotArmMotorRx[_u8MotorId].Data.u16Encoder14Bit			= (((U16)_b8Buffer[3])<<8) | ((U16)_b8Buffer[2]);
	strRobotArmMotorRx[_u8MotorId].Data.u16Encoder14BitRaw		= (((U16)_b8Buffer[5])<<8) | ((U16)_b8Buffer[4]);
	strRobotArmMotorRx[_u8MotorId].Data.u16Encoder14BitOffset	= (((U16)_b8Buffer[7])<<8) | ((U16)_b8Buffer[6]);

	return;
}

PRIVATE void _RxFncHandlerEncoderOffset(U08 _u8MotorId, U08* _b8Buffer)
{
	strRobotArmMotorRx[_u8MotorId].Data.u16Encoder14BitOffset	= (((U16)_b8Buffer[7])<<8) | ((U16)_b8Buffer[6]);

	return;
}

PRIVATE void _RxFncHandlerMultiAngle(U08 _u8MotorId, U08* _b8Buffer)
{
	strRobotArmMotorRx[_u8MotorId].Data.i64AngleMulti =	(((I64)_b8Buffer[7])<<48) | (((I64)_b8Buffer[6])<<40) | \
														(((I64)_b8Buffer[5])<<32) | (((I64)_b8Buffer[4])<<24) | \
														(((I64)_b8Buffer[3])<<16) | (((I64)_b8Buffer[2])<< 8) | \
														(((I64)_b8Buffer[1]));

	return;
}

PRIVATE void _RxFncHandlerSingleAngle(U08 _u8MotorId, U08* _b8Buffer)
{
	strRobotArmMotorRx[_u8MotorId].Data.u32AngleSingle =	(((U32)_b8Buffer[7])<<24) | (((U32)_b8Buffer[6])<<16) | \
															(((U32)_b8Buffer[5])<< 8) |  ((U32)_b8Buffer[4]);

	return;
}


/* Tx function */
PRIVATE void _TxFncHandlerControlTorque(U08 _u8MotorId, U08* _b8Buffer)
{
	I16 _TxData = strRobotArmMotorTx[_u8MotorId].Control.i16TorqueCurrent;

	_b8Buffer[1] = 0x00;
	_b8Buffer[2] = 0x00;
	_b8Buffer[3] = 0x00;
	_b8Buffer[4] = (U08)((_TxData & 0x00FF)>>0);
	_b8Buffer[5] = (U08)((_TxData & 0xFF00)>>8);
	_b8Buffer[6] = 0x00;
	_b8Buffer[7] = 0x00;

	return;
}

PRIVATE void _TxFncHandlerControlSpeed(U08 _u8MotorId, U08* _b8Buffer)
{
	I32 _TxData = strRobotArmMotorTx[_u8MotorId].Control.i32Speed;

	_b8Buffer[1] = 0x00;
	_b8Buffer[2] = 0x00;
	_b8Buffer[3] = 0x00;
	_b8Buffer[4] = (U08)((_TxData & 0x000000FF)>>0);
	_b8Buffer[5] = (U08)((_TxData & 0x0000FF00)>>8);
	_b8Buffer[6] = (U08)((_TxData & 0x00FF0000)>>16);
	_b8Buffer[7] = (U08)((_TxData & 0xFF000000)>>24);

	return;
}

PRIVATE void _TxFncHandlerControlPositionMulti1(U08 _u8MotorId, U08* _b8Buffer)
{
	I32 _TxData = strRobotArmMotorTx[_u8MotorId].Control.i32MultiAngle;

	_b8Buffer[1] = 0x00;
	_b8Buffer[2] = 0x00;
	_b8Buffer[3] = 0x00;
	_b8Buffer[4] = (U08)((_TxData & 0x000000FF)>>0);
	_b8Buffer[5] = (U08)((_TxData & 0x0000FF00)>>8);
	_b8Buffer[6] = (U08)((_TxData & 0x00FF0000)>>16);
	_b8Buffer[7] = (U08)((_TxData & 0xFF000000)>>24);

	return;
}

PRIVATE void _TxFncHandlerControlPositionMulti2(U08 _u8MotorId, U08* _b8Buffer)
{
	U16 _TxData1 = strRobotArmMotorTx[_u8MotorId].Control.u16MultiSpeed;
	I32 _TxData2 = strRobotArmMotorTx[_u8MotorId].Control.i32MultiAngle;

	_b8Buffer[1] = 0x00;
	_b8Buffer[2] = (U08)((_TxData1 & 0x00FF)>>0);
	_b8Buffer[3] = (U08)((_TxData1 & 0xFF00)>>8);
	_b8Buffer[4] = (U08)((_TxData2 & 0x000000FF)>>0);
	_b8Buffer[5] = (U08)((_TxData2 & 0x0000FF00)>>8);
	_b8Buffer[6] = (U08)((_TxData2 & 0x00FF0000)>>16);
	_b8Buffer[7] = (U08)((_TxData2 & 0xFF000000)>>24);

	return;
}

PRIVATE void _TxFncHandlerControlPositionSingle1(U08 _u8MotorId, U08* _b8Buffer)
{
	BOOL _TxData1 = strRobotArmMotorTx[_u8MotorId].Control.bDirection;
	U32  _TxData2 = strRobotArmMotorTx[_u8MotorId].Control.u32SingleAngle;

	_b8Buffer[1] = _TxData1;
	_b8Buffer[2] = 0x00;
	_b8Buffer[3] = 0x00;
	_b8Buffer[4] = (U08)((_TxData2 & 0x000000FF)>>0);
	_b8Buffer[5] = (U08)((_TxData2 & 0x0000FF00)>>8);
	_b8Buffer[6] = (U08)((_TxData2 & 0x00FF0000)>>16);
	_b8Buffer[7] = (U08)((_TxData2 & 0xFF000000)>>24);

	return;
}

PRIVATE void _TxFncHandlerControlPositionSingle2(U08 _u8MotorId, U08* _b8Buffer)
{
	BOOL _TxData1 = strRobotArmMotorTx[_u8MotorId].Control.bDirection;
	U16  _TxData2 = strRobotArmMotorTx[_u8MotorId].Control.u16SingleSpeed;
	U32  _TxData3 = strRobotArmMotorTx[_u8MotorId].Control.u32SingleAngle;

	_b8Buffer[1] = _TxData1;
	_b8Buffer[2] = (U08)((_TxData2 & 0x00FF)>>0);
	_b8Buffer[3] = (U08)((_TxData2 & 0xFF00)>>8);
	_b8Buffer[4] = (U08)((_TxData3 & 0x000000FF)>>0);
	_b8Buffer[5] = (U08)((_TxData3 & 0x0000FF00)>>8);
	_b8Buffer[6] = (U08)((_TxData3 & 0x00FF0000)>>16);
	_b8Buffer[7] = (U08)((_TxData3 & 0xFF000000)>>24);

	return;
}

PRIVATE void _TxFncHandlerControlPositionJog1(U08 _u8MotorId, U08* _b8Buffer)
{
	I32 _TxData = strRobotArmMotorTx[_u8MotorId].Control.i32JogAngle;

	_b8Buffer[1] = 0x00;
	_b8Buffer[2] = 0x00;
	_b8Buffer[3] = 0x00;
	_b8Buffer[4] = (U08)((_TxData & 0x000000FF)>>0);
	_b8Buffer[5] = (U08)((_TxData & 0x0000FF00)>>8);
	_b8Buffer[6] = (U08)((_TxData & 0x00FF0000)>>16);
	_b8Buffer[7] = (U08)((_TxData & 0xFF000000)>>24);

	return;
}

PRIVATE void _TxFncHandlerControlPositionJog2(U08 _u8MotorId, U08* _b8Buffer)
{
	U16 _TxData1 = strRobotArmMotorTx[_u8MotorId].Control.u16JogSpeed;
	I32 _TxData2 = strRobotArmMotorTx[_u8MotorId].Control.i32JogAngle;

	_b8Buffer[1] = 0x00;
	_b8Buffer[2] = (U08)(((_TxData1 & 0x00FF)>>0));
	_b8Buffer[3] = (U08)(((_TxData1 & 0xFF00)>>8));
	_b8Buffer[4] = (U08)(((_TxData2 & 0x000000FF)>>0));
	_b8Buffer[5] = (U08)(((_TxData2 & 0x0000FF00)>>8));
	_b8Buffer[6] = (U08)(((_TxData2 & 0x00FF0000)>>16));
	_b8Buffer[7] = (U08)(((_TxData2 & 0xFF000000)>>24));

	return;
}

PRIVATE void _TxFncHandlerWritePID(U08 _u8MotorId, U08* _b8Buffer)
{
	_b8Buffer[1] = 0x00;
	_b8Buffer[2] = strRobotArmMotorTx[_u8MotorId].Setting.u8AngleKp;
	_b8Buffer[3] = strRobotArmMotorTx[_u8MotorId].Setting.u8AngleKi;
	_b8Buffer[4] = strRobotArmMotorTx[_u8MotorId].Setting.u8SpeedKp;
	_b8Buffer[5] = strRobotArmMotorTx[_u8MotorId].Setting.u8SpeedKi;
	_b8Buffer[6] = strRobotArmMotorTx[_u8MotorId].Setting.u8TorqueKp;
	_b8Buffer[7] = strRobotArmMotorTx[_u8MotorId].Setting.u8TorqueKi;

	return;
}

PRIVATE void _TxFncHandlerWriteAccel(U08 _u8MotorId, U08* _b8Buffer)
{
	I32 _TxData = strRobotArmMotorTx[_u8MotorId].Setting.i32Accel;

	_b8Buffer[1] = 0x00;
	_b8Buffer[2] = 0x00;
	_b8Buffer[3] = 0x00;
	_b8Buffer[4] = (U08)((_TxData & 0x000000FF)>>0);
	_b8Buffer[5] = (U08)((_TxData & 0x0000FF00)>>8);
	_b8Buffer[6] = (U08)((_TxData & 0x00FF0000)>>16);
	_b8Buffer[7] = (U08)((_TxData & 0xFF000000)>>24);

	return;
}

PRIVATE void _TxFncHandlerWriteEncoderOffset(U08 _u8MotorId, U08* _b8Buffer)
{
	U16 _TxData = strRobotArmMotorTx[_u8MotorId].Setting.u16Encoder14BitOffset;

	_b8Buffer[1] = 0x00;
	_b8Buffer[2] = 0x00;
	_b8Buffer[3] = 0x00;
	_b8Buffer[4] = 0x00;
	_b8Buffer[5] = 0x00;
	_b8Buffer[6] = (U08)((_TxData & 0x00FF)>>0);
	_b8Buffer[7] = (U08)((_TxData & 0xFF00)>>8);

	return;
}

/********************************************************************************
 * GLOBAL FUNCTION IMPLEMENTATION
 ********************************************************************************/
GLOBAL void ApiProtocolMotorMG_RxHandler(U08 _u8MotorId, U08* _b8RxDataBuffer)	// Call above _Rx static function
{
	/* Process the command data */
	switch(_b8RxDataBuffer[0])		// Check command id
	{
	case MOTOR_CMD_SET_OFF:
	case MOTOR_CMD_SET_ON:
	case MOTOR_CMD_SET_STOP:				_RxFncHandlerSameMsg(_u8MotorId, _b8RxDataBuffer);	break;

	case MOTOR_CMD_READ_ERROR:
	case MOTOR_CMD_CLEAR_ERROR:				_RxFncHandlerState1(_u8MotorId, _b8RxDataBuffer);	break;

	case MOTOR_CMD_CONTROL_TORQUE:
	case MOTOR_CMD_CONTROL_SPEED:
	case MOTOR_CMD_CONTROL_POSITION_MULTILOOP_1:
	case MOTOR_CMD_CONTROL_POSITION_MULTILOOP_2:
	case MOTOR_CMD_CONTROL_POSITION_SINGLELOOP_1:
	case MOTOR_CMD_CONTROL_POSITION_SINGLELOOP_2:
	case MOTOR_CMD_CONTROL_POSITION_JOG_1:
	case MOTOR_CMD_CONTROL_POSITION_JOG_2:
	case MOTOR_CMD_READ_MECHANICAL_STATE:	_RxFncHandlerState2(_u8MotorId, _b8RxDataBuffer);	break;

	case MOTOR_CMD_READ_ELECTRIC_STATE:		_RxFncHandlerState3(_u8MotorId, _b8RxDataBuffer);	break;

	case MOTOR_CMD_READ_PID:
	case MOTOR_CMD_WRITE_PID_RAM:
	case MOTOR_CMD_WRITE_PID_ROM:			_RxFncHandlerPID(_u8MotorId, _b8RxDataBuffer);		break;

	case MOTOR_CMD_READ_ACCEL:
	case MOTOR_CMD_WRITE_ACCEL_RAM:			_RxFncHandlerAccel(_u8MotorId, _b8RxDataBuffer);	break;

	case MOTOR_CMD_READ_ENCODER:			_RxFncHandlerEncoder(_u8MotorId, _b8RxDataBuffer);	break;

	case MOTOR_CMD_WRITE_ENCODER_OFFSET_ROM:
	case MOTOR_CMD_WRITE_ENCODER_ZERO_ROM:	_RxFncHandlerEncoderOffset(_u8MotorId, _b8RxDataBuffer);	break;

	case MOTOR_CMD_READ_POSITION_MULTILOOP:	_RxFncHandlerMultiAngle(_u8MotorId, _b8RxDataBuffer);		break;

	case MOTOR_CMD_READ_POSITION_SINGLELOOP:_RxFncHandlerSingleAngle(_u8MotorId, _b8RxDataBuffer);		break;

//	case MOTOR_CMD_CONTROL_OPEN_LOOP:
//	case MOTOR_CMD_CLEAR_POSITION:
//	case MOTOR_CMD_MULTI_FORCE:		// not here
	default:
		// Invalid message ID
		break;
	}

	return;
};

GLOBAL void ApiProtocolMotorMG_TxHandler(enMotorId _u8MotorId, U08 _u8MessageID, U08* _b8TxDataBuffer)
{
	/* Set the command id */
	_b8TxDataBuffer[0] = _u8MessageID;

	/* Process the command data */
	switch (_u8MessageID)
	{
	case MOTOR_CMD_CONTROL_TORQUE:					_TxFncHandlerControlTorque(_u8MotorId, _b8TxDataBuffer);			break;
	case MOTOR_CMD_CONTROL_SPEED:					_TxFncHandlerControlSpeed(_u8MotorId, _b8TxDataBuffer);				break;
	case MOTOR_CMD_CONTROL_POSITION_MULTILOOP_1:	_TxFncHandlerControlPositionMulti1(_u8MotorId, _b8TxDataBuffer);	break;
	case MOTOR_CMD_CONTROL_POSITION_MULTILOOP_2:	_TxFncHandlerControlPositionMulti2(_u8MotorId, _b8TxDataBuffer);	break;
	case MOTOR_CMD_CONTROL_POSITION_SINGLELOOP_1:	_TxFncHandlerControlPositionSingle1(_u8MotorId, _b8TxDataBuffer);	break;
	case MOTOR_CMD_CONTROL_POSITION_SINGLELOOP_2:	_TxFncHandlerControlPositionSingle2(_u8MotorId, _b8TxDataBuffer);	break;
	case MOTOR_CMD_CONTROL_POSITION_JOG_1:			_TxFncHandlerControlPositionJog1(_u8MotorId, _b8TxDataBuffer);		break;
	case MOTOR_CMD_CONTROL_POSITION_JOG_2:			_TxFncHandlerControlPositionJog2(_u8MotorId, _b8TxDataBuffer);		break;

	case MOTOR_CMD_WRITE_PID_RAM:
	case MOTOR_CMD_WRITE_PID_ROM:					_TxFncHandlerWritePID(_u8MotorId, _b8TxDataBuffer);					break;

	case MOTOR_CMD_WRITE_ACCEL_RAM:					_TxFncHandlerWriteAccel(_u8MotorId, _b8TxDataBuffer);				break;
	case MOTOR_CMD_WRITE_ENCODER_OFFSET_ROM:		_TxFncHandlerWriteEncoderOffset(_u8MotorId, _b8TxDataBuffer);		break;
	case MOTOR_CMD_MULTI_FORCE:
		break;

	case MOTOR_CMD_SET_OFF:
	case MOTOR_CMD_SET_ON:
	case MOTOR_CMD_SET_STOP:
	case MOTOR_CMD_READ_PID:
	case MOTOR_CMD_READ_ACCEL:
	case MOTOR_CMD_READ_ENCODER:
	case MOTOR_CMD_WRITE_ENCODER_ZERO_ROM:
	case MOTOR_CMD_READ_POSITION_MULTILOOP:
	case MOTOR_CMD_READ_POSITION_SINGLELOOP:
	case MOTOR_CMD_READ_ERROR:
	case MOTOR_CMD_CLEAR_ERROR:
	case MOTOR_CMD_READ_MECHANICAL_STATE:
	case MOTOR_CMD_READ_ELECTRIC_STATE:
		_b8TxDataBuffer[1] = 0x00;
		_b8TxDataBuffer[2] = 0x00;
		_b8TxDataBuffer[3] = 0x00;
		_b8TxDataBuffer[4] = 0x00;
		_b8TxDataBuffer[5] = 0x00;
		_b8TxDataBuffer[6] = 0x00;
		_b8TxDataBuffer[7] = 0x00;
		break;

	default:
		// Do nothing
		break;

	}

	return;
}



GLOBAL void ApiProtocolMotorMG_SetTorque(enMotorId _u8MotorId, I16 _i16Torque)
{
	switch (_u8MotorId)
	{
	case MOTOR_1_ID:
		strRobotArmMotorTx[_u8MotorId].Control.i16TorqueCurrent = CONSTRAIN(_i16Torque, MOTOR_MG_5010_TORQUE_CONSTRAINT_LOW, MOTOR_MG_5010_TORQUE_CONSTRAINT_HIGH);
		break;
	case MOTOR_2_ID:
	case MOTOR_3_ID:
		strRobotArmMotorTx[_u8MotorId].Control.i16TorqueCurrent = CONSTRAIN(_i16Torque, MOTOR_MG_4010_TORQUE_CONSTRAINT_LOW, MOTOR_MG_4010_TORQUE_CONSTRAINT_HIGH);
		break;
	default:
		// Do nothing
		break;
	}

	return;
}

GLOBAL void ApiProtocolMotorMG_SetSpeed(enMotorId _u8MotorId, I32 _i32Speed)
{
	// 1°/s/bit  ➔ (0.01°/s/bit after gear)
	switch (_u8MotorId)
	{
	case MOTOR_1_ID:
	case MOTOR_2_ID:
	case MOTOR_3_ID:
		strRobotArmMotorTx[_u8MotorId].Control.i32Speed = CONSTRAIN(_i32Speed*100*MOTOR_MG_xx10_GEAR, -MOTOR_MG_SPEED_CONSTRAINT, MOTOR_MG_SPEED_CONSTRAINT);
		break;
	default:
		// Do nothing
		break;
	}

	return;
}

GLOBAL void ApiProtocolMotorMG_SetAngleMulti(enMotorId _u8MotorId, I32 _i32Angle, U16 _u16Speed)
{
	// Angle: 0.01°/bit ➔ 0.01°/bit after gear
	// Speed: 1°/s/bit  ➔  1°/s/bit after gear
	switch (_u8MotorId)
	{
	case MOTOR_1_ID:
	case MOTOR_2_ID:
	case MOTOR_3_ID:
		strRobotArmMotorTx[_u8MotorId].Control.i32MultiAngle = CONSTRAIN(_i32Angle*MOTOR_MG_xx10_GEAR, -MOTOR_MG_MULTI_ANGLE_CONSTRAINT, MOTOR_MG_MULTI_ANGLE_CONSTRAINT);
		strRobotArmMotorTx[_u8MotorId].Control.u16MultiSpeed = _u16Speed*MOTOR_MG_xx10_GEAR;

		break;
	default:
		// Do nothing
		break;
	}
	return;
}

GLOBAL void ApiProtocolMotorMG_SetAngleSingle(enMotorId _u8MotorId, U32 _u32Angle, U16 _u16Speed, BOOL _bDirection)
{
	// Angle: 0.01°/bit ➔ 0.01°/bit after gear
	// Speed: 1°/s/bit  ➔  1°/s/bit after gear
	switch (_u8MotorId)
	{
	case MOTOR_1_ID:
	case MOTOR_2_ID:
	case MOTOR_3_ID:
		strRobotArmMotorTx[_u8MotorId].Control.bDirection     = _bDirection;
		strRobotArmMotorTx[_u8MotorId].Control.u32SingleAngle = CONSTRAIN(_u32Angle*MOTOR_MG_xx10_GEAR, 0, MOTOR_MG_SINGLE_ANGLE_CONSTRAINT);
		strRobotArmMotorTx[_u8MotorId].Control.u16SingleSpeed = _u16Speed*MOTOR_MG_xx10_GEAR;

		break;
	default:
		// Do nothing
		break;
	}
	return;
}

GLOBAL void ApiProtocolMotorMG_SetAngleJog(enMotorId _u8MotorId, I32 _i32Angle, U16 _u16Speed)
{
	// Angle: 0.01°/bit ➔ 0.01°/bit after gear
	// Speed: 1°/s/bit  ➔  1°/s/bit after gear
	switch (_u8MotorId)
	{
	case MOTOR_1_ID:
	case MOTOR_2_ID:
	case MOTOR_3_ID:
		strRobotArmMotorTx[_u8MotorId].Control.i32JogAngle = CONSTRAIN(_i32Angle*MOTOR_MG_xx10_GEAR, -MOTOR_MG_JOG_ANGLE_CONSTRAINT, MOTOR_MG_JOG_ANGLE_CONSTRAINT);
		strRobotArmMotorTx[_u8MotorId].Control.u16JogSpeed = _u16Speed*MOTOR_MG_xx10_GEAR;
		break;
	default:
		// Do nothing
		break;
	}

	return;
}

GLOBAL void ApiProtocolMotorMG_SetPID(enMotorId _u8MotorId, U08 _u8AngleKp, U08 _u8AngleKi, U08 _u8SpeedKp, U08 _u8SpeedKi, U08 _u8TorqueKp, U08 _u8TorqueKi)
{
	switch (_u8MotorId)
	{
	case MOTOR_1_ID:
	case MOTOR_2_ID:
	case MOTOR_3_ID:
		strRobotArmMotorTx[_u8MotorId].Setting.u8AngleKp = _u8AngleKp;
		strRobotArmMotorTx[_u8MotorId].Setting.u8AngleKi = _u8AngleKi;
		strRobotArmMotorTx[_u8MotorId].Setting.u8SpeedKp = _u8SpeedKp;
		strRobotArmMotorTx[_u8MotorId].Setting.u8SpeedKi = _u8SpeedKi;
		strRobotArmMotorTx[_u8MotorId].Setting.u8TorqueKp = _u8TorqueKp;
		strRobotArmMotorTx[_u8MotorId].Setting.u8TorqueKi = _u8TorqueKi;
		break;
	default:
		// Do nothing
		break;
	}

	return;
}

GLOBAL void ApiProtocolMotorMG_SetAccel(enMotorId _u8MotorId, I32 _i32Accel)
{
	// Accel: 1dps/s
	switch (_u8MotorId)
	{
	case MOTOR_1_ID:
	case MOTOR_2_ID:
	case MOTOR_3_ID:
		strRobotArmMotorTx[_u8MotorId].Setting.i32Accel = _i32Accel;
		break;
	default:
		// Do nothing
		break;
	}

	return;
}

GLOBAL void ApiProtocolMotorMG_SetEncodeOffset(enMotorId _u8MotorId, U16 _u16Offset)
{
	// Offset: 0~360 = 0~16383
	switch (_u8MotorId)
	{
	case MOTOR_1_ID:
	case MOTOR_2_ID:
	case MOTOR_3_ID:
		strRobotArmMotorTx[_u8MotorId].Setting.u16Encoder14BitOffset = CONSTRAIN(_u16Offset, 0, MOTOR_MG_ENCODER_14BIT_OFFSET);
		break;
	default:
		// Do nothing
		break;
	}
	return;
}



//GLOBAL U08 u8MotorCmdFlag = 1;
//GLOBAL U16 u16MotorCmdCnt = 0;
//GLOBAL void ApiProtocolMotorMG_TestComm()
//{
//	switch (u8MotorCmdFlag)
//	{
//
////	case MOTOR_CMD_READ_POSITION_SINGLELOOP:
////	case MOTOR_CMD_READ_ERROR:
////	case MOTOR_CMD_CLEAR_ERROR:
////	case MOTOR_CMD_READ_MECHANICAL_STATE:
////	case MOTOR_CMD_READ_ELECTRIC_STATE:
//	case 1:
//		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_SET_ON);
//		break;
//	case 2:
//		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_SET_OFF);
//		break;
//	case 3:
//		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_SET_STOP);
//		break;
//	case 4:
//		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_READ_PID);
//		break;
//	case 5:
//		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_READ_ACCEL);
//		break;
//	case 6:
//		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_READ_ENCODER);
//		break;
//	case 7:
////		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_WRITE_ENCODER_ZERO_ROM);
//		break;
//	case 8:
//		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_READ_POSITION_MULTILOOP);
//		break;
//	case 9:
//		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_READ_POSITION_SINGLELOOP);
//		break;
//	case 10:
//		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_READ_ERROR);
//		break;
//	case 11:
//		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_CLEAR_ERROR);
//		break;
//	case 12:
//		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_READ_MECHANICAL_STATE);
//		break;
//	case 13:
//		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_READ_ELECTRIC_STATE);
//		break;
//	case 14:
//		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_CONTROL_POSITION_MULTILOOP_2);
//		break;
//	case 15:
//		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_CONTROL_POSITION_SINGLELOOP_2);
//		break;
//	case 16:
//		strRobotArmMotorTx[MOTOR_1_ID].Control.i32JogAngle = 180*100*MOTOR_1_GEARBOX;
//		strRobotArmMotorTx[MOTOR_1_ID].Control.u16JogSpeed = 90*MOTOR_1_GEARBOX;
//		AppCommCAN_SendMotorMessage(MOTOR_1_ID, MOTOR_CMD_CONTROL_POSITION_JOG_2);
//		break;
//	default:
//		break;
//	}
//
//	if (100 > u16MotorCmdCnt)		// Send 100 cmd
//	{
//		u16MotorCmdCnt++;
//	}
//	else
//	{
//		if (0 != u8MotorCmdFlag)	// Not send anymore
//		{
//			u8MotorCmdFlag = 0;
//		}
//	}
//}
