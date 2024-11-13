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
GLOBAL strMotorMgRx strRobotArmMotor[3] = {0, };
GLOBAL strMotorMsgTx strRobotArmMotorCmd[3] = {0, };


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
	case MOTOR_CMD_SET_OFF:		strRobotArmMotor[_u8MotorId].State.bOnOff = FALSE;	break;
	case MOTOR_CMD_SET_ON:		strRobotArmMotor[_u8MotorId].State.bOnOff = TRUE;	break;
	case MOTOR_CMD_SET_STOP:	strRobotArmMotor[_u8MotorId].State.bStop = TRUE;	break;
	default:
		break;
	}

	return;
}

PRIVATE void _RxFncHandlerState1(U08 _u8MotorId, U08* _b8Buffer)	// Temperature-Voltage-Error
{	// _b8Buffer[1] = *((U08*)(_b8Buffer+1))
	strRobotArmMotor[_u8MotorId].Data.i8Temp = _b8Buffer[1];
	strRobotArmMotor[_u8MotorId].Data.u16Voltage = (((U16)_b8Buffer[3])<<8) | ((U16)_b8Buffer[4]);
	strRobotArmMotor[_u8MotorId].State.u8Error = _b8Buffer[7];

	return;
}

PRIVATE void _RxFncHandlerState2(U08 _u8MotorId, U08* _b8Buffer)	// Temperature-Torque-Speed-Encoder
{
	strRobotArmMotor[_u8MotorId].Data.i8Temp = _b8Buffer[1];
	strRobotArmMotor[_u8MotorId].Data.i16TorqueCurrent	= (((I16)_b8Buffer[3])<<8) | ((I16)_b8Buffer[2]);
	strRobotArmMotor[_u8MotorId].Data.i16Speed			= (((I16)_b8Buffer[5])<<8) | ((I16)_b8Buffer[4]);
	strRobotArmMotor[_u8MotorId].Data.u16Encoder14Bit	= (((U16)_b8Buffer[7])<<8) | ((U16)_b8Buffer[6]);

	return;
}


PRIVATE void _RxFncHandlerState3(U08 _u8MotorId, U08* _b8Buffer)	// Temperature-PhaseCurrent
{
	strRobotArmMotor[_u8MotorId].Data.i16CurrPhaseA	= (((I16)_b8Buffer[3])<<8) | ((I16)_b8Buffer[2]);
	strRobotArmMotor[_u8MotorId].Data.i16CurrPhaseB	= (((I16)_b8Buffer[5])<<8) | ((I16)_b8Buffer[4]);
	strRobotArmMotor[_u8MotorId].Data.i16CurrPhaseC	= (((I16)_b8Buffer[7])<<8) | ((I16)_b8Buffer[6]);

	return;
}

PRIVATE void _RxFncHandlerPID(U08 _u8MotorId, U08* _b8Buffer)
{
	strRobotArmMotor[_u8MotorId].Setting.u8AngleKp  = _b8Buffer[2];
	strRobotArmMotor[_u8MotorId].Setting.u8AngleKi  = _b8Buffer[3];
	strRobotArmMotor[_u8MotorId].Setting.u8SpeedKp  = _b8Buffer[4];
	strRobotArmMotor[_u8MotorId].Setting.u8SpeedKi  = _b8Buffer[5];
	strRobotArmMotor[_u8MotorId].Setting.u8TorqueKp = _b8Buffer[6];
	strRobotArmMotor[_u8MotorId].Setting.u8TorqueKi = _b8Buffer[7];

	return;
}

PRIVATE void _RxFncHandlerAccel(U08 _u8MotorId, U08* _b8Buffer)
{
	strRobotArmMotor[_u8MotorId].Setting.i32Accel =	(((I32)_b8Buffer[7])<<24) | (((I32)_b8Buffer[6])<<16) | \
													(((I32)_b8Buffer[5])<< 8) |  ((I32)_b8Buffer[4]);

	return;
}

PRIVATE void _RxFncHandlerEncoder(U08 _u8MotorId, U08* _b8Buffer)
{
	strRobotArmMotor[_u8MotorId].Data.u16Encoder14Bit		= (((U16)_b8Buffer[3])<<8) | ((U16)_b8Buffer[2]);
	strRobotArmMotor[_u8MotorId].Data.u16Encoder14BitRaw	= (((U16)_b8Buffer[5])<<8) | ((U16)_b8Buffer[4]);
	strRobotArmMotor[_u8MotorId].Data.u16Encoder14BitOffset = (((U16)_b8Buffer[7])<<8) | ((U16)_b8Buffer[6]);

	return;
}

PRIVATE void _RxFncHandlerEncoderOffset(U08 _u8MotorId, U08* _b8Buffer)
{
	strRobotArmMotor[_u8MotorId].Data.u16Encoder14BitOffset = (((U16)_b8Buffer[7])<<8) | ((U16)_b8Buffer[6]);

	return;
}

PRIVATE void _RxFncHandlerMultiAngle(U08 _u8MotorId, U08* _b8Buffer)
{
	strRobotArmMotor[_u8MotorId].Data.i64AngleMulti =	(((I64)_b8Buffer[7])<<48) | (((I64)_b8Buffer[6])<<40) | \
														(((I64)_b8Buffer[5])<<32) | (((I64)_b8Buffer[4])<<24) | \
														(((I64)_b8Buffer[3])<<16) | (((I64)_b8Buffer[2])<< 8) | \
														(((I64)_b8Buffer[1]));

	return;
}

PRIVATE void _RxFncHandlerSingleAngle(U08 _u8MotorId, U08* _b8Buffer)
{
	strRobotArmMotor[_u8MotorId].Data.u32AngleSingle =	(((U32)_b8Buffer[7])<<24) | (((U32)_b8Buffer[6])<<16) | \
														(((U32)_b8Buffer[5])<< 8) |  ((U32)_b8Buffer[4]);

	return;
}


/* Tx function */
PRIVATE void _TxFncHandlerControlTorque(U08 _u8MotorId, U08* _b8Buffer)
{
	I16 _TxData = strRobotArmMotorCmd[_u8MotorId].Control.i16TorqueCurrent;

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
	I32 _TxData = strRobotArmMotorCmd[_u8MotorId].Control.i32Speed;

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
	I32 _TxData = strRobotArmMotorCmd[_u8MotorId].Control.i32MultiAngle;

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
	U16 _TxData1 = strRobotArmMotorCmd[_u8MotorId].Control.u16MultiSpeed;
	I32 _TxData2 = strRobotArmMotorCmd[_u8MotorId].Control.i32MultiAngle;

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
	BOOL _TxData1 = strRobotArmMotorCmd[_u8MotorId].Control.bDirection;
	U32  _TxData2 = strRobotArmMotorCmd[_u8MotorId].Control.u32SingleAngle;

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
	BOOL _TxData1 = strRobotArmMotorCmd[_u8MotorId].Control.bDirection;
	U16  _TxData2 = strRobotArmMotorCmd[_u8MotorId].Control.u16SingleSpeed;
	U32  _TxData3 = strRobotArmMotorCmd[_u8MotorId].Control.u32SingleAngle;

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
	I32 _TxData = strRobotArmMotorCmd[_u8MotorId].Control.i32JogAngle;

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
	U16 _TxData1 = strRobotArmMotorCmd[_u8MotorId].Control.u16JogSpeed;
	I32 _TxData2 = strRobotArmMotorCmd[_u8MotorId].Control.i32JogAngle;

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
	_b8Buffer[2] = strRobotArmMotorCmd[_u8MotorId].Setting.u8AngleKp;
	_b8Buffer[3] = strRobotArmMotorCmd[_u8MotorId].Setting.u8AngleKi;
	_b8Buffer[4] = strRobotArmMotorCmd[_u8MotorId].Setting.u8SpeedKp;
	_b8Buffer[5] = strRobotArmMotorCmd[_u8MotorId].Setting.u8SpeedKi;
	_b8Buffer[6] = strRobotArmMotorCmd[_u8MotorId].Setting.u8TorqueKp;
	_b8Buffer[7] = strRobotArmMotorCmd[_u8MotorId].Setting.u8TorqueKi;

	return;
}

PRIVATE void _TxFncHandlerWriteAccel(U08 _u8MotorId, U08* _b8Buffer)
{
	I32 _TxData = strRobotArmMotorCmd[_u8MotorId].Setting.i32Accel;

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
	U16 _TxData = strRobotArmMotorCmd[_u8MotorId].Setting.u16Encoder14BitOffset;

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
GLOBAL void ApiProtocolMotorMG_RxHandler(U32 _u32MotorIdentifier, U08* _b8RxDataBuffer)	// Call above _Rx static function
{
	/* Check Identifier is valid for MG motor or not */
	if ((MOTOR_HEADER_INIT >=_u32MotorIdentifier) \
		|| ((MOTOR_HEADER_MAX <= _u32MotorIdentifier) && (MOTOR_HEADER_MULTI != _u32MotorIdentifier)))
	{
		// Invalid, stop process
		return;
	}

	/* Adjust MotorID in range 1-32: U08 size */
	U08 _u8MotorId = (U08)MOTOR_PROTOCOL_GET_ID(_u32MotorIdentifier);

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
}
