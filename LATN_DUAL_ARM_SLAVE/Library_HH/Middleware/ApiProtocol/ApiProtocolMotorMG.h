/**
 ********************************************************************************
 ** @file    ApiProtocolMotorMG.h
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Oct 30, 2024 (created)
 ** @brief   
 ********************************************************************************
 **/

#ifndef MIDDLEWARE_APIPROTOCOL_APIPROTOCOLMOTORMG_H_
#define MIDDLEWARE_APIPROTOCOL_APIPROTOCOLMOTORMG_H_


/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include "LibraryHHInterface.h"

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/
#define MOTOR_PROTOCOL_GET_ID(header)	(header-(0x141))	// From enMotorHeader to enMotorID
#define MOTOR_PROTOCOL_GET_HEADER(id)	(id+(0x141))	// From enMotorId to enMotorHeader

#define MOTOR_PROTOCOL_DATA_FRAME_LENGTH	(8)

// Protocol communicate command
#define	MOTOR_CMD_SET_OFF						0x80	// 2.1
#define	MOTOR_CMD_SET_ON						0x88	// 2.2
#define	MOTOR_CMD_SET_STOP						0x81	// 2.3

//#define	MOTOR_CMD_CONTROL_OPEN_LOOP				0xA0	// 2.4
#define	MOTOR_CMD_CONTROL_TORQUE				0xA1	// 2.5
#define	MOTOR_CMD_CONTROL_SPEED					0xA2	// 2.6
#define	MOTOR_CMD_CONTROL_POSITION_MULTILOOP_1	0xA3	// 2.7
#define	MOTOR_CMD_CONTROL_POSITION_MULTILOOP_2	0xA4	// 2.8
#define	MOTOR_CMD_CONTROL_POSITION_SINGLELOOP_1	0xA5	// 2.9
#define	MOTOR_CMD_CONTROL_POSITION_SINGLELOOP_2	0xA6	// 2.10
#define	MOTOR_CMD_CONTROL_POSITION_JOG_1		0xA7	// 2.11
#define	MOTOR_CMD_CONTROL_POSITION_JOG_2		0xA8	// 2.12

#define	MOTOR_CMD_READ_PID						0x30	// 2.13
#define	MOTOR_CMD_WRITE_PID_RAM					0x31	// 2.14
#define	MOTOR_CMD_WRITE_PID_ROM					0x32	// 2.15
#define	MOTOR_CMD_READ_ACCEL					0x33	// 2.16
#define	MOTOR_CMD_WRITE_ACCEL_RAM				0x34	// 2.17
#define	MOTOR_CMD_READ_ENCODER					0x90	// 2.18
#define	MOTOR_CMD_WRITE_ENCODER_OFFSET_ROM		0x91	// 2.19
#define	MOTOR_CMD_WRITE_ENCODER_ZERO_ROM		0x19	// 2.20
#define	MOTOR_CMD_READ_POSITION_MULTILOOP		0x92	// 2.21
#define	MOTOR_CMD_READ_POSITION_SINGLELOOP		0x94	// 2.22
//#define MOTOR_CMD_CLEAR_POSITION				0x95	// 2.23

#define	MOTOR_CMD_READ_ERROR					0x9A	// 2.24
#define	MOTOR_CMD_CLEAR_ERROR					0x9B	// 2.25
#define	MOTOR_CMD_READ_MECHANICAL_STATE			0x9C	// 2.26
#define	MOTOR_CMD_READ_ELECTRIC_STATE			0x9D	// 2.27

#define	MOTOR_CMD_MULTI_FORCE					0xFE	// 3.1

#define MOTOR_MG_5010_TORQUE_CONSTRAINT_LOW		(-430)	// ~-7A
#define MOTOR_MG_5010_TORQUE_CONSTRAINT_HIGH	(430)	// ~ 7A
#define MOTOR_MG_4010_TORQUE_CONSTRAINT_LOW		(-279)	// ~-4.5A
#define MOTOR_MG_4010_TORQUE_CONSTRAINT_HIGH	(279)	// ~ 4.5A
#define MOTOR_MG_SPEED_CONSTRAINT				(24000)
#define MOTOR_MG_MULTI_ANGLE_CONSTRAINT			(35999999)
#define MOTOR_MG_SINGLE_ANGLE_CONSTRAINT		(35999)
#define MOTOR_MG_JOG_ANGLE_CONSTRAINT			(35999999)
#define MOTOR_MG_ENCODER_14BIT_OFFSET			(16383)

#define MOTOR_MG_xx10_GEAR						(10)

#define MOTOR_MOVE_CW							(1)
#define MOTOR_MOVE_CCW							(0)
/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef struct
{
	struct
	{
		BOOL bOnOff;
		BOOL bStop;
		U08  u8Mode;
		U08  u8Error;
	} State;

	struct
	{
		I32 i32Accel;				// Unit : 1dps/s
		U08 u8AngleKp;
		U08 u8AngleKi;
		U08 u8SpeedKp;
		U08 u8SpeedKi;
		U08 u8TorqueKp;
		U08 u8TorqueKi;
//		U16 u16Encoder18BitInit;	// Range: 0~65535 (16 high bits)
	} Setting;

	struct
	{
		I64 i64AngleMulti;			// Unit : 0.01°/bit
		U32 u32AngleSingle;			// Range: 0~35999, Unit : 0.01°/bit
		U16 u16Encoder14BitOffset;	// Range: 0-16383
		U16 u16Encoder14BitRaw;		// Range: 0~16383
		U16 u16Encoder14Bit;		// Range: 0~16383 (raw - offset)
		U16 u16Encoder18Bit;		// Range: 0~65535 (16 high bits)
		U16 u16Voltage;				// Unit : 0.1V/bit
		I16 i16TorqueCurrent;		// Range: -33A~33A (-2048 to 2048)
		I16 i16Speed;				// Unit : 1dps/bit
		I16 i16CurrPhaseA;			// Unit : 1A/64bit
		I16 i16CurrPhaseB;			// Unit : 1A/64bit
		I16 i16CurrPhaseC;			// Unit : 1A/64bit
		I08 i8Temp;					// Unit : 1°C/bit

	} Data;
} strMotorMgRx;

typedef struct
{
	struct
	{
		I16 i16TorqueCurrent;		// Range: -2048~2048 (-33A~33A )

		I32 i32Speed;				// Unit : 0.01dps/bit

		I32 i32MultiAngle;			// Unit : 0.01°/bit (Example: 36000 = 360°) (Note: before gearbox)
		U16 u16MultiSpeed;			// Unit : 1 dps/bit (Example: 360 = 360°/s) (Note: before gearbox)

		BOOL bDirection;			// 0 is CW, 1 is CCW
		U32 u32SingleAngle;			// Unit : 0.01°/bit (Example: 36000 = 360°) (Note: before gearbox)
		U16 u16SingleSpeed;			// Unit : 1 dps/bit (Example: 360 = 360°/s) (Note: before gearbox)

		I32 i32JogAngle;			// Unit : 0.01°/bit (Example: 36000 = 360°) (Note: before gearbox)
		U16 u16JogSpeed;			// Unit : 1 dps/bit (Example: 360 = 360°/s) (Note: before gearbox)
	} Control;

	struct
	{
		U08 u8AngleKp;
		U08 u8AngleKi;
		U08 u8SpeedKp;
		U08 u8SpeedKi;
		U08 u8TorqueKp;
		U08 u8TorqueKi;
		I32 i32Accel;				// Unit : 1dps/s
		U16 u16Encoder14BitOffset;	// Range: 0-16383 (Note: its motor zero point)
	} Setting;
} strMotorMsgTx;


typedef enum ENUM_MOTOR_ID
{
	MOTOR_1_ID = 0x00,
	MOTOR_2_ID,
	MOTOR_3_ID,
} enMotorId;

typedef enum ENUM_MOTOR_HEADER
{
	MOTOR_HEADER_INIT = 0x140,
	MOTOR_1_HEADER = 0x141,
	MOTOR_2_HEADER,
	MOTOR_3_HEADER,
	MOTOR_4_HEADER,
	MOTOR_5_HEADER,
	MOTOR_6_HEADER,
	MOTOR_7_HEADER,
	MOTOR_8_HEADER,
	MOTOR_9_HEADER,
	MOTOR_10_HEADER,
	MOTOR_11_HEADER,
	MOTOR_12_HEADER,
	MOTOR_13_HEADER,
	MOTOR_14_HEADER,
	MOTOR_15_HEADER,
	MOTOR_16_HEADER,
	MOTOR_17_HEADER,
	MOTOR_18_HEADER,
	MOTOR_19_HEADER,
	MOTOR_20_HEADER,
	MOTOR_21_HEADER,
	MOTOR_22_HEADER,
	MOTOR_23_HEADER,
	MOTOR_24_HEADER,
	MOTOR_25_HEADER,
	MOTOR_26_HEADER,
	MOTOR_27_HEADER,
	MOTOR_28_HEADER,
	MOTOR_29_HEADER,
	MOTOR_30_HEADER,
	MOTOR_31_HEADER,
	MOTOR_32_HEADER,
	MOTOR_33_HEADER,
	MOTOR_34_HEADER,

	MOTOR_HEADER_MAX,
	MOTOR_HEADER_MULTI = 0x280
} enMotorHeader;

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
extern  strMotorMgRx strRobotArmMotor[3];
extern strMotorMsgTx strRobotArmMotorCmd[3];
/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL void ApiProtocolMotorMG_RxHandler(U32 _u32MotorIdentifier, U08* _b8RxDataBuffer);
GLOBAL void ApiProtocolMotorMG_TxHandler(enMotorId _u8MotorId, U08 _u8MessageID, U08* _b8TxDataBuffer);

GLOBAL void ApiProtocolMotorMG_SetTorque(enMotorId _u8MotorId, I16 _i16Torque);
GLOBAL void ApiProtocolMotorMG_SetSpeed(enMotorId _u8MotorId, I32 _i32Speed);
GLOBAL void ApiProtocolMotorMG_SetAngleMulti(enMotorId _u8MotorId, I32 _i32Angle, U16 _u16Speed);
GLOBAL void ApiProtocolMotorMG_SetAngleSingle(enMotorId _u8MotorId, U32 _u32Angle, U16 _u16Speed, BOOL _bDirection);
GLOBAL void ApiProtocolMotorMG_SetAngleJog(enMotorId _u8MotorId, I32 _i32Angle, U16 _u16Speed);
GLOBAL void ApiProtocolMotorMG_SetPID(enMotorId _u8MotorId, U08 _u8AngleKp, U08 _u8AngleKi, U08 _u8SpeedKp, U08 _u8SpeedKi, U08 _u8TorqueKp, U08 _u8TorqueKi);
GLOBAL void ApiProtocolMotorMG_SetAccel(enMotorId _u8MotorId, I32 _i32Accel);
GLOBAL void ApiProtocolMotorMG_SetEncodeOffset(enMotorId _u8MotorId, U16 _u16Offset);

extern GLOBAL U08 u8MotorCmdFlag;
GLOBAL void ApiProtocolMotorMG_TestComm();

#endif /* MIDDLEWARE_APIPROTOCOL_APIPROTOCOLMOTORMG_H_ */
