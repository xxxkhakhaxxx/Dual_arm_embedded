/**
 ********************************************************************************
 ** @file    AppData.c
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Oct 30, 2024 (created)
 ** @brief   
 ********************************************************************************
 **/

/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include "AppData.h"

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


/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/

/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/

/********************************************************************************
 * GLOBAL FUNCTION IMPLEMENTATION
 ********************************************************************************/


/** ###################### **/
GLOBAL BOOL AppDataGet_IsMotorLowVoltage(U08 _u8MotorId)
{	// 1 is TRUE, 0 is FALSE
	return GETBIT(strRobotArmMotor[_u8MotorId].State.u8Error,0);	// 1st bit
}

GLOBAL BOOL AppDataGet_IsMotorHighTemp(U08 _u8MotorId)
{	// 1 is TRUE, 0 is FALSE
	return GETBIT(strRobotArmMotor[_u8MotorId].State.u8Error,3);	// 4th bit
}

PRIVATE BOOL bCanRxMsgFlag = FALSE;			// Any data in CAN Rx or not
GLOBAL BOOL AppDataGet_CanRxMsgFlag(void)
{
	return bCanRxMsgFlag;
}
GLOBAL void AppDataSet_CanRxMsgFlag(BOOL _bFlag)
{
	if (_bFlag != bCanRxMsgFlag)
	{
		bCanRxMsgFlag = _bFlag;
	}
}

static BOOL bFlag3s = FALSE;
GLOBAL BOOL AppDataGet_Flag3s(void)
{
	return bFlag3s;
}
GLOBAL void AppDataSet_Flag3s(BOOL _bFlag)
{
	if (_bFlag != bFlag3s)
	{
		bFlag3s = _bFlag;
	}

	return;
}


