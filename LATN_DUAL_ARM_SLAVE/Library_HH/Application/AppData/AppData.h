/**
 ********************************************************************************
 ** @file    AppData.h
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Oct 30, 2024 (created)
 ** @brief   
 ********************************************************************************
 **/

#ifndef APPLICATION_APPDATA_APPDATA_H_
#define APPLICATION_APPDATA_APPDATA_H_


/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include "LibraryHHInterface.h"

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/

/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
//static enMotorId _MotorInitId = MOTOR_1_ID;
//static enMotorCommSequence _MotorInitSequence = MOTOR_COMM_ON;

/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/

GLOBAL BOOL AppDataGet_IsMotorLowVoltage(U08 _u8MotorId);
GLOBAL BOOL AppDataGet_IsMotorHighTemp(U08 _u8MotorId);

GLOBAL BOOL AppDataGet_CanRxMsgFlag(void);
GLOBAL void AppDataSet_CanRxMsgFlag(BOOL _bFlag);


#endif /* APPLICATION_APPDATA_APPDATA_H_ */
