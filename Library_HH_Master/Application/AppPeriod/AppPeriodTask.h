/**
 ********************************************************************************
 ** @file    AppPeriodTask.h
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Nov 7, 2024 (created)
 ** @brief   
 ********************************************************************************
 **/

#ifndef APPLICATION_APPPERIOD_APPPERIODTASK_H_
#define APPLICATION_APPPERIOD_APPPERIODTASK_H_


/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include <LibraryHHInterface_Master.h>

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/


/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef struct
{
    float Joint_1;
    float Joint_2;
    float Joint_3;
} strRobotJointInfor;	// Unit: 1 degree/bit


/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
extern GLOBAL strRobotJointInfor strRobotJoint;


/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
GLOBAL void AppPeriodTask_TaskCall(void);


#endif /* APPLICATION_APPPERIOD_APPPERIODTASK_H_ */
