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
#define CURR_POS_J11	((float)myRobotFeedback[LEFT_ARM].Joint[0].Position)
#define CURR_POS_J21	((float)myRobotFeedback[LEFT_ARM].Joint[1].Position)
#define CURR_POS_J31	((float)myRobotFeedback[LEFT_ARM].Joint[2].Position)
#define CURR_VEL_J11	((float)myRobotFeedback[LEFT_ARM].Joint[0].Speed)
#define CURR_VEL_J21	((float)myRobotFeedback[LEFT_ARM].Joint[1].Speed)
#define CURR_VEL_J31	((float)myRobotFeedback[LEFT_ARM].Joint[2].Speed)
#define CURR_ACCEL_J11	((float)myRobotFeedback[LEFT_ARM].Joint[0].Accel)
#define CURR_ACCEL_J21	((float)myRobotFeedback[LEFT_ARM].Joint[1].Accel)
#define CURR_ACCEL_J31	((float)myRobotFeedback[LEFT_ARM].Joint[2].Accel)


/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef struct
{
	struct
	{
		float Angle;	// deg   per bit: range -360.0f ~ +360.0f
		U16 Speed;		// deg/s per bit
		U08 Direction;	// CC or CCW
	} JointPos[3];

/*	struct
	{
		I32 Speed;
	} JointVel[3];*/

	struct
	{
		I16 CurrentTor;
	} JointTor[3];

} strRobotDataCommand;

typedef struct
{
	struct
	{
		float Position;	// deg     per bit
		float Speed;	// deg/s   per bit
		float Accel;	// deg/s^2 per bit
	} Joint[3];
} strRobotDataFeedback;


/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
extern GLOBAL strRobotDataCommand  myRobotCommand[DUAL_ARM];			// Trajectory Planning data to be sent to Slave
extern GLOBAL strRobotDataFeedback myRobotFeedback[DUAL_ARM];		// Motors' data are received from Slave

/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
GLOBAL void AppPeriodTask_TaskCall(void);

GLOBAL void AppPeriodTask_StateMachineProcess(void);


#endif /* APPLICATION_APPPERIOD_APPPERIODTASK_H_ */
