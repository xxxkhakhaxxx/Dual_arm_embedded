/**
 ********************************************************************************
 ** @file    AppControl.h
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Apr 5, 2025 (created)
 ** @brief   
 ********************************************************************************
 **/

#ifndef APPLICATION_APPCONTROL_APPCONTROL_H_
#define APPLICATION_APPCONTROL_APPCONTROL_H_


/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include <LibraryHHInterface_Master.h>

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/
#define	SINGULAR_J11	(0.0f)
#define	SINGULAR_J21	(180.0f)
#define	SINGULAR_J31	(180.0f)
#define	SINGULAR_J12	(0.0f)
#define	SINGULAR_J22	(180.0f)
#define	SINGULAR_J32	(180.0f)

#define	HOME_SPEED		(20)
#define	HOME_SPEED_MAX	(40)
#define	HOME_POS_J11	(90.0f)
#define	HOME_POS_J21	(0.0f)
#define	HOME_POS_J31	(0.0f)
#define	HOME_POS_J12	(90.0f)
#define	HOME_POS_J22	(0.0f)
#define	HOME_POS_J32	(0.0f)

#define TEST_SPEED_MAX	(20)

/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef struct
{
	float Amp;		// Sine wave amplitude [Deg]
	float Freq;		// Oscillation frequency [Hz]
	float Phase;	// Phase offset [Deg]
	float Bias;		// Amplitude bias [Deg]
	float MoveTime;	// Total move time [s]
} strTpSineWave;

typedef struct
{
	struct
	{
		struct
		{
			float X;
			float Y;
			float G;
		} Start;
		
		struct
		{
			float X;
			float Y;
			float G;
		} Goal;
		
		struct
		{
			U32 Init;
			U32 MoveTime;
		} Time;
	} Line;

	struct
	{

	} Lines;

	struct
	{
		struct
		{
			float X_Org;		// [m]
			float Y_Org;		// [m]
			float Radius;		// [m]
			float Freq;			// [Hz]
			float Phase;		// [Deg]

			float Gamma;
		} Setting;

		struct
		{
			float MovedTime;	// [s]
			float TimeEnd;		// [s]
		} Ctrl;
	} Circle;

	struct
	{
		float Xm_t;		// [m]
		float Ym_t;		// [m]
		float Gm_t;		// [Rad]
	} CurrTrajectory;
} strTaskSpacePlanning;

typedef struct
{
	struct
	{
		float X_t;	// [m]
		float Y_t;	// [m]
		float G_t;	// [Rad]
	} EndEffector;

	struct
	{
		float currPos;		// [Rad]
		float prevPos;		// [Rad]
		float currVel;		// [Rad/s]
		float prevVel;		// [Rad/s]
		float currAccel;	// [Rad/s^2]
	} Joint[3];

	struct
	{
		BOOL isInit;
	} Ctrl;
} strJointSpacePlanning;


/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
GLOBAL extern strTaskSpacePlanning  myTaskTrajectory;
GLOBAL extern strJointSpacePlanning myRobotTrajectory[DUAL_ARM];


/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL void AppControl_CalRobotIK(void);
GLOBAL void AppControl_Pos_TestSquence(U08 _arm, U16 _speed);
GLOBAL void AppControl_Pos_BackToHome(U08 _arm, U16 _speed);
GLOBAL BOOL AppControl_TP_SineWaveJoint(U08 _arm, float _timeStep);	// [s]
GLOBAL BOOL AppControl_TP_CircleTool(float _timeStep);			// TP ➡ T_Mass_World
GLOBAL void AppControl_IK_Tool2EE(U08 _arm);					// T_Mass_World ➡ T_4i_0i
GLOBAL void AppControl_IK_EE2Joints(U08 _arm);					// T_4i_0i ➡ q1/q2/q3
GLOBAL void AppControl_Pos_UpdateTpData(U08 _arm);



#endif /* APPLICATION_APPCONTROL_APPCONTROL_H_ */
