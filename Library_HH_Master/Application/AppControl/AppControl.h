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

#define	HOME_SPEED		(10)
#define	HOME_SPEED_MAX	(40)
#define	HOME_POS_J11	(90.0f)
#define	HOME_POS_J21	(0.0f)
#define	HOME_POS_J31	(0.0f)
#define	HOME_POS_J12	(90.0f)
#define	HOME_POS_J22	(0.0f)
#define	HOME_POS_J32	(0.0f)

#define TP_START_SPEED		(10)
#define	TP_START_SPEED_MAX	(30)

#define TEST_SPEED_MAX	(20)
#define TEST_TORQUE_MAX	(2.0f)

/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/
typedef enum ENUM_TP_TYPE
{
	TP_TYPE_NONE = 0,

	TP_TYPE_CIRCLE,
	TP_TYPE_LINE,
	TP_TYPE_LINES,
	TP_TYPE_CUBIC,
	TP_TYPE_QUINTIC
} enTpType;

typedef enum ENUM_TOR_CONTROLLER
{
	TOR_CTRL_NONE = 0,

	TOR_CTRL_PD,
	TOR_CTRL_SPD,
	TOR_CTRL_SMC,
	TOR_CTRL_SSMC
} enTorController;

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
			float Start_X;		// [m]
			float Start_Y;		// [m]
			float Start_G;		// [Deg]
			float Goal_X;		// [m]
			float Goal_Y;		// [m]
			float Goal_G;		// [Deg]
		} Setting;	// Y = X
		
		struct
		{
			float TimeEnd;		// [s]
		} Ctrl;
	} Line;

	struct
	{
		struct
		{
			float X_Org;		// [m]
			float Y_Org;		// [m]
			float Radius;		// [m]
			float Freq;			// [Hz]
			float Phase;		// [Deg]

			float Gamma;		// [Deg]
		} Setting;

		struct
		{
			float TimeEnd;		// [s]
		} Ctrl;
	} Circle;

	struct
	{
		float Xm_t;		// [m]
		float Ym_t;		// [m]
		float Gm_t;		// [Rad]
	} CurrTrajectory;

	enTpType Type;
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

typedef struct
{
	struct
	{
		struct
		{
			float Kp[3];
			float Kd[3];
			float Alpha[3];
		} Setting;
	} PD;

	struct
	{
		struct
		{
			float Lamda[3];
			float K[3];
			float Eta[3];
			float Alpha[3];
		} Setting;
	} SMC;

	struct
	{
		BOOL isInit;
	} Ctrl;


	enTorController Type;
} strTorControl;
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
GLOBAL extern strTaskSpacePlanning  myTaskTrajectory;
GLOBAL extern strJointSpacePlanning myRobotTrajectory[DUAL_ARM];

GLOBAL extern strTorControl myController;


/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL BOOL AppControl_TP_SineWaveJoint(U08 _arm, float _timeStep);	// [s]
GLOBAL BOOL AppControl_TP_InitWorldTrajectory(enTpType _type);
GLOBAL BOOL AppControl_TP_UpdateWorldTrajectory(float _timeStep);	// TP ➡ T_Mass_World
GLOBAL void AppControl_IK_World2EE(U08 _arm);						// T_Mass_World ➡ T_4i_0i
GLOBAL void AppControl_IK_EE2Joints(U08 _arm);						// T_4i_0i ➡ q1/q2/q3

GLOBAL void AppControl_Pos_TestSequence(U16 _speed);
GLOBAL void AppControl_Pos_MoveToHome(U08 _arm, U16 _speed);
GLOBAL void AppControl_Pos_MoveToTpStart(U08 _arm, U16 _speed);
GLOBAL void AppControl_Pos_FollowTpPos(U08 _arm);

GLOBAL void AppControl_Tor_TestSequence(U08 _arm, U08 _joint);
GLOBAL BOOL AppControl_Tor_InitController(enTorController _type);
GLOBAL BOOL AppControl_Tor_ControlUpdate(U08 _arm, U08 _joint);



#endif /* APPLICATION_APPCONTROL_APPCONTROL_H_ */
