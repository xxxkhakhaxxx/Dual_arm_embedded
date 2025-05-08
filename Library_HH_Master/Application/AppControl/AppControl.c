/**
 ********************************************************************************
 ** @file    AppControl.c
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Apr 5, 2025 (created)
 ** @brief   
 ********************************************************************************
 **/

/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include "AppControl.h"

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
// Sinewave test value
PRIVATE strTpSineWave mySineTrajectory[DUAL_ARM][3] = {
		// Amp  , Freq , Phase, Bias ,  Time
	{	{ 20.0f , 0.2f , 0.0f , 90.0f, 5.0f },		// LEFT_ARM
		{ 20.0f , 0.2f , 0.0f ,  0.0f, 5.0f },		// 2025-04-10: Don't set the amplitude to high and run to fast
		{ 20.0f , 0.2f , 0.0f ,  0.0f, 5.0f }	},
	{	{ 20.0f , 0.2f , 180.0f , 90.0f, 5.0f },		// RIGHT_ARM
		{ 20.0f , 0.2f , 180.0f ,  0.0f, 5.0f },
		{ 20.0f , 0.2f , 180.0f ,  0.0f, 5.0f }	}};
PRIVATE BOOL _trajectoryState[DUAL_ARM][3] = {
		{ FALSE, FALSE, FALSE },
		{ FALSE, FALSE, FALSE }};
PRIVATE float _totalTime[DUAL_ARM] = { 0.0f , 0.0f };
PRIVATE U08	_currJoint[DUAL_ARM] = { 0 , 0 };	// Joint 1-2-3 = 0-1-2

PRIVATE U08 _btnSequenceTest = 0;


/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
GLOBAL strTrajectoryPlanning  myTrajectory = { 0, };
GLOBAL strJointSpacePlanning myRobotTrajectory[DUAL_ARM] = { 0, };
GLOBAL strTorControl myControl = { 0, };


/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE void _TP_CalJointRefData(U08 _arm, float _q1, float _q2, float _q3);	// q1/q2/q3 ➡ dq/ddq
PRIVATE U08 _Pos_CalMoveDirection(float _startAngle, float _goalAngle, float _avoidAngle);


/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
PRIVATE void _TP_CalJointRefData(U08 _arm, float _q1, float _q2, float _q3)	// Update myRobotTrajectory[_arm].Joint
{
	if ((LEFT_ARM != _arm) && (RIGHT_ARM != _arm))
	{
		return;
	}

	static float diffTime = PERIOD_TRAJECTORY_PLANNING;	// Fix time
	float newPos[3] = {_q1, _q2, _q3};

	// Process each of the three joints
	for (int i = 0; i < 3; i++)
	{
		if (FALSE == myRobotTrajectory[_arm].Ctrl.isInit)
		{	// To avoid high velocity and accelaration at first call
			myRobotTrajectory[_arm].Joint[i].currPos = newPos[i];
			myRobotTrajectory[_arm].Joint[i].prevPos = newPos[i];
			myRobotTrajectory[_arm].Joint[i].currVel = 0.0f;
			myRobotTrajectory[_arm].Joint[i].prevVel = 0.0f;
			myRobotTrajectory[_arm].Joint[i].currAccel = 0.0f;

			if (2 == i)
			{
				myRobotTrajectory[_arm].Ctrl.isInit = TRUE;
			}

			if ((TRUE == myRobotTrajectory[LEFT_ARM].Ctrl.isInit) && (TRUE == myRobotTrajectory[RIGHT_ARM].Ctrl.isInit))
			{
				AppDataSet_TPCalculated(TRUE);
			}
		}
		else
		{
			// Save current values as previous values
			myRobotTrajectory[_arm].Joint[i].prevPos = myRobotTrajectory[_arm].Joint[i].currPos;
			myRobotTrajectory[_arm].Joint[i].prevVel = myRobotTrajectory[_arm].Joint[i].currVel;

			// Get new Position, Vel, and Accel
			myRobotTrajectory[_arm].Joint[i].currPos   = newPos[i];
			myRobotTrajectory[_arm].Joint[i].currVel   =(newPos[i] - myRobotTrajectory[_arm].Joint[i].prevPos) / diffTime;
			myRobotTrajectory[_arm].Joint[i].currAccel = (myRobotTrajectory[_arm].Joint[i].currVel - myRobotTrajectory[_arm].Joint[i].prevVel) / diffTime;

		}
	}

	return;
}

PRIVATE U08 _Pos_CalMoveDirection(float _startAngle, float _goalAngle, float _avoidAngle)
{
	// Normalize all angles to [0, 360) range
	_goalAngle  = fmodf(_goalAngle,  360.0f);
	_startAngle = fmodf(_startAngle, 360.0f);
	_avoidAngle = fmodf(_avoidAngle, 360.0f);
	if (_goalAngle < 0)  _goalAngle  += 360.0f;
	if (_startAngle < 0) _startAngle += 360.0f;
	if (_avoidAngle < 0) _avoidAngle += 360.0f;

	// Check direction
	if (_startAngle < _goalAngle)
	{
		if ((_avoidAngle < _goalAngle) && (_avoidAngle > _startAngle))
		{
			return JOINT_DIR_Z_NEG;
		}
		else
		{
			return JOINT_DIR_Z_POS;
		}
	}
	else if (_startAngle > _goalAngle)
	{
		if ((_avoidAngle < _startAngle) && (_avoidAngle > _goalAngle))
		{
			return JOINT_DIR_Z_POS;
		}
		else
		{
			return JOINT_DIR_Z_NEG;
		}
	}
	else
	{
		// _startAngle == _goalAngle
	}

	return 0;
}


/********************************************************************************
 * GLOBAL FUNCTION IMPLEMENTATION
 ********************************************************************************/
GLOBAL BOOL AppControl_TP_SineWaveJoint(U08 _arm, float _timeStep)
{	// 0. Safety check
	if ((LEFT_ARM != _arm) && (RIGHT_ARM != _arm))
	{
		return FALSE;
	}

	// 1. Check if total TP end?
	if (TRUE == _trajectoryState[_arm][2])
	{	// Finish TP SineWave for all joints
		return FALSE;
	}

	// 2. Update time
	_totalTime[_arm] = _totalTime[_arm] + _timeStep;

	// 3. Check if current joint TP end?
	if (_totalTime[_arm] >= mySineTrajectory[_arm][_currJoint[_arm]].MoveTime)
	{
		_trajectoryState[_arm][_currJoint[_arm]] = TRUE;
		_totalTime[_arm] = 0.0f;

		if (_currJoint[_arm] < 2)
		{
			_currJoint[_arm]++;	// TP for next joint
		}
		else
		{	// Finish TP SineWave for all joints
			return FALSE;
		}
	}

	// 4. Caclulate sinwave for current joint
	if (FALSE == _trajectoryState[_arm][_currJoint[_arm]])
	{
		float t = _totalTime[_arm];
		float Amp  = mySineTrajectory[_arm][_currJoint[_arm]].Amp;
		float Freq = mySineTrajectory[_arm][_currJoint[_arm]].Freq;
		float Bias = mySineTrajectory[_arm][_currJoint[_arm]].Bias;
		float Phase= DEG2RAD(mySineTrajectory[_arm][_currJoint[_arm]].Phase);

		// Position: p(t) = A * sin(2πft + φ) + Bias
		// Velocity: v(t) = A * 2πf * cos(2πft + φ)
		float rotateAngle = Amp*sinf(2*PI*Freq*t + Phase) + Bias;
		float rotateSpeed = Amp*2*PI*Freq*cosf(2*PI*Freq*t + Phase);
		U08   rotateDir   = (rotateSpeed > 0) ? JOINT_DIR_Z_POS : JOINT_DIR_Z_NEG;
		rotateSpeed = fabsf(rotateSpeed);

		switch (_currJoint[_arm])
		{
		case 0:
			myRobotCommand[_arm].JointPos[0].Angle = rotateAngle;
			myRobotCommand[_arm].JointPos[0].Speed = (U16)fminf(rotateSpeed, 90.0)+1;
//			myRobotCommand[_arm].JointPos[0].Direction = _Pos_CalMoveDirection(CURR_POS_J11, rotateAngle, SINGULAR_J11);
			myRobotCommand[_arm].JointPos[0].Direction = rotateDir;
			myRobotCommand[_arm].JointPos[1].Speed = 0;
			myRobotCommand[_arm].JointPos[2].Speed = 0;
			break;

		case 1:
			myRobotCommand[_arm].JointPos[1].Angle = rotateAngle;
			myRobotCommand[_arm].JointPos[1].Speed = (U16)fminf(rotateSpeed, 90.0)+1;
//			myRobotCommand[_arm].JointPos[1].Direction = _Pos_CalMoveDirection(CURR_POS_J21, rotateAngle, SINGULAR_J21);
			myRobotCommand[_arm].JointPos[1].Direction = rotateDir;
			myRobotCommand[_arm].JointPos[0].Speed = 0;
			myRobotCommand[_arm].JointPos[2].Speed = 0;
			break;

		case 2:
			myRobotCommand[_arm].JointPos[2].Angle = rotateAngle;
			myRobotCommand[_arm].JointPos[2].Speed = (U16)fminf(rotateSpeed, 90.0)+1;
//			myRobotCommand[_arm].JointPos[2].Direction = _Pos_CalMoveDirection(CURR_POS_J31, rotateAngle, SINGULAR_J31);
			myRobotCommand[_arm].JointPos[2].Direction = rotateDir;
			myRobotCommand[_arm].JointPos[0].Speed = 0;
			myRobotCommand[_arm].JointPos[1].Speed = 0;
			break;

		default:
			return FALSE;
		}
	}

	return TRUE;	// Ok
}

GLOBAL BOOL AppControl_TP_JointTrajectoryInit(enTpType _type)
{
	U08 isSupported = FALSE;

	switch (_type)
	{
	case TP_TYPE_JOINT_SINEWAVE:
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[0].Setting.Amp      = 30.0f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[0].Setting.Freq     =  0.1f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[0].Setting.Phase    =  0.0f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[0].Setting.Bias     = 90.0f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[0].Ctrl.MovedTime   =  0.0f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[0].Ctrl.TimeStart   =  0.0f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[0].Ctrl.TimeEnd     = 30.0f;

		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[1].Setting.Amp      = 30.0f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[1].Setting.Freq     =  0.1f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[1].Setting.Phase    =  0.0f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[1].Setting.Bias     =  0.0f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[1].Ctrl.MovedTime   =  0.0f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[1].Ctrl.TimeStart   =  0.0f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[1].Ctrl.TimeEnd     = 30.0f;

		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[2].Setting.Amp      = 30.0f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[2].Setting.Freq     =  0.1f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[2].Setting.Phase    =  0.0f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[2].Setting.Bias     =  0.0f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[2].Ctrl.MovedTime   =  0.0f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[2].Ctrl.TimeStart   =  0.0f;
		myTrajectory.JointSpace.SineWave[LEFT_ARM].Joint[2].Ctrl.TimeEnd     = 30.0f;


		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[0].Setting.Amp      = 30.0f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[0].Setting.Freq     =  0.1f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[0].Setting.Phase    =180.0f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[0].Setting.Bias     = 90.0f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[0].Ctrl.MovedTime   =  0.0f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[0].Ctrl.TimeStart   =  0.0f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[0].Ctrl.TimeEnd     = 30.0f;

		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[1].Setting.Amp      = 30.0f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[1].Setting.Freq     =  0.1f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[1].Setting.Phase    =180.0f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[1].Setting.Bias     =  0.0f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[1].Ctrl.MovedTime   =  0.0f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[1].Ctrl.TimeStart   =  0.0f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[1].Ctrl.TimeEnd     = 30.0f;

		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[2].Setting.Amp      = 30.0f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[2].Setting.Freq     =  0.1f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[2].Setting.Phase    =180.0f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[2].Setting.Bias     =  0.0f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[2].Ctrl.MovedTime   =  0.0f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[2].Ctrl.TimeStart   =  0.0f;
		myTrajectory.JointSpace.SineWave[RIGHT_ARM].Joint[2].Ctrl.TimeEnd     = 30.0f;

		myTrajectory.Type = TP_TYPE_JOINT_SINEWAVE;
		isSupported = TRUE;
		break;

	default:
		isSupported = FALSE;
		break;
	}

	return isSupported;
}

GLOBAL BOOL AppControl_TP_JointTrajectoryUpdate(float _timeStep)
{
	static BOOL isMoving = FALSE;
	static enTpType _type = TP_TYPE_NONE;
	static float Amp_i, Freq_i, Phase_i, Bias_i, Ts_i, Te_i, Tm_i;
	static float T_global = 0.0f;
	static float Pos[3];

	U08 arm_idx, joint_idx;

	// 1. Check if first time called
	if (TP_TYPE_NONE == _type)
	{
		_type = myTrajectory.Type;	// Get type, remember to use InitWorldTrajectory first

		switch (_type)
		{
		case TP_TYPE_JOINT_SINEWAVE:
			_timeStep = 0.0f;	// Start at 0.0s
			isMoving = TRUE;
			break;
		default:
			isMoving = FALSE;
			return isMoving;	// Exit
		}
	}

	T_global = T_global + _timeStep;

	for (arm_idx = 0 ; arm_idx < DUAL_ARM ; arm_idx++)
	{
		for (joint_idx = 0 ; joint_idx < JOINTS_PER_ARM ; joint_idx++)
		{
			Amp_i  = myTrajectory.JointSpace.SineWave[arm_idx].Joint[joint_idx].Setting.Amp*D2R;	// [Rad]
			Freq_i = myTrajectory.JointSpace.SineWave[arm_idx].Joint[joint_idx].Setting.Freq;
			Phase_i= myTrajectory.JointSpace.SineWave[arm_idx].Joint[joint_idx].Setting.Phase*D2R;	// [Rad]
			Bias_i = myTrajectory.JointSpace.SineWave[arm_idx].Joint[joint_idx].Setting.Bias*D2R;	// [Rad]
			Ts_i   = myTrajectory.JointSpace.SineWave[arm_idx].Joint[joint_idx].Ctrl.TimeStart;
			Te_i   = myTrajectory.JointSpace.SineWave[arm_idx].Joint[joint_idx].Ctrl.TimeEnd;
			Tm_i   = myTrajectory.JointSpace.SineWave[arm_idx].Joint[joint_idx].Ctrl.MovedTime;

			// Calculate moved time
			if (T_global < Ts_i)		// Before start time
			{
				Tm_i = 0.0f;
			}
			else if ((Ts_i <= T_global) && (T_global <= Te_i))	// During move time
			{
				Tm_i = Tm_i + _timeStep;
			}
			else	// After end time
			{
				Tm_i = Tm_i;	// Keep it
			}

			Pos[joint_idx] = Amp_i*sinf(2*PI*Freq_i*Tm_i + Phase_i) + Bias_i;					// [Rad]
			myTrajectory.JointSpace.CurrentPos[arm_idx].Joint[joint_idx].Pos = Pos[joint_idx];	// [Rad]
			myTrajectory.JointSpace.SineWave[arm_idx].Joint[joint_idx].Ctrl.MovedTime = Tm_i;
		}
		_TP_CalJointRefData(arm_idx, Pos[0], Pos[1], Pos[2]);
	}


	return isMoving;
}

GLOBAL BOOL AppControl_TP_TaskTrajectoryInit(enTpType _type)
{
	U08 isSupported = FALSE;

	switch (_type)
	{
	case TP_TYPE_TASK_CIRCLE:
		myTrajectory.TaskSpace.Circle.Setting.X_Org  = 0.0f;		// [m]
		myTrajectory.TaskSpace.Circle.Setting.Y_Org  = 0.35f;		// [m]
		myTrajectory.TaskSpace.Circle.Setting.Radius = 0.05f;		// [m]
		myTrajectory.TaskSpace.Circle.Setting.Freq   = 0.1f;		// [Hz]
		myTrajectory.TaskSpace.Circle.Setting.Phase  = 0.0f;		// [Deg]
		myTrajectory.TaskSpace.Circle.Setting.Gamma  = 0.0f;		// [Deg]
		myTrajectory.TaskSpace.Circle.Ctrl.TimeEnd   = 10.0f;		// [s]
		myTrajectory.Type = TP_TYPE_TASK_CIRCLE;

		isSupported = TRUE;
		break;
	case TP_TYPE_TASK_LINE:
		myTrajectory.TaskSpace.Line.Setting.Start_X = 0.0f;
		myTrajectory.TaskSpace.Line.Setting.Start_Y = 0.3f;
		myTrajectory.TaskSpace.Line.Setting.Start_G = 0.0f;
		myTrajectory.TaskSpace.Line.Setting.Goal_X = 0.0f;
		myTrajectory.TaskSpace.Line.Setting.Goal_Y = 0.4f;
		myTrajectory.TaskSpace.Line.Setting.Goal_G = 0.0f;
		myTrajectory.TaskSpace.Line.Ctrl.TimeEnd   = 5.0f;
		myTrajectory.Type = TP_TYPE_TASK_LINE;

		isSupported = TRUE;
		break;

	case TP_TYPE_TASK_LINES:
	case TP_TYPE_TASK_CUBIC:
	case TP_TYPE_TASK_QUINTIC:
	case TP_TYPE_NONE:
	default:
		isSupported = FALSE;
		break;
	}

	return isSupported;
}

GLOBAL BOOL AppControl_TP_TaskTrajectoryUpdate(float _timeStep)
{
	static BOOL isMoving = FALSE;
	static enTpType _type = TP_TYPE_NONE;

	static float Xo, Yo, Ro, Go, Freq, Phase;			// Cicle
	static float Xs, Ys, Gs, Xg, Yg, Gg, timePercent;	// Line

	static float _endTime;
	static float _movedTime = 0.0f;

	// 1. Check if first time called
	if (TP_TYPE_NONE == _type)
	{
		_type = myTrajectory.Type;	// Get type, remember to use InitWorldTrajectory first

		switch (_type)
		{
		case TP_TYPE_TASK_CIRCLE:
			Xo = myTrajectory.TaskSpace.Circle.Setting.X_Org;
			Yo = myTrajectory.TaskSpace.Circle.Setting.Y_Org;
			Ro = myTrajectory.TaskSpace.Circle.Setting.Radius;
			Freq  = myTrajectory.TaskSpace.Circle.Setting.Freq;
			Go    = DEG2RAD(myTrajectory.TaskSpace.Circle.Setting.Gamma);
			Phase = DEG2RAD(myTrajectory.TaskSpace.Circle.Setting.Phase);
			_endTime = myTrajectory.TaskSpace.Circle.Ctrl.TimeEnd;
			break;
		case TP_TYPE_TASK_LINE:
			Xs = myTrajectory.TaskSpace.Line.Setting.Start_X;
			Ys = myTrajectory.TaskSpace.Line.Setting.Start_Y;
			Xg = myTrajectory.TaskSpace.Line.Setting.Goal_X;
			Yg = myTrajectory.TaskSpace.Line.Setting.Goal_Y;
			Gs = DEG2RAD(myTrajectory.TaskSpace.Line.Setting.Start_G);
			Gg = DEG2RAD(myTrajectory.TaskSpace.Line.Setting.Goal_G);
			_endTime = myTrajectory.TaskSpace.Line.Ctrl.TimeEnd;
			break;
		case TP_TYPE_TASK_LINES:
		case TP_TYPE_TASK_CUBIC:
		case TP_TYPE_TASK_QUINTIC:
		case TP_TYPE_NONE:
		default:
			isMoving = FALSE;
			return isMoving;	// Exit
		}

		_timeStep = 0.0f;	// Start from 0.00s
		isMoving = TRUE;
	}

	// 2. Check end TP or still continue
	if (_movedTime >= _endTime)
	{	// Finished
		isMoving = FALSE;
	}
	else
	{
		_movedTime  = _movedTime + _timeStep;

		// 3. Update trajectory
		switch (_type)
		{
		case TP_TYPE_TASK_CIRCLE:
			myTrajectory.TaskSpace.CurrentPos.Xm_t = Ro*sinf(2*PI*Freq*_movedTime + Phase) + Xo;	// xm(t) = Ro*cos(2πft + φ) + Xo
			myTrajectory.TaskSpace.CurrentPos.Ym_t = Ro*cosf(2*PI*Freq*_movedTime + Phase) + Yo;	// ym(t) = Ro*sin(2πft + φ) + Yo
			myTrajectory.TaskSpace.CurrentPos.Gm_t = Go;	// Currently, keep it constant [Rad]
			break;
		case TP_TYPE_TASK_LINE:
			timePercent = _movedTime / myTrajectory.TaskSpace.Line.Ctrl.TimeEnd;
			myTrajectory.TaskSpace.CurrentPos.Xm_t = Xs + timePercent*(Xg - Xs);
			myTrajectory.TaskSpace.CurrentPos.Ym_t = Ys + timePercent*(Yg - Ys);
			myTrajectory.TaskSpace.CurrentPos.Gm_t = Gs + timePercent*(Gg - Gs);
			break;
		case TP_TYPE_TASK_LINES:
		case TP_TYPE_TASK_CUBIC:
		case TP_TYPE_TASK_QUINTIC:
		default:

			break;;
		}
	}

	return isMoving;
}

GLOBAL void AppControl_IK_World2EE(U08 _arm)
{
	if ((LEFT_ARM != _arm) && (RIGHT_ARM != _arm))
	{
		return;
	}

	float Xm = myTrajectory.TaskSpace.CurrentPos.Xm_t;
	float Ym = myTrajectory.TaskSpace.CurrentPos.Ym_t;
	float Gm = myTrajectory.TaskSpace.CurrentPos.Gm_t;	// [Rad]

	switch (_arm)
	{
	case LEFT_ARM:
		myRobotTrajectory[LEFT_ARM].EndEffector.X_t = Xm + BASE_PARAM_C - OBJ_HALF_WIDTH_A*cosf(Gm);
		myRobotTrajectory[LEFT_ARM].EndEffector.Y_t = Ym - BASE_PARAM_D - OBJ_HALF_WIDTH_A*sinf(Gm);
		myRobotTrajectory[LEFT_ARM].EndEffector.G_t = Gm;
		break;
	case RIGHT_ARM:
		myRobotTrajectory[RIGHT_ARM].EndEffector.X_t = -Xm + BASE_PARAM_C - OBJ_HALF_WIDTH_A*cosf(Gm);
		myRobotTrajectory[RIGHT_ARM].EndEffector.Y_t =  Ym - BASE_PARAM_D + OBJ_HALF_WIDTH_A*sinf(Gm);
		myRobotTrajectory[RIGHT_ARM].EndEffector.G_t = -Gm;
		break;

	default:
		return;
	}

	return;
}

GLOBAL void AppControl_IK_EE2Joints(U08 _arm)
{
	static BOOL invalidTrajectory = FALSE;

	if ((LEFT_ARM != _arm) && (RIGHT_ARM != _arm))
	{
		return;
	}

	if (TRUE == invalidTrajectory)
	{
		return;
	}

	float x4, y4, g4;	// input	[m] & [Rad]
	float q1, q2, q3;	// output	[Rad]
	float x3, y3, c2, s2, c1, s1, temp1, temp2, temp3;	// process


	/* Get end-effector data */
	x4 = myRobotTrajectory[_arm].EndEffector.X_t;
	y4 = myRobotTrajectory[_arm].EndEffector.Y_t;
	g4 = myRobotTrajectory[_arm].EndEffector.G_t;	// [Rad]

	/* Calculate x3 y3 */
	x3 = x4 - L3*cosf(g4);
	y3 = y4 - L3*sinf(g4);

	/* Calculate q2 */
	c2 = (x3*x3 + y3*y3 - L1*L1 - L2*L2) / (2.0*L1*L2);
	s2 = -sqrtf(1.0f - c2*c2);	// q2 < 0
//	s2 = sqrt(1.0 - c2*c2);		// q2 > 0
	if (TRUE == isnan(s2))
	{	// Stop continue TP
		invalidTrajectory = TRUE;
		AppDataSet_TPCalculated(FALSE);
//		AppDataSet_MasterState(MASTER_STATE_ERROR);	// TODO Create error state
		return;
	}
	q2 = atan2f(s2, c2);

	/* Calculate q1 */
	temp1 = L1 + L2*cosf(q2);
	temp2 =      L2*sinf(q2);
	temp3 = temp1*temp1 + temp2*temp2;
	c1 = (x3*temp1 + y3*temp2) / temp3;
	s1 = (y3*temp1 - x3*temp2) / temp3;
	q1 = atan2f(s1,c1);

	/* Calculate q3 */
	q3 = g4 - q2 - q1;

	/* Update joints' referene data - dq, ddq */
	_TP_CalJointRefData(_arm, q1, q2, q3);

	return;
}


GLOBAL void AppControl_Pos_TestSequence(U16 _speed)
{
	// 1. Test Speed constraints
	if (_speed > TEST_SPEED_MAX)	// For safety
	{
		_speed = TEST_SPEED_MAX;
	}

	switch (_btnSequenceTest)
	{
	case 0:	// back home using auto check dir function
		myRobotCommand[LEFT_ARM].JointPos[0].Angle = 90.0;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = _Pos_CalMoveDirection(CURR_POS_J11, 90.0, SINGULAR_J11);
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = _Pos_CalMoveDirection(CURR_POS_J21, 0.0, SINGULAR_J21);
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = _Pos_CalMoveDirection(CURR_POS_J31, 0.0, SINGULAR_J31);

		myRobotCommand[RIGHT_ARM].JointPos[0].Angle = 90.0;
		myRobotCommand[RIGHT_ARM].JointPos[0].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[0].Direction = _Pos_CalMoveDirection(CURR_POS_J12, 90.0, SINGULAR_J12);
		myRobotCommand[RIGHT_ARM].JointPos[1].Angle = 0.0;
		myRobotCommand[RIGHT_ARM].JointPos[1].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[1].Direction = _Pos_CalMoveDirection(CURR_POS_J22, 0.0, SINGULAR_J22);
		myRobotCommand[RIGHT_ARM].JointPos[2].Angle = 0.0;
		myRobotCommand[RIGHT_ARM].JointPos[2].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[2].Direction = _Pos_CalMoveDirection(CURR_POS_J32, 0.0, SINGULAR_J32);

		_btnSequenceTest = 1;
		break;

	case 1:
		myRobotCommand[LEFT_ARM].JointPos[0].Angle = 180.0;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = _Pos_CalMoveDirection(CURR_POS_J11, 180.0, SINGULAR_J11);
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = -90.0;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = _Pos_CalMoveDirection(CURR_POS_J21, -90.0, SINGULAR_J21);
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = -90.0;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = _Pos_CalMoveDirection(CURR_POS_J31, -90.0, SINGULAR_J31);

		myRobotCommand[RIGHT_ARM].JointPos[0].Angle = 180.0;
		myRobotCommand[RIGHT_ARM].JointPos[0].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[0].Direction = _Pos_CalMoveDirection(CURR_POS_J12, 180.0, SINGULAR_J12);
		myRobotCommand[RIGHT_ARM].JointPos[1].Angle = -90.0;
		myRobotCommand[RIGHT_ARM].JointPos[1].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[1].Direction = _Pos_CalMoveDirection(CURR_POS_J22, -90.0, SINGULAR_J22);
		myRobotCommand[RIGHT_ARM].JointPos[2].Angle = -90.0;
		myRobotCommand[RIGHT_ARM].JointPos[2].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[2].Direction = _Pos_CalMoveDirection(CURR_POS_J32, -90.0, SINGULAR_J32);

		_btnSequenceTest = 2;
		break;

	case 2:
		myRobotCommand[LEFT_ARM].JointPos[0].Angle = 90.0;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = _Pos_CalMoveDirection(CURR_POS_J11, 90.0, SINGULAR_J11);
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = _Pos_CalMoveDirection(CURR_POS_J21, 0.0, SINGULAR_J21);
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = _Pos_CalMoveDirection(CURR_POS_J31, 0.0, SINGULAR_J31);

		myRobotCommand[RIGHT_ARM].JointPos[0].Angle = 90.0;
		myRobotCommand[RIGHT_ARM].JointPos[0].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[0].Direction = _Pos_CalMoveDirection(CURR_POS_J12, 90.0, SINGULAR_J12);
		myRobotCommand[RIGHT_ARM].JointPos[1].Angle = 0.0;
		myRobotCommand[RIGHT_ARM].JointPos[1].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[1].Direction = _Pos_CalMoveDirection(CURR_POS_J22, 0.0, SINGULAR_J22);
		myRobotCommand[RIGHT_ARM].JointPos[2].Angle = 0.0;
		myRobotCommand[RIGHT_ARM].JointPos[2].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[2].Direction = _Pos_CalMoveDirection(CURR_POS_J32, 0.0, SINGULAR_J32);

		_btnSequenceTest = 1;
		break;
	default:
		_btnSequenceTest = 1;
		break;
	}

	return;
}

GLOBAL void AppControl_Pos_MoveToHome(U08 _arm, U16 _speed)
{
	if (_speed > HOME_SPEED_MAX)
	{
		_speed = HOME_SPEED_MAX;
	}

	if (LEFT_ARM == _arm)
	{
		myRobotCommand[LEFT_ARM].JointPos[0].Angle = HOME_POS_J11;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = _Pos_CalMoveDirection(CURR_POS_J11, HOME_POS_J11, SINGULAR_J11);
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = HOME_POS_J21;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = _Pos_CalMoveDirection(CURR_POS_J21, HOME_POS_J21, SINGULAR_J21);
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = HOME_POS_J31;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = _Pos_CalMoveDirection(CURR_POS_J31, HOME_POS_J31, SINGULAR_J31);
	}
	else if (RIGHT_ARM == _arm)
	{
		myRobotCommand[RIGHT_ARM].JointPos[0].Angle = HOME_POS_J12;
		myRobotCommand[RIGHT_ARM].JointPos[0].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[0].Direction = _Pos_CalMoveDirection(CURR_POS_J12, HOME_POS_J12, SINGULAR_J12);
		myRobotCommand[RIGHT_ARM].JointPos[1].Angle = HOME_POS_J22;
		myRobotCommand[RIGHT_ARM].JointPos[1].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[1].Direction = _Pos_CalMoveDirection(CURR_POS_J22, HOME_POS_J22, SINGULAR_J22);
		myRobotCommand[RIGHT_ARM].JointPos[2].Angle = HOME_POS_J32;
		myRobotCommand[RIGHT_ARM].JointPos[2].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[2].Direction = _Pos_CalMoveDirection(CURR_POS_J32, HOME_POS_J32, SINGULAR_J32);
	}

	return;
}

GLOBAL void AppControl_Pos_MoveToTpStart(U08 _arm, U16 _speed)	// Trajectory -> Command
{
	static float q1, q2, q3;

	if (_speed > TP_START_SPEED_MAX)
	{
		_speed = TP_START_SPEED_MAX;
	}

	if (LEFT_ARM == _arm)
	{
		q1 = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[0].currPos);
		q2 = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[1].currPos);
		q3 = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[2].currPos);

		myRobotCommand[LEFT_ARM].JointPos[0].Angle = q1;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = _Pos_CalMoveDirection(CURR_POS_J11, q1, SINGULAR_J11);
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = q2;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = _Pos_CalMoveDirection(CURR_POS_J21, q2, SINGULAR_J21);
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = q3;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = _Pos_CalMoveDirection(CURR_POS_J31, q3, SINGULAR_J31);
	}
	else if (RIGHT_ARM == _arm)
	{
		q1 = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[0].currPos);
		q2 = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[1].currPos);
		q3 = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[2].currPos);

		myRobotCommand[RIGHT_ARM].JointPos[0].Angle = q1;
		myRobotCommand[RIGHT_ARM].JointPos[0].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[0].Direction = _Pos_CalMoveDirection(CURR_POS_J12, q1, SINGULAR_J12);
		myRobotCommand[RIGHT_ARM].JointPos[1].Angle = q2;
		myRobotCommand[RIGHT_ARM].JointPos[1].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[1].Direction = _Pos_CalMoveDirection(CURR_POS_J22, q2, SINGULAR_J22);
		myRobotCommand[RIGHT_ARM].JointPos[2].Angle = q3;
		myRobotCommand[RIGHT_ARM].JointPos[2].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[2].Direction = _Pos_CalMoveDirection(CURR_POS_J32, q3, SINGULAR_J32);
	}

	return;
}

GLOBAL void AppControl_Pos_FollowTpPos(U08 _arm)
{
	static float q1, q2, q3, v1, v2, v3;
	static U08 d1, d2, d3;

	if (LEFT_ARM == _arm)
	{
		q1 = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[0].currPos);
		q2 = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[1].currPos);
		q3 = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[2].currPos);
		v1 = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[0].currVel);
		v2 = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[1].currVel);
		v3 = RAD2DEG(myRobotTrajectory[LEFT_ARM].Joint[2].currVel);
		d1 = (v1 > 0) ? JOINT_DIR_Z_POS : JOINT_DIR_Z_NEG;
		d2 = (v2 > 0) ? JOINT_DIR_Z_POS : JOINT_DIR_Z_NEG;
		d3 = (v3 > 0) ? JOINT_DIR_Z_POS : JOINT_DIR_Z_NEG;

		myRobotCommand[LEFT_ARM].JointPos[0].Angle = q1;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = (U16)(fabsf(v1)+1);
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = d1;
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = q2;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = (U16)(fabsf(v2)+1);
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = d2;
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = q3;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = (U16)(fabsf(v3)+1);
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = d3;
	}
	else if (RIGHT_ARM == _arm)
	{
		q1 = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[0].currPos);
		q2 = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[1].currPos);
		q3 = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[2].currPos);
		v1 = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[0].currVel);
		v2 = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[1].currVel);
		v3 = RAD2DEG(myRobotTrajectory[RIGHT_ARM].Joint[2].currVel);
		d1 = (v1 > 0) ? JOINT_DIR_Z_POS : JOINT_DIR_Z_NEG;
		d2 = (v2 > 0) ? JOINT_DIR_Z_POS : JOINT_DIR_Z_NEG;
		d3 = (v3 > 0) ? JOINT_DIR_Z_POS : JOINT_DIR_Z_NEG;

		myRobotCommand[RIGHT_ARM].JointPos[0].Angle = q1;
		myRobotCommand[RIGHT_ARM].JointPos[0].Speed = (U16)(fabsf(v1)+1);
		myRobotCommand[RIGHT_ARM].JointPos[0].Direction = d1;
		myRobotCommand[RIGHT_ARM].JointPos[1].Angle = q2;
		myRobotCommand[RIGHT_ARM].JointPos[1].Speed = (U16)(fabsf(v2)+1);
		myRobotCommand[RIGHT_ARM].JointPos[1].Direction = d2;
		myRobotCommand[RIGHT_ARM].JointPos[2].Angle = q3;
		myRobotCommand[RIGHT_ARM].JointPos[2].Speed = (U16)(fabsf(v3)+1);
		myRobotCommand[RIGHT_ARM].JointPos[2].Direction = d3;
	}

	return;
}

GLOBAL void AppControl_Tor_TestSequence(U08 _arm, U08 _joint)
{
	// 1. Safety check
	if (
	(LEFT_ARM != _arm) && (RIGHT_ARM != _arm) && \
	(0 != _joint) && (1 != _joint) && (2 !=_joint))
	{
		return;
	}

	switch (_btnSequenceTest)
	{
	case 0:
		myRobotCommand[_arm].JointTor[_joint].Tor = 0.1f;
		AppDataSet_LedState(LED_3_ORANGE, TRUE);
		AppDataSet_LedState(LED_4_GREEN, FALSE);
		AppDataSet_LedState(LED_6_BLUE, FALSE);
		_btnSequenceTest = 1;
		break;
	case 1:
		myRobotCommand[_arm].JointTor[_joint].Tor = 0.2f;
		AppDataSet_LedState(LED_3_ORANGE, FALSE);
		AppDataSet_LedState(LED_4_GREEN, TRUE);
		AppDataSet_LedState(LED_6_BLUE, FALSE);
		_btnSequenceTest = 2;
		break;
	case 2:
		myRobotCommand[_arm].JointTor[_joint].Tor = 0.3f;
		AppDataSet_LedState(LED_3_ORANGE, TRUE);
		AppDataSet_LedState(LED_4_GREEN, TRUE);
		AppDataSet_LedState(LED_6_BLUE, FALSE);
		_btnSequenceTest = 3;
		break;
	case 3:
		myRobotCommand[_arm].JointTor[_joint].Tor = 0.4f;
		AppDataSet_LedState(LED_3_ORANGE, FALSE);
		AppDataSet_LedState(LED_4_GREEN, FALSE);
		AppDataSet_LedState(LED_6_BLUE, TRUE);
		_btnSequenceTest = 4;
		break;
	case 4:
		myRobotCommand[_arm].JointTor[_joint].Tor = 0.5f;
		AppDataSet_LedState(LED_3_ORANGE, TRUE);
		AppDataSet_LedState(LED_4_GREEN, FALSE);
		AppDataSet_LedState(LED_6_BLUE, TRUE);
		_btnSequenceTest = 5;
		break;
	case 5:
		myRobotCommand[_arm].JointTor[_joint].Tor = 0.6f;
		AppDataSet_LedState(LED_3_ORANGE, FALSE);
		AppDataSet_LedState(LED_4_GREEN, TRUE);
		AppDataSet_LedState(LED_6_BLUE, TRUE);
		_btnSequenceTest = 6;
		break;
	case 6:
		myRobotCommand[_arm].JointTor[_joint].Tor = 0.7f;
		AppDataSet_LedState(LED_3_ORANGE, TRUE);
		AppDataSet_LedState(LED_4_GREEN, TRUE);
		AppDataSet_LedState(LED_6_BLUE, TRUE);
		_btnSequenceTest = 7;
		break;
	case 7:
		myRobotCommand[_arm].JointTor[_joint].Tor = 0.8f;
		AppDataSet_LedState(LED_3_ORANGE, FALSE);
		AppDataSet_LedState(LED_4_GREEN, FALSE);
		AppDataSet_LedState(LED_6_BLUE, FALSE);
		_btnSequenceTest = 0;
		break;
	default:
		_btnSequenceTest = 0;
		break;
	}

	return;
}

GLOBAL BOOL AppControl_Tor_ControllerInit(enTorController _type)
{
	U08 isSupported = FALSE;

	switch (_type)
	{
	case TOR_CTRL_PD:
		myControl.PD.Setting.Kp[0] = 22.0f;
		myControl.PD.Setting.Kp[1] = 15.0f;
		myControl.PD.Setting.Kp[2] = 6.0f;

		myControl.PD.Setting.Kd[0] = 0.4f;
		myControl.PD.Setting.Kd[1] = 0.15f;
		myControl.PD.Setting.Kd[2] = 0.01f;

		myControl.PD.Setting.Alpha[0] = 0.0f;
		myControl.PD.Setting.Alpha[1] = 0.0f;
		myControl.PD.Setting.Alpha[2] = 0.0f;
		myControl.Type = TOR_CTRL_PD;

		isSupported = TRUE;
		break;
	case TOR_CTRL_SPD:
		myControl.PD.Setting.Kp[0] = 22.0f;
		myControl.PD.Setting.Kp[1] = 15.0f;
		myControl.PD.Setting.Kp[2] = 6.0f;

		myControl.PD.Setting.Kd[0] = 0.4f;
		myControl.PD.Setting.Kd[1] = 0.15f;
		myControl.PD.Setting.Kd[2] = 0.01f;

		myControl.PD.Setting.Alpha[0] = 0.3f;
		myControl.PD.Setting.Alpha[1] = 0.3f;
		myControl.PD.Setting.Alpha[2] = 0.3f;
		myControl.Type = TOR_CTRL_SPD;

		isSupported = TRUE;
		break;
	case TOR_CTRL_SMC:
	case TOR_CTRL_SSMC:
	case TOR_CTRL_NONE:
	default:
		isSupported = FALSE;
		break;
	}

	return isSupported;
}

GLOBAL BOOL AppControl_Tor_ControlUpdateJoint(U08 _arm, U08 _joint)
{
	static enTorController _type = TOR_CTRL_NONE;

	// Params
	static float Kp_i, Kd_i;
	// Inputs
	static float q_r_i, dq_r_i, q_i, dq_i, e_i, de_i;
	// Outputs
	static float torque_i;

	// Get control params (1 time only)
	if (TOR_CTRL_NONE == _type)
	{
		_type = myControl.Type;

		switch (_type)
		{
		case TOR_CTRL_PD:
			Kp_i = myControl.PD.Setting.Kp[_joint];
			Kd_i = myControl.PD.Setting.Kd[_joint];
			break;
		case TOR_CTRL_SPD:
		case TOR_CTRL_SMC:
		case TOR_CTRL_SSMC:
		case TOR_CTRL_NONE:
		default:

			return FALSE;
		}
	}

	switch (_type)
	{
	case TOR_CTRL_PD:
		 q_r_i = myRobotTrajectory[_arm].Joint[_joint].currPos;
		dq_r_i = myRobotTrajectory[_arm].Joint[_joint].currVel;
		 q_i = DEG2RAD(myRobotFeedback[_arm].Joint[_joint].Position);
		dq_i = DEG2RAD(myRobotFeedback[_arm].Joint[_joint].Speed);

		 e_i =  q_r_i -  q_i;
		de_i = dq_r_i - dq_i;

		torque_i = Kp_i*e_i + Kd_i*de_i;

		myRobotCommand[_arm].JointTor[_joint].Tor = CONSTRAIN(torque_i, -CONTROL_MAX_TORQUE, CONTROL_MAX_TORQUE);
		break;
	case TOR_CTRL_SPD:
	case TOR_CTRL_SMC:
	case TOR_CTRL_SSMC:
	case TOR_CTRL_NONE:
	default:

		return FALSE;
	}

	return TRUE;
}

GLOBAL BOOL AppControl_Tor_ControlUpdateSingleArm(U08 _arm)
{
	static enTorController _type = TOR_CTRL_NONE;
	static float Kp[3], Kd[3];
	static float Err[3], dErr[3];
	static float Tor[3];
	static U08 _idx;

	if ((LEFT_ARM != _arm) && (RIGHT_ARM != _arm))
	{
		return FALSE;
	}

	// Get control params (1 time only)
	if (TOR_CTRL_NONE == _type)
	{
		_type = myControl.Type;

		switch (_type)
		{
		case TOR_CTRL_PD:
			for (_idx = 0 ; _idx < JOINTS_PER_ARM ; _idx++)
			{
				Kp[_idx] = myControl.PD.Setting.Kp[_idx];
				Kd[_idx] = myControl.PD.Setting.Kd[_idx];
			}
			break;
		case TOR_CTRL_SMC:
		default:
			// Not support
			return FALSE;
		}
	}

	// Calculate control signal
	switch (_type)
	{
	case TOR_CTRL_PD:
		for (_idx = 0 ; _idx < JOINTS_PER_ARM ; _idx++)
		{
			 Err[_idx] = myRobotTrajectory[_arm].Joint[_idx].currPos - DEG2RAD(myRobotFeedback[_arm].Joint[_idx].Position);
			dErr[_idx] = myRobotTrajectory[_arm].Joint[_idx].currVel - DEG2RAD(myRobotFeedback[_arm].Joint[_idx].Speed);

			Tor[_idx] = Kp[_idx]*Err[_idx] + Kd[_idx]*dErr[_idx];

			myRobotCommand[_arm].JointTor[_idx].Tor = CONSTRAIN(Tor[_idx], -CONTROL_MAX_TORQUE, CONTROL_MAX_TORQUE);
		}
		break;
	case TOR_CTRL_SMC:
	default:
		// Not support
		return FALSE;
	}

	return TRUE;
}

GLOBAL BOOL AppControl_Tor_ControlUpdateDualArm(U08 _arm)
{
	static enTorController _type = TOR_CTRL_NONE;
	static float Kp[3], Kd[3], Al[3];
	static float Err[6], dErr[6], ErrSync[6], dErrSync[6], ErrCC[6], dErrCC[6];
	static float Tor[6];
	static U08 _idx;

	if (DUAL_ARM != _arm)
	{
		return FALSE;
	}

	// Get control params (1 time only)
	if (TOR_CTRL_NONE == _type)
	{
		_type = myControl.Type;

		switch (_type)
		{
		case TOR_CTRL_SPD:
			for (_idx = 0 ; _idx < JOINTS_PER_ARM ; _idx++)
			{
				Kp[_idx] = myControl.PD.Setting.Kp[_idx];
				Kd[_idx] = myControl.PD.Setting.Kd[_idx];
				Al[_idx] = myControl.PD.Setting.Alpha[_idx];
			}
			break;
		case TOR_CTRL_SSMC:
		default:
			// Not support
			return FALSE;
		}
	}

	// Calculate control signal
	switch (_type)
	{
	case TOR_CTRL_SPD:
		// Calculate error
		Err[0] = myRobotTrajectory[LEFT_ARM].Joint[0].currPos - DEG2RAD(myRobotFeedback[LEFT_ARM].Joint[0].Position);
		Err[1] = myRobotTrajectory[LEFT_ARM].Joint[1].currPos - DEG2RAD(myRobotFeedback[LEFT_ARM].Joint[1].Position);
		Err[2] = myRobotTrajectory[LEFT_ARM].Joint[2].currPos - DEG2RAD(myRobotFeedback[LEFT_ARM].Joint[2].Position);
		Err[3] = myRobotTrajectory[RIGHT_ARM].Joint[0].currPos - DEG2RAD(myRobotFeedback[RIGHT_ARM].Joint[0].Position);
		Err[4] = myRobotTrajectory[RIGHT_ARM].Joint[1].currPos - DEG2RAD(myRobotFeedback[RIGHT_ARM].Joint[1].Position);
		Err[5] = myRobotTrajectory[RIGHT_ARM].Joint[2].currPos - DEG2RAD(myRobotFeedback[RIGHT_ARM].Joint[2].Position);

		dErr[0] = myRobotTrajectory[LEFT_ARM].Joint[0].currVel - DEG2RAD(myRobotFeedback[LEFT_ARM].Joint[0].Speed);
		dErr[1] = myRobotTrajectory[LEFT_ARM].Joint[1].currVel - DEG2RAD(myRobotFeedback[LEFT_ARM].Joint[1].Speed);
		dErr[2] = myRobotTrajectory[LEFT_ARM].Joint[2].currVel - DEG2RAD(myRobotFeedback[LEFT_ARM].Joint[2].Speed);
		dErr[3] = myRobotTrajectory[RIGHT_ARM].Joint[0].currVel - DEG2RAD(myRobotFeedback[RIGHT_ARM].Joint[0].Speed);
		dErr[4] = myRobotTrajectory[RIGHT_ARM].Joint[1].currVel - DEG2RAD(myRobotFeedback[RIGHT_ARM].Joint[1].Speed);
		dErr[5] = myRobotTrajectory[RIGHT_ARM].Joint[2].currVel - DEG2RAD(myRobotFeedback[RIGHT_ARM].Joint[2].Speed);

		// Calculate synschronous error
		ErrSync[0] = Err[0] - Err[3];
		ErrSync[1] = Err[1] - Err[4];
		ErrSync[2] = Err[2] - Err[5];
		ErrSync[3] = -ErrSync[0];
		ErrSync[4] = -ErrSync[1];
		ErrSync[5] = -ErrSync[2];

		dErrSync[0] = dErr[0] - dErr[3];
		dErrSync[1] = dErr[1] - dErr[4];
		dErrSync[2] = dErr[2] - dErr[5];
		dErrSync[3] = -dErrSync[0];
		dErrSync[4] = -dErrSync[1];
		dErrSync[5] = -dErrSync[2];

		// Calculate cross-coupling error
		ErrCC[0] = Err[0] + Al[0]*ErrSync[0];
		ErrCC[1] = Err[1] + Al[1]*ErrSync[1];
		ErrCC[2] = Err[2] + Al[2]*ErrSync[2];
		ErrCC[3] = Err[3] + Al[0]*ErrSync[3];
		ErrCC[4] = Err[4] + Al[1]*ErrSync[4];
		ErrCC[5] = Err[5] + Al[2]*ErrSync[5];

		dErrCC[0] = dErr[0] + Al[0]*dErrSync[0];
		dErrCC[1] = dErr[1] + Al[1]*dErrSync[1];
		dErrCC[2] = dErr[2] + Al[2]*dErrSync[2];
		dErrCC[3] = dErr[3] + Al[0]*dErrSync[3];
		dErrCC[4] = dErr[4] + Al[1]*dErrSync[4];
		dErrCC[5] = dErr[5] + Al[2]*dErrSync[5];

		// Calculate Torque
		Tor[0] = Kp[0]*ErrCC[0] + Kd[0]*dErrCC[0];
		Tor[1] = Kp[1]*ErrCC[1] + Kd[1]*dErrCC[1];
		Tor[2] = Kp[2]*ErrCC[2] + Kd[2]*dErrCC[2];
		Tor[3] = Kp[0]*ErrCC[3] + Kd[0]*dErrCC[3];
		Tor[4] = Kp[1]*ErrCC[4] + Kd[1]*dErrCC[4];
		Tor[5] = Kp[2]*ErrCC[5] + Kd[2]*dErrCC[5];

		myRobotCommand[LEFT_ARM].JointTor[0].Tor = CONSTRAIN(Tor[0], -CONTROL_MAX_TORQUE, CONTROL_MAX_TORQUE);
		myRobotCommand[LEFT_ARM].JointTor[1].Tor = CONSTRAIN(Tor[1], -CONTROL_MAX_TORQUE, CONTROL_MAX_TORQUE);
		myRobotCommand[LEFT_ARM].JointTor[2].Tor = CONSTRAIN(Tor[2], -CONTROL_MAX_TORQUE, CONTROL_MAX_TORQUE);
		myRobotCommand[RIGHT_ARM].JointTor[0].Tor = CONSTRAIN(Tor[3], -CONTROL_MAX_TORQUE, CONTROL_MAX_TORQUE);
		myRobotCommand[RIGHT_ARM].JointTor[1].Tor = CONSTRAIN(Tor[4], -CONTROL_MAX_TORQUE, CONTROL_MAX_TORQUE);
		myRobotCommand[RIGHT_ARM].JointTor[2].Tor = CONSTRAIN(Tor[5], -CONTROL_MAX_TORQUE, CONTROL_MAX_TORQUE);
		break;
	case TOR_CTRL_SSMC:
	default:
		// Not support
		return FALSE;
	}

	return TRUE;
}
