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
PRIVATE I32 Px = 0;
PRIVATE I32 Py = 0;
PRIVATE float Yaw = 0;

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
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
GLOBAL strTaskSpacePlanning  myTaskTrajectory = { 0, };
GLOBAL strJointSpacePlanning myRobotTrajectory[DUAL_ARM] = { 0, };


/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE U08 _Pos_CalMoveDirection(float _startAngle, float _goalAngle, float _avoidAngle);
PRIVATE void _TP_CalJointRefData(U08 _arm, float _q1, float _q2, float _q3);	// q1/q2/q3 ➡ dq/ddq


/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/
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

PRIVATE void _TP_CalJointRefData(U08 _arm, float _q1, float _q2, float _q3)
{
	if ((LEFT_ARM != _arm) && (RIGHT_ARM != _arm))
	{
		return;
	}

	static float diffTime = PERIOD_TRAJECTORY_PLANNING / 1000.0f;	// Fix time
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


/********************************************************************************
 * GLOBAL FUNCTION IMPLEMENTATION
 ********************************************************************************/
/**
* Input:
*  - Px: unit mm, type I32, Example: Px = 1000(mm)
*  - Py: unit mm, type I32, Example: Py = 500(mm)
*  - Yaw: unit 0.01°, type float, Example: Yaw = 2.5°
* Output:
*  - Θ: unit 0.01°, type float pointer, pointer allocation include J1, J2, and J3
 */
GLOBAL void AppControl_CalRobotIK(void)
{
	float x3, y3, c2, s2, c1, s1;
	float _q1, _q2;

	/* Calculating x3 y3 */
	x3 = (float)((float)Px - L3*(float)cos(Yaw * D2R));
	y3 = (float)((float)Py - L3*(float)sin(Yaw * D2R));

	/* Calculating Θ_2 */
	c2 = (float)((x3*x3 + y3*y3 - L1*L1 - L2*L2) / (2.0*L1*L2));
	s2 = -sqrt(1.0 - c2*c2);
//	s2 = sqrt(1.0 - c2*c2);

	_q2 = atan2(s2,c2);

	/* Calculating Θ_1 */
	c1 = (x3*(L1 + L2*cos(_q2)) +  y3*L2*sin(_q2)) \
		  / ((L1 + L2*cos(_q2))*(L1 + L2*cos(_q2)) + (L2*sin(_q2))*(L2*sin(_q2)));

	s1 = (y3*(L1 + L2*cos(_q2)) -  x3*L2*sin(_q2)) \
		  / ((L1 + L2*cos(_q2))*(L1 + L2*cos(_q2)) + (L2*sin(_q2))*(L2*sin(_q2)));

	_q1 = atan2(s1,c1);

	/* Calculating Θ_3 */
	myRobotCommand[LEFT_ARM].JointPos[0].Angle = (U32)RAD_TO_DEG001(_q1);
	myRobotCommand[LEFT_ARM].JointPos[1].Angle = (U32)RAD_TO_DEG001(_q2);
	myRobotCommand[LEFT_ARM].JointPos[2].Angle = (U32)RAD_TO_DEG001(Yaw - _q1 - _q2);
	return;
}


PRIVATE U08 _btnSequenceTest = 0;
GLOBAL void AppControl_Pos_TestSquence(U08 _arm, U16 _speed)
{	// 1. Safety check
	if ((LEFT_ARM != _arm) && (RIGHT_ARM != _arm))
	{
		return;
	}

	// 2. Test Speed constraints
	if (_speed > TEST_SPEED_MAX)	// For safety
	{
		_speed = TEST_SPEED_MAX;
	}

#if (0)	// Tested
	switch (_btnSequenceTest)
	{
	case 0:		// <90 | <0 | <0  -->  90 | 0 | 0
		myRobotCommand[LEFT_ARM].JointPos[0].Angle = 90.0;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = JOINT_DIR_Z_POS;
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = JOINT_DIR_Z_POS;
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = JOINT_DIR_Z_POS;

		myRobotCommand[RIGHT_ARM].JointPos[0].Angle = 90.0;
		myRobotCommand[RIGHT_ARM].JointPos[0].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[0].Direction = JOINT_DIR_Z_POS;
		myRobotCommand[RIGHT_ARM].JointPos[1].Angle = 0.0;
		myRobotCommand[RIGHT_ARM].JointPos[1].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[1].Direction = JOINT_DIR_Z_POS;
		myRobotCommand[RIGHT_ARM].JointPos[2].Angle = 0.0;
		myRobotCommand[RIGHT_ARM].JointPos[2].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[2].Direction = JOINT_DIR_Z_POS;

		_btnSequenceTest = 1;
		break;

	case 1:		// 90 | 0 | 0  -->  180 | -90 | -90
		myRobotCommand[LEFT_ARM].JointPos[0].Angle = 180.0;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = JOINT_DIR_Z_POS;
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = -90.0;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = JOINT_DIR_Z_NEG;
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = -90.0;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = JOINT_DIR_Z_NEG;

		myRobotCommand[RIGHT_ARM].JointPos[0].Angle = 180.0;
		myRobotCommand[RIGHT_ARM].JointPos[0].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[0].Direction = JOINT_DIR_Z_POS;
		myRobotCommand[RIGHT_ARM].JointPos[1].Angle = -90.0;
		myRobotCommand[RIGHT_ARM].JointPos[1].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[1].Direction = JOINT_DIR_Z_NEG;
		myRobotCommand[RIGHT_ARM].JointPos[2].Angle = -90.0;
		myRobotCommand[RIGHT_ARM].JointPos[2].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[2].Direction = JOINT_DIR_Z_NEG;

		_btnSequenceTest = 2;
		break;

	case 2:		// 180 | -90 | -90  -->  90 | 0 | 0
		myRobotCommand[LEFT_ARM].JointPos[0].Angle = 90.0;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = JOINT_DIR_Z_NEG;
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = JOINT_DIR_Z_POS;
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speed;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = JOINT_DIR_Z_POS;

		myRobotCommand[RIGHT_ARM].JointPos[0].Angle = 90.0;
		myRobotCommand[RIGHT_ARM].JointPos[0].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[0].Direction = JOINT_DIR_Z_NEG;
		myRobotCommand[RIGHT_ARM].JointPos[1].Angle = 0.0;
		myRobotCommand[RIGHT_ARM].JointPos[1].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[1].Direction = JOINT_DIR_Z_POS;
		myRobotCommand[RIGHT_ARM].JointPos[2].Angle = 0.0;
		myRobotCommand[RIGHT_ARM].JointPos[2].Speed = _speed;
		myRobotCommand[RIGHT_ARM].JointPos[2].Direction = JOINT_DIR_Z_POS;

		_btnSequenceTest = 1;
		break;

	default:
		_btnSequenceTest = 0;
		break;
	}
#else // (Tested)
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



#endif
	return;
}

GLOBAL void AppControl_Pos_BackToHome(U08 _arm, U16 _speed)
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
		rotateSpeed = fabs(rotateSpeed);

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

GLOBAL BOOL AppControl_TP_CircleTool(float _timeStep)
{
	static BOOL isInit = FALSE;
	static BOOL isMoving = FALSE;
	static float t, Xo, Yo, Ro, Freq, Phase;

	// 1. Init desired trajectory
	if (FALSE == isInit)
	{
		myTaskTrajectory.Circle.Setting.X_Org  = 0.0f;		// [m]
		myTaskTrajectory.Circle.Setting.Y_Org  = 0.35f;		// [m]
		myTaskTrajectory.Circle.Setting.Radius = 0.05f;		// [m]
		myTaskTrajectory.Circle.Setting.Freq   = 0.1f;		// [Hz]
		myTaskTrajectory.Circle.Setting.Phase  = 0.0f;		// [Deg]
		myTaskTrajectory.Circle.Setting.Gamma  = 0.0f;		// [Deg]

		myTaskTrajectory.Circle.Ctrl.MovedTime =  0.0f;		// [s]
		myTaskTrajectory.Circle.Ctrl.TimeEnd   = 10.0f;		// [s]
		_timeStep = 0.0f;	// Start from 0.0s

		isInit = TRUE;
		isMoving = TRUE;
	}

	// 2. Check TP end or not
	if (myTaskTrajectory.Circle.Ctrl.MovedTime >= myTaskTrajectory.Circle.Ctrl.TimeEnd)
	{	// Finished
		isMoving = FALSE;
	}
	else
	{
		// 3. Get params
		t  = myTaskTrajectory.Circle.Ctrl.MovedTime + _timeStep;
		Xo = myTaskTrajectory.Circle.Setting.X_Org;
		Yo = myTaskTrajectory.Circle.Setting.Y_Org;
		Ro = myTaskTrajectory.Circle.Setting.Radius;
		Freq  = myTaskTrajectory.Circle.Setting.Freq;
		Phase = DEG2RAD(myTaskTrajectory.Circle.Setting.Phase);

		// 4. Update trajectory
		myTaskTrajectory.CurrTrajectory.Xm_t = Ro*sinf(2*PI*Freq*t + Phase) + Xo;	// xm(t) = Ro*cos(2πft + φ) + Xo
		myTaskTrajectory.CurrTrajectory.Ym_t = Ro*cosf(2*PI*Freq*t + Phase) + Yo;	// ym(t) = Ro*sin(2πft + φ) + Yo
		myTaskTrajectory.CurrTrajectory.Gm_t = DEG2RAD(myTaskTrajectory.Circle.Setting.Gamma);	// Currently, keep it constant [Rad]

		// 5. Update moved time
		myTaskTrajectory.Circle.Ctrl.MovedTime = t;
	}

	return isMoving;
}

GLOBAL void AppControl_IK_Tool2EE(U08 _arm)
{
	if ((LEFT_ARM != _arm) && (RIGHT_ARM != _arm))
	{
		return;
	}

	float Xm = myTaskTrajectory.CurrTrajectory.Xm_t;
	float Ym = myTaskTrajectory.CurrTrajectory.Ym_t;
	float Gm = myTaskTrajectory.CurrTrajectory.Gm_t;	// [Rad]

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
//		AppDataSet_MasterState(MASTER_STATE_ERROR);	// TODO
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

GLOBAL void AppControl_Pos_UpdateTpData(U08 _arm)
{
	static U16 _speed = 10;
	float q1, q2, q3;
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


