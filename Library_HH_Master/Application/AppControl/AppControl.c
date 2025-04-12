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

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * PRIVATE FUNCTION DECLARATION
 ********************************************************************************/
PRIVATE U08 _Pos_CalMoveDirection(float _startAngle, float _goalAngle, float _avoidAngle);


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
GLOBAL void AppControl_TP_SineWave(U08 _arm, float _timeStep)
{	// 0. Safety check
	if ((LEFT_ARM != _arm) && (RIGHT_ARM != _arm))
	{
		return;
	}

	// 1. Check if total TP end?
	if (TRUE == _trajectoryState[_arm][2])
	{	// Finish TP SineWave for all joints
		return;
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
			return;
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
			return;
		}
	}

	return;
}

