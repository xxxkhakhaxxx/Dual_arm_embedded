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


PRIVATE U16 _speedMag = 10;				// Deg/s
PRIVATE U08 _btnSequence = 0;
GLOBAL void AppControl_Pos_TestSquence(void)
{
#if (0)	// Tested
	_speedMag = 90;
	switch (_btnSequence)
	{
	case 0:		// <90 | <0 | <0  -->  90 | 0 | 0
		myRobotCommand[LEFT_ARM].JointPos[0].Angle = 90.0;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = JOINT_DIR_Z_POS;
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = JOINT_DIR_Z_POS;
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = JOINT_DIR_Z_POS;
		_btnSequence = 1;
		break;

	case 1:		// 90 | 0 | 0  -->  180 | -90 | -90
		myRobotCommand[LEFT_ARM].JointPos[0].Angle = 180.0;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = JOINT_DIR_Z_POS;
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = -90.0;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = JOINT_DIR_Z_NEG;
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = -90.0;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = JOINT_DIR_Z_NEG;
		_btnSequence = 2;
		break;

	case 2:		// 180 | -90 | -90  -->  90 | 0 | 0
		myRobotCommand[LEFT_ARM].JointPos[0].Angle = 90.0;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = JOINT_DIR_Z_NEG;
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = JOINT_DIR_Z_POS;
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = JOINT_DIR_Z_POS;
		_btnSequence = 1;
		break;

	default:
		_btnSequence = 0;
		break;
	}
#else // (Tested)
	_speedMag = 45;
	switch (_btnSequence)
	{
	case 0:	// back home using auto check dir function
		myRobotCommand[LEFT_ARM].JointPos[0].Angle = 90.0;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = _Pos_CalMoveDirection(CURR_POS_J11, 90.0, SINGULAR_J11);
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = _Pos_CalMoveDirection(CURR_POS_J21, 0.0, SINGULAR_J21);
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = _Pos_CalMoveDirection(CURR_POS_J31, 0.0, SINGULAR_J31);
		_btnSequence = 1;
		break;
		
	case 1:
		myRobotCommand[LEFT_ARM].JointPos[0].Angle = 180.0;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = _Pos_CalMoveDirection(CURR_POS_J11, 180.0, SINGULAR_J11);
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = -90.0;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = _Pos_CalMoveDirection(CURR_POS_J21, -90.0, SINGULAR_J21);
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = -90.0;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = _Pos_CalMoveDirection(CURR_POS_J31, -90.0, SINGULAR_J31);
		_btnSequence = 2;
		break;
	case 2:
	
		myRobotCommand[LEFT_ARM].JointPos[0].Angle = 90.0;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = _Pos_CalMoveDirection(CURR_POS_J11, 90.0, SINGULAR_J11);
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = _Pos_CalMoveDirection(CURR_POS_J21, 0.0, SINGULAR_J21);
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = _Pos_CalMoveDirection(CURR_POS_J31, 0.0, SINGULAR_J31);
		_btnSequence = 1;
		break;
	default:
		_btnSequence = 1;
		break;
	}



#endif
	return;
}

GLOBAL void AppControl_Pos_BackToHome(U08 _arm, U16 _speed)
{
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
	/*else if (RIGHT_ARM == _arm)
	{

	}*/

	return;
}

PRIVATE strTpSineWave mySineTrajectory[3] = {
		// Amp  , Freq , Phase, Bias ,  Time
		{ 20.0f , 0.2f , 0.0f , 90.0f, 5.0f },
		{ 20.0f , 0.2f , 0.0f ,  0.0f, 5.0f },		// 2025-04-10: Don't set the amplitude to high and run to fast
		{ 20.0f , 0.2f , 0.0f ,  0.0f, 5.0f }};
PRIVATE float _totalTime = 0.0f;
PRIVATE U08	_currJoint = 0;	// Joint 1-2-3 = 0-1-2
PRIVATE BOOL _trajectoryState[3] = { FALSE, FALSE, FALSE };
GLOBAL void AppControl_TP_SineWave(float _timeStep)
{
	// 1. Check if total TP end?
	if (TRUE == _trajectoryState[2])
	{	// Finish TP SineWave for all joints
		return;
	}

	// 2. Update time
	_totalTime = _totalTime + _timeStep;

	// 3. Check if current joint TP end?
	if (_totalTime >= mySineTrajectory[_currJoint].MoveTime)
	{
		_trajectoryState[_currJoint] = TRUE;
		_totalTime = 0.0f;

		if (_currJoint < 2)
		{
			_currJoint++;	// TP for next joint
		}
		else
		{	// Finish TP SineWave for all joints
			return;
		}
	}

	// 4. Caclulate sinwave for current joint
	if (FALSE == _trajectoryState[_currJoint])
	{
		float t = _totalTime;
		float Amp  = mySineTrajectory[_currJoint].Amp;
		float Freq = mySineTrajectory[_currJoint].Freq;
		float Bias = mySineTrajectory[_currJoint].Bias;
		float Phase= DEG2RAD(mySineTrajectory[_currJoint].Phase);

		// Position: p(t) = A * sin(2πft + φ) + Bias
		// Velocity: v(t) = A * 2πf * cos(2πft + φ)
		float rotateAngle = Amp*sinf(2*PI*Freq*t + Phase) + Bias;
		float rotateSpeed = Amp*2*PI*Freq*cosf(2*PI*Freq*t + Phase);
		U08   rotateDir   = (rotateSpeed > 0) ? JOINT_DIR_Z_POS : JOINT_DIR_Z_NEG;
		rotateSpeed = fabs(rotateSpeed);

		switch (_currJoint)
		{
		case 0:
			myRobotCommand[LEFT_ARM].JointPos[0].Angle = rotateAngle;
			myRobotCommand[LEFT_ARM].JointPos[0].Speed = (U16)fminf(rotateSpeed, 90.0)+1;
//			myRobotCommand[LEFT_ARM].JointPos[0].Direction = _Pos_CalMoveDirection(CURR_POS_J11, rotateAngle, SINGULAR_J11);
			myRobotCommand[LEFT_ARM].JointPos[0].Direction = rotateDir;
			myRobotCommand[LEFT_ARM].JointPos[1].Speed = 0;
			myRobotCommand[LEFT_ARM].JointPos[2].Speed = 0;
			break;

		case 1:
			myRobotCommand[LEFT_ARM].JointPos[1].Angle = rotateAngle;
			myRobotCommand[LEFT_ARM].JointPos[1].Speed = (U16)fminf(rotateSpeed, 90.0)+1;
//			myRobotCommand[LEFT_ARM].JointPos[1].Direction = _Pos_CalMoveDirection(CURR_POS_J21, rotateAngle, SINGULAR_J21);
			myRobotCommand[LEFT_ARM].JointPos[1].Direction = rotateDir;
			myRobotCommand[LEFT_ARM].JointPos[0].Speed = 0;
			myRobotCommand[LEFT_ARM].JointPos[2].Speed = 0;
			break;

		case 2:
			myRobotCommand[LEFT_ARM].JointPos[2].Angle = rotateAngle;
			myRobotCommand[LEFT_ARM].JointPos[2].Speed = (U16)fminf(rotateSpeed, 90.0)+1;
//			myRobotCommand[LEFT_ARM].JointPos[2].Direction = _Pos_CalMoveDirection(CURR_POS_J31, rotateAngle, SINGULAR_J31);
			myRobotCommand[LEFT_ARM].JointPos[2].Direction = rotateDir;
			myRobotCommand[LEFT_ARM].JointPos[0].Speed = 0;
			myRobotCommand[LEFT_ARM].JointPos[1].Speed = 0;
			break;

		default:
			return;
		}
	}

	return;
}

