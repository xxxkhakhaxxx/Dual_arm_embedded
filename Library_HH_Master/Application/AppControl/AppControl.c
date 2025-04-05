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


/********************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATION
 ********************************************************************************/


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


PRIVATE float _angleMag = 10.00;		// Degree range -180 ~ +180
PRIVATE U16 _speedMag = 10;				// Deg/s
PRIVATE U08 _btnSequence = FALSE;
GLOBAL void AppControl_Pos_TestSquence(void)
{
#if 0
	_speedMag = 10;
	switch (_btnSequence)
	{
	case 0:		// 90 | 0 | 0
		myRobotCommand[LEFT_ARM].JointPos[0].Angle = 90.0;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = JOINT_DIR_Z_NEG;
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = JOINT_DIR_Z_POS;
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = JOINT_DIR_Z_NEG;
		_btnSequence = 1;
		break;

	case 1:		// 180 | 0 | 0
		myRobotCommand[LEFT_ARM].JointPos[0].Angle = 180.0;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = JOINT_DIR_Z_NEG;
		_btnSequence = 2;
		break;

	case 2:		// 180 | -90 | 0
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = -90.0;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = JOINT_DIR_Z_NEG;
		_btnSequence = 3;
		break;

	case 3:		// 180 | -90 | -90
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = -90.0;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = JOINT_DIR_Z_POS;
		_btnSequence = 4;
		break;

	case 4:		// 90 | 0 | 0
		myRobotCommand[LEFT_ARM].JointPos[0].Angle = 90.0;
		myRobotCommand[LEFT_ARM].JointPos[0].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[0].Direction = JOINT_DIR_Z_POS;
		myRobotCommand[LEFT_ARM].JointPos[1].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[1].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[1].Direction = JOINT_DIR_Z_POS;
		myRobotCommand[LEFT_ARM].JointPos[2].Angle = 0.0;
		myRobotCommand[LEFT_ARM].JointPos[2].Speed = _speedMag;
		myRobotCommand[LEFT_ARM].JointPos[2].Direction = JOINT_DIR_Z_NEG;
		_btnSequence = 1;
		break;

	default:
		_btnSequence = 0;
		break;
	}
#elif (1)
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
#else



#endif
	return;
}

