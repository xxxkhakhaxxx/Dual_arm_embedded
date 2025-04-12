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


/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL void AppControl_CalRobotIK(void);
GLOBAL void AppControl_Pos_TestSquence(U08 _arm, U16 _speed);
GLOBAL void AppControl_Pos_BackToHome(U08 _arm, U16 _speed);
GLOBAL void AppControl_TP_SineWave(U08 _arm, float _timeStep);		// Unit: s



#endif /* APPLICATION_APPCONTROL_APPCONTROL_H_ */
