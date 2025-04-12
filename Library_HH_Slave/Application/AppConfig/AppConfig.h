/**
 ********************************************************************************
 ** @file    AppConfig.h
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Oct 30, 2024 (created)
 ** @brief   
 ********************************************************************************
 **/

#ifndef APPLICATION_APPCONFIG_APPCONFIG_H_
#define APPLICATION_APPCONFIG_APPCONFIG_H_


/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include <LibraryHHInterface_Slave.h>

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/
#define PROJECT_MCU		STM32F1XX_FW_1_8_6

#define DUAL_ARM_LEFT	1
#define DUAL_ARM_RIGHT	2
#define THIS_IS_ARM 	DUAL_ARM_RIGHT

#define JOINT_DIR_Z_POS		TRUE	// Positive direction in kinematics
#define JOINT_DIR_Z_NEG		FALSE	// Negative direction in kinematics

// Mapping direction and offset from Real joint to Kinematics
#if defined(THIS_IS_ARM) && (THIS_IS_ARM == DUAL_ARM_LEFT)
	// Left arm setting code here
	#define J1_DIR_KINE2REAL	((I08)(1))
	#define J2_DIR_KINE2REAL	((I08)(-1))
	#define J3_DIR_KINE2REAL	((I08)(1))
	#define J1_DIR_REAL2KINE	((I08)(1))
	#define J2_DIR_REAL2KINE	((I08)(-1))
	#define J3_DIR_REAL2KINE	((I08)(1))

	// Joint calibration: J_real = J_kinematics*J_dir + J_offset
	#define J1_OFFSET_REAL2KINE	(-252.73f)
	#define J2_OFFSET_REAL2KINE	(-57.83f)
	#define J3_OFFSET_REAL2KINE	(-42.73f)
	#define J1_OFFSET_KINE2REAL	((I32)(10727))
	#define J2_OFFSET_KINE2REAL	((I32)(30217))
	#define J3_OFFSET_KINE2REAL	((I32)(31727))

#elif defined(THIS_IS_ARM) && (THIS_IS_ARM == DUAL_ARM_RIGHT)
	// Right arm setting code here
	#define J1_DIR_KINE2REAL	((I08)(-1))
	#define J2_DIR_KINE2REAL	((I08)(1))
	#define J3_DIR_KINE2REAL	((I08)(-1))
	#define J1_DIR_REAL2KINE	((I08)(-1))
	#define J2_DIR_REAL2KINE	((I08)(1))
	#define J3_DIR_REAL2KINE	((I08)(-1))

	#define J1_OFFSET_REAL2KINE	(-31.09f)
	#define J2_OFFSET_REAL2KINE	(-18.15f)
	#define J3_OFFSET_REAL2KINE	(-2.21f)
	#define J1_OFFSET_KINE2REAL	((I32)(32897))
	#define J2_OFFSET_KINE2REAL	((I32)(34185))
	#define J3_OFFSET_KINE2REAL	((I32)(35779))
#else
	#error "Choose arm: DUAL_ARM_LEFT or DUAL_ARM_RIGHT"
#endif


//#define TEST_UART_SEND				// Init-> Auto send and Receive: Used for checking which UART channel work with DMA and IT
//#define TEST_UART_CYCLE_NO_FEEDBACK	// Init-> Wait Master-> FeedBack Master-> Wait Master-> Control Motor-> Motor Feedback-> Feedback Master
//#define TEST_UART_CYCLE_MOTOR_DATA	// Init-> ... -> wait Master -> Read motor data -> Cal speed + accel -> Feedback  motor data -> wait Master.
//#define SLAVE_NO_MOTOR_COMM

#define TOTAL_MOTOR_FOR_ONE_ARM	(3)
#define MOTOR_1		LINGKONG_MG5010E_i10
#define MOTOR_2		LINGKONG_MG4010E_i10
#define MOTOR_3		LINGKONG_MG4010E_i10

#if ((MOTOR_1==LINGKONG_MG5010E_i10) || (MOTOR_1==LINGKONG_MG4010E_i10))
#define MOTOR_1_GEARBOX		(10)
#endif

#if ((MOTOR_2==LINGKONG_MG5010E_i10) || (MOTOR_2==LINGKONG_MG4010E_i10))
#define MOTOR_2_GEARBOX		(10)
#endif
#if ((MOTOR_3==LINGKONG_MG5010E_i10) || (MOTOR_3==LINGKONG_MG4010E_i10))
#define MOTOR_3_GEARBOX		(10)
#endif


/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/


/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/



#endif /* APPLICATION_APPCONFIG_APPCONFIG_H_ */
