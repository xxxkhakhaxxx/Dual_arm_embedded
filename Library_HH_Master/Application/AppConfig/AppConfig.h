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
#include <LibraryHHInterface_Master.h>

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/
#define PROJECT_MCU		STM32F4XX_FW_1_28_1

/* --------------- PROGRAM LOGIC MACRO --------------- */
//#define MASTER_NO_CONTROL				// Master will not send control command
//#define MASTER_NO_GUI					// Master will not send data to GUI
//#define MASTER_GUI_TP_CHECK				// Check TP via GUI - only use in Pos control
//#define TEST_MOTOR_FILTER				// Only use in Pos control - Dual arm

//#define MASTER_CONTROL_POS
//#define MASTER_CONTROL_VEL
#define MASTER_CONTROL_TOR
//#define MASTER_CONTROLLER_PD_SPD
#define MASTER_CONTROLLER_SMC_SSMC

//#define SLAVE_1_ENA		(TRUE)
#define SLAVE_2_ENA		(TRUE)

/* --------------- ROBOT PARAMS MACRO --------------- */
#define LEFT_ARM		(0)			// For array idx
#define RIGHT_ARM		(1)			// For array idx
#define DUAL_ARM		(2)			// For array init
#define JOINTS_PER_ARM	(3)

/* ------------ KINEMATICS PARAMS MACRO -------------- */
#define JOINT_DIR_Z_POS	TRUE	// Positive direction in kinematics
#define JOINT_DIR_Z_NEG	FALSE	// Negative direction in kinematics

#define OBJ_HALF_WIDTH_A	(0.05f)	// [m]
#define OBJ_HALF_HEIGHT_B	(0.05f)	// [m]
#define BASE_PARAM_C	(0.224f)	// [m]
#define BASE_PARAM_D	(0.000f)	// [m]
#define L1				(0.233f)	// [m]	20 + 180 + 33
#define L2				(0.233f)	// [m]	20 + 180 + 33
#define L3				(0.138f)	// [m]  20 + 100 + 18

#define M1				(0.368f)	// [Kg]
#define M2				(0.356f)	// [Kg]
#define M3				(0.040f)	// [Kg]
//#define M12				(0.368f)	// [Kg]
//#define M22				(0.330f)	// [Kg]
//#define M32				(0.042f)	// [Kg]

#define L1_L2_M2		(L1*L2*M2)
#define L1_L2_M3		(L1*L2*M3)
#define L1_L3_M2		(L1*L3*M2)
#define L1_L3_M3		(L1*L3*M3)
#define L2_L3_M2		(L2*L3*M2)
#define L2_L3_M3		(L2*L3*M3)
#define M1_M2_M3		(M1+M2+M3)
#define M2_M3			(M2+M3)

/* --------------- CONTROL PARAMS MACRO -------------- */
#define CONTROL_MAX_TORQUE	(3.0f)	// [Nm]


/* ----------------- TIME PARAMS MACRO ---------------- */
#define PERIOD_CONTROL		(0.01f)	// Update this base on your timer 2 [s]
#define PERIOD_GUI_SEND		(0.02f)	// [s]
#define GUI_SEND_CNT_MAX	(PERIOD_GUI_SEND/PERIOD_CONTROL - 1)
#define PERIOD_TRAJECTORY_PLANNING	PERIOD_CONTROL	// [s]
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
