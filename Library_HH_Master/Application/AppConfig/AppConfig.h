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
//#define	MASTER_NO_GUI					// Master will not send data to GUI
#define MASTER_GUI_TP_CHECK				// Check TP via GUI

#define	MASTER_CONTROL_POS
//#define MASTER_CONTROL_VEL
//#define MASTER_CONTROL_TOR

#define SLAVE_1_ENA		(TRUE)
#define SLAVE_2_ENA		(TRUE)

/* --------------- PROGRAM LOGIC MACRO --------------- */
#define LEFT_ARM		(0)			// For array idx
#define RIGHT_ARM		(1)			// For array idx
#define DUAL_ARM		(2)			// For array init

/* ------------- KINEMATICS PARAMS MACRO --------------- */
#define JOINT_DIR_Z_POS	TRUE	// Positive direction in kinematics
#define JOINT_DIR_Z_NEG	FALSE	// Negative direction in kinematics

#define OBJ_HALF_WIDTH_A	(0.05f)	// [m]
#define OBJ_HALF_HEIGHT_B	(0.05f)	// [m]
#define BASE_PARAM_C	(0.224f)	// [m]
#define BASE_PARAM_D	(0.000f)	// [m]
#define L1				(0.233f)	// [m]
#define L2				(0.233f)	// [m]
#define L3				(0.138f)	// [m]
#define M11				(0.368f)	// [Kg]
#define M21				(0.356f)	// [Kg]
#define M31				(0.040f)	// [Kg]
#define M12				(0.368f)	// [Kg]
#define M22				(0.330f)	// [Kg]
#define M32				(0.042f)	// [Kg]


/* ----------------- GUI PARAMS MACRO ---------------- */
#define PERIOD_CONTROL		(20)	// Update this base on your timer 2 [ms]
#define PERIOD_GUI_SEND		(20)	// [ms]
#define GUI_SEND_CNT_MAX	(PERIOD_GUI_SEND/PERIOD_CONTROL - 1)
#define PERIOD_TRAJECTORY_PLANNING	(0.02f)	// [s]
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
