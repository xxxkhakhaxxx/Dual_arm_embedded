/**
 ********************************************************************************
 ** @file    AppCommSPI.h
 ** @author  HH (hunghoang.1806@gmail.com)
 ** @date    Oct 30, 2024 (created)
 ** @brief   
 ********************************************************************************
 **/

#ifndef APPLICATION_APPCOMMUNICATE_APPCOMMSPI_H_
#define APPLICATION_APPCOMMUNICATE_APPCOMMSPI_H_


/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include "LibraryHHInterface_Master.h"

/********************************************************************************
 * MACROS AND DEFINES
 ********************************************************************************/
typedef enum ENUM_SLAVE_ID
{
	SLAVE_1_ID = 0x00,
	SLAVE_2_ID = 0x01
} enSlaveId;

typedef enum ENUM_MASTER_SEND_MSG_ID
{
	MASTER_MSG_ANGLE_KINEMATICS = 0x00,
	MASTER_MSG_ANGLE_DIRECT,
	MASTER_MSG_FORCE
} enMasterSendMsgId;

typedef enum ENUM_SLAVE_SEND_MSG_ID
{
	SLAVE_MSG_POSITION = 0x00,
	SLAVE_MSG_POSITION_SPEED,
	SLAVE_MSG_POSITION_SPEED_TORQUE
} enSlaveSendMsgId;
/********************************************************************************
 * TYPEDEFS AND ENUMS
 ********************************************************************************/


/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/


/********************************************************************************
 * GLOBAL FUNCTION DECLARATION
 ********************************************************************************/
GLOBAL void AppCommSPI_UserSetup(SPI_HandleTypeDef* hspi);
GLOBAL void AppCommSPI_SendSlaveMessage(enSlaveId _SlaveId, enMasterSendMsgId _TxMsgId);
GLOBAL void AppCommSPI_GetSlaveMessage(void);

/* HAL function implement */
GLOBAL void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
GLOBAL void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
#endif /* APPLICATION_APPCOMMUNICATE_APPCOMMSPI_H_ */
