/* USER CODE BEGIN */
/**
 ******************************************************************************
 * File Name          : p2p_server_app.h
 * Description        : Header for p2p_server_app.c module
 ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __P2P_SERVER_APP_H
#define __P2P_SERVER_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_ble.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  PEER_CONN_HANDLE_EVT,
  PEER_DISCON_HANDLE_EVT,
} P2P_APP__Opcode_Notification_evt_t;

typedef struct
{
  P2P_APP__Opcode_Notification_evt_t   P2P_Evt_Opcode;
  uint16_t                              ConnectionHandle;
}P2P_APP_ConnHandle_Not_evt_t;


typedef enum
{
  P2PS_STM__NOTIFY_ENABLED_EVT,
  P2PS_STM_NOTIFY_DISABLED_EVT,
  P2PS_STM_READ_EVT,
  P2PS_STM_WRITE_EVT,
  P2PS_STM_BOOT_REQUEST_EVT,
} P2PS_STM_Opcode_evt_t;

typedef struct
{
  uint8_t * pPayload;
  uint8_t     Length;
}P2PS_STM_Data_t;  

typedef struct
{
  P2PS_STM_Opcode_evt_t     P2P_Evt_Opcode;
  P2PS_STM_Data_t           DataTransfered;
  uint16_t                  ConnectionHandle;
  uint8_t                   ServiceInstance;
}P2PS_STM_App_Notification_evt_t;

/* USER CODE BEGIN ET */
 #define SENSORTYPE_MANU_POS 4
#define SENSORINTERRUPT_MANU_POS 5
#define SENSORDATA1_MANU_POS1 6
#define SENSORDATA1_MANU_POS2 7
#define SENSORDATA2_MANU_POS1 8
#define SENSORDATA2_MANU_POS2 9
#define SENSORDATA3_MANU_POS1 10
#define SENSORDATA3_MANU_POS2 11
#define SENSORDATA4_MANU_POS1 12
#define SENSORDATA4_MANU_POS2 13
#define SENSORBATTERY_MANU_POS 14
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ---------------------------------------------*/
  void P2PS_APP_Init( void );
  void P2PS_APP_Notification( P2P_APP_ConnHandle_Not_evt_t *pNotification );
  void Adv_Request(APP_SERVER_ConnStatus_t New_Status);
  void Adv_Cancel( void );
  void AdvData_Change(void);
  APP_SERVER_ConnStatus_t APP_BLE_Get_Server_Connection_Status(void);
  void Add_Service(void);
  void Remove_Service(void);
  tBleStatus Server_Update_Char(uint8_t payload_length, uint8_t *pPayload); // Server to Client
/* USER CODE BEGIN EF */
 
/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*__P2P_SERVER_APP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
