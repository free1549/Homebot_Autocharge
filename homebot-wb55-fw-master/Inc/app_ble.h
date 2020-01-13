/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * File Name          : app_ble.h
  * Description        : Application configuration file for BLE Middleware.
  *
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
#ifndef APP_BLE_H
#define APP_BLE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hci_tl.h"
#include "ble.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/

typedef enum
{
  APP_BLE_SERVER,
  APP_BLE_CLIENT,
} APP_BLE_Mode_t;

typedef enum
{
  APP_BLE_IDLE,
  APP_BLE_FAST_ADV,
  APP_BLE_LP_ADV,
  APP_BLE_SCAN,
  APP_BLE_LP_CONNECTING,
  APP_BLE_CONNECTED_AS_SERVER, // server role
  APP_BLE_CONNECTED_AS_CLIENT, // client role
} APP_BLE_ConnStatus_t;

typedef enum
{
  APP_SERVER_IDLE,
  APP_SERVER_FAST_ADV,
  APP_SERVER_LP_ADV,
  APP_SERVER_LP_CONNECTING,
  APP_SERVER_CONNECTED, // server role
} APP_SERVER_ConnStatus_t;

typedef enum
{
  APP_CLIENT_IDLE,
  APP_CLIENT_CONNECTING,
  APP_CLIENT_DISCOVER_SERVICES,
  APP_CLIENT_DISCOVER_CHARACS,
  APP_CLIENT_DISCOVER_WRITE_DESC,
  APP_CLIENT_DISCOVER_NOTIFICATION_CHAR_DESC,
  APP_CLIENT_ENABLE_NOTIFICATION_DESC,
  APP_CLIENT_DISABLE_NOTIFICATION_DESC,
  APP_CLIENT_CONNECTED,
} APP_CLIENT_ConnStatus_t;

/* Private typedef -----------------------------------------------------------*/
/**
 * security parameters structure
 */ 
typedef struct _tSecurityParams
{
  /**
   * IO capability of the device
   */
  uint8_t ioCapability;

  /**
   * Authentication requirement of the device
   * Man In the Middle protection required?
   */
  uint8_t mitm_mode;

  /**
   * bonding mode of the device
   */
  uint8_t bonding_mode;

  /**
   * Flag to tell whether OOB data has
   * to be used during the pairing process
   */
  uint8_t OOB_Data_Present; 

  /**
   * OOB data to be used in the pairing process if
   * OOB_Data_Present is set to TRUE
   */
  uint8_t OOB_Data[16]; 

  /**
   * this variable indicates whether to use a fixed pin
   * during the pairing process or a passkey has to be
   * requested to the application during the pairing process
   * 0 implies use fixed pin and 1 implies request for passkey
   */
  uint8_t Use_Fixed_Pin; 

  /**
   * minimum encryption key size requirement
   */
  uint8_t encryptionKeySizeMin;

  /**
   * maximum encryption key size requirement
   */
  uint8_t encryptionKeySizeMax;

  /**
   * fixed pin to be used in the pairing process if
   * Use_Fixed_Pin is set to 1
   */
  uint32_t Fixed_Pin;

  /**
   * this flag indicates whether the host has to initiate
   * the security, wait for pairing or does not have any security
   * requirements.\n
   * 0x00 : no security required
   * 0x01 : host should initiate security by sending the slave security
   *        request command
   * 0x02 : host need not send the clave security request but it
   * has to wait for paiirng to complete before doing any other
   * processing
   */
  uint8_t initiateSecurity;
}tSecurityParams;

/**
 * global context
 * contains the variables common to all 
 * services
 */ 
typedef struct _tBLEProfileGlobalContext
{

  /**
   * security requirements of the host
   */ 
  tSecurityParams bleSecurityParam;

  /**
   * gap service handle
   */
  uint16_t gapServiceHandle;

  /**
   * device name characteristic handle
   */ 
  uint16_t devNameCharHandle;

  /**
   * appearance characteristic handle
   */ 
  uint16_t appearanceCharHandle;

  /**
   * connection handle of the current active connection
   * When not in connection, the handle is set to 0xFFFF
   */ 
  uint16_t connectionHandle;

  /**
   * length of the UUID list to be used while advertising
   */ 
  uint8_t advtServUUIDlen;

  /**
   * the UUID list to be used while advertising
   */ 
  uint8_t advtServUUID[100];

}BleGlobalContext_t;

typedef struct
{
  BleGlobalContext_t BleApplicationContext_legacy;
  APP_SERVER_ConnStatus_t Device_Connection_Status;
  uint8_t Notification_Status; /* used to chek if P2P Server is enabled to Notify */
  uint16_t SvcHdle;				        /**< Service handle */
  uint16_t WriteClientToServerCharHdle;	  /**< Characteristic handle */
  uint16_t NotifyServerToClientCharHdle;	/**< Characteristic handle */
}BleServerApplicationContext_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */  

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ---------------------------------------------*/
  void APP_BLE_Init( void ); 

  void MacAddressString(uint8_t *mac_str, tBDAddr addr);
  void MacAddressStringToBytes(uint8_t *mac_str, tBDAddr addr);
  void SendDisconnected(tBDAddr mac);

/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*APP_BLE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
