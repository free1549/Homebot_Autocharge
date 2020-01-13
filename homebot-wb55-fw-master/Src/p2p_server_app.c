/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : p2p_server_app.c
 * Description        : P2P Server Application
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "p2p_server_app.h"
#include "stm32_seq.h"
#include "app_ble.h"
#include "hi2st-parser.h"
#include "common_blesvc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

PLACE_IN_SECTION("BLE_APP_CONTEXT") BleServerApplicationContext_t BleServerApplicationContext;
PLACE_IN_SECTION("BLE_APP_CONTEXT") uint16_t AdvIntervalMin, AdvIntervalMax;

/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t Server_Event_Handler(void *pckt);


/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* Hardware Characteristics Service */
/*
 The following 128bits UUIDs have been generated from the random UUID
 generator:
 D973F2E0-B19E-11E2-9E96-0800200C9A66: Service 128bits UUID
 D973F2E1-B19E-11E2-9E96-0800200C9A66: Characteristic_1 128bits UUID
 D973F2E2-B19E-11E2-9E96-0800200C9A66: Characteristic_2 128bits UUID
 */
#define COPY_P2P_SERVICE_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x00,0x00,0xfe,0x40,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_P2P_WRITE_CHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0xfe,0x41,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_P2P_NOTIFY_UUID(uuid_struct)        COPY_UUID_128(uuid_struct,0x00,0x00,0xfe,0x42,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)

/**
 * Advertising Data
 */
static const char local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME ,'F','O','R','C','E','M','E'};
uint8_t *server_bd_addr;

/* 31 bytes fix */
uint8_t adv_data[31] = {
    /* Advertising data: Flags AD Type */
    0x02, // size 2 bytes
    AD_TYPE_FLAGS, // data type : 0x01 (Flags)
    FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE | FLAG_BIT_BR_EDR_NOT_SUPPORTED, // data : 06 (0x02 LE General Discoverable Mode, 0x04 : BR/EDR Not supported
    /* Advertising data: device name*/
    0x08, //size
    AD_TYPE_COMPLETE_LOCAL_NAME,  // 0x09 : Complete local Name
    0, 0, 0, 0, 0, 0, 0,//data : device name : max length 7 
    /* Advertising data: Complete List of 128-bit Service Class UUIDs*/
    0x11,       // size : 17bytes
    AD_TYPE_128_BIT_SERV_UUID_CMPLT_LIST,       //data type : 0x07 Complete List of 128-bit Service Class UUIDs
    0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe3,0xf2,0x73,0xd9
};

uint8_t manuf_data[15] = {
    sizeof(manuf_data)-1,
    AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
    'V' /* Company ID  */,
    'A' /* Company ID  */,
    0x00 /*Sensor Type*/,
    0x00 /* Interrupt*/,
    0x00 /* Sensor Data 1_1 */,
    0x00 /* Sensor Data 1_2 */,
    0x00 /* Sensor Data 2_1 */,
    0x00 /* Sensor Data 2_2 */,
    0x00 /* Sensor Data 3_1 */,
    0x00 /* Sensor Data 3_2 */,
    0x00 /* Sensor Data 4_1 */,
    0x00 /* Sensor Data 4_2 */,
    0x00 /* Battery %(percent) */,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern APP_BLE_Mode_t BleMode;

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
  switch(pNotification->P2P_Evt_Opcode)
  {
    case P2PS_STM__NOTIFY_ENABLED_EVT:
      BleServerApplicationContext.Notification_Status = 1;
      break;

    case P2PS_STM_NOTIFY_DISABLED_EVT:
      BleServerApplicationContext.Notification_Status = 0;
      break;
      
    case P2PS_STM_WRITE_EVT:
      for(uint8_t i=0; i<pNotification->DataTransfered.Length; i++)
  		{
  			//addToBleParserFromWB55(pNotification->DataTransfered.pPayload[i], 0);
  			addToBleParserFromAndroid(pNotification->DataTransfered.pPayload[i]);
  		}
      UTIL_SEQ_SetTask( 1<<CFG_TASK_PARSE_BLE_ID, CFG_SCH_PRIO_0);
      break;

    default:
      break;
  }

  return;
}

void P2PS_APP_Notification(P2P_APP_ConnHandle_Not_evt_t *pNotification)
{
  switch(pNotification->P2P_Evt_Opcode)
  {
  case PEER_CONN_HANDLE_EVT :
    /* Connection as client */
    BleServerApplicationContext.BleApplicationContext_legacy.connectionHandle = pNotification->ConnectionHandle;
    BleServerApplicationContext.Device_Connection_Status = APP_SERVER_CONNECTED;
    break;

  case PEER_DISCON_HANDLE_EVT :
    if (pNotification->ConnectionHandle == BleServerApplicationContext.BleApplicationContext_legacy.connectionHandle)
    {
      BleServerApplicationContext.BleApplicationContext_legacy.connectionHandle = 0;
      BleServerApplicationContext.Notification_Status = 0;
      Adv_Request(APP_SERVER_FAST_ADV);
    }
    break;

  default:
    break;
  }

  return;
}

 // copy from p2p_stm.c
 
/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Server_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blue_aci *blue_evt;
  aci_gatt_attribute_modified_event_rp0    * attribute_modified;
  P2PS_STM_App_Notification_evt_t Notification;

  if(BleMode != APP_BLE_SERVER)
  {
    return SVCCTL_EvtNotAck;
  }

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch(event_pckt->evt)
  {
    case EVT_VENDOR:
    {
      blue_evt = (evt_blue_aci*)event_pckt->data;
      switch(blue_evt->ecode)
      {
        case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
       {
          attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blue_evt->data;
            if(attribute_modified->Attr_Handle == (BleServerApplicationContext.NotifyServerToClientCharHdle + 2))
            {
              /**
               * Descriptor handle
               */
              return_value = SVCCTL_EvtAckFlowEnable;
              /**
               * Notify to application
               */
              if(attribute_modified->Attr_Data[0] & COMSVC_Notification)
              {
                Notification.P2P_Evt_Opcode = P2PS_STM__NOTIFY_ENABLED_EVT;
                P2PS_STM_App_Notification(&Notification);
              }
              else
              {
                Notification.P2P_Evt_Opcode = P2PS_STM_NOTIFY_DISABLED_EVT;
                P2PS_STM_App_Notification(&Notification);
              }
            }
            
            else if(attribute_modified->Attr_Handle == (BleServerApplicationContext.WriteClientToServerCharHdle + 1))
            {
              Notification.P2P_Evt_Opcode = P2PS_STM_WRITE_EVT;
              Notification.DataTransfered.Length=attribute_modified->Attr_Data_Length;
              Notification.DataTransfered.pPayload=attribute_modified->Attr_Data;
              P2PS_STM_App_Notification(&Notification);  
            }
        }
        break;

        default:
          break;
      }
    }
    break; /* HCI_EVT_VENDOR_SPECIFIC */

    default:
      break;
  }

  return(return_value);
}/* end SVCCTL_EvtAckStatus_t */

void Add_Service(void)
{ 
  Char_UUID_t  uuid16;

  /**
   *	Register the event handler to the BLE controller
   */
  SVCCTL_RegisterSvcHandler(Server_Event_Handler);
  
    /**
     *  Peer To Peer Service
     *
     * Max_Attribute_Records = 2*no_of_char + 1
     * service_max_attribute_record = 1 for Peer To Peer service +
     *                                2 for P2P Write characteristic +
     *                                2 for P2P Notify characteristic +
     *                                1 for client char configuration descriptor +
     *                                
     */
    COPY_P2P_SERVICE_UUID(uuid16.Char_UUID_128);
    aci_gatt_add_service(UUID_TYPE_128,
                      (Service_UUID_t *) &uuid16,
                      PRIMARY_SERVICE,
                      8,
                      &(BleServerApplicationContext.SvcHdle));

    /**
     *  Add Write Characteristic
     */
    COPY_P2P_WRITE_CHAR_UUID(uuid16.Char_UUID_128);
    aci_gatt_add_char(BleServerApplicationContext.SvcHdle,
                      UUID_TYPE_128, &uuid16,
                      20,                                   
                      CHAR_PROP_WRITE_WITHOUT_RESP,
                      ATTR_PERMISSION_NONE,
                      GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
                      10, /* encryKeySize */
                      1, /* isVariable */
                      &(BleServerApplicationContext.WriteClientToServerCharHdle));

    /**
     *   Add Notify Characteristic
     */
    COPY_P2P_NOTIFY_UUID(uuid16.Char_UUID_128);
    aci_gatt_add_char(BleServerApplicationContext.SvcHdle,
                      UUID_TYPE_128, &uuid16,
                      20,
                      CHAR_PROP_NOTIFY,
                      ATTR_PERMISSION_NONE,
                      GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
                      10, /* encryKeySize */
                      1, /* isVariable: 1 */
                      &(BleServerApplicationContext.NotifyServerToClientCharHdle));
}

void Remove_Service(void)
{ 
  if(BleServerApplicationContext.SvcHdle == 0xFFFF)
  {
    return;
  }
    /**
     *  Peer To Peer Service
     *
     * Max_Attribute_Records = 2*no_of_char + 1
     * service_max_attribute_record = 1 for Peer To Peer service +
     *                                2 for P2P Write characteristic +
     *                                2 for P2P Notify characteristic +
     *                                1 for client char configuration descriptor +
     *                                
     */

    /**
     *  Delete Write Characteristic
     */
    aci_gatt_del_char(BleServerApplicationContext.SvcHdle, BleServerApplicationContext.WriteClientToServerCharHdle);

    /**
     *   Delete Notify Characteristic
     */
    aci_gatt_del_char(BleServerApplicationContext.SvcHdle, BleServerApplicationContext.NotifyServerToClientCharHdle);

    /**
     *   Delete Service
     */
    aci_gatt_del_service(BleServerApplicationContext.SvcHdle);
}

void P2PS_APP_Init(void)
{
  /**
   * Initialization of the BLE App Context
   */
  BleServerApplicationContext.Device_Connection_Status = APP_SERVER_IDLE;
  BleServerApplicationContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;
  BleServerApplicationContext.SvcHdle = 0xFFFF;

  /**
   * Make device discoverable
   */
  BleServerApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = NULL;
  BleServerApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 0;
  /* Initialize intervals for reconnexion without intervals update */
  AdvIntervalMin = CFG_FAST_CONN_ADV_INTERVAL_MIN;
  AdvIntervalMax = CFG_FAST_CONN_ADV_INTERVAL_MAX;

  /**
   * Start to Advertise to be connected by P2P Client
   */
   if(BleMode == APP_BLE_SERVER)
  {
     Add_Service();
     Adv_Request(APP_SERVER_FAST_ADV);
  }

  return;
}

/*************************************************************
 *
 *SPECIFIC FUNCTIONS FOR SERVER
 *
 *************************************************************/
void Adv_Cancel( void )
{
  if (BleServerApplicationContext.Device_Connection_Status != APP_SERVER_CONNECTED)
  {
    tBleStatus result = 0x00;
    result = aci_gap_set_non_discoverable();

    BleServerApplicationContext.Device_Connection_Status = APP_SERVER_IDLE;
    if (result == BLE_STATUS_SUCCESS)
    {
    }
    else
    {
    }
  }
  return;
}

void Adv_Request(APP_SERVER_ConnStatus_t New_Status)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint16_t Min_Inter, Max_Inter;
 
  if (New_Status == APP_SERVER_FAST_ADV)
  {
    Min_Inter = AdvIntervalMin;
    Max_Inter = AdvIntervalMax;
  }
  else
  {
    Min_Inter = CFG_LP_CONN_ADV_INTERVAL_MIN;
    Max_Inter = CFG_LP_CONN_ADV_INTERVAL_MAX;
  }

    if ((New_Status == APP_SERVER_LP_ADV)
        && ((BleServerApplicationContext.Device_Connection_Status == APP_SERVER_FAST_ADV)
            || (BleServerApplicationContext.Device_Connection_Status == APP_SERVER_LP_ADV)))
    {
      /* Connection in ADVERTISE mode have to stop the current advertising */
      ret = aci_gap_set_non_discoverable();
      if (ret == BLE_STATUS_SUCCESS)
      {
      }
      else
      {
      }
    }

    BleServerApplicationContext.Device_Connection_Status = New_Status;
    /* Start Fast or Low Power Advertising */
    ret = aci_gap_set_discoverable(
        ADV_IND,
        Min_Inter,
        Max_Inter,
        PUBLIC_ADDR,
        NO_WHITE_LIST_USE, /* use white list */
        sizeof(local_name),
        (uint8_t*) &local_name,
        BleServerApplicationContext.BleApplicationContext_legacy.advtServUUIDlen,
        BleServerApplicationContext.BleApplicationContext_legacy.advtServUUID,
        0,
        0);

//    manuf_data[SENSORTYPE_MANU_POS] = sensor_type;

    /* Update Advertising data */
    ret = aci_gap_update_adv_data(sizeof(manuf_data), (uint8_t*) manuf_data);

     if (ret == BLE_STATUS_SUCCESS)
    {
      if (New_Status == APP_SERVER_FAST_ADV)
      {
      }
      else
      {
      }
    }
    else
    {
      if (New_Status == APP_SERVER_FAST_ADV)
      {
      }
      else
      {
      }
    }

  return;
}

void AdvData_Change(void)
{
#if 0
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint8_t bat_per = (uint8_t)((Batgauge.Data.Voltage * 100) / 3700);
  uint8_t bInterrupt = 0;
  uint16_t data1 = 0, data2 = 0, data3 = 0, data4 = 0;

  if(bat_per > 100)
  {
    bat_per = 100;
  }

  if(sensor_type == (SENSOR_TYPE_TEMPERATURE | SENSOR_TYPE_HUMIDITY | SENSOR_TYPE_ECO2 | SENSOR_TYPE_TVOC))
  {
    data1 =  (uint16_t) (shtc1_temp * 10);// temperature
    data2 =  (uint16_t) (shtc1_hum * 10);// humidity
    data3 = (uint16_t) ECO2_ppm;
    data4 = (uint16_t) TVOC_ppb;
  }
  else if(sensor_type == SENSOR_TYPE_PIR)
  {
    bInterrupt = pir_irq;
  }
  else if(sensor_type == SENSOR_TYPE_HALL)
  {
    bInterrupt = hall_irq;
    data1 = HAL_GPIO_ReadPin(HALL_IRQ_GPIO_Port, HALL_IRQ_Pin);      
  }
  

  manuf_data[SENSORTYPE_MANU_POS] = sensor_type;
  manuf_data[SENSORINTERRUPT_MANU_POS] = bInterrupt;

  manuf_data[SENSORDATA1_MANU_POS1] = data1 >> 8;
  manuf_data[SENSORDATA1_MANU_POS2] = data1 & 0xFF;

  manuf_data[SENSORDATA2_MANU_POS1] = data2 >> 8;
  manuf_data[SENSORDATA2_MANU_POS2] = data2 & 0xFF;

  manuf_data[SENSORDATA3_MANU_POS1] = data3 >> 8;
  manuf_data[SENSORDATA3_MANU_POS2] = data3 & 0xFF;

  manuf_data[SENSORDATA4_MANU_POS1] = data4 >> 8;
  manuf_data[SENSORDATA4_MANU_POS2] = data4 & 0xFF;

  manuf_data[SENSORBATTERY_MANU_POS] = bat_per;

  ret = aci_gap_update_adv_data(sizeof(manuf_data), (uint8_t*) manuf_data);
#endif
}

APP_SERVER_ConnStatus_t APP_BLE_Get_Server_Connection_Status(void)
{
    return BleServerApplicationContext.Device_Connection_Status;
}

tBleStatus Server_Update_Char(uint8_t payload_length, uint8_t *pPayload) // Server to Client
{
  tBleStatus result = BLE_STATUS_SUCCESS;

  if(BleServerApplicationContext.Notification_Status == 1)
  {
    result = aci_gatt_update_char_value(BleServerApplicationContext.SvcHdle,
                            BleServerApplicationContext.NotifyServerToClientCharHdle,
                             0, /* charValOffset */
                            payload_length, /* charValueLen */
                            (uint8_t *)  pPayload);
  }

  return result;
}/* end Write_Char() */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
