/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : p2p_client_app.c
 * Description        : P2P Client Application
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
#include "p2p_client_app.h"

#include "stm32_seq.h"
#include "app_ble.h"
#include "hi2st-parser.h"
#include "timer.h"

/* USER CODE BEGIN Includes */
#define SENSOR_CLIENT
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

typedef enum
{
  P2P_START_TIMER_EVT,
  P2P_STOP_TIMER_EVT,
  P2P_NOTIFICATION_INFO_RECEIVED_EVT,
} P2P_Client_Opcode_Notification_evt_t;

typedef struct
{
  uint8_t * pPayload;
  uint8_t     Length;
}P2P_Client_Data_t;

typedef struct
{
  P2P_Client_Opcode_Notification_evt_t  P2P_Client_Evt_Opcode;
  P2P_Client_Data_t DataTransfered;
}P2P_Client_App_Notification_evt_t;

typedef struct
{
  /**
   * state of the P2P Client
   * state machine
   */
  APP_CLIENT_ConnStatus_t state;

  /**
   * connection handle
   */
  uint16_t connHandle;

  /**
   * handle of the P2P service
   */
  uint16_t P2PServiceHandle;

  /**
   * end handle of the P2P service
   */
  uint16_t P2PServiceEndHandle;

  /**
   * handle of the Tx characteristic - Write To Server
   *
   */
  uint16_t P2PWriteToServerCharHdle;

  /**
   * handle of the client configuration
   * descriptor of Tx characteristic
   */
  uint16_t P2PWriteToServerDescHandle;

  /**
   * handle of the Rx characteristic - Notification From Server
   *
   */
  uint16_t P2PNotificationCharHdle;

  /**
   * handle of the client configuration
   * descriptor of Rx characteristic
   */
  uint16_t P2PNotificationDescHandle;

  /**
   * mac address
   */
  tBDAddr macAddress;
  /**
   * device name
   */
  uint8_t devName[20];
  /**
   * device name length
   */
  uint8_t devName_len;
}ClientConnectContext_t;

typedef struct
{
  /**
   * device mac address
   */
  tBDAddr macAddress;

  /**
   * device name
   */
  uint8_t devName[20];
  /**
   * device name length
   */
  uint8_t devName_len;
}ClientDiscoveryContext_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
#define UNPACK_2_BYTE_PARAMETER(ptr)  \
        (uint16_t)((uint16_t)(*((uint8_t *)ptr))) |   \
        (uint16_t)((((uint16_t)(*((uint8_t *)ptr + 1))) << 8))
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

//PLACE_IN_SECTION("BLE_APP_CONTEXT") static ClientConnectContext_t aClientConnectContext[BLE_CFG_CLT_MAX_CON_NBR_CB];
//PLACE_IN_SECTION("BLE_APP_CONTEXT") static ClientDiscoveryContext_t aClientDiscoveryContext[BLE_CFG_CLT_MAX_CON_NBR_CB];
static ClientConnectContext_t aClientConnectContext[BLE_CFG_CLT_MAX_CON_NBR_CB];
static ClientDiscoveryContext_t aClientDiscoveryContext[BLE_CFG_CLT_MAX_DISCOVERY_NBR_CB];
uint8_t discovery_counter = 0;

uint8_t elssen_connect_start = 0;
uint8_t elssen_connect_connected = 0;
uint8_t elssen_check_cnt = 0;

/**
 * END of Section BLE_APP_CONTEXT
 */
/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;
extern APP_BLE_Mode_t BleMode;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void Gatt_Notification(P2P_Client_App_Notification_evt_t *pNotification, uint8_t index);
static SVCCTL_EvtAckStatus_t Client_Event_Handler(void *Event);
int8_t GetClientConIndexByHandle(uint16_t handle);
uint8_t GetDiscoveryDeviceInfoByMac(tBDAddr mac, uint8_t *dev_name);
uint8_t CheckElssenDevice(uint8_t index);

/* USER CODE BEGIN PFP */

void Update_Service( void );

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void P2PC_APP_Init(void)
{
  uint8_t index =0;
/* USER CODE BEGIN P2PC_APP_Init_1 */
  UTIL_SEQ_RegTask( 1<< CFG_TASK_SEARCH_SERVICE_ID, UTIL_SEQ_RFU, Update_Service );

/* USER CODE END P2PC_APP_Init_1 */
  for(index = 0; index < BLE_CFG_CLT_MAX_CON_NBR_CB; index++)
  {
    aClientConnectContext[index].state= APP_CLIENT_IDLE;
  }

  /**
   *  Register the event handler to the BLE controller
   */
  SVCCTL_RegisterCltHandler(Client_Event_Handler);

#ifdef SENSOR_CLIENT
  if(BleMode == APP_BLE_CLIENT)
  {
    Scan_Request();
  }
#endif
  return;
}

void P2PC_APP_Notification(P2P_APP_ConnHandle_Not_evt_t *pNotification)
{
  uint8_t result;
  int8_t index;

  switch(pNotification->P2P_Evt_Opcode)
  {
  case PEER_CONN_HANDLE_EVT :
      /* Connection as client */
      //BleServerApplicationContext.BleApplicationContext_legacy.connectionHandle = connection_complete_event->Connection_Handle;
      //BleServerApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_CLIENT;
      index = 0;
      while((index < BLE_CFG_CLT_MAX_CON_NBR_CB) && (aClientConnectContext[index].state != APP_CLIENT_CONNECTING))
      {
        index++;
      }

      aClientConnectContext[index].connHandle = pNotification->ConnectionHandle;

      result = aci_gatt_disc_all_primary_services(pNotification->ConnectionHandle);
      if (result == BLE_STATUS_SUCCESS)
      {
      }
      else
      {
      }
      break;

    case PEER_DISCON_HANDLE_EVT :
      index = GetClientConIndexByHandle(pNotification->ConnectionHandle);
      if(index != -1)
      {
        if(CheckElssenDevice(index) == 1)
        {
          ElssenDisconnected();
        }
        SendDisconnected(aClientConnectContext[index].macAddress);
        aClientConnectContext[index].state = APP_CLIENT_IDLE;
        aClientConnectContext[index].connHandle = 0;
        aClientConnectContext[index].P2PNotificationCharHdle = 0;
        aClientConnectContext[index].P2PNotificationDescHandle = 0;
        aClientConnectContext[index].P2PServiceHandle = 0;
        memset(aClientConnectContext[index].macAddress, 0, sizeof(tBDAddr));
        memset(aClientConnectContext[index].devName, 0, aClientConnectContext[index].devName_len);
        aClientConnectContext[index].devName_len = 0;
      }
      break;

    default:
      break;
  }
  return;
}

APP_CLIENT_ConnStatus_t APP_BLE_Get_Client_Connection_Status( uint16_t Connection_Handle )
{
  uint8_t index;

  index = 0;
  while(index < BLE_CFG_CLT_MAX_CON_NBR_CB)
  {
    if(aClientConnectContext[index].connHandle == Connection_Handle)
    {
      return aClientConnectContext[index].state;
    }
    index++;
  }

  return APP_CLIENT_IDLE;
}

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Client_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blue_aci *blue_evt;

  P2P_Client_App_Notification_evt_t Notification;

  if(BleMode != APP_BLE_CLIENT)
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

        case EVT_BLUE_ATT_READ_BY_GROUP_TYPE_RESP:
        {
          aci_att_read_by_group_type_resp_event_rp0 *pr = (void*)blue_evt->data;
          uint8_t numServ, i, idx;
          uint16_t uuid, handle;

          uint8_t index;
          handle = pr->Connection_Handle;
          index = 0;
#if 0
          while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                  (aClientConnectContext[index].state != APP_BLE_IDLE))
          {
            APP_CLIENT_ConnStatus_t status;

            status = APP_BLE_Get_Client_Connection_Status(aClientConnectContext[index].connHandle);

            if((aClientConnectContext[index].state == APP_CLIENT_CONNECTED)&&
                    (status == APP_CLIENT_IDLE))
            {
              /* Handle deconnected */

              aClientConnectContext[index].state = APP_CLIENT_IDLE;
              aClientConnectContext[index].connHandle = 0xFFFF;
              break;
            }
            index++;
          }
#endif
          while((index < BLE_CFG_CLT_MAX_CON_NBR_CB) &&
                  (aClientConnectContext[index].state != APP_CLIENT_CONNECTING))
          {
            index++;
          }

          if(index < BLE_CFG_CLT_MAX_CON_NBR_CB)
          {
            aClientConnectContext[index].connHandle= handle;

            numServ = (pr->Data_Length) / pr->Attribute_Data_Length;

            /* the event data will be
             * 2bytes start handle
             * 2bytes end handle
             * 2 or 16 bytes data
             * we are interested only if the UUID is 16 bit.
             * So check if the data length is 6
             */
//#if (UUID_128BIT_FORMAT==1)           
          if (pr->Attribute_Data_Length == 20)
          {
              idx = 16;
              for (i=0; i<numServ; i++)
              {
                uuid = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx]);
                //if(uuid == P2P_SERVICE_UUID)
                if(uuid == P2P_SERVICE_UUID || uuid == 0xABCD) // Elssen
                //if(uuid == P2P_SERVICE_UUID || uuid == 0xF2E0) // BlueNRG F2E0
                //if(aClientConnectContext[index].state == APP_CLIENT_CONNECTING)
                {
                  aClientConnectContext[index].P2PServiceHandle = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx-16]);
                  aClientConnectContext[index].P2PServiceEndHandle = UNPACK_2_BYTE_PARAMETER (&pr->Attribute_Data_List[idx-14]);
                  aClientConnectContext[index].state = APP_CLIENT_DISCOVER_CHARACS ;
                }
                idx += 6;
              }
            }
          }
        }
        break;

        case EVT_BLUE_ATT_READ_BY_TYPE_RESP:
        {

          aci_att_read_by_type_resp_event_rp0 *pr = (void*)blue_evt->data;
          uint8_t idx;
          uint16_t uuid, handle;

          /* the event data will be
           * 2 bytes start handle
           * 1 byte char properties
           * 2 bytes handle
           * 2 or 16 bytes data
           */

          uint8_t index;

          index = 0;
          while(index < BLE_CFG_CLT_MAX_CON_NBR_CB)
          {
            if(aClientConnectContext[index].state == APP_CLIENT_DISCOVER_CHARACS || aClientConnectContext[index].state == APP_CLIENT_DISCOVER_WRITE_DESC)
            {
              break;
            }
            index++;
          }

          if(index < BLE_CFG_CLT_MAX_CON_NBR_CB)
          {

            /* we are interested in only 16 bit UUIDs */
            idx = 17;
            if (pr->Handle_Value_Pair_Length == 21)
            {
              pr->Data_Length -= 1;
              while(pr->Data_Length > 0)
              {
                uuid = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx]);
                /* store the characteristic handle not the attribute handle */
                handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx-14]);

                //if(uuid == P2P_WRITE_CHAR_UUID)
                if(uuid == P2P_WRITE_CHAR_UUID || uuid == 0xFFF6) // Elssen
                //if(aClientConnectContext[index].state == APP_CLIENT_DISCOVER_CHARACS)
                {
                  aClientConnectContext[index].state = APP_CLIENT_DISCOVER_WRITE_DESC;
                  aClientConnectContext[index].P2PWriteToServerCharHdle = handle;
                }
                //else if(uuid == P2P_NOTIFY_CHAR_UUID)
                else if(uuid == P2P_NOTIFY_CHAR_UUID || uuid == 0xFFF7) // Elssen
                //else if(aClientConnectContext[index].state == APP_CLIENT_DISCOVER_WRITE_DESC)
                {
                  aClientConnectContext[index].state = APP_CLIENT_DISCOVER_NOTIFICATION_CHAR_DESC;
                  aClientConnectContext[index].P2PNotificationCharHdle = handle;
                }

#if 0
                aClientConnectContext[index].P2PWriteToServerCharHdle = 17;
                aClientConnectContext[index].P2PNotificationCharHdle = 14;
                aClientConnectContext[index].state = APP_CLIENT_ENABLE_NOTIFICATION_DESC;

                //if(uuid == P2P_WRITE_CHAR_UUID)
                if(uuid == P2P_WRITE_CHAR_UUID || uuid == 0xF2E2) // BluNRG 0xF2E2
                {
                  //aClientConnectContext[index].state = APP_CLIENT_DISCOVER_WRITE_DESC;
                  //aClientConnectContext[index].P2PWriteToServerCharHdle = handle;
                aClientConnectContext[index].state = APP_CLIENT_ENABLE_NOTIFICATION_DESC;

                }

                //else if(uuid == P2P_NOTIFY_CHAR_UUID)
                else if(uuid == P2P_NOTIFY_CHAR_UUID || uuid == 0xF2E1) // BlueNRG 0xF2E1
                {
                  //aClientConnectContext[index].state = APP_CLIENT_DISCOVER_NOTIFICATION_CHAR_DESC;
                  aClientConnectContext[index].state = APP_CLIENT_ENABLE_NOTIFICATION_DESC;
                  //aClientConnectContext[index].P2PNotificationCharHdle = handle;
                }
#endif

                pr->Data_Length -= 21;
                idx += 21;
              }
            }
          }
        }
        break;

        case EVT_BLUE_ATT_FIND_INFORMATION_RESP:
        {
          aci_att_find_info_resp_event_rp0 *pr = (void*)blue_evt->data;

          uint8_t numDesc, idx, i;
          uint16_t uuid, handle;

          /*
           * event data will be of the format
           * 2 bytes handle
           * 2 bytes UUID
           */

          uint8_t index;

          index = 0;
          while((index < BLE_CFG_CLT_MAX_CON_NBR_CB) &&
                  (aClientConnectContext[index].connHandle != pr->Connection_Handle))

            index++;

          if(index < BLE_CFG_CLT_MAX_CON_NBR_CB)
          {

            numDesc = (pr->Event_Data_Length) / 4;
            /* we are interested only in 16 bit UUIDs */
            idx = 0;
            if (pr->Format == UUID_TYPE_16)
            {
              for (i=0; i<numDesc; i++)
              {
                handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_UUID_Pair[idx]);
                uuid = UNPACK_2_BYTE_PARAMETER(&pr->Handle_UUID_Pair[idx+2]);

                if(uuid == CLIENT_CHAR_CONFIG_DESCRIPTOR_UUID)
                {
                  if( aClientConnectContext[index].state == APP_CLIENT_DISCOVER_NOTIFICATION_CHAR_DESC)
                  {
                    aClientConnectContext[index].P2PNotificationDescHandle = handle;
                    aClientConnectContext[index].state = APP_CLIENT_ENABLE_NOTIFICATION_DESC;

                  }
                }
                idx += 4;
              }
            }
          }
        }
        break; /*EVT_BLUE_ATT_FIND_INFORMATION_RESP*/

        case EVT_BLUE_GATT_NOTIFICATION:
        {
          aci_gatt_notification_event_rp0 *pr = (void*)blue_evt->data;
          uint8_t index;

          index = 0;
          while((index < BLE_CFG_CLT_MAX_CON_NBR_CB) &&
                  (aClientConnectContext[index].connHandle != pr->Connection_Handle))
            index++;

          if(index < BLE_CFG_CLT_MAX_CON_NBR_CB)
          {

            if ( (pr->Attribute_Handle == aClientConnectContext[index].P2PNotificationCharHdle))
            {

              Notification.P2P_Client_Evt_Opcode = P2P_NOTIFICATION_INFO_RECEIVED_EVT;
              Notification.DataTransfered.Length = pr->Attribute_Value_Length;
              Notification.DataTransfered.pPayload = &pr->Attribute_Value[0];

              Gatt_Notification(&Notification, index);

              /* INFORM APPLICATION BUTTON IS PUSHED BY END DEVICE */

            }
          }
        }
        break;/* end EVT_BLUE_GATT_NOTIFICATION */

        case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
        {
          aci_gatt_proc_complete_event_rp0 *pr = (void*)blue_evt->data;

          uint8_t index;

          index = 0;
          while((index < BLE_CFG_CLT_MAX_CON_NBR_CB) &&
                  (aClientConnectContext[index].connHandle != pr->Connection_Handle))
            index++;

          if(index < BLE_CFG_CLT_MAX_CON_NBR_CB)
          {

            UTIL_SEQ_SetTask( 1<<CFG_TASK_SEARCH_SERVICE_ID, CFG_SCH_PRIO_0);

          }
        }
        break; /*EVT_BLUE_GATT_PROCEDURE_COMPLETE*/
        default:
          break;
      }
    }

    break; /* HCI_EVT_VENDOR_SPECIFIC */

    default:
      break;
  }

  return(return_value);
}/* end BLE_CTRL_Event_Acknowledged_Status_t */

void Gatt_Notification(P2P_Client_App_Notification_evt_t *pNotification, uint8_t index)
{
  switch(pNotification->P2P_Client_Evt_Opcode)
  {
    case P2P_NOTIFICATION_INFO_RECEIVED_EVT:
      // check from Elssen device
      if(CheckElssenDevice(index) == TRUE)
      {
        // pass data to hi
        HAL_UART_Transmit(&huart1, pNotification->DataTransfered.pPayload, pNotification->DataTransfered.Length, 1000);
      }
      break;

    default:
      break;
  }
  return;
}

int8_t GetClientIdleIndex()
{
    uint8_t index =0;
 
    for(index = 0; index < BLE_CFG_CLT_MAX_CON_NBR_CB; index++)
    {
        if(aClientConnectContext[index].state == APP_CLIENT_IDLE)
        {
          return index;
        }
    }

    return (-1);
}

int8_t GetClientAddressIndex(tBDAddr bdAddr)
{
    uint8_t index =0;
 
    for(index = 0; index < BLE_CFG_CLT_MAX_CON_NBR_CB; index++)
    {
        if(memcmp(aClientConnectContext[index].macAddress , bdAddr, sizeof(tBDAddr)) == 0)
        {
          return index;
        }
    }

    return (-1);
}

int8_t GetElssenDiscoveryIndex(void)
{
  uint8_t index;
  uint8_t elssen_name[6] = {'E', 'L', '-', 'P', '-', 'T'};

  for(index = 0; index < discovery_counter; index++)
  {
    if(memcmp(aClientDiscoveryContext[index].devName, elssen_name, sizeof(elssen_name)) == 0)
    {
      return index;
    }
  }

  return (-1);
}

uint8_t CheckElssenDevice(uint8_t index)
{
    uint8_t elssen_name[6] = {'E', 'L', '-', 'P', '-', 'T'};
 
    if(memcmp(aClientConnectContext[index].devName, elssen_name, sizeof(elssen_name)) == 0)
    {
      return TRUE;
    }

    return FALSE;
}



void Connect_Request( tBDAddr connectAddr )
{
  tBleStatus result;
  int8_t index =0;

  if (BleMode == APP_BLE_CLIENT)
  {
    index = GetClientIdleIndex();
    if(index == -1)
    {
      return;
    }

    result = aci_gap_create_connection(SCAN_P,
                                       SCAN_L,
                                       PUBLIC_ADDR, connectAddr,
                                       PUBLIC_ADDR,
                                       CONN_P1,
                                       CONN_P2,
                                       0,
                                       SUPERV_TIMEOUT,
                                       CONN_L1,
                                       CONN_L2);

    if (result == BLE_STATUS_SUCCESS)
    {
      aClientConnectContext[index].state = APP_CLIENT_CONNECTING;
      memcpy(aClientConnectContext[index].macAddress, connectAddr, sizeof(tBDAddr));
    }
    else
    {
      aClientConnectContext[index].state = APP_CLIENT_IDLE;
    }
  }
  return;
}

void Disconnect_Request( tBDAddr disconnectAddr )
{
  tBleStatus result;
  int8_t index =0;

  if(BleMode == APP_BLE_CLIENT)
  {
    index = GetClientAddressIndex(disconnectAddr);
    if(index == -1)
    {
      return;
    }

    result = aci_gap_terminate(aClientConnectContext[index].connHandle, 0x13); // 0x13 : user terminate

    if (result == BLE_STATUS_SUCCESS)
    {
      aClientConnectContext[index].state = APP_CLIENT_IDLE;
    }
  }
  return;
}

void Scan_Request( void )
{
  tBleStatus result;
  //if(P2P_Client_APP_Get_State() == APP_CLIENT_IDLE)
  {
    //result = aci_gap_start_general_discovery_proc(SCAN_P, SCAN_L, PUBLIC_ADDR, 1);
    result = aci_gap_start_general_discovery_proc(SCAN_P, SCAN_L, PUBLIC_ADDR, 1);
    if (result == BLE_STATUS_SUCCESS)
    {
      discovery_counter = 0;
    //uart_printf(" \r\n\r** START GENERAL DISCOVERY (SCAN) **  \r\n\r");
    }
    else
    {
    //uart_printf("-- BLE_App_Start_Limited_Disc_Req, Failed %d \r\n\r", result);
    }
  }


#ifdef SENSOR_CLIENT
  if(BleMode == APP_BLE_CLIENT && elssen_connect_connected == 0)
  {
    setTimer(Timer_Scan_Canecl, 1000, TIME_RESET);
  }
#endif
  return;
}

void Scan_Stop( void )
{
  tBleStatus result;
  //if (BleServerApplicationContext.Device_Connection_Status == APP_BLE_SCAN)
  {
    result = aci_gap_terminate_gap_proc(GAP_GENERAL_DISCOVERY_PROC);
    if (result == BLE_STATUS_SUCCESS)
    {
    //uart_printf(" \r\n\r** START GENERAL DISCOVERY (SCAN) **  \r\n\r");
    }
    else
    {
    //uart_printf("-- BLE_App_Start_Limited_Disc_Req, Failed %d \r\n\r", result);
    }
  }
  return;
}

void SendConnected(tBDAddr mac)
{
#if 0
  ToHi tohi = TO_HI__INIT;
  uint8_t mac_str[20] = {0};
  uint8_t name_str[20] = {0};

  MacAddressString(mac_str, mac);
  GetDiscoveryDeviceInfoByMac(mac, name_str);

  tohi.has_connected = 1;
  tohi.connected= 1;
  tohi.connected_mac = (char *) mac_str;
  tohi.dev_name = (char *) name_str;

  pushToHi(&huart1, &tohi);
#endif
}

uint8_t P2P_Client_APP_Get_State( void ) {
  uint8_t index;

  index = 0;
  while(index < BLE_CFG_CLT_MAX_CON_NBR_CB)
  {
    if(aClientConnectContext[index].state != APP_CLIENT_IDLE)
    {
      return aClientConnectContext[index].state;
    }
    index++;
  }

  return APP_CLIENT_IDLE;
}

uint8_t GetDiscoveryDeviceInfoByMac(tBDAddr mac, uint8_t *dev_name)
{
  for(uint8_t index = 0; index < discovery_counter; index++)
  {
    if(memcmp(aClientDiscoveryContext[index].macAddress, mac, sizeof(tBDAddr)) == 0)
    {
      memcpy(dev_name, aClientDiscoveryContext[index].devName, aClientDiscoveryContext[index].devName_len);
      return aClientDiscoveryContext[index].devName_len;
    }
  }

  return 0;
}

uint8_t FilterByName(uint8_t *name, uint8_t name_len)
{
  uint8_t forceMeterName[20] = {'F', 'O', 'R', 'C', 'E', 'M', 'E'};
  uint8_t elssenDeviceName[20] = {'E', 'L', '-', 'P', '-', 'T'};

  if(memcmp(forceMeterName, name, name_len) == 0)
  {
    return 1;
  }

  if(memcmp(elssenDeviceName, name, name_len) == 0)
  {
    return 1;
  }

  return 0;
}

int8_t IsInDiscoveryDeviceList(tBDAddr mac)
{
  for(uint8_t index = 0; index < discovery_counter; index++)
  {
    if(memcmp(aClientDiscoveryContext[index].macAddress, mac, sizeof(tBDAddr)) == 0)
    {
      return index;
    }
  }

  return -1;
}

void AddDiscoveryDeviceList(tBDAddr mac, uint8_t *name, uint8_t name_len)
{
  if(IsInDiscoveryDeviceList(mac) == -1 && FilterByName(name, name_len) == 1)
  {
    memcpy(aClientDiscoveryContext[discovery_counter].macAddress, mac, sizeof(tBDAddr));
    memcpy(aClientDiscoveryContext[discovery_counter].devName, name, name_len);    
    aClientDiscoveryContext[discovery_counter].devName_len = name_len;
    discovery_counter++;
  }
}

void ElssenAutoConnectionStart(void)
{
#ifdef SENSOR_CLIENT
  int8_t elssen_index;

  if(BleMode != APP_BLE_CLIENT)
    return;

  elssen_index = GetElssenDiscoveryIndex();

  if(elssen_index == (-1))
  {
    Scan_Request();
  }
  else
  {
    Connect_Request(aClientDiscoveryContext[elssen_index].macAddress);
    elssen_connect_start = 1;
    elssen_connect_connected = 0;
    elssen_check_cnt = 0;
    setTimer(Timer_Elssen_connection_check, 1000, TIME_RESET);
  }
#endif
}

void ElssenConnected(void)
{
#ifdef SENSOR_CLIENT
  if(BleMode == APP_BLE_CLIENT && elssen_connect_start == 1)
  {
    elssen_connect_start = 0;
    elssen_connect_connected = 1;
    Scan_Request();
    killTimer(Timer_Elssen_connection_check);
  }
#endif
}

void ElssenConnectStop(void)
{
  tBleStatus result;
  if(elssen_connect_start == 1)
  {
    result = aci_gap_terminate_gap_proc(GAP_GENERAL_CONNECTION_ESTABLISHMENT_PROC);
    if (result == BLE_STATUS_SUCCESS)
    {
    //uart_printf(" \r\n\r** START GENERAL DISCOVERY (SCAN) **  \r\n\r");
    }
    else
    {
    //uart_printf("-- BLE_App_Start_Limited_Disc_Req, Failed %d \r\n\r", result);
    }
    elssen_connect_start = 0;
    aClientConnectContext[0].state = APP_CLIENT_IDLE;
    Scan_Request();
  }
  else if(elssen_connect_connected == 1)
  {
    //ElssenDisconnect();
    Scan_Request();
  }
  else
  {
    Scan_Request();
  }
}

void ElssenDisconnect(void)
{
#if 0

#ifdef SENSOR_CLIENT
  int8_t elssen_index;
  
  elssen_index = GetElssenDiscoveryIndex();

  if(elssen_index == (-1))
  {
    Scan_Request();
  }
  else
  {
    Disconnect_Request(aClientDiscoveryContext[elssen_index].macAddress);
    elssen_connect_start = 0;
    elssen_connect_connected = 0;
    elssen_check_cnt = 0;
    setTimer(Timer_Elssen_connection_check, 1000, TIME_RESET);
  }
#endif
#endif

}

void ElssenDisconnected(void)
{
#ifdef SENSOR_CLIENT
  if(BleMode == APP_BLE_CLIENT)
  {
    killTimer(Timer_Elssen_connection_check);
    elssen_connect_connected = 0;
    Scan_Request();
  }
#endif
}


/**
 * @brief  Feature Characteristic update
 * @param  pFeatureValue: The address of the new value to be written
 * @retval None
 */
#if 1
tBleStatus Write_Char(uint8_t payload_length, uint8_t *pPayload, uint8_t index)// Client to server
{

  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;


  ret =aci_gatt_write_without_resp(aClientConnectContext[index].connHandle,
                             aClientConnectContext[index].P2PWriteToServerCharHdle,
                             payload_length, /* charValueLen */
                             (uint8_t *)  pPayload);

  return ret;
}/* end Write_Char() */
#endif

void Update_Service()
{
  uint16_t enable = 0x0001;

  HAL_UART_Transmit(&huart1, "Update Service\r\n", 16, 1000);

  uint8_t index;

  index = 0;
  while(index < BLE_CFG_CLT_MAX_CON_NBR_CB)
  {


    switch(aClientConnectContext[index].state)
    {

      case APP_CLIENT_DISCOVER_SERVICES:
        break;
      case APP_CLIENT_DISCOVER_CHARACS:
        aci_gatt_disc_all_char_of_service(aClientConnectContext[index].connHandle,
                                          aClientConnectContext[index].P2PServiceHandle,
                                          aClientConnectContext[index].P2PServiceEndHandle);

        break;
      case APP_CLIENT_DISCOVER_WRITE_DESC: /* Not Used - No decriptor */
        aci_gatt_disc_all_char_desc(aClientConnectContext[index].connHandle,
                                    aClientConnectContext[index].P2PWriteToServerCharHdle,
                                    aClientConnectContext[index].P2PWriteToServerCharHdle+2);

        break;
      case APP_CLIENT_DISCOVER_NOTIFICATION_CHAR_DESC:
        aci_gatt_disc_all_char_desc(aClientConnectContext[index].connHandle,
                                    aClientConnectContext[index].P2PNotificationCharHdle,
                                    aClientConnectContext[index].P2PNotificationCharHdle+2);

        break;
      case APP_CLIENT_ENABLE_NOTIFICATION_DESC:
        aci_gatt_write_char_desc(aClientConnectContext[index].connHandle,
                                 aClientConnectContext[index].P2PNotificationDescHandle,
                                 2,
                                 (uint8_t *)&enable);
        //aci_gatt_write_char_desc(aClientConnectContext[index].connHandle,
          //                       aClientConnectContext[index].P2PNotificationCharHdle + 1,
            //                     2,
              //                   (uint8_t *)&enable);

        aClientConnectContext[index].state = APP_CLIENT_CONNECTED;
        aClientConnectContext[index].devName_len = GetDiscoveryDeviceInfoByMac(aClientConnectContext[index].macAddress, aClientConnectContext[index].devName);
        SendConnected(aClientConnectContext[index].macAddress);
        if(CheckElssenDevice(index) == 1)
        {
          ElssenConnected();
        }
        break;
      case APP_CLIENT_DISABLE_NOTIFICATION_DESC :
        aci_gatt_write_char_desc(aClientConnectContext[index].connHandle,
                                 aClientConnectContext[index].P2PNotificationDescHandle,
                                 2,
                                 (uint8_t *)&enable);

        aClientConnectContext[index].state = APP_CLIENT_CONNECTED;
        //SendConnected(aClientConnectContext[index].macAddress);
        break;
      default:
        break;
    }
    index++;
  }
  return;
}

/* USER CODE BEGIN LF */

int8_t GetClientConIndexByHandle(uint16_t handle)
{
  uint8_t index = 0;

  if(index < BLE_CFG_CLT_MAX_CON_NBR_CB)
  {
    if(aClientConnectContext[index].connHandle == handle)
    {
      return index;
    }
  }

  return -1;
}

/* USER CODE END LF */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
