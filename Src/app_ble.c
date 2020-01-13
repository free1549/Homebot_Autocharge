/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * File Name          : app_ble.c
  * Description        : Application file for BLE Middleware.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "app_common.h"

#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include "app_ble.h"

#include "stm32_seq.h"
#include "shci.h"
#include "stm32_lpm.h"
#include "p2p_server_app.h"
#include "p2p_client_app.h"
#include "simple.pb-c.h"
#include "hi2st-parser.h"
#include "sensor.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
APP_BLE_Mode_t BleMode = APP_BLE_SERVER; // APP_BLE_SERVER, APP_BLE_CLIENT

#define APPBLE_GAP_DEVICE_NAME_LENGTH 7
#define FAST_ADV_TIMEOUT               (30*1000*1000/CFG_TS_TICK_VAL) /**< 30s */
#define INITIAL_ADV_TIMEOUT            (60*1000*1000/CFG_TS_TICK_VAL) /**< 60s */

#define BD_ADDR_SIZE_LOCAL    6

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

static const uint8_t M_bd_addr[BD_ADDR_SIZE_LOCAL] =
    {
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000000000FF)),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00000000FF00) >> 8),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x000000FF0000) >> 16),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000FF000000) >> 24),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00FF00000000) >> 32),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0xFF0000000000) >> 40)
    };

static uint8_t bd_addr_udn[BD_ADDR_SIZE_LOCAL];

/**
*   Identity root key used to derive LTK and CSRK 
*/
static const uint8_t BLE_CFG_IR_VALUE[16] = CFG_BLE_IRK;

/**
* Encryption root key used to derive LTK and CSRK
*/
static const uint8_t BLE_CFG_ER_VALUE[16] = CFG_BLE_ERK;


tBDAddr SERVER_REMOTE_BDADDR;

//PLACE_IN_SECTION("BLE_APP_CONTEXT") static BleServerApplicationContext_t BleServerApplicationContext;
//PLACE_IN_SECTION("BLE_APP_CONTEXT") static uint16_t AdvIntervalMin, AdvIntervalMax;
extern BleServerApplicationContext_t BleServerApplicationContext;
extern uint16_t AdvIntervalMin, AdvIntervalMax;
extern uint8_t *server_bd_addr;



P2P_APP_ConnHandle_Not_evt_t handleNotification;
APP_BLE_ConnStatus_t BleAppContextStatus;

HCI_TL_CmdStatus_t BleHCICmdStatus;

#if L2CAP_REQUEST_NEW_CONN_PARAM != 0
#define SIZE_TAB_CONN_INT            2
float tab_conn_interval[SIZE_TAB_CONN_INT] = {50, 1000} ; /* ms */
uint8_t index_con_int, mutex; 
#endif 


/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void BLE_UserEvtRx( void * pPayload );
static void BLE_StatusNot( HCI_TL_CmdStatus_t status );
static void Ble_Tl_Init( void );
static void Ble_Hci_Gap_Gatt_Init(void);
static const uint8_t* BleGetBdAddress( void );

#if(L2CAP_REQUEST_NEW_CONN_PARAM != 0)  
static void BLE_SVC_L2CAP_Conn_Update(uint16_t Connection_Handle);
#endif

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void APP_BLE_Init( void )
{
/* USER CODE BEGIN APP_BLE_Init_1 */

/* USER CODE END APP_BLE_Init_1 */
  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
  {
    {{0,0,0}},                          /**< Header unused */
    {0,                                 /** pBleBufferAddress not used */
    0,                                  /** BleBufferSize not used */
    CFG_BLE_NUM_GATT_ATTRIBUTES,
    CFG_BLE_NUM_GATT_SERVICES,
    CFG_BLE_ATT_VALUE_ARRAY_SIZE,
    CFG_BLE_NUM_LINK,
    CFG_BLE_DATA_LENGTH_EXTENSION,
    CFG_BLE_PREPARE_WRITE_LIST_SIZE,
    CFG_BLE_MBLOCK_COUNT,
    CFG_BLE_MAX_ATT_MTU,
    CFG_BLE_SLAVE_SCA,
    CFG_BLE_MASTER_SCA,
    CFG_BLE_LSE_SOURCE,
    CFG_BLE_MAX_CONN_EVENT_LENGTH,
    CFG_BLE_HSE_STARTUP_TIME,
    CFG_BLE_VITERBI_MODE,
    CFG_BLE_LL_ONLY,
    0}
  };

  /**
   * Initialize Ble Transport Layer
   */
  Ble_Tl_Init( );

  /**
   * Do not allow standby in the application
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

  /**
   * Register the hci transport layer to handle BLE User Asynchronous Events
   */
  UTIL_SEQ_RegTask( 1<<CFG_TASK_HCI_ASYNCH_EVT_ID, UTIL_SEQ_RFU, hci_user_evt_proc);

  /**
   * Starts the BLE Stack on CPU2
   */
  SHCI_C2_BLE_Init( &ble_init_cmd_packet );

  /**
   * Initialization of HCI & GATT & GAP layer
   */
  Ble_Hci_Gap_Gatt_Init();

  /**
   * Initialization of the BLE Services
   */
  SVCCTL_Init();

  /**
   * From here, all initialization are BLE application specific
   */
  /**
   * Initialization of ADV - Ad Manufacturer Element - Support OTA Bit Mask
   */
  /**
   * Initialization of the BLE App Context
   */

#if(RADIO_ACTIVITY_EVENT != 0)  
  aci_hal_set_radio_activity_mask(0x0006);
#endif  
  
#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0 )
  index_con_int = 0; 
  mutex = 1; 
#endif
  /**
   * Initialize P2P Server Application
   */
  P2PS_APP_Init();
  P2PC_APP_Init();

  /**
   * Start scanning
   */
  //Scan_Request();

/* USER CODE BEGIN APP_BLE_Init_2 */

/* USER CODE END APP_BLE_Init_2 */
  return;
}

void printMacAddress(uint8_t *mac)
{
  //uart_printf("Mac : ");

  for(uint8_t i = 0; i < 6; i++)
  {
    //uart_printf("%02X:", mac[5 - i]);
  }
  //uart_printf(" ");
}

void MacAddressString(uint8_t *mac_str, tBDAddr addr)
{
  uint8_t pos = 0;
  uint8_t temp;

  //sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
  for(int8_t i = 5; i >= 0; i--)
  {
    temp = addr[i] >> 4;
    if(temp > 9)
    {
      temp -= 10;
      temp += 'A';
    }
    else
    {
      temp += '0';
    }
    mac_str[pos++] = temp;
    temp = addr[i] & 0x0F;
    if(temp > 9)
    {
      temp -= 10;
      temp += 'A';
    }
    else
    {
      temp += '0';
    }
    mac_str[pos++] = temp;
    if(i != 0)
    {
      mac_str[pos++] = ':';
    }
  }
}

void MacAddressStringToBytes(uint8_t *mac_str, uint8_t *addr)
{
  uint8_t len = 16;
  uint8_t temp;

  //sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
  for(int8_t i = 0; i < 6; i++)
  {
    temp = 0;
    if(mac_str[len] >= 'A')
    {
      temp = mac_str[len] - 'A';
      temp += 10;
    }
    else
    {
      temp = mac_str[len] - '0';
    }

    addr[i] = temp;
    len--;
 
    if(mac_str[len] >= 'A')
    {
      temp = mac_str[len] - 'A';
      temp += 10;
    }
    else
    {
      temp = mac_str[len] - '0';
    }
    addr[i] += temp << 4;
    len--;
    len--; // :
  }
}

#if 0
int32_t GetRSSI()
{
  uint8_t rssi_level[3];
  int32_t rsi_dbm;
  uint32_t rssi_int16 ;
  uint32_t reg_agc;

  aci_hal_read_raw_rssi(rssi_level);

  // extract the data rssi_int16 + agc
  rssi_int16 = ((rssi_level[0]) >> 16) & 0xFFFFU ;
  reg_agc = (rssi_level[1]) & 0xffU;

  // check if rssi is too low
  if((rssi_int16 == 0U) || (reg_agc > 0xbU))
  {
    rsi_dbm = 127 ;
  }
  else
  {
    rsi_dbm = (int32_t)reg_agc * 6 - 127 ;
    while(rssi_int16 > 30U)
    {
      rsi_dbm += 6 ;
      rssi_int16 = (rssi_int16 >> 1) ;
    }
    rsi_dbm += (int32_t)(uint32_t)((417U*rssi_int16 + 18080U)>>10) ;
  }

  return rsi_dbm ;  

}
#endif

uint8_t GetRSSIProcess(uint8_t *data_pos)
{
  Advertising_Report_t Advertising_Report[1];

  //int i;

  //for (i = 0; i < rp0->Num_Reports; i++) 
  {
    data_pos += 1;
    Osal_MemCpy( &Advertising_Report[0], data_pos, 9 );
    Advertising_Report[0].Data = &data_pos[9];
    data_pos += 9 + data_pos[8];
    Advertising_Report[0].RSSI = data_pos[0];
    Advertising_Report[0].RSSI = 255 - Advertising_Report[0].RSSI;
  }

  return Advertising_Report[0].RSSI;  

}

void SendScanData(tBDAddr mac, uint8_t * name, uint8_t rssi)
{
#if 0
  ToHi tohi = TO_HI__INIT;

  uint8_t mac_str[20] = {0};
  MacAddressString(mac_str, mac);

  tohi.mac_address = (char *)mac_str;
  tohi.dev_name = (char *)name;
  tohi.has_dev_rssi = 1;
  tohi.dev_rssi = rssi;
  //tohi.dev_rssi = abs(GetRSSI());

  pushToHi(&huart1, &tohi);
#endif
}

void SendDisconnected(tBDAddr mac)
{
#if 0
  ToHi tohi = TO_HI__INIT;
  uint8_t mac_str[20] = {0};

  MacAddressString(mac_str, mac);

  tohi.has_disconnected = 1;
  tohi.disconnected = 1;
  tohi.disconnected_mac = (char *) mac_str;

  pushToHi(&huart1, &tohi);
#endif
}

SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification( void *pckt )
{
  hci_event_pckt *event_pckt;
  evt_le_meta_event *meta_evt;
  hci_le_connection_complete_event_rp0 *connection_complete_event;
  evt_blue_aci *blue_evt;
  hci_le_advertising_report_event_rp0 * le_advertising_event;
  event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) pckt)->data;
  uint8_t event_type, event_data_size;
  int k = 0;
  uint8_t adtype, adlength;
  hci_le_phy_update_complete_event_rp0 *evt_le_phy_update_complete; 
  uint8_t TX_PHY, RX_PHY;
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  switch (event_pckt->evt)
  {
    case EVT_VENDOR:
    {
      handleNotification.P2P_Evt_Opcode = PEER_DISCON_HANDLE_EVT;
      blue_evt = (evt_blue_aci*) event_pckt->data;
      /* USER CODE BEGIN EVT_VENDOR */

      /* USER CODE END EVT_VENDOR */
      switch (blue_evt->ecode)
      {
      /* USER CODE BEGIN ecode */

      /* USER CODE END ecode */

        case EVT_BLUE_GAP_PROCEDURE_COMPLETE:
        {
        /* USER CODE BEGIN EVT_BLUE_GAP_PROCEDURE_COMPLETE */

        /* USER CODE END EVT_BLUE_GAP_PROCEDURE_COMPLETE */
          aci_gap_proc_complete_event_rp0 *gap_evt_proc_complete = (void*) blue_evt->data;
          /* CHECK GAP GENERAL DISCOVERY PROCEDURE COMPLETED & SUCCEED */
          if (gap_evt_proc_complete->Procedure_Code == GAP_GENERAL_DISCOVERY_PROC
              && gap_evt_proc_complete->Status == 0x00)
          {
              /* USER CODE BEGIN GAP_GENERAL_DISCOVERY_PROC */
              ElssenAutoConnectionStart();

              /* USER CODE END GAP_GENERAL_DISCOVERY_PROC */
            //uart_printf("-- GAP GENERAL DISCOVERY PROCEDURE_COMPLETED\n");
            //Scan_Request();
          }
          else if (gap_evt_proc_complete->Procedure_Code == GAP_GENERAL_CONNECTION_ESTABLISHMENT_PROC
              && gap_evt_proc_complete->Status == 0x00)
          {
            // elssen connection failed then scan again
            Scan_Request();
          }
        }
        break;
        default:
          /* USER CODE BEGIN ecode_default */

          /* USER CODE END ecode_default */
          break;
       
      }
    }
    break; 

    case EVT_DISCONN_COMPLETE:
    {
      hci_disconnection_complete_event_rp0 *disconnection_complete_event;
      disconnection_complete_event = (hci_disconnection_complete_event_rp0 *) event_pckt->data;

      handleNotification.P2P_Evt_Opcode = PEER_DISCON_HANDLE_EVT;
      handleNotification.ConnectionHandle = disconnection_complete_event->Connection_Handle;
      if(BleMode == APP_BLE_SERVER)
      {
        P2PS_APP_Notification(&handleNotification);
      }
      else
      {
        P2PC_APP_Notification(&handleNotification);
      }
    }
    break; /* EVT_DISCONN_COMPLETE */

    case EVT_LE_META_EVENT:
      {
        meta_evt = (evt_le_meta_event*) event_pckt->data;
        switch (meta_evt->subevent)
        {
          case EVT_LE_CONN_UPDATE_COMPLETE: 
            break;
          case EVT_LE_PHY_UPDATE_COMPLETE:
            evt_le_phy_update_complete = (hci_le_phy_update_complete_event_rp0*)meta_evt->data;
            if (evt_le_phy_update_complete->Status == 0)
            {
              //uart_printf("EVT_UPDATE_PHY_COMPLETE, status ok \n");

            }
            else
            {
              //uart_printf("EVT_UPDATE_PHY_COMPLETE, status nok \n");

            }
            ret = hci_le_read_phy(BleServerApplicationContext.BleApplicationContext_legacy.connectionHandle,&TX_PHY,&RX_PHY);
            if (ret == BLE_STATUS_SUCCESS)
            {
              if ((TX_PHY == TX_2M) && (RX_PHY == RX_2M))
              {
                //uart_printf("PHY Param  TX= %d, RX= %d \n", TX_PHY, RX_PHY);

              }
              else
              {
                //uart_printf("PHY Param  TX= %d, RX= %d \n", TX_PHY, RX_PHY);

              } 
            }
            else
            {
              //uart_printf("Read conf not succeess \n");

            }
            break;
          case EVT_LE_CONN_COMPLETE:
            {
              /**
                       * The connection is done, there is no need anymore to schedule the LP ADV
                       */
              connection_complete_event = (hci_le_connection_complete_event_rp0 *) meta_evt->data;
            
              //if (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_CONNECTING)
              {
                handleNotification.P2P_Evt_Opcode = PEER_CONN_HANDLE_EVT;
                handleNotification.ConnectionHandle = connection_complete_event->Connection_Handle;

                if(BleMode == APP_BLE_SERVER)
                {
                  P2PS_APP_Notification(&handleNotification);
                }
                else
                {
                  P2PC_APP_Notification(&handleNotification);
                }

              }
            }
          break; /* HCI_EVT_LE_CONN_COMPLETE */

          case EVT_LE_ADVERTISING_REPORT:
          {
            uint8_t *adv_report_data;
            uint8_t adv_rssi;
            le_advertising_event = (hci_le_advertising_report_event_rp0 *) meta_evt->data;

            event_type = le_advertising_event->Advertising_Report[0].Event_Type;

            event_data_size = le_advertising_event->Advertising_Report[0].Length_Data;

            adv_report_data = (uint8_t*)(&le_advertising_event->Advertising_Report[0].Length_Data) + 1;
            k = 0;

            adv_rssi = adv_report_data[event_data_size]; // next byte of data end
            adv_rssi = 255 - adv_rssi;

            if(event_data_size == 0)
            {
              return SVCCTL_UserEvtFlowEnable;
            }

            /* search AD TYPE 0x09 (Complete Local Name) */
            /* search AD Type 0x02 (16 bits UUIDS) */
            if (event_type == ADV_IND || event_type == SCAN_RSP)
            {

              /* ISOLATION OF BD ADDRESS AND LOCAL NAME */

              while(k < event_data_size)
              {
                adlength = adv_report_data[k];
                adtype = adv_report_data[k + 1];
                switch (adtype)
                {
                  case AD_TYPE_FLAGS: /* now get flags */
                    break;

                  case AD_TYPE_TX_POWER_LEVEL: /* Tx power level */
                  break;
                  case AD_TYPE_MANUFACTURER_SPECIFIC_DATA: /* Manufacturer Specific */
                    if (adv_report_data[k + 2] == 'V' && adv_report_data[k + 3] == 'A') // Check Company ID
                    {
                      SENSOR_Data_t sensorData;

                      sensorData.bInterrupt = adv_report_data[k + SENSORINTERRUPT_MANU_POS];
                      sensorData.sensortype = adv_report_data[k + SENSORTYPE_MANU_POS];
                      sensorData.data1 = adv_report_data[k + SENSORDATA1_MANU_POS1] << 8 | adv_report_data[k + SENSORDATA1_MANU_POS2];
                      sensorData.data2 = adv_report_data[k + SENSORDATA2_MANU_POS1] << 8 | adv_report_data[k + SENSORDATA2_MANU_POS2];
                      sensorData.data3 = adv_report_data[k + SENSORDATA3_MANU_POS1] << 8 | adv_report_data[k + SENSORDATA3_MANU_POS2];
                      sensorData.data4 = adv_report_data[k + SENSORDATA4_MANU_POS1] << 8 | adv_report_data[k + SENSORDATA4_MANU_POS2];
                      sensorData.data4 = adv_report_data[k + SENSORDATA4_MANU_POS1] << 8 | adv_report_data[k + SENSORDATA4_MANU_POS2];
                      sensorData.voltage = adv_report_data[k + SENSORBATTERY_MANU_POS];
                      sensorData.bNewData1 = 1;
                      sensorData.bNewData2 = 1;
                      sensorData.bNewData3 = 1;
                      sensorData.bNewData4 = 1;
                      memcpy(sensorData.macAddress, le_advertising_event->Advertising_Report[0].Address, sizeof(tBDAddr));

                      AddSensorDataList(&sensorData);
                    }
                    break;
                  case AD_TYPE_SERVICE_DATA: /* service data 16 bits */
                    break;
                  case AD_TYPE_COMPLETE_LOCAL_NAME:
                    {
                      uint8_t dev_name[50] = {0};
                      memcpy(dev_name, &adv_report_data[k+2], adlength - 1);

                      //printMacAddress(le_advertising_event->Advertising_Report[0].Address);
                      //uart_printf("name : %s\r\n", dev_name);

                      AddDiscoveryDeviceList(le_advertising_event->Advertising_Report[0].Address, dev_name, adlength - 1);
                      //SendScanData(le_advertising_event->Advertising_Report[0].Address, dev_name, adv_rssi);
                    }
                    break;
                  default:
                    break;
                } /* end switch Data[k+adlength] */
                k += adlength + 1;
              } /* end while */

            } /* end if ADV_IND */
          }

            break;

          default:
            break;
        }
      }
      break; /* HCI_EVT_LE_META_EVENT */
    default:

      break;
  }

  return (SVCCTL_UserEvtFlowEnable);
}

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
static void Ble_Tl_Init( void )
{
  HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;

  Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&BleCmdBuffer;
  Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
  hci_init(BLE_UserEvtRx, (void*) &Hci_Tl_Init_Conf);

  return;
}

 static void Ble_Hci_Gap_Gatt_Init(void){

  uint8_t role;
  uint8_t index;
  uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;
  uint32_t srd_bd_addr[2];
  uint16_t appearance[1] = { BLE_CFG_GAP_APPEARANCE }; 

  /**
   * Initialize HCI layer
   */
  /*HCI Reset to synchronise BLE Stack*/
  hci_reset();

  /**
   * Write the BD Address
   */
  server_bd_addr = (uint8_t *) BleGetBdAddress();
  aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                            CONFIG_DATA_PUBADDR_LEN,
                            (uint8_t*) server_bd_addr);
  
  /**
   * Static random Address
   * The two upper bits shall be set to 1
   * The lowest 32bits is read from the UDN to differentiate between devices
   * The RNG may be used to provide a random number on each power on
   */
  srd_bd_addr[1] =  0x0000ED6E;
  srd_bd_addr[0] =  LL_FLASH_GetUDN( );
  aci_hal_write_config_data( CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, (uint8_t*)srd_bd_addr );

  /**
   * Write Identity root key used to derive LTK and CSRK 
   */
    aci_hal_write_config_data( CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, (uint8_t*)BLE_CFG_IR_VALUE );
    
   /**
   * Write Encryption root key used to derive LTK and CSRK
   */
    aci_hal_write_config_data( CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, (uint8_t*)BLE_CFG_ER_VALUE );

  /**
   * Set TX Power to 0dBm.
   */
  aci_hal_set_tx_power_level(1, CFG_TX_POWER);

  /**
   * Initialize GATT interface
   */
  aci_gatt_init();

  /**
   * Initialize GAP interface
   */
  role = 0;

#if (BLE_CFG_PERIPHERAL == 1)
  role |= GAP_PERIPHERAL_ROLE;
#endif

#if (BLE_CFG_CENTRAL == 1)
  role |= GAP_CENTRAL_ROLE;
#endif

  if (role > 0)
  {
    const char *name = "STM32WB";
    aci_gap_init(role, 0,
                 APPBLE_GAP_DEVICE_NAME_LENGTH,
                 &gap_service_handle, &gap_dev_name_char_handle, &gap_appearance_char_handle);

    if (aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, strlen(name), (uint8_t *) name))
    {
      BLE_DBG_SVCCTL_MSG("Device Name aci_gatt_update_char_value failed.\n");
    }
  }

  if(aci_gatt_update_char_value(gap_service_handle,
                                gap_appearance_char_handle,
                                0,
                                2,
                                (uint8_t *)&appearance))
  {
    BLE_DBG_SVCCTL_MSG("Appearance aci_gatt_update_char_value failed.\n");
  }
  
  BleServerApplicationContext.BleApplicationContext_legacy.gapServiceHandle = gap_service_handle;
  BleServerApplicationContext.BleApplicationContext_legacy.devNameCharHandle = gap_dev_name_char_handle;
  BleServerApplicationContext.BleApplicationContext_legacy.appearanceCharHandle = gap_appearance_char_handle;
  /**
   * Initialize Default PHY
   */
  hci_le_set_default_phy(ALL_PHYS_PREFERENCE,TX_2M_PREFERRED,RX_2M_PREFERRED); 

  /**
   * Initialize IO capability
   */
  BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY;
  aci_gap_set_io_capability(BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);

  /**
   * Initialize authentication
   */
  BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode = CFG_MITM_PROTECTION;
  BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.OOB_Data_Present = 0;
  BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin = 8;
  BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax = 16;
  BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin = 1;
  BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin = 111111;
  BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode = 1;
  for (index = 0; index < 16; index++)
  {
    BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.OOB_Data[index] = (uint8_t) index;
  }

  aci_gap_set_authentication_requirement(BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
                                         BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
                                           0,
                                         0,
                                         BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
                                         BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
                                         BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin,
                                         BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin,
                                         0);

  /**
   * Initialize whitelist
   */
   if (BleServerApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
   {
     aci_gap_configure_whitelist();
   }
}

const uint8_t* BleGetBdAddress( void )
{
  const uint8_t *bd_addr;
  uint32_t udn;
  uint32_t company_id;
  uint32_t device_id;

  udn = LL_FLASH_GetUDN();

  if(udn != 0xFFFFFFFF)
  {
    company_id = LL_FLASH_GetSTCompanyID();
    device_id = LL_FLASH_GetDeviceID();

    bd_addr_udn[0] = (uint8_t)(udn & 0x000000FF);
    bd_addr_udn[1] = (uint8_t)( (udn & 0x0000FF00) >> 8 );
    bd_addr_udn[2] = (uint8_t)( (udn & 0x00FF0000) >> 16 );
    bd_addr_udn[3] = (uint8_t)device_id;
    bd_addr_udn[4] = (uint8_t)(company_id & 0x000000FF);;
    bd_addr_udn[5] = (uint8_t)( (company_id & 0x0000FF00) >> 8 );

    bd_addr = (const uint8_t *)bd_addr_udn;
  }
  else
  {
    bd_addr = M_bd_addr;
  }

  return bd_addr;
}

#if(L2CAP_REQUEST_NEW_CONN_PARAM != 0)  
void BLE_SVC_L2CAP_Conn_Update(uint16_t Connection_Handle)
{
  if(mutex == 1) { 
    mutex = 0;
    index_con_int = (index_con_int + 1)%SIZE_TAB_CONN_INT;
    uint16_t interval_min = CONN_P(tab_conn_interval[index_con_int]);
    uint16_t interval_max = CONN_P(tab_conn_interval[index_con_int]);
    uint16_t slave_latency = L2CAP_SLAVE_LATENCY;
    uint16_t timeout_multiplier = L2CAP_TIMEOUT_MULTIPLIER;
    tBleStatus result;

    result = aci_l2cap_connection_parameter_update_req(BleServerApplicationContext.BleApplicationContext_legacy.connectionHandle,
                                                       interval_min, interval_max,
                                                       slave_latency, timeout_multiplier);
    if( result == BLE_STATUS_SUCCESS )
    {
    }
    else
    {
    }
  }
  return;
}
#endif

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void hci_notify_asynch_evt(void* pdata)
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_HCI_ASYNCH_EVT_ID, CFG_SCH_PRIO_0);
  return;
}

void hci_cmd_resp_release(uint32_t flag)
{
  UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);
  return;
}

void hci_cmd_resp_wait(uint32_t timeout)
{
  UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);
  return;
}

static void BLE_UserEvtRx( void * pPayload )
{
  SVCCTL_UserEvtFlowStatus_t svctl_return_status;
  tHCI_UserEvtRxParam *pParam;

  pParam = (tHCI_UserEvtRxParam *)pPayload; 

  svctl_return_status = SVCCTL_UserEvtRx((void *)&(pParam->pckt->evtserial));
  if (svctl_return_status != SVCCTL_UserEvtFlowDisable)
  {
    pParam->status = HCI_TL_UserEventFlow_Enable;
  }
  else
  {
    pParam->status = HCI_TL_UserEventFlow_Disable;
  }
}

static void BLE_StatusNot( HCI_TL_CmdStatus_t status )
{
  uint32_t task_id_list;

  BleHCICmdStatus = status;

  switch (status)
  {
    case HCI_TL_CmdBusy:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_PauseTask(task_id_list);

      break;

    case HCI_TL_CmdAvailable:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_ResumeTask(task_id_list);    

      break;

    default:
      break;
  }
  return;
}

void SVCCTL_ResumeUserEventFlow( void )
{
  hci_resume_flow();
  return;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
