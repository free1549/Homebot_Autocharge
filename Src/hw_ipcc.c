/**
 ******************************************************************************
  * File Name          : hw_ipcc.c
  * Description        : Hardware IPCC source file for BLE/THREAD 
  *                      middleWare.
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

/* Includes ------------------------------------------------------------------*/
#include "app_common.h"
#include "mbox_def.h"

/* Global variables ---------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define HW_IPCC_TX_PENDING( channel ) ( !(LL_C1_IPCC_IsActiveFlag_CHx( IPCC, channel )) ) &&  (((~(IPCC->C1MR)) & (channel << 16U)))
#define HW_IPCC_RX_PENDING( channel )  (LL_C2_IPCC_IsActiveFlag_CHx( IPCC, channel )) && (((~(IPCC->C1MR)) & (channel << 0U)))

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static void (*FreeBufCb)( void );

/* Private function prototypes -----------------------------------------------*/
static void HW_IPCC_BLE_EvtHandler( void );
static void HW_IPCC_BLE_AclDataEvtHandler( void );
static void HW_IPCC_MM_FreeBufHandler( void );
static void HW_IPCC_SYS_CmdEvtHandler( void );
static void HW_IPCC_SYS_EvtHandler( void );
static void HW_IPCC_TRACES_EvtHandler( void );

#ifdef THREAD_WB
static void HW_IPCC_OT_CmdEvtHandler( void );
static void HW_IPCC_THREAD_NotEvtHandler( void );
static void HW_IPCC_THREAD_CliNotEvtHandler( void );
#endif

#ifdef MAC_802_15_4_WB
static void HW_IPCC_MAC_802_15_4_CmdEvtHandler( void );
static void HW_IPCC_MAC_802_15_4_NotEvtHandler( void );
#endif

#ifdef ZIGBEE_WB
static void HW_IPCC_ZIGBEE_CmdEvtHandler( void );
static void HW_IPCC_ZIGBEE_StackNotifEvtHandler( void );
static void HW_IPCC_ZIGBEE_CliNotifEvtHandler( void );
#endif

/* Public function definition -----------------------------------------------*/

/******************************************************************************
 * INTERRUPT HANDLER
 ******************************************************************************/
void HW_IPCC_Rx_Handler( void )
{
  if (HW_IPCC_RX_PENDING( HW_IPCC_SYSTEM_EVENT_CHANNEL ))
  {
      HW_IPCC_SYS_EvtHandler();
  }
#ifdef MAC_802_15_4_WB
  else if (HW_IPCC_RX_PENDING( HW_IPCC_MAC_802_15_4_NOTIFICATION_ACK_CHANNEL ))
  {
    HW_IPCC_MAC_802_15_4_NotEvtHandler();
  }
#endif /* MAC_802_15_4_WB */
#ifdef THREAD_WB
  else if (HW_IPCC_RX_PENDING( HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL ))
  {
    HW_IPCC_THREAD_NotEvtHandler();
  }
  else if (HW_IPCC_RX_PENDING( HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL ))
  {
    HW_IPCC_THREAD_CliNotEvtHandler();
  }
#endif /* THREAD_WB */
#ifdef ZIGBEE_WB
  else if (HW_IPCC_RX_PENDING( HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL ))
  {
    HW_IPCC_ZIGBEE_StackNotifEvtHandler();
  }
  else if (HW_IPCC_RX_PENDING( HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL ))
  {
      HW_IPCC_ZIGBEE_CliNotifEvtHandler();
  }
#endif /* ZIGBEE_WB */
  else if (HW_IPCC_RX_PENDING( HW_IPCC_BLE_EVENT_CHANNEL ))
  {
    HW_IPCC_BLE_EvtHandler();
  }
  else if (HW_IPCC_RX_PENDING( HW_IPCC_TRACES_CHANNEL ))
  {
    HW_IPCC_TRACES_EvtHandler();
  }

  return;
}

void HW_IPCC_Tx_Handler( void )
{
  if (HW_IPCC_TX_PENDING( HW_IPCC_SYSTEM_CMD_RSP_CHANNEL ))
  {
    HW_IPCC_SYS_CmdEvtHandler();
  }
#ifdef MAC_802_15_4_WB
  else if (HW_IPCC_TX_PENDING( HW_IPCC_MAC_802_15_4_CMD_RSP_CHANNEL ))
  {
    HW_IPCC_MAC_802_15_4_CmdEvtHandler();
  }
#endif /* MAC_802_15_4_WB */
#ifdef THREAD_WB
  else if (HW_IPCC_TX_PENDING( HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL ))
  {
    HW_IPCC_OT_CmdEvtHandler();
  }
#endif /* THREAD_WB */
#ifdef ZIGBEE_WB
  if (HW_IPCC_TX_PENDING( HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL ))
  {
      HW_IPCC_ZIGBEE_CmdEvtHandler();
  }
#endif /* ZIGBEE_WB */
  else if (HW_IPCC_TX_PENDING( HW_IPCC_SYSTEM_CMD_RSP_CHANNEL ))
  {
    HW_IPCC_SYS_CmdEvtHandler();
  }
  else if (HW_IPCC_TX_PENDING( HW_IPCC_MM_RELEASE_BUFFER_CHANNEL ))
  {
    HW_IPCC_MM_FreeBufHandler();
  }
  else if (HW_IPCC_TX_PENDING( HW_IPCC_HCI_ACL_DATA_CHANNEL ))
  {
    HW_IPCC_BLE_AclDataEvtHandler();
  }

  return;
}
/******************************************************************************
 * GENERAL
 ******************************************************************************/
void HW_IPCC_Enable( void )
{
  /**
   * In case the SBSFU is implemented, it may have already set the C2BOOT bit to startup the CPU2.
   * In that case, to keep the mechanism transparent to the user application, it shall call the system command
   * SHCI_C2_Reinit( ) before jumping to the application.
   * When the CPU2 receives that command, it waits for its event input to be set to restart the CPU2 firmware.
   * This is required because once C2BOOT has been set once, a clear/set on C2BOOT has no effect.
   * When SHCI_C2_Reinit( ) is not called, generating an event to the CPU2 does not have any effect
   * So, by default, the application shall both set the event flag and set the C2BOOT bit.
   */
  __SEV( );       /* Set the internal event flag and send an event to the CPU2 */
  __WFE( );       /* Clear the internal event flag */
  LL_PWR_EnableBootC2( );

  return;
}

void HW_IPCC_Init( void )
{
  LL_AHB3_GRP1_EnableClock( LL_AHB3_GRP1_PERIPH_IPCC );

  LL_C1_IPCC_EnableIT_RXO( IPCC );
  LL_C1_IPCC_EnableIT_TXF( IPCC );

  HAL_NVIC_EnableIRQ(IPCC_C1_RX_IRQn);
  HAL_NVIC_EnableIRQ(IPCC_C1_TX_IRQn);

  return;
}

/******************************************************************************
 * BLE
 ******************************************************************************/
void HW_IPCC_BLE_Init( void )
{
  LL_C1_IPCC_EnableReceiveChannel( IPCC, HW_IPCC_BLE_EVENT_CHANNEL );

  return;
}

void HW_IPCC_BLE_SendCmd( void )
{
  LL_C1_IPCC_SetFlag_CHx( IPCC, HW_IPCC_BLE_CMD_CHANNEL );

  return;
}

static void HW_IPCC_BLE_EvtHandler( void )
{
  HW_IPCC_BLE_RxEvtNot();

  LL_C1_IPCC_ClearFlag_CHx( IPCC, HW_IPCC_BLE_EVENT_CHANNEL );

  return;
}

void HW_IPCC_BLE_SendAclData( void )
{
  LL_C1_IPCC_SetFlag_CHx( IPCC, HW_IPCC_HCI_ACL_DATA_CHANNEL );
  LL_C1_IPCC_EnableTransmitChannel( IPCC, HW_IPCC_HCI_ACL_DATA_CHANNEL );

  return;
}

static void HW_IPCC_BLE_AclDataEvtHandler( void )
{
  LL_C1_IPCC_DisableTransmitChannel( IPCC, HW_IPCC_HCI_ACL_DATA_CHANNEL );

  HW_IPCC_BLE_AclDataAckNot();

  return;
}

__weak void HW_IPCC_BLE_AclDataAckNot( void ){};
__weak void HW_IPCC_BLE_RxEvtNot( void ){};

/******************************************************************************
 * SYSTEM
 ******************************************************************************/
void HW_IPCC_SYS_Init( void )
{
  LL_C1_IPCC_EnableReceiveChannel( IPCC, HW_IPCC_SYSTEM_EVENT_CHANNEL );

  return;
}

void HW_IPCC_SYS_SendCmd( void )
{
  LL_C1_IPCC_SetFlag_CHx( IPCC, HW_IPCC_SYSTEM_CMD_RSP_CHANNEL );
  LL_C1_IPCC_EnableTransmitChannel( IPCC, HW_IPCC_SYSTEM_CMD_RSP_CHANNEL );

  return;
}

static void HW_IPCC_SYS_CmdEvtHandler( void )
{
  LL_C1_IPCC_DisableTransmitChannel( IPCC, HW_IPCC_SYSTEM_CMD_RSP_CHANNEL );

  HW_IPCC_SYS_CmdEvtNot();

  return;
}

static void HW_IPCC_SYS_EvtHandler( void )
{
  HW_IPCC_SYS_EvtNot();

  LL_C1_IPCC_ClearFlag_CHx( IPCC, HW_IPCC_SYSTEM_EVENT_CHANNEL );

  return;
}

__weak void HW_IPCC_SYS_CmdEvtNot( void ){};
__weak void HW_IPCC_SYS_EvtNot( void ){};

/******************************************************************************
 * MAC 802.15.4
 ******************************************************************************/
#ifdef MAC_802_15_4_WB
void HW_IPCC_MAC_802_15_4_Init( void )
{
  LL_C1_IPCC_EnableReceiveChannel( IPCC, HW_IPCC_MAC_802_15_4_NOTIFICATION_ACK_CHANNEL );

  return;
}

void HW_IPCC_MAC_802_15_4_SendCmd( void )
{
  LL_C1_IPCC_SetFlag_CHx( IPCC, HW_IPCC_MAC_802_15_4_CMD_RSP_CHANNEL );
  LL_C1_IPCC_EnableTransmitChannel( IPCC, HW_IPCC_MAC_802_15_4_CMD_RSP_CHANNEL );

  return;
}

void HW_IPCC_MAC_802_15_4_SendAck( void )
{
  LL_C1_IPCC_ClearFlag_CHx( IPCC, HW_IPCC_MAC_802_15_4_NOTIFICATION_ACK_CHANNEL );
  LL_C1_IPCC_EnableReceiveChannel( IPCC, HW_IPCC_MAC_802_15_4_NOTIFICATION_ACK_CHANNEL );

  return;
}

static void HW_IPCC_MAC_802_15_4_CmdEvtHandler( void )
{
  LL_C1_IPCC_DisableTransmitChannel( IPCC, HW_IPCC_MAC_802_15_4_CMD_RSP_CHANNEL );

  HW_IPCC_MAC_802_15_4_CmdEvtNot();

  return;
}

static void HW_IPCC_MAC_802_15_4_NotEvtHandler( void )
{
  LL_C1_IPCC_DisableReceiveChannel( IPCC, HW_IPCC_MAC_802_15_4_NOTIFICATION_ACK_CHANNEL );

  HW_IPCC_MAC_802_15_4_EvtNot();

  return;
}
__weak void HW_IPCC_MAC_802_15_4_CmdEvtNot( void ){};
__weak void HW_IPCC_MAC_802_15_4_EvtNot( void ){};
#endif

/******************************************************************************
 * THREAD
 ******************************************************************************/
#ifdef THREAD_WB
void HW_IPCC_THREAD_Init( void )
{
  LL_C1_IPCC_EnableReceiveChannel( IPCC, HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL );
  LL_C1_IPCC_EnableReceiveChannel( IPCC, HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL );

  return;
}

void HW_IPCC_OT_SendCmd( void )
{
  LL_C1_IPCC_SetFlag_CHx( IPCC, HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL );
  LL_C1_IPCC_EnableTransmitChannel( IPCC, HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL );

  return;
}

void HW_IPCC_CLI_SendCmd( void )
{
  LL_C1_IPCC_SetFlag_CHx( IPCC, HW_IPCC_THREAD_CLI_CMD_CHANNEL );

  return;
}

void HW_IPCC_THREAD_SendAck( void )
{
  LL_C1_IPCC_ClearFlag_CHx( IPCC, HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL );
  LL_C1_IPCC_EnableReceiveChannel( IPCC, HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL );

  return;
}

void HW_IPCC_THREAD_CliSendAck( void )
{
  LL_C1_IPCC_ClearFlag_CHx( IPCC, HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL );
  LL_C1_IPCC_EnableReceiveChannel( IPCC, HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL );

  return;
}

static void HW_IPCC_OT_CmdEvtHandler( void )
{
  LL_C1_IPCC_DisableTransmitChannel( IPCC, HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL );

  HW_IPCC_OT_CmdEvtNot();

  return;
}

static void HW_IPCC_THREAD_NotEvtHandler( void )
{
  LL_C1_IPCC_DisableReceiveChannel( IPCC, HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL );

  HW_IPCC_THREAD_EvtNot();

  return;
}

static void HW_IPCC_THREAD_CliNotEvtHandler( void )
{
  LL_C1_IPCC_DisableReceiveChannel( IPCC, HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL );

  HW_IPCC_THREAD_CliEvtNot();

  return;
}

__weak void HW_IPCC_OT_CmdEvtNot( void ){};
__weak void HW_IPCC_CLI_CmdEvtNot( void ){};
__weak void HW_IPCC_THREAD_EvtNot( void ){};

#endif /* THREAD_WB */

/******************************************************************************
 * ZIGBEE
 ******************************************************************************/
#ifdef ZIGBEE_WB
void HW_IPCC_ZIGBEE_Init( void )
{
  LL_C1_IPCC_EnableReceiveChannel( IPCC, HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL );
  LL_C1_IPCC_EnableReceiveChannel( IPCC, HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL );

  return;
}

void HW_IPCC_ZIGBEE_SendAppliCmd( void )
{
  LL_C1_IPCC_SetFlag_CHx( IPCC, HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL );
  LL_C1_IPCC_EnableTransmitChannel( IPCC, HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL );

  return;
}

void HW_IPCC_ZIGBEE_SendCliCmd( void )
{
  LL_C1_IPCC_SetFlag_CHx( IPCC, HW_IPCC_THREAD_CLI_CMD_CHANNEL );

   return;
}

void HW_IPCC_ZIGBEE_SendAppliCmdAck( void )
{
  LL_C1_IPCC_ClearFlag_CHx( IPCC, HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL );
  LL_C1_IPCC_EnableReceiveChannel( IPCC, HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL );

  return;
}

void HW_IPCC_ZIGBEE_SendCliCmdAck( void )
{
    LL_C1_IPCC_ClearFlag_CHx( IPCC, HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL );
    LL_C1_IPCC_EnableReceiveChannel( IPCC, HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL );

    return;
}

static void HW_IPCC_ZIGBEE_CmdEvtHandler( void )
{
    LL_C1_IPCC_DisableTransmitChannel( IPCC, HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL );

    HW_IPCC_ZIGBEE_AppliCmdNotification();

    return;
}

static void HW_IPCC_ZIGBEE_StackNotifEvtHandler( void )
{
    LL_C1_IPCC_DisableReceiveChannel( IPCC, HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL );

    HW_IPCC_ZIGBEE_AppliAsyncEvtNotification();

    return;
}

static void HW_IPCC_ZIGBEE_CliNotifEvtHandler( void )
{
  LL_C1_IPCC_DisableReceiveChannel( IPCC, HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL );

  HW_IPCC_ZIGBEE_CliEvtNotification();

  return;
}

__weak void HW_IPCC_ZIGBEE_AppliCmdNotification( void ){};
__weak void HW_IPCC_ZIGBEE_AppliAsyncEvtNotification( void ){};
__weak void HW_IPCC_ZIGBEE_CliEvtNotification( void ){};
#endif /* ZIGBEE_WB */

/******************************************************************************
 * MEMORY MANAGER
 ******************************************************************************/
void HW_IPCC_MM_SendFreeBuf( void (*cb)( void ) )
{
  if ( LL_C1_IPCC_IsActiveFlag_CHx( IPCC, HW_IPCC_MM_RELEASE_BUFFER_CHANNEL ) )
  {
    FreeBufCb = cb;
    LL_C1_IPCC_EnableTransmitChannel( IPCC, HW_IPCC_MM_RELEASE_BUFFER_CHANNEL );
  }
  else
  {
    cb();

    LL_C1_IPCC_SetFlag_CHx( IPCC, HW_IPCC_MM_RELEASE_BUFFER_CHANNEL );
  }

  return;
}

static void HW_IPCC_MM_FreeBufHandler( void )
{
  LL_C1_IPCC_DisableTransmitChannel( IPCC, HW_IPCC_MM_RELEASE_BUFFER_CHANNEL );

  FreeBufCb();

  LL_C1_IPCC_SetFlag_CHx( IPCC, HW_IPCC_MM_RELEASE_BUFFER_CHANNEL );

  return;
}

/******************************************************************************
 * TRACES
 ******************************************************************************/
void HW_IPCC_TRACES_Init( void )
{
  LL_C1_IPCC_EnableReceiveChannel( IPCC, HW_IPCC_TRACES_CHANNEL );

  return;
}

static void HW_IPCC_TRACES_EvtHandler( void )
{
  HW_IPCC_TRACES_EvtNot();

  LL_C1_IPCC_ClearFlag_CHx( IPCC, HW_IPCC_TRACES_CHANNEL );

  return;
}

__weak void HW_IPCC_TRACES_EvtNot( void ){};

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
