/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32wbxx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32wbxx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hi2st-parser.h"
#include "uart_trace.h"
#include "hw.h"
#include "uart_trace.h"
#include "app_conf.h"
#include "stm32_seq.h"
#include "motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_lpuart1_tx;
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern motor_TypeDef DCMOTOR;

extern uint8_t addToBleParserFromHi(uint8_t t);
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
    /* USER CODE BEGIN SVCall_IRQn 0 */

    /* USER CODE END SVCall_IRQn 0 */
    /* USER CODE BEGIN SVCall_IRQn 1 */

    /* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
    /* USER CODE BEGIN PendSV_IRQn 0 */

    /* USER CODE END PendSV_IRQn 0 */
    /* USER CODE BEGIN PendSV_IRQn 1 */

    /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    /* USER CODE BEGIN SysTick_IRQn 1 */
    HAL_SYSTICK_IRQHandler();
    DCMOTOR.Value.l_ts++;
    DCMOTOR.Value.r_ts++;
    //DCMOTOR.Value.ts++;
    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32WBxx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32wbxx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles EXTI line0 interrupt.
 */
void EXTI0_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI0_IRQn 0 */

    /* USER CODE END EXTI0_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
    /* USER CODE BEGIN EXTI0_IRQn 1 */

    /* USER CODE END EXTI0_IRQn 1 */
}

/**
 * @brief This function handles EXTI line1 interrupt.
 */
void EXTI1_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI1_IRQn 0 */

    /* USER CODE END EXTI1_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
    /* USER CODE BEGIN EXTI1_IRQn 1 */

    /* USER CODE END EXTI1_IRQn 1 */
}

void EXTI4_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI1_IRQn 0 */

    /* USER CODE END EXTI1_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
    /* USER CODE BEGIN EXTI1_IRQn 1 */
    /*HAL_GPIO_WritePin(GPIOA, PW_ON_Pin, GPIO_PIN_RESET);*/
	//uart_prtinf("off\r\n");
    /* USER CODE END EXTI1_IRQn 1 */
}

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}

/**
 * @brief This function handles DMA1 channel1 global interrupt.
 */
void DMA1_Channel1_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

    /* USER CODE END DMA1_Channel1_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_lpuart1_tx);
    /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

    /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel2 global interrupt.
 */
void DMA1_Channel2_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

    /* USER CODE END DMA1_Channel2_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_adc1);
    /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

    /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
 * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
 */
void TIM1_UP_TIM16_IRQHandler(void)
{
    /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

    /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
    HAL_TIM_IRQHandler(&htim1);
    HAL_TIM_IRQHandler(&htim16);
    /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

    /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
 * @brief This function handles TIM1 trigger and commutation interrupts and TIM17 global interrupt.
 */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
    /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */

    /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */
    HAL_TIM_IRQHandler(&htim1);
    HAL_TIM_IRQHandler(&htim17);
    /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */

    /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart1, "a", 1);
    uart_printf("recv_uart\r\n");
}

void USART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */

    /* USER CODE END USART1_IRQn 0 */
    //HAL_UART_IRQHandler(&huart1);
    /* USER CODE BEGIN USART1_IRQn 1 */
#if 1
    uint32_t isrflags   = READ_REG(huart1.Instance->ISR);
    uint32_t cr1its     = READ_REG(huart1.Instance->CR1);
    uint32_t errorflags = 0x00U;

    /* If no error occurs */
    //errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
    errorflags = (isrflags & 0x0f);
    if(errorflags == RESET)
    {
        /* UART in mode Receiver -------------------------------------------------*/
        //if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
        if(isrflags & USART_ISR_RXNE)
        {
            //SetFIFO((FifoHandle)uart_data, (uint8_t)READ_REG(huart2.Instance->DR));
            addToBleParserFromHi((uint8_t)READ_REG(huart1.Instance->RDR));
      	    UTIL_SEQ_SetTask( 1<<CFG_TASK_PARSE_BLE_ID, CFG_SCH_PRIO_0);
            //UART_Receive_IT(huart);
            //LEDL_B_ON;
            return;
        }
    }  
    //uint32_t dmarequest = 0x00U;
    uint32_t cr3its   = READ_REG(huart1.Instance->CR3);
    /* If some errors occur */
    if((errorflags != RESET) && (((cr3its & USART_CR3_EIE) != RESET) || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET)))
    {
        /* UART parity error interrupt occurred ----------------------------------*/
        if(((isrflags & USART_ISR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
        {
            huart1.ErrorCode |= HAL_UART_ERROR_PE;
        }

        /* UART noise error interrupt occurred -----------------------------------*/
        if(((isrflags & USART_ISR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
        {
            huart1.ErrorCode |= HAL_UART_ERROR_NE;
        }

        /* UART frame error interrupt occurred -----------------------------------*/
#if 1
        if(((isrflags & USART_ISR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
        {
            huart1.ErrorCode |= HAL_UART_ERROR_FE;
        }
#endif
        /* UART Over-Run interrupt occurred --------------------------------------*/
        if(((isrflags & USART_ISR_ORE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
        { 
            huart1.ErrorCode |= HAL_UART_ERROR_ORE;
        }

        /* Call UART Error Call back function if need be --------------------------*/  
        if(huart1.ErrorCode != HAL_UART_ERROR_NONE)
        {
            /* UART in mode Receiver -----------------------------------------------*/
            if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
            {
                //SetFIFO((FifoHandle)uart_data, (uint8_t)READ_REG(huart1.Instance->DR));
                addToBleParserFromHi((uint8_t)READ_REG(huart1.Instance->RDR));
                UTIL_SEQ_SetTask( 1<<CFG_TASK_PARSE_BLE_ID, CFG_SCH_PRIO_0);

                //UART_Receive_IT(huart1);
            }

            /* Non Blocking error : transfer could go on. 
               Error is notified to user through user error callback */
            HAL_UART_ErrorCallback(&huart1);       
			// READ_REG(huart1.Instance->RDR);
            SET_BIT(huart1.Instance->ICR, USART_ICR_PECF|USART_ICR_FECF|USART_ICR_ORECF);
            huart1.ErrorCode = HAL_UART_ERROR_NONE;
        }

        return;
    } /* End if some error occurs */
#if 0
    /* UART in mode Transmitter ------------------------------------------------*/
    if(((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
    {
        UART_Transmit_IT(&huart2);
        return;
    }
#endif

    /* UART in mode Transmitter end --------------------------------------------*/
    if(((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
    {
        // UART_EndTransmit_IT(&huart2);

        /* Disable the UART Transmit Complete Interrupt */    
        CLEAR_BIT(huart1.Instance->CR1, USART_CR1_TCIE);
        /* Tx process is ended, restore huart->gState to Ready */
        huart1.gState = HAL_UART_STATE_READY;
        HAL_UART_TxCpltCallback(&huart1);

        return;
    }
#endif
    /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles LPUART1 global interrupt.
  */
void LPUART1_IRQHandler(void)
{
#if 0
  /* USER CODE BEGIN LPUART1_IRQn 0 */

    /* USER CODE END LPUART1_IRQn 0 */
    HAL_UART_IRQHandler(&hlpuart1);
    /* USER CODE BEGIN LPUART1_IRQn 1 */
    READ_REG(hlpuart1.Instance->RDR);
	HAL_UART_Receive_IT(&hlpuart1, "a", 1);
    /* USER CODE END LPUART1_IRQn 1 */
#endif
    // for PC test
    /* USER CODE BEGIN USART1_IRQn 0 */

    /* USER CODE END USART1_IRQn 0 */
    //HAL_UART_IRQHandler(&huart1);
    /* USER CODE BEGIN USART1_IRQn 1 */
#if 1
    uint32_t isrflags   = READ_REG(hlpuart1.Instance->ISR);
    uint32_t cr1its     = READ_REG(hlpuart1.Instance->CR1);
    uint32_t errorflags = 0x00U;

    /* If no error occurs */
    //errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
    errorflags = (isrflags & 0x0f);
    if(errorflags == RESET)
    {
        /* UART in mode Receiver -------------------------------------------------*/
        //if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
        if(isrflags & USART_ISR_RXNE)
        {
            //SetFIFO((FifoHandle)uart_data, (uint8_t)READ_REG(huart2.Instance->DR));
            addToBleParserFromHi((uint8_t)READ_REG(hlpuart1.Instance->RDR));
                UTIL_SEQ_SetTask( 1<<CFG_TASK_PARSE_BLE_ID, CFG_SCH_PRIO_0);
            //UART_Receive_IT(huart);
            //LEDL_B_ON;
            return;
        }
    }  
    //uint32_t dmarequest = 0x00U;
    uint32_t cr3its   = READ_REG(hlpuart1.Instance->CR3);
    /* If some errors occur */
    if((errorflags != RESET) && (((cr3its & USART_CR3_EIE) != RESET) || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET)))
    {
        /* UART parity error interrupt occurred ----------------------------------*/
        if(((isrflags & USART_ISR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
        {
            hlpuart1.ErrorCode |= HAL_UART_ERROR_PE;
        }

        /* UART noise error interrupt occurred -----------------------------------*/
        if(((isrflags & USART_ISR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
        {
            hlpuart1.ErrorCode |= HAL_UART_ERROR_NE;
        }

        /* UART frame error interrupt occurred -----------------------------------*/
#if 1
        if(((isrflags & USART_ISR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
        {
            hlpuart1.ErrorCode |= HAL_UART_ERROR_FE;
        }
#endif
        /* UART Over-Run interrupt occurred --------------------------------------*/
        if(((isrflags & USART_ISR_ORE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
        { 
            hlpuart1.ErrorCode |= HAL_UART_ERROR_ORE;
        }

        /* Call UART Error Call back function if need be --------------------------*/  
        if(hlpuart1.ErrorCode != HAL_UART_ERROR_NONE)
        {
            /* UART in mode Receiver -----------------------------------------------*/
            if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
            {
                //SetFIFO((FifoHandle)uart_data, (uint8_t)READ_REG(huart1.Instance->DR));
                addToBleParserFromHi((uint8_t)READ_REG(hlpuart1.Instance->RDR));
                UTIL_SEQ_SetTask( 1<<CFG_TASK_PARSE_BLE_ID, CFG_SCH_PRIO_0);
                //UART_Receive_IT(huart1);
            }

            /* Non Blocking error : transfer could go on. 
               Error is notified to user through user error callback */
            HAL_UART_ErrorCallback(&hlpuart1);       
			// READ_REG(huart1.Instance->RDR);
            SET_BIT(hlpuart1.Instance->ICR, USART_ICR_PECF|USART_ICR_FECF|USART_ICR_ORECF);
            hlpuart1.ErrorCode = HAL_UART_ERROR_NONE;
        }

        return;
    } /* End if some error occurs */
#if 0
    /* UART in mode Transmitter ------------------------------------------------*/
    if(((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
    {
        UART_Transmit_IT(&huart2);
        return;
    }
#endif

    /* UART in mode Transmitter end --------------------------------------------*/
    if(((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
    {
        // UART_EndTransmit_IT(&huart2);

        /* Disable the UART Transmit Complete Interrupt */    
        CLEAR_BIT(hlpuart1.Instance->CR1, USART_CR1_TCIE);
        /* Tx process is ended, restore huart->gState to Ready */
        hlpuart1.gState = HAL_UART_STATE_READY;
        HAL_UART_TxCpltCallback(&hlpuart1);

        return;
    }
#endif

}

/* USER CODE BEGIN 1 */

void RTC_WKUP_IRQHandler(void)
{
    HW_TS_RTC_Wakeup_Handler();
}

void IPCC_C1_TX_IRQHandler(void)
{
    HW_IPCC_Tx_Handler();

    return;
}

void IPCC_C1_RX_IRQHandler(void)
{
    HW_IPCC_Rx_Handler();
    return;
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
