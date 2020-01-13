/**
  ******************************************************************************
  * @file           : Fifo.h
  * @brief          : Header for Fifo.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * 
  * COPYRIGHT(c) 2018 VARRAM
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FIFO_H__
#define __FIFO_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
#include "stm32wb55xx.h"

/* Private define ------------------------------------------------------------*/

/* Fifo Struct define ------------------------------------------------------------*/

typedef struct FifoStruct
{
    uint8_t ucHead;     /**< Head index  */ 
    uint8_t ucTail;     /**< Tail index  */
    uint8_t ucSize;     /**< Buffer size */
    uint8_t ucData[1];  /**< the first byte of the buffer */
} FifoStruct;
typedef FifoStruct * FifoHandle;

/* Fifo Function define ------------------------------------------------------------*/
void InitFIFO(uint8_t * gpFIFOBuffer, uint8_t cLength);
uint8_t GetFIFO(FifoHandle fh, uint8_t * pData);
uint8_t SetFIFO(FifoHandle fh, uint8_t ucData);
void ClearFIFO(FifoHandle fh);
uint8_t FIFOIsEmpty(FifoHandle fh);
uint8_t FIFOIsFull(FifoHandle fh);

#endif /* __FIFO_H__ */

/************************ (C) COPYRIGHT VARRAM *****END OF FILE****/
