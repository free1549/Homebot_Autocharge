
/**
  ******************************************************************************
  * @file           : Fifo.c
  * @brief          : Fifo program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 VARRAM
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Fifo.h"

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


void InitFIFO(uint8_t * gpFIFOBuffer, uint8_t cLength)
{
    FifoHandle fh = (FifoHandle)gpFIFOBuffer;
    
    if (cLength <= 3) return;

    fh->ucSize = (uint8_t)(cLength - 3);
    
    ClearFIFO(fh);
    return;
}

uint8_t GetFIFO(FifoHandle fh, uint8_t * pData)
{
    if (fh->ucHead != fh->ucTail)
    {
        *pData = fh->ucData[fh->ucHead];
        fh->ucHead++;
        if (fh->ucHead >= fh->ucSize) fh->ucHead = 0;
        return 1;
    }
    return 0;
}

uint8_t SetFIFO(FifoHandle fh, uint8_t ucData)
{
    if ((fh->ucTail + 1 == fh->ucHead) ||
        (fh->ucTail + 1 - fh->ucSize == fh->ucHead))
        return 0;          /* Full */
    fh->ucData[fh->ucTail] = ucData;
    fh->ucTail++;
    if (fh->ucTail >= fh->ucSize) fh->ucTail = 0;
    return 1;
}

void ClearFIFO(FifoHandle fh)
{
    fh->ucHead = 0;
    fh->ucTail = 0;
}

uint8_t FIFOIsEmpty(FifoHandle fh)
{
    return  fh->ucHead != fh->ucTail;
}

uint8_t FIFOIsFull(FifoHandle fh)
{
    return (fh->ucTail + 1 == fh->ucHead) || (fh->ucTail + 1 - fh->ucSize == fh->ucHead);
}

/************************ (C) COPYRIGHT VARRAM *****END OF FILE****/
