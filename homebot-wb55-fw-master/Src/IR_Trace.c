#include "IR_Trace.h"
#include "fifo.h"
#include "timer.h"

typedef enum
{
    HEADER = 0,
    LONG_R,
    LONG_C,
    LONG_L,
    SHORT_R,
    SHORT_C,
    SHORT_L,
    TAIL,
    ERR = 255,
}IR_DATA;


static uint8_t IrDataBuff[40];
static IR_DIRECTION CurrentDirection, BeforeDirection;
static uint8_t TimerFlag;

#define T_HEADER    1000
#define T_TAIL      825
#define T_LONG_R    225
#define T_LONG_C    150
#define T_LONG_L    75
#define T_SHORT_R   550
#define T_SHORT_C   650
#define T_SHORT_L   750
#define T_GAP       15

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint32_t count_10us = 0;
    count_10us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    htim->Instance->CNT = 0;
    
    setTimer(Timer_IRCHECK,	50, TIME_RESET);
    //uart_printf("%d\r\n", count_10us);

    if(count_10us > T_HEADER-T_GAP && count_10us < T_HEADER+T_GAP)
    {
        SetFIFO((FifoHandle)IrDataBuff, HEADER);//header
    }
    else if(count_10us > T_TAIL-T_GAP && count_10us < T_TAIL+T_GAP)
    {
        SetFIFO((FifoHandle)IrDataBuff, TAIL);//tail
    }
    else if(count_10us > T_LONG_R-T_GAP && count_10us < T_LONG_R+T_GAP)
    {
        SetFIFO((FifoHandle)IrDataBuff, LONG_R);//long_r
    }
    else if(count_10us > T_LONG_C-T_GAP && count_10us < T_LONG_C+T_GAP)
    {
        SetFIFO((FifoHandle)IrDataBuff, LONG_C);//long_c
    }
    else if(count_10us > T_LONG_L-T_GAP && count_10us < T_LONG_L+T_GAP)
    {
        SetFIFO((FifoHandle)IrDataBuff, LONG_L);//long_l
    }
    else if(count_10us > T_SHORT_R-T_GAP && count_10us < T_SHORT_R+T_GAP)
    {
        SetFIFO((FifoHandle)IrDataBuff, SHORT_R);//short_r
    }
    else if(count_10us > T_SHORT_C-T_GAP && count_10us < T_SHORT_C+T_GAP)
    {
        SetFIFO((FifoHandle)IrDataBuff, SHORT_C);//short_c
    }
    else if(count_10us > T_SHORT_L-T_GAP && count_10us < T_SHORT_L+T_GAP)
    {
        SetFIFO((FifoHandle)IrDataBuff, SHORT_L);//short_l
    }
    else
    {
        SetFIFO((FifoHandle)IrDataBuff, ERR);
    }
}

void Ir_Init_Buffer()
{
    InitFIFO(IrDataBuff, sizeof(IrDataBuff));
    ClearFIFO((FifoHandle)IrDataBuff);
    for(uint8_t i=3; i<sizeof(IrDataBuff); i++)
    {
        IrDataBuff[i] = 0xAA;
    }
}

static void Ir_Clear_Buffer()
{
    ClearFIFO((FifoHandle)IrDataBuff);
    for(uint8_t i=3; i<sizeof(IrDataBuff); i++)
    {
        IrDataBuff[i] = 0xAA;
    }
}

#if 0
static uint8_t Ir_Check_Header(uint8_t *d, uint8_t *i)
{    
    *i++;
    GetFIFO((FifoHandle)IrDataBuff, d);
    if(*d == TAIL || *d == HEADER)//header 뒤 헤더나 테일이면.
    {
        Ir_Clear_Buffer();
        return 1;
    }
    else
    {
        return 0;
    }
}
#endif

static void Ir_Clear_Direction()
{
    CurrentDirection.long_distance.ir_long = 0;
    CurrentDirection.short_distance.ir_short = 0;
}

static uint8_t Ir_Detect_Direction()
{
    uint8_t header = FALSE;
    
    //for(uint8_t i=3; i<sizeof(IrDataBuff); i++)
    while(FIFOIsEmpty((FifoHandle)IrDataBuff))
    {
        uint8_t d = 0;
        
        GetFIFO((FifoHandle)IrDataBuff, &d);

        if(d == HEADER && header == FALSE)
        {
            header = TRUE;
        }
        else if(header == TRUE)
        {
            switch(d)
            {
                case LONG_R :
                {
                    CurrentDirection.long_distance.bytes.long_r++;   
                    break;
                }
                case LONG_C :
                {
                    CurrentDirection.long_distance.bytes.long_c++;   
                    break;
                }
                case LONG_L :
                {
                    CurrentDirection.long_distance.bytes.long_l++;   
                    break;
                }
                case SHORT_R :
                {
                    CurrentDirection.short_distance.bytes.short_r++;   
                    break;
                }
                case SHORT_C :
                {
                    CurrentDirection.short_distance.bytes.short_c++;   
                    break;
                }
                case SHORT_L :
                {
                    CurrentDirection.short_distance.bytes.short_l++;
                    break;
                }
                case TAIL :
                {                                        
                    return 0;
                    break;
                }
                case HEADER :
                {                                
                    return 1;
                    break;
                }
                default :
                {
                    break;
                }
            }
        }
    }  
        
    return 1;
}

static void Set_TimerFlag()
{
    TimerFlag = 1;
}

void Clear_IrFlag()
{
    TimerFlag = 0;
}

uint8_t Is_IrTimer()
{
    return TimerFlag;
}


void Ir_Getdata(IR_DIRECTION* cdirection, IR_DIRECTION* bdirection)
{
    *cdirection = CurrentDirection;
    *bdirection = BeforeDirection;
}

void Ir_Process()
{
    uint8_t check_status = 0;
    Set_TimerFlag();
    
    Ir_Clear_Direction();
    //Ir_Clear_Buffer();

    check_status = Ir_Detect_Direction();

    if(check_status)
    {        
        Ir_Clear_Direction();
        Ir_Clear_Buffer();
    }
    else
    {
        BeforeDirection = CurrentDirection;
    }

    Ir_Clear_Buffer();
}

IR_DIRECTION Ir_GetDirection()
{
    return CurrentDirection;
}

