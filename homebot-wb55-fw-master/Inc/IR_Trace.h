#ifndef __IR_TRACE_H__
#define __IR_TRACE_H__

#include "main.h"

#define DETECT_FULL_SENSING     (0x00020202)
#define DETECT_HALF_SENSING     (0x00010101)
#define DETECT_LECE_SENSING     (0x00010100)
#define DETECT_RICE_SENSING     (0x00000101)
#define DETECT_LEFT_SENSING     (0x00010000)
#define DETECT_RIGHT_SENSING    (0x00000001)
#define DETECT_CENTER_SENSING   (0x00000100)
#define DETECT_NONE_SENSING     (0x00000000)

#define FALSE (0)
#define TRUE (!FALSE)

typedef struct 
{    
    union
    {        
        uint32_t ir_long;
        struct
        {
            uint8_t long_r;
            uint8_t long_c;
            uint8_t long_l;
            uint8_t notused;
        }bytes;
    }long_distance;
    
    union
    {
        uint32_t ir_short;
        struct
        {
            uint8_t short_r;
            uint8_t short_c;
            uint8_t short_l;
            uint8_t notused;
        }bytes;
    }short_distance;
}IR_DIRECTION;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void Ir_Init_Buffer();
void Ir_Process();
void Ir_Getdata(IR_DIRECTION* cdirection, IR_DIRECTION* bdirection);
uint8_t Is_IrTimer();
void Clear_IrFlag();
IR_DIRECTION Ir_GetDirection();

#endif
