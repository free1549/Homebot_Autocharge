#ifndef __TRACE_STATION_H__
#define __TRACE_STATION_H__

#include "main.h"

typedef struct
{
    int powergood;
    int processstep;
    int detectirstep;
    int thinkstep;
    int direction;
    int goaround;
    int detectlev;
    IR_DIRECTION ir;
}MODE_DEBUG;

void Charge_Process();
int Get_ChargeMode();
void Set_ChargeMode();
void Clear_ChargeMode();
MODE_DEBUG Get_Debug();
void Clear_InnerStaticVar();

#endif