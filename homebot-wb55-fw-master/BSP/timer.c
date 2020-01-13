
//#include "stm32f4xx_hal.h"
#include "main.h"
#include "timer.h"
#include "stdlib.h"
#include "app_conf.h"
#include "stm32_seq.h"

struct TIMER_BIT	 TIMER_SYSTEM[TIMER_SYSTEM_QTY];
extern RTC_HandleTypeDef hrtc;
uint32_t nTimerOnCnt = 0;

/////////////////////////////////////////////////////////////////////////
// Program ??? ???? Timer Check
/////////////////////////////////////////////////////////////////////////
void timerReset(Timer_type TimerNo)
{
  TIMER_SYSTEM[TimerNo].Count  = 0;
  TIMER_SYSTEM[TimerNo].Flag   = OFF;
  TIMER_SYSTEM[TimerNo].Reset  = OFF;
  TIMER_SYSTEM[TimerNo].On   = OFF;
}

// Timer Set
void setTimer(Timer_type TimerNo, unsigned int TimePeriod, int nMode)
{
  if(TIMER_SYSTEM[TimerNo].Start && (nMode == TIME_CONTINUE)) return;
  TIMER_SYSTEM[TimerNo].Count = 0;
  TIMER_SYSTEM[TimerNo].Period= TimePeriod;
  TIMER_SYSTEM[TimerNo].Flag  = OFF;
  TIMER_SYSTEM[TimerNo].Reset = OFF;
  TIMER_SYSTEM[TimerNo].Use = ON;
  TIMER_SYSTEM[TimerNo].Start = ON;
  TIMER_SYSTEM[TimerNo].On  = OFF;
}

// Timer Kill
void killTimer(Timer_type TimerNo)
{
  TIMER_SYSTEM[TimerNo].Start = OFF;
  TIMER_SYSTEM[TimerNo].On  = OFF;
  TIMER_SYSTEM[TimerNo].Flag  = OFF;
  TIMER_SYSTEM[TimerNo].Reset = OFF;
  TIMER_SYSTEM[TimerNo].Count = 0;
  TIMER_SYSTEM[TimerNo].Period= 0;
  TIMER_SYSTEM[TimerNo].Use   = OFF;
}

uint8_t liveCheckTimer(Timer_type TimerNo)
{
  return TIMER_SYSTEM[TimerNo].Use;
}

uint8_t onCheckTimer(Timer_type TimerNo)
{
  return TIMER_SYSTEM[TimerNo].On;
}

uint16_t getPeriodTimer(Timer_type TimerNo)
{
  return TIMER_SYSTEM[TimerNo].Period;
}


/*sys IRQ*/
void HAL_SYSTICK_Callback(void)
{
  int i;   
  char nAddTime=1;
  
  for(i = 0; i < TIMER_SYSTEM_QTY; i++)
  {  
    if(TIMER_SYSTEM[i].Start)
    {
      if(!TIMER_SYSTEM[i].Reset && !TIMER_SYSTEM[i].On)
      {
        if((TIMER_SYSTEM[i].Count+nAddTime) < TIMER_SYSTEM[i].Period) 
          TIMER_SYSTEM[i].Count+=nAddTime;
        else
        {
          TIMER_SYSTEM[i].Count = TIMER_SYSTEM[i].Period;
          TIMER_SYSTEM[i].On = ON;
          nTimerOnCnt++;
          UTIL_SEQ_SetTask( 1<<CFG_TASK_TIMER_CHECK_ID, CFG_SCH_PRIO_0);
        }
      }
    }
    else
      TIMER_SYSTEM[i].On = OFF;
  }
}

uint16_t GetSystemTimeStamp(void)
{
	uint16_t subSec = 0;
/*  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;
	uint8_t datalog[100];
  HAL_RTC_GetTime(&hrtc, &stimestructure, RTC_FORMAT_BIN);*/
//  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
  /*subSec = ((((((int)RTC_SYNCH_PREDIV) - ((int)stimestructure.SubSeconds)) * 100) /
             (RTC_SYNCH_PREDIV + 1)) & 0xff);
  */
	  //subSec = ((((255 - ((int)stimestructure.SubSeconds)) * 1000) /(255 + 1)) & 0xFFff);
 subSec = ((((255 - ((int)hrtc.Instance->SSR)) * 1000) /(255 + 1)) & 0xFFff);
 // uart_printf(" %03d \r\n",subSec);
  
//	  sprintf(datalog," %03d \r\n",subSec);
//	  HAL_UART_Transmit_DMA(&huart1, (uint8_t*) datalog, strlen(datalog));

	return subSec;
}

