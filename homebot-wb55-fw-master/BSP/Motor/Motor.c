#include "stm32wbxx_hal.h"
#include "main.h"
#include "timer.h"
#include "Motor.h"
#include "encoders.h"
//#include "beeper.h"
//#include "led.h"
//#include "uart_trace.h"
#include <stdlib.h>
//#include <arm_math.h> 
#include <math.h>
#include "simple.pb-c.h"

float p_gain=10.0f;//35  30.0f;
float i_gain=20.0f;//100.0f;  60.0f;
float d_gain=0.015f;//0.015f;  0.003f;

long l_power_difference=0;
float l_proportional=0;
float l_derivative=0;
float l_integral=0;
int32_t lcurrpm=0;

long r_power_difference=0;
float r_proportional=0;
float r_derivative=0;
float r_integral=0;
int32_t rcurrpm=0;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim11;
extern struct TIMER_BIT	 TIMER_SYSTEM[TIMER_SYSTEM_QTY];
extern uint16_t ADC_value[3];
extern int iAccuracy;
extern float fAzimuth, fPitch, fRoll;
extern float des_fAzimuth;
extern uint32_t tossingTimeCheck;
extern SYSTEM_key sys;
extern ToHi tohi;

motor_TypeDef DCMOTOR;

Control_Motor_TypeDef 	cDCMOTOR;
move_TypeDef  *_moving;

#ifdef PI
#undef PI
#endif
#define PI 3.141592653589793238462643383f
#define DtoR 0.01745329252f

/*
   속도 +-999
   */
void Mobile_DC(motor_TypeDef * Motor)
{
  if(((Motor->Value.l_sour!=0)||(Motor->Value.r_sour!=0)))
  {
    MOBILE_WAKEUP;
  }

 Motor->Value.rmsts=1;
 Motor->Value.lmsts=1;
 if((Motor->Value.r_sour>=-(MAX_DC_SPEED))&&(Motor->Value.r_sour<=(MAX_DC_SPEED)))//999? ??? ?? ?? ??? ? ?? ??.
 {

   if(Motor->Value.r_sour>0)
   {
     RIGHT_DIR_CW;
     MTIM.Instance->CCR1 = abs(Motor->Value.r_sour);
     Motor->Value.sts|=0x02;
   }
   //CCW
   else if(Motor->Value.r_sour<0)
   {
     RIGHT_DIR_CCW;
     MTIM.Instance->CCR1 = abs(Motor->Value.r_sour);
     Motor->Value.sts|=0x02;
   }
   // 0 ??...
   else
   {
     Motor->Value.rmsts=0;
     MTIM.Instance->CCR1 = 0;
     Motor->Value.sts&=~0x02;
   }
 }

 if((Motor->Value.l_sour>=-(MAX_DC_SPEED))&&(Motor->Value.l_sour<=(MAX_DC_SPEED)))//999? ??? ?? ?? ??? ? ?? ??.
 {

   if(Motor->Value.l_sour>0)
   {
     LEFT_DIR_CW;
     MTIM.Instance->CCR2 = abs(Motor->Value.l_sour);
     Motor->Value.sts|=0x04;
   }
   //CCW
   else if(Motor->Value.l_sour<0)
   {
     LEFT_DIR_CCW;
     MTIM.Instance->CCR2 = abs(Motor->Value.l_sour);
     Motor->Value.sts|=0x04;
   }
   // 0 ??...
   else
   {
     Motor->Value.lmsts=0;
     MTIM.Instance->CCR2 = 0;
     Motor->Value.sts&=~0x04;
   }
 }
 
// tohi.lspeed=Motor->Value.l_sour;
// tohi.rspeed=Motor->Value.r_sour;
 //    uart_printf("%d %d %d\r\n",Motor->Value.r_sour,Motor->Value.l_sour,Motor->Value.cap);
 if(Motor->Value.sts == 0x00) {MOBILE_SLEEP; }
}

// FFxx board. toss
void Neck_DC(motor_TypeDef * Motor)
{
  if(Motor->Value.n_sour!=0 ) {
    MOBILE_WAKEUP;
  }
  if((Motor->Value.n_sour>=-(MAX_DC_SPEED))&&(Motor->Value.n_sour<=(MAX_DC_SPEED)))
  {
    if(Motor->Value.n_sour>0)
    {	
      NECK_DIR_CW;
      STIM.Instance->CCR3 = abs(Motor->Value.n_sour);
       Motor->Value.sts|=0x01;
    }
    //CCW
    else if(Motor->Value.n_sour<0)
    {
      NECK_DIR_CCW;
      STIM.Instance->CCR3 = abs(Motor->Value.n_sour);
       Motor->Value.sts|=0x01;
    }
    // 0 ??...
    else
    {
      STIM.Instance->CCR3 = 0;
       Motor->Value.sts&=~0x01;
    }
  }
 // tohi.t_speed=Motor->Value.n_sour;
  if(Motor->Value.sts == 0x00) {MOBILE_SLEEP; }
}




/*??? ??.*/
long Angle_Gap_Chk(long Angle_Sour,long Angle_Des)
{	
  long gap=0;
  gap = Angle_Sour-Angle_Des;
  if(gap<-180000)gap+=360000;
  else if(gap>180000)gap-=360000;
  gap=gap%360000;
  return	gap;
}
long Value_Gap_Chk(long Sour,long Des)
{	
  long gap=0;
  gap =Sour- Des;
  if(gap<-180000)gap+=360000;
  else if(gap>180000)gap-=360000;
  gap=gap%360000;
  return	gap;
}
void Setspeed(SYSTEM_key *key,float des_angle)
{  
  long proportional=0;
  long power_difference=0;

  if(key->stop_req==1)// && (iAccuracy<2))
  {
    proportional=0;
    key->lspd=0;
    key->rspd=0;
    return;
  } 

  proportional = Angle_Gap_Chk((long)(key->yaw*1000),(long)(des_angle*1000));

  if(proportional!=0)
  {
    power_difference = (proportional>>7);/// + derivative*3;    //?? /20    derivative*3/2
  }
  int max = key->curspd;
  if(power_difference > max)	power_difference = max;
  if(power_difference < -max)power_difference = -max;

  if(proportional>50000)  //??? 50 ?? ??? ??.
  {
    key->lspd=-max>>1;
    key->rspd=max>>1;
  }
  else if(proportional<-50000)
  {
    key->lspd=max>>1;
    key->rspd=-max>>1;
  }
  else
  {
    //////////////////////////////??...
    //if(key->joystick==J_GO)
    {
      if(power_difference < 0)
      {
        key->lspd=max;
        key->rspd=max+(int)power_difference;
      }
      else if(power_difference > 0)	
      {
        key->lspd=max-(int)power_difference;
        key->rspd=max;
      }
      else //??
      {
        key->lspd=max;
        key->rspd=max;
      }
    }

  }
}




void Motor_Speed_setting(motor_TypeDef * Motor,char Use,int16_t L,int16_t  R)
{	
  if(Use == ON)  //????..
  {
    Motor->Value.use=Use;
    //Motor->Value.cap=1;
    
    if(R>Motor->Value.r_sour)  Motor->Value.r_des_dir=CW;
    else  Motor->Value.r_des_dir=CCW;

    if(L>Motor->Value.l_sour)  Motor->Value.l_des_dir=CW;
    else  Motor->Value.l_des_dir=CCW;

    Motor->Value.r_dest=R;
    Motor->Value.l_dest=L;
  }
  else 
  {	
    Motor->Value.use=Use;
    Motor->Value.r_dest=R;
    Motor->Value.r_sour=R;
    Motor->Value.r_cap=0;
    Motor->Value.l_dest=L;
    Motor->Value.l_sour=L;
    //Motor->Value.LM_CAP=0;
    //Mobile_DC(SLOW_DECAY,Motor->Value.l_dest,Motor->Value.r_dest);
  }
}



int32_t lcnt=0;

void LSetspeed(motor_TypeDef * Motor)
{  
	//float dt=Motor->Value.ts;
	float dt = Motor->Value.l_ts/1000.0f;
    Motor->Value.l_ts = 0;
	//dt=0.02f;

	static float last_proportional=0;
	static uint8_t lnew=0;
	float cmd_rps=0;
	float cmd_cpr=0;
	float l_error=0;
	//if(abs(Motor->Value.l_cmdrpm)<10)Motor->Value.l_cmdrpm=0;
	if(Motor->Value.l_cmdrpm==0)
	{
		lnew=1;
		l_derivative=0;
		l_integral=0;
		last_proportional=0;
		Motor->Value.l_sour=0;
		lcurrpm=0;
		DCMOTOR.Value.lcurrpm=lcurrpm;
	//	tohi.has_lenc=DCMOTOR.Value.lcurrpm;
		return;
	}
	else if(lnew)
	{
		setCountsResetLeft();
        lnew=0;
	}
	
	cmd_rps=Motor->Value.l_cmdrpm/60.0f;//최당 회전 수도 환산..
	cmd_rps*=MGEAR; //모터 기준의 기어비 적용 - 모터 자체 rps 환산.
	cmd_cpr =cmd_rps * MCPR;  //초당 회전해야할 cpr 값..
	cmd_cpr /= (1/dt);	//해당 함수 호출시 발생해야 하는 pulse값.
	
	// RPM을 구해야 한다..
	// MCOR(12) * MGEAR(200) = 2400  1회전당 Pulse
	lcnt=getCountsAndResetLeft(); //해당 함수 호출시 발생 하는 pulse값.
	lcurrpm=(int32_t)(lcnt*(1/dt)/MCPR/MGEAR*60);
	DCMOTOR.Value.lcurrpm=lcurrpm;
//	tohi.has_lenc=DCMOTOR.Value.lcurrpm;
	l_error = cmd_cpr-(float)lcnt;
//	uart_printf("%05.3f %07.3f %05.3f %05.3f \r\n",dt,(float)l_error,cmd_cpr,(float)lcnt);
	if(i_gain==0)l_integral=0;
	l_proportional=((float)l_error*p_gain);
	l_integral+=((float)l_error*i_gain*dt);
	l_derivative=-1*((float)(l_proportional-last_proportional)*(d_gain/dt));
	last_proportional = l_proportional; 	//지난 에러 
	if(l_integral>MAX_DC_SPEED)l_integral=MAX_DC_SPEED;
	if(l_integral<-MAX_DC_SPEED)l_integral=-MAX_DC_SPEED;
	if(l_proportional!=0)
	{
		l_power_difference =(long)(l_proportional+l_integral+l_derivative);/// + derivative*3;	  //장차 /20	derivative*3/2	+ rintegral*i_gain;///
	}

	if(l_power_difference > MAX_DC_SPEED)	l_power_difference = MAX_DC_SPEED;
	if(l_power_difference < -MAX_DC_SPEED)  l_power_difference = -MAX_DC_SPEED;

	//if(l_power_difference<0)l_power_difference=0;
	
	Motor->Value.l_sour=l_power_difference;
}


int32_t rcnt=0;

void RSetspeed(motor_TypeDef * Motor)
{  
	//float dt=Motor->Value.ts;
	float dt = Motor->Value.r_ts/1000.0f;;
    Motor->Value.r_ts = 0;

	//dt=0.02f;
	static float last_proportional=0;
	static uint8_t rnew=0;
	float cmd_rps=0;
	float cmd_cpr=0;
	float r_error=0;
	if(DCMOTOR.Value.r_cmdrpm==0)
	{
		rnew=1;
		r_derivative=0;
		r_integral=0;
		last_proportional=0;
		Motor->Value.r_sour=0;
		rcurrpm=0;
		DCMOTOR.Value.rcurrpm=rcurrpm;
	//	tohi.has_renc=DCMOTOR.Value.rcurrpm;
		return;
	}
	else if(rnew)
	{
		setCountsResetRight();rnew=0;
	}
	
	cmd_rps=Motor->Value.r_cmdrpm/60.0f;//최당 회전 수도 환산..
	cmd_rps*=MGEAR; //모터 기준의 기어비 적용 - 모터 자체 rps 환산.
	cmd_cpr =cmd_rps * MCPR;  //초당 회전해야할 cpr 값..
	cmd_cpr /= (1/dt);	//해당 함수 호출시 발생해야 하는 pulse값.
	
	rcnt=getCountsAndResetRight(); //해당 함수 호출시 발생 하는 pulse값.
	rcurrpm=(int32_t)(rcnt*(1/dt)/MCPR/MGEAR*60);
	DCMOTOR.Value.rcurrpm=rcurrpm;
//	tohi.has_renc=DCMOTOR.Value.rcurrpm;
	r_error = cmd_cpr-(float)rcnt;
	
	if(i_gain==0)r_integral=0;
	r_proportional=((float)r_error*p_gain);
	r_integral+=((float)r_error*i_gain*dt);
	r_derivative=-1*((float)(r_proportional-last_proportional)*(d_gain/dt));
	last_proportional = r_proportional;     //지난 에러 
	if(r_integral>MAX_DC_SPEED)r_integral=MAX_DC_SPEED;
	if(r_integral<-MAX_DC_SPEED)r_integral=-MAX_DC_SPEED;
	if(r_proportional!=0)
	{
		r_power_difference =(long)(r_proportional+r_integral+r_derivative);/// + derivative*3;    //장차 /20    derivative*3/2  + rintegral*i_gain;///
	}
	if(r_power_difference > MAX_DC_SPEED)	r_power_difference =  MAX_DC_SPEED;
	if(r_power_difference < -MAX_DC_SPEED)  r_power_difference = -MAX_DC_SPEED;
	
	Motor->Value.r_sour=r_power_difference;
    Motor->Value.ts = 0;
}
