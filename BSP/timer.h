// Define to prevent recursive inclusion -------------------------------------
#ifndef __TIMER_H
#define __TIMER_H

#define Timer_TOF_10ms	  	Timer1
#define Timer_IMU_20ms	  	Timer2
#define Timer_MOTOR 		Timer3
#define Timer_BatCheck 		Timer4
#define Timer_IRCHECK		Timer5
#define Timer_Scan_Canecl		Timer6
#define Timer_Sensor_Data_1000ms		Timer7
#define Timer_Elssen_connection_check		Timer8

#define	TIMER_SYSTEM_QTY	11
#define	TIME_CONTINUE		0
#define	TIME_RESET			1

struct  TIMER_BIT{      // bit   description
	uint16_t		Period	;		// unit ms
	uint16_t		Count	;		// unit ms
	uint16_t	 	Start	:1;
	uint16_t		On		:1;
	uint16_t		Flag	:1;
	uint16_t		Reset	:1;
	uint16_t		Pause	:1;
	uint16_t		Use		:1;
	uint16_t		rsvd	:10;
};
typedef enum
{
  Timer1 		= 0U,
  Timer2		= 1U,
  Timer3		= 2U,
  Timer4		= 3U,
  Timer5		= 4U,
  Timer6		= 5U,
  Timer7		= 6U,
  Timer8		= 7U,
  Timer9		= 8U,
  Timer10		= 9U,
  Timer11		= 10U,
}Timer_type;  /*0~10*/

#define Timer_MOTOR_DELAY	Timer1

void timerReset(Timer_type TimerNo);
void setTimer(Timer_type TimerNo, unsigned int TimePeriod, int nMode);
void killTimer(Timer_type TimerNo);
uint8_t liveCheckTimer(Timer_type TimerNo);
uint8_t onCheckTimer(Timer_type TimerNo);
uint16_t GetSystemTimeStamp(void);

#endif // __TIMER_H
