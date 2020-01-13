// Define to prevent recursive inclusion -------------------------------------
#ifndef __MOTOR_H
#define __MOTOR_H

#define MTIM	 htim1
#define STIM	 htim1

//#define MAX_DC_SPEED 2500
#define DEFAULT_DC_SPEED 1000

#define MGEAR 250
#define MCPR 12


#define MOBILE_WAKEUP 	SetGpioE->Bit.b4=1 //set
#define MOBILE_SLEEP  	SetGpioE->Bit.b4=0 //reset

#define NECK_DIR_CW 	SetGpioC->Bit.b6=1 //set
#define NECK_DIR_CCW  	SetGpioC->Bit.b6=0 //reset

#define LEFT_DIR_CW 	SetGpioC->Bit.b5=1 //set
#define LEFT_DIR_CCW  	SetGpioC->Bit.b5=0 //reset

#define RIGHT_DIR_CW 	SetGpioC->Bit.b4=0 //set
#define RIGHT_DIR_CCW  	SetGpioC->Bit.b4=1 //reset


#define CMD_GO			0x83 //GO
#define CMD_RIGHT 		0x23 //right
#define CMD_LEFT 		0xc3 //left
#define CMD_BACK 		0x43 //back
#define CMD_STOP 		0x03 //stop

typedef enum
{
  CCW = 0U,
  CW
}Motor_dir;

typedef enum 
{
  SLOW_DECAY    = 0x00U,
  FAST_DECAY    = 0x01U,
} ControlMode;


struct RXX_MOTOR
{
	int16_t RM;
	int16_t LM;
	int8_t STEP_DIR;
	uint8_t STEP_USE;
	int16_t STEP2;
    int16_t VRADC;
    char MODE;
    char USE;
};


typedef union 
{
	struct RXX_MOTOR Value;
	char Buf[sizeof(struct RXX_MOTOR)];
} Control_Motor_TypeDef;

struct RX_MOTOR
{
	int16_t r_dest;
	int16_t r_sour;
    int16_t r_cap; 
    int16_t r_des_dir;
    int16_t r_cmdrpm;
	int32_t rcurrpm;
	int16_t l_dest;
	int16_t l_sour;
    int16_t l_cap;
    int16_t l_des_dir;
    int16_t l_cmdrpm;
	int32_t lcurrpm;
    int16_t lpos;
	int16_t n_sour;
    int16_t npos;
    int16_t spd;
    char cap;
    char use;
    char iAccuracy;
    char FLAG1;
    char Dec;
    uint8_t rmsts:2; //0 stop  1; go run  2: back run
    uint8_t lmsts:2; //0 stop  1; go run  2: back run
    uint8_t sts:4; //0 stop  sts!=0  run
    uint16_t mtrdelay;
    uint16_t mtrdelay_start:4;
    uint16_t mtrdelay_end:4;
	
	uint16_t curTStamp;
	uint16_t preTStamp;
	float ts; //sampling interval, unit is second.
	uint8_t restartTS;
    float l_ts;
    float r_ts;
};

typedef union 
{
	struct RX_MOTOR Value;
	char Buf[sizeof(struct RX_MOTOR)];
}motor_TypeDef;


typedef struct {
	int16_t lspd;
	int16_t  rspd;
	uint16_t  delay;	//ms
	uint8_t  action_type;	//0 : noaction 1: tossing  2: 
	uint8_t  accenable;	
	uint8_t  end;	
} move_TypeDef;

#define USR_SPD	(2000)

//*/


void Mobile_DC(motor_TypeDef * Motor);
//id Tossing_DC(int tspd);
//id Motion_Func_Start(SYSTEM_key *key);
//id ACC_DEC_Mobile_USE(SYSTEM_key *key);
//id Func_Move_Step(SYSTEM_key *key);
//id Tossing(SYSTEM_key *key);
//id JoysticToCtrl(SYSTEM_key *key,int16_t angle);//,char spd)

#endif 

