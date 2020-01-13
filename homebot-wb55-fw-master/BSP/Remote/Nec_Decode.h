/*
 * NEC_Decode.h
 *
 *  Created on: Mar 9, 2016
 *      Author: peter
 */

#ifndef INC_NEC_DECODE_H_
#define INC_NEC_DECODE_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

//REMOTE VALUE
#define CMD_MODE0		0xB0 //SHOOT MODE 0
#define CMD_MODE1 		0x70 //right
#define CMD_MODE2		0xF0 //left

#define CMD_SPK1 		0x8E //SPK BEEP 1
#define CMD_SPK2		0x4E //SPK BEEP 2

#define CMD_GO			0x83 //GO
#define CMD_RIGHT 		0x23 //right
#define CMD_LEFT 		0xc3 //left
#define CMD_BACK 		0x43 //back
#define CMD_STOP 		0x03 //stop
#define CMD_PLAY 		0xB4 //PLAY

#define CMD_TIMEUP 		0x2a //TIME up
#define CMD_TIMEDOWN 	0xca //TIME down
#define CMD_FAST 		0x56 //VELOCITY UP
#define CMD_SLOW 		0xd6 //VELOCITY DOWN
#define CMD_SPKUP 		0x20 //SPK VOL UP
#define CMD_SPKDOWN 	0xA0 //SPK VOL DOWN



typedef enum {
    NEC_NOT_EXTENDED, NEC_EXTENDED
} NEC_TYPE; //Extended NEC Set Var

typedef enum {
    NEC_INIT, NEC_AGC_OK, NEC_AGC_FAIL, NEC_FAIL, NEC_OK, NEC_ERR
} NEC_STATE; //Decoding State Var

typedef struct {
    int rawTimerData[32]; //Capture Address+Command Var
	int rawHeaderData[2]; //Capture Header Var
    uint8_t decoded[4]; //Decoded Var
	uint8_t B_Decoded[4]; //Before Decoded Var
	uint8_t Cmd_ok;
	uint8_t Systick_Flag;
	uint16_t Systick_Count;
	uint16_t data;
	uint16_t data_bit;
	
	uint16_t RL;
	uint16_t CL;
	uint16_t LL;
	uint16_t RS;
	uint16_t CS;
	uint16_t LS;
	
    NEC_STATE state; //Decoding State Var
	NEC_STATE Before_State; //Before Decoding State Var

    TIM_HandleTypeDef *timerHandle; //Timer Var

    uint32_t timerChannel; //Timer Channel Var
  //  HAL_TIM_ActiveChannel timerChannelActive; //Timer Channel Var 

    uint16_t timingBitBoundary; //Bit(0 or 1) Check Var
    uint16_t timingAgcBoundary; //Heaer Check Var
    NEC_TYPE type;

    void (*NEC_DecodedCallback)(uint16_t, uint8_t,int16_t,int16_t);
    void (*NEC_ErrorCallback)(int16_t,int16_t);
    void (*NEC_RepeatCallback)();
} NEC;



void Nec_Init(TIM_HandleTypeDef* htim,uint32_t Channel);

void NEC_TIM_IC_CaptureCallback(NEC* handle);

void NEC_Read(NEC* handle);

void myNecDecodedCallback(uint16_t address, uint8_t cmd ,int16_t HH,int16_t HL);

void myNecErrorCallback(int16_t HH,int16_t HL);

void myNecRepeatCallback(void);

#endif /* INC_NEC_DECODE_H_ */
