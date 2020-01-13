/*
 * NEC_Decode.c
 *
 *  Created on: Mar 9, 2016
 *      Author: peter
 */
#include "string.h"
#include "NEC_Decode.h"
#include "Fifo.h"
#include "main.h"

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;

uint8_t RemoteDataBuf[20]={0,};

NEC nec;

#define debug_print  HAL_UART_Transmit_DMA

void B_Decoded_Set(void){
	for(int a=0; a<4; a++)
	{
		nec.B_Decoded[a] = nec.decoded[a]; 
	}
}
	
void NecRepeatCheck(uint8_t ADD, uint8_t COM, uint8_t ADD_C, uint8_t COM_C){
	if(((nec.B_Decoded[0] == ADD) && (nec.B_Decoded[2] == COM)) && ((nec.B_Decoded[1] == ADD_C) && (nec.B_Decoded[3] == COM_C)))
  	{
		//LED_ON();
	}	
}

void myNecDecodedCallback(uint16_t address, uint8_t cmd ,int16_t HH,int16_t HL) {
 //   char buff[100];
	//B_Decoded_Set();
	// SetFIFO((FifoHandle)RemoteDataBuf, (uint8_t)cmd);
//	nec.Cmd_ok=1;
//	nec.Systick_Count=0;
//	Remote_motor(cmd);
	nec.Systick_Flag = 1;//Systick Delay 10ms and Call func NEC_Read()
//	uart_printf("A:0x%02x\tC:0x%02x\r\n", address, cmd);
   // snprintf(buff, 100, "A:0x%x\tC:0x%x %d\r\n", address, cmd,nec.Systick_Count);
	//HAL_UART_Transmit_DMA(&huart1, (uint8_t*) buff, strlen(buff));
	//SetTimer(Timer_150ms, 150, TIME_RESET);
}

void myNecErrorCallback(int16_t HH,int16_t HL) {
    char buff[100];
    snprintf(buff, 100, "E=%d %02d\r\n", HH,HL);
	debug_print(&huart1, (uint8_t*) buff, strlen(buff));
	nec.state = NEC_INIT;
	nec.rawHeaderData[0] = 0;
	nec.rawHeaderData[1] = 0;
	nec.decoded[1]=0;
	nec.decoded[2]=0;
	nec.Cmd_ok=0;
	nec.Systick_Flag = 1;//Systick Delay 10ms and Call func NEC_Read()
}

void myNecRepeatCallback(void) {
   // char buff[100];
	//nec.Cmd_ok=1;

	
	//NecRepeatCheck(0x80, 0xC0, 0x7F, 0x3F);
	//nec.Systick_Count=0;
	//Remote_motor(1,nec.B_Decoded[2]);
	//nec.Systick_Flag = 1;//Systick Delay 10ms and Call func NEC_Read()
	
  //  uart_printf("R:0x%x\r\n", nec.decoded[2]);
 //   snprintf(buff, 100, "R:0x%x\r\n", nec.decoded[2]);
//	debug_print(&huart1, (uint8_t*) buff, strlen(buff));
}
#if 1
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  char buff[100];
  snprintf(buff, 100, "kkkk=%d\r\n", nec.rawHeaderData[1]);
  debug_print(&huart1, (uint8_t*) buff, strlen(buff));

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_TIM_ErrorCallback could be implemented in the user file
   */
}
#endif
uint32_t t=0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t cnt=0;
	t=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
	if (nec.state == NEC_INIT )
	{	
		if((t < 1100)&& (t> 900)) //header
		{	
            nec.state = NEC_AGC_OK;
            nec.data_bit=0;
            nec.data=0;
  		    HAL_TIM_IC_Stop_IT(htim,  nec.timerChannel);
			HAL_TIM_Base_Start_IT(&htim6);
			//uart_printf(":%d\r\n",t);
		}
	}
}

void Nec_Init(TIM_HandleTypeDef* htim,uint32_t Channel)
{
	HAL_TIM_IC_Start_IT(htim,  Channel);
  	nec.timerHandle = htim;
    nec.timerChannel = Channel;
   // nec.timerChannelActive = HAL_TIM_ACTIVE_CHANNEL_1;
    nec.timingBitBoundary = 550;//(2.25ms+1.12ms)/2 : bit timing
    nec.timingAgcBoundary = 1100;//(13.5ms+11.5ms)/2 : header timing 
    nec.type = NEC_EXTENDED;
    nec.NEC_DecodedCallback = myNecDecodedCallback;
    nec.NEC_ErrorCallback = myNecErrorCallback;
    nec.NEC_RepeatCallback = myNecRepeatCallback;

	nec.Systick_Flag=0;
	nec.Systick_Count=0;
}
void Nec_DeInit(TIM_HandleTypeDef* htim,uint32_t Channel)
{
	HAL_TIM_IC_Stop_IT(htim,  Channel);
  	nec.timerHandle = htim;
    nec.timerChannel = Channel;
   // nec.timerChannelActive = HAL_TIM_ACTIVE_CHANNEL_1;
    nec.timingBitBoundary = 550;//(2.25ms+1.12ms)/2 : bit timing
    nec.timingAgcBoundary = 1100;//(13.5ms+11.5ms)/2 : header timing 
    nec.type = NEC_EXTENDED;
    nec.NEC_DecodedCallback = myNecDecodedCallback;
    nec.NEC_ErrorCallback = myNecErrorCallback;
    nec.NEC_RepeatCallback = myNecRepeatCallback;

	nec.Systick_Flag=0;
	nec.Systick_Count=0;
}

void stationPoscheck(void)
{
	
	uart_printf(":%d\r\n",t);
	
	//nec.LS
	
}
