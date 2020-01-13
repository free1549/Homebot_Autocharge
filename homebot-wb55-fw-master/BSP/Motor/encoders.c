

#include "stm32wbxx_hal.h"
#include "main.h"
#include "encoders.h"
#include "stdlib.h"
#include "stdbool.h"

static bool lastLeftA;
static bool lastLeftB;
static bool lastRightA;
static bool lastRightB;

static bool errorLeft;
static bool errorRight;

// These count variables are uint16_t instead of int16_t because
// signed integer overflow is undefined behavior in C++.
uint16_t countLeft;
uint16_t countRight;
int16_t countLeftSending;
int16_t countRightSending;

int16_t lenc;
int16_t renc;


static void setLeftEncoder(void)
{
    bool newLeftA = GetLeft_B();
    bool newLeftB = GetLeft_XOR() ^ newLeftA; //xor 하여 a상 추출.

    countLeft += (lastLeftA ^ newLeftB) - (newLeftA ^ lastLeftB);
    countLeftSending += (lastLeftA ^ newLeftB) - (newLeftA ^ lastLeftB);
    lenc += (lastLeftA ^ newLeftB) - (newLeftA ^ lastLeftB);

    if((lastLeftA ^ newLeftA) & (lastLeftB ^ newLeftB))
    {
        errorLeft = true;
    }
//	if(newLeftA==SET)TP_ON;
//	else if(newLeftA!=SET)TP_OFF;
//	uart_printf("[%d %d]\r\n",lastLeftA,lastLeftB);
//	uart_printf("%d += (%d ^ %d) - (%d ^ %d) \r\n",countLeft,lastLeftA,newLeftB,newLeftA,lastLeftB);
    lastLeftA = newLeftA;
    lastLeftB = newLeftB;
}

static void setRightEncoder(void)
{
    bool newRightB = GetRight_B();
    bool newRightA = GetRight_XOR() ^ newRightB;

    countRight += (lastRightA ^ newRightB) - (newRightA ^ lastRightB);
    countRightSending += (lastRightA ^ newRightB) - (newRightA ^ lastRightB);
    renc += (lastRightA ^ newRightB) - (newRightA ^ lastRightB);

    if((lastRightA ^ newRightA) & (lastRightB ^ newRightB))
    {
        errorRight = true;
    }

    lastRightA = newRightA;
    lastRightB = newRightB;
}

void encoder_init(void)
{
    lastLeftB = GetLeft_B();
    lastLeftA = GetLeft_XOR() ^ lastLeftB;
    countLeft = 0;
    countLeftSending = 0;
    lenc = 0;
    errorLeft = 0;

    lastRightB = GetRight_B();
    lastRightA = GetRight_XOR() ^ lastRightB;
    countRight = 0;
    countRightSending = 0;
    renc = 0;
    errorRight = 0;
	//uart_printf("[%d %d] [%d %d]\r\n",lastLeftA,lastLeftB,lastRightA,lastRightB);
}

void encoder_init_check(void)
{
   static bool initialized = 0;
   if (!initialized)
   {
	   initialized = true;
	   encoder_init();
   }
}

int16_t getCountsLeft(void)
{
    encoder_init_check();

    int16_t counts = countLeft;
    return counts;
}

int16_t getCountsRight(void)
{
    encoder_init_check();

    int32_t counts = countRight;
    return counts;
}

int16_t getCountsAndResetLeft(void)
{
    encoder_init_check();

    int16_t counts = countLeft;
    countLeft = 0;
    return counts;
}

int16_t getCountsAndResetRight(void)
{
    encoder_init_check();

    int16_t counts = countRight;
    countRight = 0;
    return counts;
}
int16_t setCountsResetLeft(void)
{
    encoder_init_check();
    countLeft = 0;
    return countLeft;
}

int16_t setCountsResetRight(void)
{
    encoder_init_check();
    countRight = 0;
    return countRight;
}


bool checkErrorLeft(void)
{
    encoder_init_check();

    bool error = errorLeft;
    errorLeft = 0;
    return error;
}

bool checkErrorRight(void)
{
    encoder_init_check();

    bool error = errorRight;
    errorRight = 0;
    return error;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(GPIO_Pin);
  //setLeftEncoder();
 
  switch(GPIO_Pin)
  {
	case R_SIG:
		setRightEncoder();
	break;
  	case L_SIG:
		//TP_TOGGLE;
  		setLeftEncoder();
	break;
  }
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}



