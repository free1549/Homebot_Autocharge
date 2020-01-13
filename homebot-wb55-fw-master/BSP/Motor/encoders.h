// Define to prevent recursive inclusion -------------------------------------
#ifndef __ENCODERS_H
#define __ENCODERS_H

#define L_SIG GPIO_PIN_1
#define R_SIG GPIO_PIN_0


//#define GetGpioB	((SFR16*)GPIOB+0x10) //IDR;/*!< GPIO port input data register,Address offset: 0x10*/
//#define GetGpioC	((SFR16*)GPIOC+0x10) //IDR;/*!< GPIO port input data register,Address offset: 0x10*/

#define GetLeft_XOR()	GetGpioB->Bit.b1
#define GetLeft_B()	 	GetGpioB->Bit.b14

#define GetRight_XOR() 	GetGpioB->Bit.b0
#define GetRight_B()  	GetGpioB->Bit.b13
int16_t getCountsAndResetLeft(void);
int16_t getCountsAndResetRight(void);
int16_t setCountsResetRight(void);
int16_t setCountsResetLeft(void);

#endif // __ENCODERS_H
