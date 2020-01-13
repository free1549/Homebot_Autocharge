// Define to prevent recursive inclusion -------------------------------------
#ifndef __TOFVL53L1_H
#define __TOFVL53L1_H
#include "stm32wbxx_hal.h"

#define FRONT_SHUT_TOF_l1_ON 		SetGpioD->Bit.b0=1 //set
#define FRONT_SHUT_TOF_l1_OFF  	SetGpioD->Bit.b0=0 //reset

#define FRONTL_SHUT_TOF_l1_ON 		SetGpioC->Bit.b6=1 //set
#define FRONTL_SHUT_TOF_l1_OFF  	SetGpioC->Bit.b6=0 //reset

#define FRONTR_SHUT_TOF_l1_ON 		SetGpioC->Bit.b7=1 //set
#define FRONTR_SHUT_TOF_l1_OFF  	SetGpioC->Bit.b7=0 //reset


enum XNUCLEO53L1A1_dev_e{
    XNUCLEO53L1A1_DEV_LEFT =  0,    //!< left satellite device P21 header : 'l'
    XNUCLEO53L1A1_DEV_CENTER  =  1, //!< center (built-in) vl053 device : 'c"
    XNUCLEO53L1A1_DEV_RIGHT=  2     //!< Right satellite device P22 header : 'r'
};

typedef enum {
//	TOF_RESET = 0,
//	TOF_DETECT,
  VL53l1_DETECT =0,
  VL53l1_BOOT ,
  VL53l1_INITSTART,
  VL53l1_WAIT_MEASURE,
  VL53l1_GET_MEASURE,
} VL53l1_MeasureStep;

#endif // __TOFVL53L1_H
