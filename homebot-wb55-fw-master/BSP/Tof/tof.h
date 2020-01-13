// Define to prevent recursive inclusion -------------------------------------
#ifndef __TOF_H
#define __TOF_H
#include "stm32wbxx_hal.h"


#include "X-NUCLEO-53L0A1.h"
#include "vl53l0x_api.h"
#include "vl53l0x_api_calibration.h"
extern I2C_HandleTypeDef hi2c2;

#define FRONT_SHUT_TOF_ON 		SetGpioB->Bit.b14=1 //set
#define FRONT_SHUT_TOF_OFF  	SetGpioB->Bit.b14=0 //reset

#define DOWNL_SHUT_TOF_ON 		SetGpioB->Bit.b15=1 //set
#define DOWNL_SHUT_TOF_OFF  	SetGpioB->Bit.b15=0 //reset

#define FRONTL_SHUT_TOF_ON 		SetGpioC->Bit.b6=1 //set
#define FRONTL_SHUT_TOF_OFF  	SetGpioC->Bit.b6=0 //reset

#define FRONTR_SHUT_TOF_ON 		SetGpioC->Bit.b7=1 //set
#define FRONTR_SHUT_TOF_OFF  	SetGpioC->Bit.b7=0 //reset

#define DOWNR_SHUT_TOF_ON 		SetGpioC->Bit.b8=1 //set
#define DOWNR_SHUT_TOF_OFF  	SetGpioC->Bit.b8=0 //reset

typedef enum {
	LONG_RANGE 		= 0, /*!< Long range mode */
	HIGH_SPEED 		= 1, /*!< High speed mode */
	HIGH_ACCURACY	= 2, /*!< High accuracy mode */
} RangingConfig_e;

typedef enum {
//	TOF_RESET = 0,
//	TOF_DETECT,
  TOF_DETECT =0,
  TOF_START ,
  TOF_WAIT_START,
  TOF_WAIT_MEASURE,
  TOF_GET_MEASURE,
} TofMeasureStep;

uint8_t TOF_MeasureRange(uint16_t Id);

#endif // __TOF_H
