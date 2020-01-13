/**
 ******************************************************************************
 * @file    accelerometer.h
 * @author  MEMS Application Team
 * @version V4.2.0
 * @date    01-February-2018
 * @brief   This header file contains the functions prototypes for the
 *          accelerometer driver
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ACCELEROMETER_H
#define __ACCELEROMETER_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "sensor.h"

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup COMMON COMMON
 * @{
 */

/** @addtogroup ACCELEROMETER ACCELEROMETER
 * @{
 */

/** @addtogroup ACCELEROMETER_Public_Types ACCELEROMETER Public types
 * @{
 */

/**
 * @brief  ACCELEROMETER driver structure definition
 */
typedef struct
{
  DrvStatusTypeDef ( *Init            ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *DeInit          ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Enable   ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Disable  ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_WhoAmI      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Check_WhoAmI    ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_Axes        ) ( DrvContextTypeDef*, SensorAxes_t* );
  DrvStatusTypeDef ( *Get_AxesRaw     ) ( DrvContextTypeDef*, SensorAxesRaw_t* );
  DrvStatusTypeDef ( *Get_Sensitivity ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Get_ODR         ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Set_ODR         ) ( DrvContextTypeDef*, SensorOdr_t );
  DrvStatusTypeDef ( *Set_ODR_Value   ) ( DrvContextTypeDef*, float );
  DrvStatusTypeDef ( *Get_FS          ) ( DrvContextTypeDef*, float* );
  DrvStatusTypeDef ( *Set_FS          ) ( DrvContextTypeDef*, SensorFs_t );
  DrvStatusTypeDef ( *Set_FS_Value    ) ( DrvContextTypeDef*, float );
  DrvStatusTypeDef ( *Get_Axes_Status ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Set_Axes_Status ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Read_Reg        ) ( DrvContextTypeDef*, uint8_t, uint8_t* );
  DrvStatusTypeDef ( *Write_Reg       ) ( DrvContextTypeDef*, uint8_t, uint8_t );
  DrvStatusTypeDef ( *Get_DRDY_Status ) ( DrvContextTypeDef*, uint8_t* );
//  DrvStatusTypeDef ( *Free_Fall_Detection ) ( DrvContextTypeDef*, SensorIntPin_t );
} ACCELERO_Drv_t;

/*ACCELERO_Drv_t LSM6DS3_X_Drv =
{
  LSM6DS3_X_Init,
  LSM6DS3_X_DeInit,
  LSM6DS3_X_Sensor_Enable,
  LSM6DS3_X_Sensor_Disable,
  LSM6DS3_X_Get_WhoAmI,
  LSM6DS3_X_Check_WhoAmI,
  LSM6DS3_X_Get_Axes,
  LSM6DS3_X_Get_AxesRaw,
  LSM6DS3_X_Get_Sensitivity,
  LSM6DS3_X_Get_ODR,
  LSM6DS3_X_Set_ODR,
  LSM6DS3_X_Set_ODR_Value,
  LSM6DS3_X_Get_FS,
  LSM6DS3_X_Set_FS,
  LSM6DS3_X_Set_FS_Value,
  LSM6DS3_X_Get_Axes_Status,
  LSM6DS3_X_Set_Axes_Status,
  LSM6DS3_X_Read_Reg,
  LSM6DS3_X_Write_Reg,
  LSM6DS3_X_Get_DRDY_Status,
  LSM6DS3_X_Enable_Free_Fall_Detection
};
*/
/**
 * @brief  ACCELEROMETER data structure definition
 */
typedef struct
{
  void *pComponentData; /* Component specific data. */
  void *pExtData;       /* Other data. */
} ACCELERO_Data_t;

typedef enum
{
  X_AXIS = 0,
  Y_AXIS,
  Z_AXIS,
  ALL_ACTIVE
} ACTIVE_AXIS_t;

typedef enum
{
  NORMAL_MODE,
  HIGH_RES_MODE,
  LOW_PWR_MODE
} OP_MODE_t;

typedef enum
{
  INT1_DRDY_DISABLED,
  INT1_DRDY_ENABLED
} INT1_DRDY_CONFIG_t;

typedef enum
{
  DRDY_PULSED,
  DRDY_LATCHED
} DRDY_MODE_t;


/**
 * @brief  ACCELEROMETER hardware features status data structure definition
 */
typedef struct
{
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
} ACCELERO_Event_Status_t;

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __ACCELEROMETER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
