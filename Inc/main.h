/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


    /* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
#include "simple.pb-c.h"

    /* Private includes ----------------------------------------------------------*/
    /* USER CODE BEGIN Includes */

    /* USER CODE END Includes */

    /* Exported types ------------------------------------------------------------*/
    /* USER CODE BEGIN ET */

    /* USER CODE END ET */

    /* Exported constants --------------------------------------------------------*/
    /* USER CODE BEGIN EC */

    /* USER CODE END EC */

    /* Exported macro ------------------------------------------------------------*/
    /* USER CODE BEGIN EM */

    /* USER CODE END EM */

    /* Exported functions prototypes ---------------------------------------------*/
    void Error_Handler(void);

    /* USER CODE BEGIN EFP */

    /* USER CODE END EFP */

    /* Private defines -----------------------------------------------------------*/
#define MAX_DC_SPEED 1600
#define SOC_PW_Pin GPIO_PIN_13
#define SOC_PW_GPIO_Port GPIOC
#define BOOT0_Pin GPIO_PIN_3
#define BOOT0_GPIO_Port GPIOH
#define STM_I2C1_SCL_Pin GPIO_PIN_8
#define STM_I2C1_SCL_GPIO_Port GPIOB
#define STM_I2C1_SDA_Pin GPIO_PIN_9
#define STM_I2C1_SDA_GPIO_Port GPIOB
#define TILT_ADC0_Pin GPIO_PIN_0
#define TILT_ADC0_GPIO_Port GPIOC
#define BAT_ADC1_Pin GPIO_PIN_1
#define BAT_ADC1_GPIO_Port GPIOC
#define IMU_INT1_Pin GPIO_PIN_2
#define IMU_INT1_GPIO_Port GPIOC
#define CHG_ENABLE_Pin GPIO_PIN_3
#define CHG_ENABLE_GPIO_Port GPIOC
#define BOOST_STATUS_Pin GPIO_PIN_0
#define BOOST_STATUS_GPIO_Port GPIOA
#define CHARGE_STATUS_Pin GPIO_PIN_1
#define CHARGE_STATUS_GPIO_Port GPIOA
#define PUSH_SW_Pin GPIO_PIN_4
#define PUSH_SW_GPIO_Port GPIOA
#define PW_ON_Pin GPIO_PIN_5
#define PW_ON_GPIO_Port GPIOA
#define IR_RECEIVE_Pin GPIO_PIN_6
#define IR_RECEIVE_GPIO_Port GPIOA
#define BAT_CHECK_EN_Pin GPIO_PIN_7
#define BAT_CHECK_EN_GPIO_Port GPIOA
#define DIR_CH1_Pin GPIO_PIN_4
#define DIR_CH1_GPIO_Port GPIOC
#define DIR_CH2_Pin GPIO_PIN_5
#define DIR_CH2_GPIO_Port GPIOC
#define IR_C_Pin GPIO_PIN_2
#define IR_C_GPIO_Port GPIOB
#define ENC_RX4_Pin GPIO_PIN_0
#define ENC_RX4_GPIO_Port GPIOB
#define ENC_RX4_EXTI_IRQn EXTI0_IRQn
#define ENC_LX4_Pin GPIO_PIN_1
#define ENC_LX4_GPIO_Port GPIOB
#define ENC_LX4_EXTI_IRQn EXTI1_IRQn
#define MOTO_DRIVE_SLEEP_Pin GPIO_PIN_4
#define MOTO_DRIVE_SLEEP_GPIO_Port GPIOE
#define PW_LED_Pin GPIO_PIN_12
#define PW_LED_GPIO_Port GPIOB
#define ENC_RB_Pin GPIO_PIN_13
#define ENC_RB_GPIO_Port GPIOB
#define ENC_LB_Pin GPIO_PIN_14
#define ENC_LB_GPIO_Port GPIOB
#define PG_STATUS_Pin GPIO_PIN_15
#define PG_STATUS_GPIO_Port GPIOB
#define DIR_CH3_Pin GPIO_PIN_6
#define DIR_CH3_GPIO_Port GPIOC
#define TP_Pin GPIO_PIN_15
#define TP_GPIO_Port GPIOA
#define FRONT_SHUT_TOF_Pin GPIO_PIN_10
#define FRONT_SHUT_TOF_GPIO_Port GPIOC
#define DOWNL_SHUT_TOF_Pin GPIO_PIN_11
#define DOWNL_SHUT_TOF_GPIO_Port GPIOC
#define DOWNR_SHUT_TOF_Pin GPIO_PIN_12
#define DOWNR_SHUT_TOF_GPIO_Port GPIOC
#define FRONTL_SHUT_TOF_Pin GPIO_PIN_0
#define FRONTL_SHUT_TOF_GPIO_Port GPIOD
#define FRONTR_SHUT_TOF_Pin GPIO_PIN_1
#define FRONTR_SHUT_TOF_GPIO_Port GPIOD
#define VMREF_RLFO_Pin GPIO_PIN_3
#define VMREF_RLFO_GPIO_Port GPIOB
#define STC_RSTIO_Pin GPIO_PIN_4
#define STC_RSTIO_GPIO_Port GPIOB
#define STC_ALM_Pin GPIO_PIN_5
#define STC_ALM_GPIO_Port GPIOB
    /* USER CODE BEGIN Private defines */
#define TILTPOS		(ADC_value[0])
#define BATADC_VAL	(ADC_value[1])
#define ST_READ_PG  HAL_GPIO_ReadPin(GPIOB, PG_STATUS_Pin)

    typedef union {
        volatile uint16_t DATA;
        struct {
            volatile unsigned b0:1;
            volatile unsigned b1:1;
            volatile unsigned b2:1;
            volatile unsigned b3:1;
            volatile unsigned b4:1;
            volatile unsigned b5:1;
            volatile unsigned b6:1;
            volatile unsigned b7:1;
            volatile unsigned b8:1;
            volatile unsigned b9:1;
            volatile unsigned b10:1;
            volatile unsigned b11:1;
            volatile unsigned b12:1;
            volatile unsigned b13:1;
            volatile unsigned b14:1;
            volatile unsigned b15:1;
        } Bit;
    } SFR16;

    /********************************Gpio Group bit contorl **************************************************/
#define GetGpioA	((SFR16*)(GPIOA_BASE+0x10))
#define GetGpioB	((SFR16*)(GPIOB_BASE+0x10))
#define GetGpioC	((SFR16*)(GPIOC_BASE+0x10))
#define GetGpioD	((SFR16*)(GPIOD_BASE+0x10))
#define GetGpioE	((SFR16*)(GPIOE_BASE+0x10))

    /******************* (C) COPYRIGHT 2007 INSEM Inc ***************************************END OF FILE****/

    /********************************Gpio Group bit contorl **************************************************/
#define SetGpioA	((SFR16*)(GPIOA_BASE+0x14))
#define SetGpioB	((SFR16*)(GPIOB_BASE+0x14))
#define SetGpioC	((SFR16*)(GPIOC_BASE+0x14))
#define SetGpioD	((SFR16*)(GPIOD_BASE+0x14))
#define SetGpioE	((SFR16*)(GPIOE_BASE+0x14))
    /******************* (C) COPYRIGHT 2007 INSEM Inc ***************************************END OF FILE****/


    typedef enum
    {
        OFF	=0U,
        ON	=1U,
    }ON_OFF;


    typedef struct 
    {
        uint16_t control_mode:3;		//0:STANBY  1: BLE 2:STANDALONE  3:Sleep
        uint16_t usb_con:2;	//3:�ʱⰪ 1:����� 0:�̿��� 
        uint16_t chg_sts:2;  //3:�ʱⰪ 1:����   0:������ . 2.�Һ���..
        uint16_t func_tof:1;	//0: �ʱⰪ 1:TOF �Ÿ� Ȯ��
        uint16_t tossingmode:4;
        uint16_t ai_on:1;// �ӽ� ������ PC �������� ��ſ�.
        uint16_t fdummy:3;

        uint8_t soundkey;	//	:4; //0: �ʱⰪ ble ���� �Է� ���� sound key 1~
        uint8_t funckey;	//	:4; //0: �ʱⰪ ble ���� �Է� �޴� func key 1~6   (function Ű�� �з� ��..

        uint8_t backup_funckey;//	:4; //0: �ʱⰪ ble ���� �Է� �޴� func key 1~6   (function Ű�� �з� ��..
        uint8_t joystick;//	:4; //0: �ʱⰪ ble ���� �Է� �޴� joystick key	  (��ư�� ��..

        uint8_t bejoystick;//	:4; //0: �ʱⰪ ble ���� �Է� �޴� joystick key	  ������ ���.(��ư�� ��..

        uint8_t tossingpwd;// :8;

        uint8_t tof_rang;	//cm
        uint8_t tof_org_rang;	//cm
        uint16_t maxspd;//		:10; //0: �ִ� �ӵ� �� ~1000   TEST ���� �߰� ����..(H/W �� ������...���..
        uint16_t backup_maxspd;//		:10; //0: �ִ� �ӵ� �� ~1000   TEST ���� �߰� ����..(H/W �� ������...���..
        uint16_t curspd;//		:10; //0: �ִ� �ӵ� �� ~1000   TEST ���� �߰� ����..(H/W �� ������...���..
        int16_t lspd; //0: 0~10 ���� ����.
        int16_t rspd; //0: 0~10 ���� ����.
        ///20
        float cmd_yaw;
        float yaw;
        float pitch;
        float roll;
        float accx;
        float accy;
        float accz;

        float gyrox;
        float gyroy;
        float gyroz;
        ////28
        uint8_t user_id[16];

        uint16_t set_led:3;
        uint16_t set_r_led:3;
        uint16_t set_l_led:3;
        uint16_t ble_mode_led:3; 
        uint16_t led_mode:4; //0:normal  1:õõ�� ���� ���Ҵ� �Ѵ�.2:õõ�� �����´�. 3:

        uint16_t led_period;

        uint16_t bat_voltage;//voltage*100
        int8_t bat_level_conver; //���� app���� ������ ����.
        uint8_t bat_base_level;  //������ ����..
        uint8_t bat_level;	//���� ó�� �� ����..

        uint16_t bat_safe_time_s; //sec

        uint8_t bat_recharge_cnt;
        uint8_t bat_check_cnt:4;
        uint8_t bat_low:1;
        uint8_t bat_dummy:3;

        uint8_t imu_iAccuracy:2;
        uint8_t yaw_req:1;
        uint8_t yaw_req_cnt:5;

        //uint8_t acc_req:1;
        uint8_t acc_req:1;
        uint8_t stop_req:1;
        uint8_t acc_irq_1:1;
        uint8_t nstep:5;


        uint32_t petAcheType:8; // ����(+1), ����(+2), ȣ��(+4), ��ȭ(+8)
        // �ʱⰪ 0 (���°� ����)

        uint32_t petType:2;     // ������(1), �����(2)
        // �ʱⰪ 0 (eeprom.c ���� 0�̸� �������� ������.)

        uint32_t petAge:5;      // 0 ~ 30��
        // �ʱⰪ 0

        uint32_t petBodyType:3; // ���ָ���(1), ����(2) ����(3) ����(4) ��(5)
        // �ʱⰪ 3 (eeprom.c ���� 0�̸� 3���� ������.)

        uint32_t petKind:2;     // ó������(0), ����(1), ����(2), ����(3)
        // �ʱⰪ 0

        uint32_t familiarity:3; // ģ�е� (������ 0 ~ 6 ������)
        // �ʱⰪ 3

        uint32_t driveMode:3;   // �Ϲ�����(0), ��������(1), �������(2), ������(3)
        // �ʱⰪ 0  �ڡڡڡڡ� eeprom.c  { if(driveMode==0) driveMode = 2 } ����

        uint32_t soundOnDrive:1;// �⺻�Ҹ���(0), �����Ҹ�����(1)
        // �ʱⰪ 1

        uint32_t setCatFish:1;  // ���˴������(0), ���˴�����(1)
        // �ʱⰪ 0

        uint32_t adjust_dummy:4; 



        uint8_t tof_enable;

        uint8_t tof_hz; //sample rate

        uint8_t setuping:1; //0:settup �ƴ� . 1:settup��..
        uint8_t setup_step:2; //setup step
        uint8_t sleep:2; // 0 : normal, 1:sleep
        uint8_t sleepdebug:3; //debug



        uint8_t schedule_count;

        uint8_t tofrawerror:1;	//tof ȹ�� �Լ����� ���� ǥ��.. 6cm����..
        uint8_t toferror:1;		//���������� üũ ����.
        uint8_t hoferror:1;		//���������� üũ ����.
        uint8_t imuerror:1;	//���������� üũ ����.
        uint8_t imuiniterror:1; //�ʱ�ȭ ���� 
        uint8_t maginiterror:1; //�ʱ�ȭ ���� 
        uint8_t mobileSleepStatus:2;   // bit 1 : dont sleep | bit 2 : auto-drive sleep check time.

        uint8_t tofRawErrorCount;	//tof ȹ�� �Լ����� ���� ǥ��..

        //  uint16_t any_dummy;
        //10 
    }SYSTEM_key;

    /* USER CODE END Private defines */

void onBleMessageFromAndroid(uint8_t* buf, size_t requiredBytes);
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
