/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "rf.h"
#include "rtc.h"
#include "app_ble.h"
#include "app_entry.h"
#include "app_common.h"
#include "tim.h"
#include "usb.h"
#include "gpio.h"
#include "stm32_seq.h"
#include "tof.h"
#include "tof_vl53l1.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uuid.h"
#include "simple.pb-c.h"
#include "hi2st-parser.h"
#include "timer.h"
#include "Motor.h"
#include "UpdateFW.h"
#include "uart_trace.h"
#include "IR_Trace.h"
#include "Trace_Station.h"
#include "p2p_client_app.h"
#include "Sensor.h"
/* USER CODE END Includes */

#ifndef M_PI
#define M_PI 3.1415926535f
#endif

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void TOFOnOff(uint8_t bOn);
void MotorOnOff(uint8_t bOn);
void DimmingLEDOnOff(uint8_t bOn);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SYSTEM_key  sys;
uint16_t ADC_value[3];
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart1;
extern motor_TypeDef DCMOTOR;
extern int16_t countLeftSending;
extern int16_t countRightSending;
extern int16_t lenc;
extern int16_t renc;
extern APP_BLE_Mode_t BleMode;

extern uint8_t elssen_check_cnt;
extern uint32_t nTimerOnCnt;

ToHi tohi = TO_HI__INIT;
extern ProtobufCAllocator protobuf_c_to_st_allocator;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static uint8_t step = 1;
float m_ROBOT_WHEEL_DISTANCE;
float m_ROBOT_WHEEL_RADIUS  ;
float m_ENCODER_PER_1_CYCLE ;
float m_ROBOT_MOVE_PER_TICK ;
float lx, ly, cx, cy, rx, ry;
float theta;

float bat_vol;
static int BreakMode;

void Sys_Getdata(SYSTEM_key* sdata)
{
    *sdata = sys;
}

void sensorRequestProcess(ToSt *to_st) {
    // Wheel Motor Control
    if( to_st->has_l_speed)
    {
        DCMOTOR.Value.l_cmdrpm = to_st->l_speed;
    }
    if( to_st->has_r_speed)
    {
        DCMOTOR.Value.r_cmdrpm = to_st->r_speed;
    }

    if(to_st->has_neck_speed)
    {
        DCMOTOR.Value.n_sour=to_st->neck_speed;
    }

    // set sensor 'has' values
    if(to_st->has_request_st_info)
    {
        if(to_st->request_st_info&ST_INFO__TOF)      tohi.has_tof=1;
        if(to_st->request_st_info&ST_INFO__ACC)
        {
            tohi.has_accx=1;
            tohi.has_accy=1;
            tohi.has_accz=1;
        }
        if(to_st->request_st_info&ST_INFO__GYRO)
        {
            tohi.has_gyro_x=1;
            tohi.has_gyro_y=1;
            tohi.has_gyro_z=1;
        }
        if(to_st->request_st_info&ST_INFO__IMU)
        {
            IR_DIRECTION temp;
            temp = Ir_GetDirection();
            tohi.has_yaw=1;
            tohi.has_pitch=1;
            tohi.has_roll=1;
            tohi.yaw = (float)temp.long_distance.bytes.long_r;
            tohi.pitch = (float)temp.long_distance.bytes.long_c;
            tohi.roll = (float)temp.long_distance.bytes.long_l;
            tohi.gyro_x = (float)temp.long_distance.bytes.long_r;
            tohi.gyro_y = (float)temp.long_distance.bytes.long_c;
            tohi.gyro_z = (float)temp.long_distance.bytes.long_l;
            tohi.has_battery=1;
            tohi.battery = ST_READ_PG;
        }

        if(to_st->request_st_info&ST_INFO__ENC)
        {
            tohi.has_lenc=1;
            tohi.has_renc=1;
            tohi.lenc = countLeftSending;
            tohi.renc = countRightSending;
            countLeftSending = countRightSending = 0; 
        }

        if(to_st->request_st_info&ST_INFO__BATTERY)
        {
            //tohi.has_battery=1;
        }

    }

    pushToHi(&huart1, &tohi);

    tohi.has_tof=0;
    tohi.has_accx=0;
    tohi.has_accy=0;
    tohi.has_accz=0;
    tohi.has_gyro_x=0;
    tohi.has_gyro_y=0;
    tohi.has_gyro_z=0;
    tohi.has_yaw=0;
    tohi.has_pitch=0;
    tohi.has_roll=0;
    tohi.has_lenc=0;
    tohi.has_renc=0;
}

void onBleMessageFromHi(uint8_t* buf, size_t requiredBytes)
{
    ToSt* tost = to_st__unpack(&protobuf_c_to_st_allocator, requiredBytes, buf);

    if( !tost ) {
        return;
    }

    // HIS -> ST ( F/W update | Motor Control )
    sensorRequestProcess(tost);

    // HOST -> ST ( Control Direct )
    // Wheel Motor Control
    if( tost->has_l_speed)
    {
        DCMOTOR.Value.l_cmdrpm = tost->l_speed;
    }
    if( tost->has_r_speed)
    {
        DCMOTOR.Value.r_cmdrpm = tost->r_speed;
    }

    // Neck Motor Control
    if(tost->has_neck_speed)
    {
        DCMOTOR.Value.n_sour = tost->neck_speed;
    }

    if(tost->sensor_set)
    {
        SENSOR_Set_t sensorset;

        MacAddressStringToBytes((uint8_t *)tost->sensor_set->sensor_mac_address, sensorset.macAddress);

        sensorset.sensortype = tost->sensor_set->sensor_type;
        
        sensorset.bInterruptAlarm = tost->sensor_set->interrupt_alarm;
        
        sensorset.notify_period = tost->sensor_set->notify_period;
        
        sensorset.period_alarm = tost->sensor_set->period_alarm;
        
        sensorset.upper_threshold = tost->sensor_set->upper_threshold;
        
        sensorset.lower_threshold = tost->sensor_set->lower_threshold;
    
        AddSensorSetList(&sensorset, 1);
    }

	if(tost->has_do_charge)
    {
        if(tost->do_charge)
        {
            Set_ChargeMode();
        }
        else
        {
            Clear_ChargeMode();
            Clear_InnerStaticVar();
        }
    }
	
    if(tost->has_test_1)
    {
      TOFOnOff(tost->test_1);
    }

    if(tost->has_test_2)
    {
      MotorOnOff(tost->test_2);
    }

    if(tost->has_test_3)
    {
      DimmingLEDOnOff(tost->test_3);
    }
	if(tost->has_test_10)
    {
        BreakMode ^= 1;
    }

    to_st__free_unpacked(tost, &protobuf_c_to_st_allocator); 
}

// Set-Up only. Another function will comes with [HOST -> HI -> ST] root.
void onBleMessageFromAndroid(uint8_t* buf, size_t requiredBytes)
{
  ToSt* tost = to_st__unpack(&protobuf_c_to_st_allocator, requiredBytes, buf);

  if( tost->has_l_speed)
  {
      DCMOTOR.Value.l_cmdrpm = tost->l_speed;
  }
  if( tost->has_r_speed)
  {
      DCMOTOR.Value.r_cmdrpm = tost->r_speed;
  }

  to_st__free_unpacked(tost, &protobuf_c_to_st_allocator); 
}

void battery_monitor_update(void) {
    //uart_printf("BAT %u\r\n", pin_value);
    bat_vol = (BATADC_VAL * 3.3) / 4096;
    bat_vol *= 2;
}

void gnpSame(int16_t l, int16_t r) {
    float moveDist = l * m_ROBOT_MOVE_PER_TICK;
    lx += moveDist * cosf(theta + (M_PI/2.f));
    ly += moveDist * sinf(theta + (M_PI/2.f));
    rx += moveDist * cosf(theta + (M_PI/2.f));
    ry += moveDist * sinf(theta + (M_PI/2.f));
}

void gnpTurn(int16_t l, int16_t r) {
    int16_t absl = l;
    int16_t absr = r;
    if( l < 0 ) absl = -absl;
    if( r < 0 ) absr = -absr;

    float moveL = absl * m_ROBOT_MOVE_PER_TICK;
    float moveR = absr * m_ROBOT_MOVE_PER_TICK;
    float ox = (moveL*rx + moveR*lx) / (moveL + moveR);
    float oy = (moveL*ry + moveR*ly) / (moveL + moveR);
    float lx_dist = ox-lx, ly_dist = oy-ly;
    float ldist = sqrtf( lx_dist*lx_dist + ly_dist*ly_dist );
    float rx_dist = ox-rx, ry_dist = oy-ry;
    float rdist = sqrtf( rx_dist*rx_dist + ry_dist*ry_dist );
    float dtheta;
    if( absl > absr ) dtheta = moveL / ldist;
    else        dtheta = moveR / rdist;
    if( l > r ) {
        if(dtheta > 0.f) dtheta = -dtheta;
    } else {
        if(dtheta < 0.f) dtheta = -dtheta;
    }

    theta += dtheta;

    lx = ox + ldist * cosf(M_PI + theta);
    ly = oy + ldist * sinf(M_PI + theta);
    rx = ox + rdist * cosf(theta);
    ry = oy + rdist * sinf(theta);
}

void gnpDiff(int16_t l, int16_t r) {
    float moveL = l * m_ROBOT_MOVE_PER_TICK;
    float moveR = r * m_ROBOT_MOVE_PER_TICK;
    float ox = (moveL*rx - moveR*lx) / (moveL-moveR);
    float oy = (moveL*ry - moveR*ly) / (moveL-moveR);
    float lx_dist = ox-lx, ly_dist = oy-ly;
    float ldist = sqrtf( lx_dist*lx_dist + ly_dist*ly_dist );
    float rx_dist = ox-rx, ry_dist = oy-ry;
    float rdist = sqrtf( rx_dist*rx_dist + ry_dist*ry_dist );
    float dtheta;
    if( l ) dtheta = moveL / ldist;
    else dtheta = moveR / rdist;
    if( l > r ) {
        if(dtheta > 0.f) dtheta = -dtheta;
    } else {
        if(dtheta < 0.f) dtheta = -dtheta;
    }
    theta += dtheta;

    if( l < 0 ) l = -l;
    if( r < 0 ) r = -r;

    if( l < r ) {
        lx = ox + ldist * cosf(theta);
        ly = oy + ldist * sinf(theta);
        rx = ox + rdist * cosf(theta);
        ry = oy + rdist * sinf(theta);
    } else {
        lx = ox + ldist * cosf(M_PI + theta);
        ly = oy + ldist * sinf(M_PI + theta);
        rx = ox + rdist * cosf(M_PI + theta);
        ry = oy + rdist * sinf(M_PI + theta);
    }
}

void calpos() {
#if 0
    int16_t l = -lenc;
    int16_t r = -renc;
    lenc = renc = 0;

    if( l || r ) {
        if( l == r ) {
            gnpSame(l, r);
        } else if( l * r < 0 ) {
            gnpTurn(l, r);
        } else {
            gnpDiff(l, r);
        }
    }
    if( theta < 0.f ) theta += 2.f*M_PI;
    else if( theta > 2.f*M_PI ) theta -= 2.f*M_PI;
    tohi.cx = (lx+rx)/2.f;
    tohi.cy = (ly+ry)/2.f;
    tohi.th = theta;
#endif
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Parse_Ble()
{
  uint8_t remain_size = 0;
  remain_size = parseBleFromHi(&onBleMessageFromHi);
  if(remain_size != 0)
  {
    UTIL_SEQ_SetTask( 1<<CFG_TASK_PARSE_BLE_ID, CFG_SCH_PRIO_0);
  }

  remain_size = parseBleFromAndroid(&onBleMessageFromAndroid);
  if(remain_size != 0)
  {
    UTIL_SEQ_SetTask( 1<<CFG_TASK_PARSE_BLE_ID, CFG_SCH_PRIO_0);
  }
}


void Timer_Check()
{
  int state_curr = 0;
  int state_prev = 0;  
  uint16_t dt=0;
  static int16_t tspd=0;

  if(onCheckTimer(Timer_BatCheck)) {
      timerReset(Timer_BatCheck);
      static int b = 0;
      b++;
      if(b % 2) {
          //HAL_GPIO_WritePin(GPIOB, PW_LED_Pin, GPIO_PIN_RESET);
      }
      else {
          //HAL_GPIO_WritePin(GPIOB, PW_LED_Pin, GPIO_PIN_SET);
      }

      battery_monitor_update();
  }
  if(onCheckTimer(Timer_IMU_20ms)) //2.5ms ¼Ò¿ä...
  { 
  	  if(Get_ChargeMode())
      {
        UTIL_SEQ_SetTask( 1<<CFG_TASK_CONTROL_ROBOT_ID, CFG_SCH_PRIO_0);
      }
      timerReset(Timer_IMU_20ms);
      /* get the algorithm state, enable or disable */
      state_curr = GetAlgoState();
      if(state_curr)
      {
          /* if previous state is 0, current state is 1, that means restart the algorithm*/
          if(state_curr != state_prev)
          {
              Enable_Algo();
          }
          /*Read sensor data and call the algorithm function*/
          dt=AlgoPoll();

          calpos();
      }
      state_prev = state_curr;
      /* need to delay some time to get a stable sampling rate */

      tohi.has_power_button_pressed=1;
      tohi.power_button_pressed = HAL_GPIO_ReadPin(GPIOA, PUSH_SW_Pin);

      if( tohi.power_button_pressed ) {
          lx = 1.5f - (m_ROBOT_WHEEL_DISTANCE / 2.f);
          ly = 1.5f;
          cx = 1.5f;
          cy = 1.5f;
          rx = 1.5f + (m_ROBOT_WHEEL_DISTANCE / 2.f);
          ry = 1.5f;
          theta = 0.f;
      }

      {
          static int8_t button_pressed_count = 0;

          // read continuous button pressed time (20ms x 100 = 2 sec)
          if( HAL_GPIO_ReadPin(GPIOA, PUSH_SW_Pin) ) {
              button_pressed_count ++;
          } else {
              button_pressed_count = 0;
          }

          // if over 2 sec button pushed
          if( button_pressed_count > 100 ) {
              static int printcount = 0;
              if( ++printcount > 5 ) { // 100ms
                uart_printf("if button un-pressed exit\r\n");
                printcount = 0;
              }
              button_pressed_count = 100;
              
              // exit hi 
              HAL_GPIO_WritePin(GPIOC, SOC_PW_Pin, GPIO_PIN_RESET);

              // exit st
              HAL_GPIO_WritePin(GPIOA, PW_ON_Pin, GPIO_PIN_RESET);
          }
      }

  }
  if(onCheckTimer(Timer_TOF_10ms))
  {
      timerReset(Timer_TOF_10ms);

      //if(TOF_53l1_MeasureRange(1)==VL53l1_GET_MEASURE)
      {
      }
  }
  if(onCheckTimer(Timer_MOTOR))
  {
      timerReset(Timer_MOTOR);

#define ENCODER_MODE
#ifdef ENCODER_MODE
#if 1
    if(BreakMode)
    {
      if(sys.tof_org_rang < 350)
      {
        //LEFT_BREAK();
        //RIGHT_BREAK();
        DCMOTOR.Value.r_cmdrpm = 0;
        DCMOTOR.Value.l_cmdrpm = 0;
        DCMOTOR.Value.r_sour = 0;
        DCMOTOR.Value.l_sour = 0;
      }
    }
#endif
      if(DCMOTOR.Value.spd!=tspd)
      {
          DCMOTOR.Value.r_cmdrpm=DCMOTOR.Value.spd;
          DCMOTOR.Value.l_cmdrpm=DCMOTOR.Value.spd;
          tspd=DCMOTOR.Value.spd;
      }
      //DCMOTOR.Value.ts=(float)getPeriodTimer(Timer_MOTOR)/1000.0f;
      //DCMOTOR.Value.ts /= 1000.0f;
      LSetspeed(&DCMOTOR);
      RSetspeed(&DCMOTOR);
#else
      DCMOTOR.Value.r_sour=DCMOTOR.Value.spd;
      DCMOTOR.Value.l_sour=DCMOTOR.Value.spd;
#endif
      Mobile_DC(&DCMOTOR);
      if(TILTPOS<1900 && (DCMOTOR.Value.n_sour<0))DCMOTOR.Value.n_sour=0;
      if(TILTPOS>2300 && (DCMOTOR.Value.n_sour>0))DCMOTOR.Value.n_sour=0;
      Neck_DC(&DCMOTOR);
  }


  // adverting Sensor data change
  if(onCheckTimer(Timer_Sensor_Data_1000ms))
  {
    timerReset(Timer_Sensor_Data_1000ms);
    //if(BleMode == APP_BLE_SERVER)
    //{
      //AdvData_Change();
      //pir_irq = 0;
      //hall_irq = 0;
    //}
    //else if(BleMode == APP_BLE_CLIENT)
    if(BleMode == APP_BLE_CLIENT)
    {
      CheckSensorDataSend();
    }
  }

  if(onCheckTimer(Timer_Scan_Canecl))
  {
    killTimer(Timer_Scan_Canecl);
    Scan_Stop();
  }

  if(onCheckTimer(Timer_Elssen_connection_check))
  {
    elssen_check_cnt++;
    if(elssen_check_cnt >= 10) // Connect and receive data in 10 seconds
    {
      ElssenConnectStop();
      killTimer(Timer_Elssen_connection_check);
    }
    else
    {
      timerReset(Timer_Elssen_connection_check);
    }
  }
  
  if(onCheckTimer(Timer_IRCHECK))
  {
      timerReset(Timer_IRCHECK);       
      Ir_Process();
  }

  if(nTimerOnCnt > 0)
  {
    nTimerOnCnt--;
    UTIL_SEQ_SetTask( 1<<CFG_TASK_TIMER_CHECK_ID, CFG_SCH_PRIO_0);
  }

}


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */


    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    /*if(HAL_GPIO_ReadPin(GPIOA, PUSH_SW_Pin))*/
    {
        HAL_GPIO_WritePin(GPIOA, PW_ON_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, SOC_PW_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, PW_LED_Pin, GPIO_PIN_RESET);
    }
        /*HAL_GPIO_WritePin(GPIOC, SOC_PW_Pin, GPIO_PIN_SET);*/
        /*HAL_GPIO_WritePin(GPIOA, PW_ON_Pin, GPIO_PIN_SET);*/
    HAL_GPIO_WritePin(BAT_CHECK_EN_GPIO_Port, BAT_CHECK_EN_Pin, GPIO_PIN_SET);

    MX_DMA_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_USART1_UART_Init();
    HAL_UART_Receive_IT(&huart1, "a", 1);
    MX_USB_PCD_Init();
    MX_ADC1_Init();
    MX_TIM17_Init();
    MX_I2C3_Init();
    MX_LPUART1_UART_Init();
    //HAL_UART_Receive_IT(&hlpuart1, "a", 1);
    MX_TIM2_Init();
    MX_TIM16_Init();
    //MX_RF_Init();
    MX_RTC_Init();
    /* USER CODE BEGIN 2 */
    
    APPE_Init();
    /* registerCallbackBleFromHi(&onBleMessageFromHi); */
    /* registerCallbackBleFromAndroid(&onBleMessageFromAndroid); */
    //setPw(ON);
    //setSocPw(ON);
    HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_value,2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  //V_ref
    setTimer(Timer_TOF_10ms,	10, TIME_RESET);
    setTimer(Timer_IMU_20ms,	20, TIME_RESET);
    setTimer(Timer_MOTOR, 	20, TIME_RESET);
    setTimer(Timer_BatCheck,	500, TIME_RESET);
#if 1
    initializeAllSensors();//GYRO ACCEL
    enableAllSensors();
    Imu_mag_init();
    encoder_init_check();
#endif
    uart_printf("Dev Set\r\n");
    //HAL_UART_Transmit(&huart1, "uart1\r\n", 10, 10);
	Ir_Init_Buffer();

    UTIL_SEQ_RegTask( 1<< CFG_TASK_PARSE_BLE_ID, UTIL_SEQ_RFU, Parse_Ble );
    UTIL_SEQ_RegTask( 1<< CFG_TASK_TIMER_CHECK_ID, UTIL_SEQ_RFU, Timer_Check );
    UTIL_SEQ_RegTask( 1<< CFG_TASK_CONTROL_ROBOT_ID, UTIL_SEQ_RFU, Charge_Process );
    setTimer(Timer_Sensor_Data_1000ms, 1000, TIME_RESET);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

m_ROBOT_WHEEL_DISTANCE = 0.1320f;
m_ROBOT_WHEEL_RADIUS   = 0.0628f;
m_ENCODER_PER_1_CYCLE  = 900.f;
m_ROBOT_MOVE_PER_TICK  = 2 * M_PI * m_ROBOT_WHEEL_RADIUS /                 m_ENCODER_PER_1_CYCLE;
lx = 1.5f - (m_ROBOT_WHEEL_DISTANCE / 2.f);
ly = 1.5f;
cx = 1.5f;
cy = 1.5f;
rx = 1.5f + (m_ROBOT_WHEEL_DISTANCE / 2.f);
ry = 1.5f;
theta = 0.f;

    while (1)
    {
        //HAL_GPIO_TogglePin(GPIOA, TP_Pin);
        UTIL_SEQ_Run( UTIL_SEQ_DEFAULT );
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSI1|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSI;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO3, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}


/* USER CODE BEGIN 4 */
void TOFOnOff(uint8_t bOn)
{
  if(bOn == 1)
  {
    HAL_GPIO_WritePin(FRONTL_SHUT_TOF_GPIO_Port, FRONTR_SHUT_TOF_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(FRONTL_SHUT_TOF_GPIO_Port, FRONTR_SHUT_TOF_Pin, GPIO_PIN_SET);
  }
}

void MotorOnOff(uint8_t bOn)
{
  if(bOn == 1)
  {
    HAL_GPIO_WritePin(MOTO_DRIVE_SLEEP_GPIO_Port, MOTO_DRIVE_SLEEP_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTO_DRIVE_SLEEP_GPIO_Port, MOTO_DRIVE_SLEEP_Pin, GPIO_PIN_SET);
  }
}

void DimmingLEDOnOff(uint8_t bOn)
{
  if(bOn == 1)
  {
    HAL_GPIO_WritePin(TP_GPIO_Port, TP_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(TP_GPIO_Port, TP_Pin, GPIO_PIN_SET);
  }
}


/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{ 
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
