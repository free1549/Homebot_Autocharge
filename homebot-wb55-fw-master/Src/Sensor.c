
//#include "stm32f4xx_hal.h"
#include "main.h"
#include "Sensor.h"
#include "timer.h"
#include "p2p_server_app.h"
//#include "bq27441.h"
#include "hi2st-parser.h"
//#include "envSensor.h"
#include "flash_if.h"


uint8_t hall_irq = 0;
uint8_t pir_irq = 0;
GPIO_PinState pir_board_status = GPIO_PIN_RESET;
uint8_t sensor_type = (SENSOR_TYPE_TEMPERATURE | SENSOR_TYPE_HUMIDITY | SENSOR_TYPE_ECO2 | SENSOR_TYPE_TVOC); //SENSOR_TYPE_PIR, SENSOR_TYPE_HALL, (SENSOR_TYPE_TEMPERATURE | SENSOR_TYPE_HUMIDITY | SENSOR_TYPE_ECO2 | SENSOR_TYPE_TVOC));

SENSOR_Data_t sensor_data_list[20];
uint8_t sensor_data_list_cnt = 0;

SENSOR_Set_t sensor_set_list[20];
uint8_t sensor_set_list_cnt = 0;



extern UART_HandleTypeDef huart1;

extern float TMP117_temp;
extern float shtc1_hum;
extern float shtc1_temp;
extern uint16_t ECO2_ppm;
extern uint16_t TVOC_ppb;
extern uint8_t *server_bd_addr;
extern HAL_StatusTypeDef env_board_status;
extern APP_BLE_Mode_t BleMode; // APP_BLE_SERVER, APP_BLE_CLIENT
//extern BQ27441_Struct Batgauge;

void CheckThreshold(uint8_t data_index);

extern uint64_t Read_Flash(uint32_t Address);
void Sensor_Init()
{
#if 0
  uint32_t tickstart;

  // variable init for client
  g_temperature_set.interrupt_alarm = 1;
  g_temperature_set.notify_period = 5; //sec
  g_temperature_set.period_alarm = 1;
  g_temperature_set.upper_threshold = 40;
  g_temperature_set.lower_threshold = 0;

  g_humidity_set.interrupt_alarm = 1;
  g_humidity_set.notify_period = 5; //sec
  g_humidity_set.period_alarm = 1;
  g_humidity_set.upper_threshold = 90;
  g_humidity_set.lower_threshold = 30;

  g_hall_set.interrupt_alarm = 1;
  g_hall_set.notify_period = 5; //sec
  g_hall_set.period_alarm = 1;

  g_pir_set.interrupt_alarm = 1;
  g_pir_set.notify_period = 5; //sec
  g_pir_set.period_alarm = 1;

  g_eco2_set.interrupt_alarm = 1;
  g_eco2_set.notify_period = 5; //sec
  g_eco2_set.period_alarm = 1;
  g_eco2_set.upper_threshold = 1000;
  g_eco2_set.lower_threshold = 0;

  g_tvoc_set.interrupt_alarm = 1;
  g_tvoc_set.notify_period = 5; //sec
  g_tvoc_set.period_alarm = 1;
  g_tvoc_set.upper_threshold = 40;
  g_tvoc_set.lower_threshold = 0;
  /////////////////////////////////

  SGP30IagInit();

  /* Get timeout */
  tickstart = HAL_GetTick();

  sensor_type = SENSOR_TYPE_PIR; //SENSOR_TYPE_PIR, SENSOR_TYPE_HALL, (SENSOR_TYPE_TEMPERATURE | SENSOR_TYPE_HUMIDITY | SENSOR_TYPE_ECO2 | SENSOR_TYPE_TVOC))

#if 0
  /* Wait till PIR is ready */
  while (pir_board_status == GPIO_PIN_RESET)
  {
    pir_board_status = HAL_GPIO_ReadPin(PIR_COUT_GPIO_Port, PIR_COUT_Pin);
  
    if ((HAL_GetTick() - tickstart) > 1000)
    {
      break;
    }
  }
  
  sensor_type = SENSOR_TYPE_HALL;
  
  if(pir_board_status == GPIO_PIN_SET)
  {
    sensor_type = SENSOR_TYPE_PIR;
  }
  
  if(env_board_status == HAL_OK)
  {
    sensor_type = SENSOR_TYPE_TEMPERATURE | SENSOR_TYPE_HUMIDITY | SENSOR_TYPE_ECO2 | SENSOR_TYPE_TVOC;
  }
#endif

  if(sensor_type == (SENSOR_TYPE_TEMPERATURE | SENSOR_TYPE_HUMIDITY | SENSOR_TYPE_ECO2 | SENSOR_TYPE_TVOC))
  {
    setTimer(Timer_SHTC1_1000ms, 1000, TIME_RESET);
    setTimer(Timer_SGP30_1000ms, 1000, TIME_RESET);
  }
#endif
}

int8_t GetSensorDataListIndex(tBDAddr mac)
{
  for(uint8_t index = 0; index < sensor_data_list_cnt; index++)
  {
    if(memcmp(sensor_data_list[index].macAddress, mac, sizeof(tBDAddr)) == 0)
    {
      return index;
    }
  }

  return -1;
}

void AddSensorDataList(SENSOR_Data_t *sensor_data)
{
  int8_t index = GetSensorDataListIndex(sensor_data->macAddress);
  int8_t set_index;
  SENSOR_Set_t sensorset;

  if(index == (-1))
  {
    index = sensor_data_list_cnt;
    memcpy(&sensor_data_list[sensor_data_list_cnt], sensor_data, sizeof(SENSOR_Data_t));
    sensor_data_list_cnt++;

    // check set list, if not in set list then set default set to send data
    // one device has multi sensors
    if(sensor_data_list[index].sensortype == (SENSOR_TYPE_TEMPERATURE | SENSOR_TYPE_HUMIDITY | SENSOR_TYPE_ECO2 | SENSOR_TYPE_TVOC))
    {
      set_index = GetSensorSetListIndex(sensor_data->macAddress, SENSOR_TYPE_TEMPERATURE);
      if(set_index == (-1))
      {
        memcpy(sensorset.macAddress, sensor_data->macAddress, sizeof(tBDAddr));
        sensorset.sensortype = SENSOR_TYPE_TEMPERATURE;
        AddSensorSetList(&sensorset, 0);
        
        sensorset.sensortype = SENSOR_TYPE_HUMIDITY;
        AddSensorSetList(&sensorset, 0);

        sensorset.sensortype = SENSOR_TYPE_ECO2;
        AddSensorSetList(&sensorset, 0);

        sensorset.sensortype = SENSOR_TYPE_TVOC;
        AddSensorSetList(&sensorset, 0);
      }
    }
    else // one device has only one sensor
    {
      set_index = GetSensorSetListIndex(sensor_data->macAddress, sensor_data->sensortype);
      if(set_index == (-1))
      {
        memcpy(sensorset.macAddress, sensor_data->macAddress, sizeof(tBDAddr));
        sensorset.sensortype = sensor_data->sensortype;
        AddSensorSetList(&sensorset, 0);
      }
    }
  }
  else
  {
    memcpy(&sensor_data_list[index], sensor_data, sizeof(SENSOR_Data_t));
  }

  if(sensor_data_list[index].sensortype == SENSOR_TYPE_PIR || sensor_data_list[index].sensortype == SENSOR_TYPE_HALL)
  {
    set_index = GetSensorSetListIndex(sensor_data->macAddress, sensor_data_list[index].sensortype);
    if(set_index != (-1))
    {
      if(sensor_data_list[index].bInterrupt == 1)
      {
        if(sensor_set_list[set_index].bInterruptAlarm == 1 && sensor_set_list[set_index].bInterruptAlarm_sended == 0)
        {
          sensor_set_list[set_index].bInterruptAlarm_sended = 1;
          SendSensorData(sensor_data_list[index].macAddress, sensor_data_list[index].sensortype, 1);
        }
      }
      else if(sensor_set_list[set_index].bInterruptAlarm_sended == 1)
      {
        sensor_set_list[set_index].bInterruptAlarm_sended = 0;
        SendSensorData(sensor_data_list[index].macAddress, sensor_data_list[index].sensortype, 1); // for test
      }
    }
  }
  
  // Check threshold (temperatur, humidty, eco2, tvoc)

  if(sensor_data_list[index].sensortype == (SENSOR_TYPE_TEMPERATURE | SENSOR_TYPE_HUMIDITY | SENSOR_TYPE_ECO2 | SENSOR_TYPE_TVOC))
  {
    CheckThreshold(index);
  }
}

void CheckThreshold(uint8_t data_index)
{
  int8_t index;
  uint16_t value;

  // data1 : temperature
  index = GetSensorSetListIndex(sensor_data_list[data_index].macAddress, SENSOR_TYPE_TEMPERATURE);

  if(index != (-1))
  {
    if(sensor_data_list[data_index].data1 <= sensor_set_list[index].upper_threshold && sensor_data_list[data_index].data1 >= sensor_set_list[index].lower_threshold) // normal state
    {
      sensor_set_list[index].bUpper_threshold_sended = 0;
      sensor_set_list[index].bLower_threshold_sended = 0;
    }
    else if(sensor_data_list[data_index].data1 > sensor_set_list[index].upper_threshold)
    {
      if(sensor_set_list[index].bUpper_threshold_sended == 0 && sensor_set_list[index].bInterruptAlarm == 1)
      {
        sensor_set_list[data_index].bUpper_threshold_sended = 1;
        sensor_set_list[data_index].bLower_threshold_sended = 0;
        SendSensorData(sensor_set_list[data_index].macAddress, sensor_set_list[data_index].sensortype, 1);
      }
    }
    else if(sensor_data_list[data_index].data1 < sensor_set_list[index].lower_threshold)
    {
      if(sensor_set_list[index].bLower_threshold_sended == 0 && sensor_set_list[index].bInterruptAlarm == 1)
      {
        sensor_set_list[data_index].bUpper_threshold_sended = 0;
        sensor_set_list[data_index].bLower_threshold_sended = 1;
        SendSensorData(sensor_set_list[data_index].macAddress, sensor_set_list[data_index].sensortype, 1);
      }
    }
  }

  // data2 : humidity

  index = GetSensorSetListIndex(sensor_data_list[data_index].macAddress, SENSOR_TYPE_HUMIDITY);

  if(index != (-1))
  {
    if(sensor_data_list[data_index].data2 <= sensor_set_list[index].upper_threshold && sensor_data_list[data_index].data2 >= sensor_set_list[index].lower_threshold) // normal state
    {
      sensor_set_list[index].bUpper_threshold_sended = 0;
      sensor_set_list[index].bLower_threshold_sended = 0;
    }
    else if(sensor_data_list[data_index].data2 > sensor_set_list[index].upper_threshold)
    {
      if(sensor_set_list[index].bUpper_threshold_sended == 0 && sensor_set_list[index].bInterruptAlarm == 1)
      {
        sensor_set_list[data_index].bUpper_threshold_sended = 1;
        sensor_set_list[data_index].bLower_threshold_sended = 0;
        SendSensorData(sensor_set_list[data_index].macAddress, sensor_set_list[data_index].sensortype, 1);
      }
    }
    else if(sensor_data_list[data_index].data2 < sensor_set_list[index].lower_threshold)
    {
      if(sensor_set_list[index].bLower_threshold_sended == 0 && sensor_set_list[index].bInterruptAlarm == 1)
      {
        sensor_set_list[data_index].bUpper_threshold_sended = 0;
        sensor_set_list[data_index].bLower_threshold_sended = 1;
        SendSensorData(sensor_set_list[data_index].macAddress, sensor_set_list[data_index].sensortype, 1);
      }
    }
  }

  // data3 : eco2
  index = GetSensorSetListIndex(sensor_data_list[data_index].macAddress, SENSOR_TYPE_ECO2);

  if(index != (-1))
  {
    if(sensor_data_list[data_index].data3 <= sensor_set_list[index].upper_threshold && sensor_data_list[data_index].data3 >= sensor_set_list[index].lower_threshold) // normal state
    {
      sensor_set_list[index].bUpper_threshold_sended = 0;
      sensor_set_list[index].bLower_threshold_sended = 0;
    }
    else if(sensor_data_list[data_index].data3 > sensor_set_list[index].upper_threshold)
    {
      if(sensor_set_list[index].bUpper_threshold_sended == 0 && sensor_set_list[index].bInterruptAlarm == 1)
      {
        sensor_set_list[data_index].bUpper_threshold_sended = 1;
        sensor_set_list[data_index].bLower_threshold_sended = 0;
        SendSensorData(sensor_set_list[data_index].macAddress, sensor_set_list[data_index].sensortype, 1);
      }
    }
    else if(sensor_data_list[data_index].data3 < sensor_set_list[index].lower_threshold)
    {
      if(sensor_set_list[index].bLower_threshold_sended == 0 && sensor_set_list[index].bInterruptAlarm == 1)
      {
        sensor_set_list[data_index].bUpper_threshold_sended = 0;
        sensor_set_list[data_index].bLower_threshold_sended = 1;
        SendSensorData(sensor_set_list[data_index].macAddress, sensor_set_list[data_index].sensortype, 1);
      }
    }
  }

  // data4 : tvoc
  index = GetSensorSetListIndex(sensor_data_list[data_index].macAddress, SENSOR_TYPE_TVOC);

  if(index != (-1))
  {
    if(sensor_data_list[data_index].data3 <= sensor_set_list[index].upper_threshold && sensor_data_list[data_index].data3 >= sensor_set_list[index].lower_threshold) // normal state
    {
      sensor_set_list[index].bUpper_threshold_sended = 0;
      sensor_set_list[index].bLower_threshold_sended = 0;
    }
    else if(sensor_data_list[data_index].data3 > sensor_set_list[index].upper_threshold)
    {
      if(sensor_set_list[index].bUpper_threshold_sended == 0 && sensor_set_list[index].bInterruptAlarm == 1)
      {
        sensor_set_list[data_index].bUpper_threshold_sended = 1;
        sensor_set_list[data_index].bLower_threshold_sended = 0;
        SendSensorData(sensor_set_list[data_index].macAddress, sensor_set_list[data_index].sensortype, 1);
      }
    }
    else if(sensor_data_list[data_index].data3 < sensor_set_list[index].lower_threshold)
    {
      if(sensor_set_list[index].bLower_threshold_sended == 0 && sensor_set_list[index].bInterruptAlarm == 1)
      {
        sensor_set_list[data_index].bUpper_threshold_sended = 0;
        sensor_set_list[data_index].bLower_threshold_sended = 1;
        SendSensorData(sensor_set_list[data_index].macAddress, sensor_set_list[data_index].sensortype, 1);
      }
    }
  }

}

void SendSensorData(tBDAddr macAddr, uint8_t sensortype, uint8_t bForce)
{
  int8_t data_index;
  ToHi toHi = TO_HI__INIT;
  ToHost tohost = TO_HOST__INIT;

  uint8_t mac_str[20] = {0};
  SensorData sensor_data = SENSOR_DATA__INIT;
  uint8_t bSend = 0;

  data_index = GetSensorDataListIndex(macAddr);

  if(data_index != (-1))
  {

    if(sensortype == SENSOR_TYPE_HALL)
    {
      if(sensor_data_list[data_index].bNewData1 == 1 || bForce == 1)
      {
        sensor_data.has_hall_state = 1;
        sensor_data.hall_state = sensor_data_list[data_index].data1;
        sensor_data_list[data_index].bNewData1 = 0;
        
        //if(sensor_data_list[data_index].bInterrupt == 1)
        {
          sensor_data.has_hall_interrupt = 1;
          sensor_data.hall_interrupt = sensor_data_list[data_index].bInterrupt;
          sensor_data_list[data_index].bInterrupt = 0;
        }
        bSend = 1;
      }
    }
    if(sensortype == SENSOR_TYPE_TEMPERATURE)
    {
      if(sensor_data_list[data_index].bNewData1 == 1 || bForce == 1)
      {
        sensor_data.has_sensor_temperature = 1;
        sensor_data.sensor_temperature = sensor_data_list[data_index].data1;
        sensor_data_list[data_index].bNewData1 = 0;
        bSend = 1;
      }
    }
    if(sensortype == SENSOR_TYPE_HUMIDITY)
    {
      if(sensor_data_list[data_index].bNewData2 == 1 || bForce == 1)
      {
        sensor_data.has_sensor_humidity = 1;
        sensor_data.sensor_humidity = sensor_data_list[data_index].data2;
        sensor_data_list[data_index].bNewData2 = 0;
        bSend = 1;
      }
    }
    if(sensortype == SENSOR_TYPE_ECO2)
    {
      if(sensor_data_list[data_index].bNewData3 == 1 || bForce == 1)
      {
        sensor_data.has_eco2_ppm = 1;
        sensor_data.eco2_ppm = sensor_data_list[data_index].data3;
        sensor_data_list[data_index].bNewData3 = 0;
        bSend = 1;
      }    
    }
    if(sensortype == SENSOR_TYPE_TVOC)
    {
      if(sensor_data_list[data_index].bNewData4 == 1 || bForce == 1)
      {
        sensor_data.has_tvoc_ppb = 1;
        sensor_data.tvoc_ppb = sensor_data_list[data_index].data4;
        sensor_data_list[data_index].bNewData4 = 0;
        bSend = 1;
      }
    }
    
    if(sensortype == SENSOR_TYPE_PIR)
    {
      if(bForce == 1)
      {
        //if(sensor_data_list[data_index].bInterrupt == 1)
        {
          sensor_data.has_pir_interrupt = 1;
          sensor_data.pir_interrupt = sensor_data_list[data_index].bInterrupt;
          sensor_data_list[data_index].bInterrupt = 0;
          bSend = 1;
        }
      }
    }

    if(bSend == 1)
    {
      sensor_data.has_sensor_voltage = 1;
      sensor_data.sensor_voltage = sensor_data_list[data_index].voltage;
      
      MacAddressString(mac_str, sensor_data_list[data_index].macAddress);
      
      tohost.sensor_data = &sensor_data;
      tohost.sensor_data->sensor_mac_address = (char *) mac_str;
      
      tohost.sensor_data->has_sensor_type = 1;
      tohost.sensor_data->sensor_type = sensortype;
      
      toHi.tohost_bypass = &tohost;
      
      pushToHi(&huart1, &toHi);
    }
  }
}

void CheckSensorDataSend(void)
{
  uint8_t set_index;

  for(set_index = 0; set_index < sensor_set_list_cnt; set_index++)
  {
    if(sensor_set_list[set_index].period_cnt > 0)
    {
      sensor_set_list[set_index].period_cnt--;
    }

    if(sensor_set_list[set_index].period_cnt == 0)
    {
      sensor_set_list[set_index].period_cnt = sensor_set_list[set_index].notify_period;

      SendSensorData(sensor_set_list[set_index].macAddress, sensor_set_list[set_index].sensortype, 0);
    }
  }  
}

// one device can have different type of sensors
int8_t GetSensorSetListIndex(tBDAddr mac, uint8_t sensortype)
{
  for(uint8_t index = 0; index < sensor_set_list_cnt; index++)
  {
    if(memcmp(sensor_set_list[index].macAddress, mac, sizeof(tBDAddr)) == 0)
    {
      if(sensortype == sensor_set_list[index].sensortype)
      {
        return index;
      }
    }
  }

  return -1;
}

void AddSensorSetList(SENSOR_Set_t *sensorset, uint8_t bUserSet)
{
  int8_t index;

  index = GetSensorSetListIndex(sensorset->macAddress, sensorset->sensortype);

  if(index == (-1))
  {
    index = sensor_set_list_cnt;
    memcpy(&sensor_set_list[sensor_set_list_cnt].macAddress, &sensorset->macAddress, sizeof(tBDAddr));
    sensor_set_list_cnt++;

    sensor_set_list[index].sensortype = sensorset->sensortype;
    sensor_set_list[index].bInterruptAlarm = 1;
    sensor_set_list[index].notify_period = 5;
    sensor_set_list[index].period_alarm = 1;
    sensor_set_list[index].upper_threshold = 0xFFFFFFFF;
    sensor_set_list[index].lower_threshold = 0;
    sensor_set_list[index].period_cnt = 5;
    SendSensorData(sensor_set_list[index].macAddress, sensor_set_list[index].sensortype, 1);
  }

  if(bUserSet == 1)
  {
    sensor_set_list[index].sensortype = sensorset->sensortype;
    
    sensor_set_list[index].bInterruptAlarm = sensorset->bInterruptAlarm;
    
    sensor_set_list[index].notify_period = sensorset->notify_period; //sec
    
    sensor_set_list[index].period_alarm = sensorset->period_alarm;
    
    sensor_set_list[index].upper_threshold = sensorset->upper_threshold;
    
    sensor_set_list[index].lower_threshold = sensorset->lower_threshold;

    sensor_set_list[index].period_cnt = sensorset->notify_period; //sec
  }
}

#define USER_DATA_ADDRESS                0x08004000
#define SENSOR_SAVE_ADDRESS                0x08004400

void SaveSensorSetList()
{
  uint8_t flashPos = 0;
  uint32_t set_size = sizeof(sensor_set_list);

  uint32_t SectorError;

  FLASH_EraseInitTypeDef pEraseInit;

  // read user data to save after erase flash
  uint64_t userdata[128] = {0}; // 1Kbytes : SENSOR_SAVE_ADDRESS - USER_DATA_ADDRESS = 0x400

  for(flashPos = 0; flashPos < 128; flashPos++)
  {
    userdata[flashPos] = Read_Flash(USER_DATA_ADDRESS + (flashPos * 8)); //read 8 bytes
  }

  /* Unlock the Flash to enable the flash control register access *************/ 
  FLASH_If_Init();

  pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  pEraseInit.Page = 4;
  pEraseInit.NbPages= 1;

  if (HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK)
  {
     /* Error occurred while page erase */
     //return (1);
     return;
  }
  
  //while((set_size % 8)) // size of struct is always 8's mulitple
  //{
    //set_size++;
  //}

  FLASH_If_Write(USER_DATA_ADDRESS, userdata, sizeof(userdata)/sizeof(uint64_t));

  FLASH_If_Write(SENSOR_SAVE_ADDRESS, (uint64_t *) sensor_set_list, set_size/sizeof(uint64_t));

}

void ReadSensorSetList()
{
  uint8_t flashPos = 0;
  uint32_t set_size = sizeof(sensor_set_list);
  uint8_t * setListAddr = (uint8_t *) &sensor_set_list;
  uint64_t readData;
    
  for(flashPos = 0; flashPos < 128; flashPos++)
  {
    readData = Read_Flash(USER_DATA_ADDRESS + (flashPos * 8)); //read 8 bytes
    memcpy(setListAddr, &readData, sizeof(readData));
    setListAddr += sizeof(readData);
  }
}
