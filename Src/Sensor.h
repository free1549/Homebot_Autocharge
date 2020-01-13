// Define to prevent recursive inclusion -------------------------------------
#ifndef __SENSOR_H
#define __SENSOR_H

#include "simple.pb-c.h"
#include "ble.h"

typedef enum
{
  SENSOR_TYPE_HALL = 0, // only main bd
  SENSOR_TYPE_TEMPERATURE = 0x01, // main bd + env bd
  SENSOR_TYPE_HUMIDITY = 0x02, // main bd + env bd
  SENSOR_TYPE_PIR = 0x04, // main bd + pir bdb
  SENSOR_TYPE_ECO2 = 0x08, // main bd + sgp30 bdb
  SENSOR_TYPE_TVOC = 0x10, // main bd + sgp30 bdb
} SENSOR_TYPE_t;

typedef struct
{
  tBDAddr macAddress;
  uint8_t bInterrupt;
  uint8_t sensortype;
  uint16_t data1;
  uint16_t data2;
  uint16_t data3;
  uint16_t data4;
  uint8_t voltage;
  uint8_t bNewData1;
  uint8_t bNewData2;
  uint8_t bNewData3;
  uint8_t bNewData4;
} SENSOR_Data_t;

typedef struct
{
  tBDAddr macAddress;
  uint8_t sensortype;
  uint8_t bInterruptAlarm;
  uint8_t bInterruptAlarm_sended;
  uint32_t notify_period;
  uint8_t period_alarm;
  uint32_t upper_threshold;
  uint32_t lower_threshold;
  uint32_t period_cnt;
  uint8_t bUpper_threshold_sended;
  uint8_t bLower_threshold_sended;
} SENSOR_Set_t;

void Sensor_Init();
void SendSensorData(tBDAddr macAddr, uint8_t sensortype, uint8_t bForce);
void AddSensorDataList(SENSOR_Data_t *sensor_data);
void AddSensorSetList(SENSOR_Set_t *sensorset, uint8_t bUserSet);
int8_t GetSensorSetListIndex(tBDAddr mac, uint8_t sensortype);
void CheckSensorDataSend(void);

#endif // __SENSOR_H
