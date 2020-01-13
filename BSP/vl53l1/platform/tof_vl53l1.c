#include "stm32wbxx_hal.h"
#include "main.h"
#include "tof_vl53l1.h"
#include "uart_trace.h"

#include "VL53L1X_API.h"

#include "simple.pb-c.h"
extern ToHi tohi;

VL53L1_Dev_t vl53l1_dev;
extern I2C_HandleTypeDef hi2c3;

int VL53L1A1_ResetId(int DevNo, int state) {
    int status;
    
    switch( DevNo ){
    case XNUCLEO53L1A1_DEV_CENTER :
    case 'c' :
    	FRONT_SHUT_TOF_l1_OFF;
        if( state )FRONT_SHUT_TOF_l1_ON;
        break;
    case XNUCLEO53L1A1_DEV_LEFT :
    case 'l' :
    	FRONTL_SHUT_TOF_l1_OFF;
        if( state )FRONTL_SHUT_TOF_l1_ON;
        break;
    case 'r' :
    case XNUCLEO53L1A1_DEV_RIGHT :
    	FRONTR_SHUT_TOF_l1_OFF;
        if( state )FRONTR_SHUT_TOF_l1_ON;
        break;
    default:
        status = -1;
        break;
    }
    return status;
}

int TOF_53l1_DetectSensors(uint16_t Id) {
	int status = VL53L1_ERROR_NONE;
	int FinalAddress;
	uint8_t byteData, sensorState=0;
	uint16_t wordData;

	/* Reset all */
	status = VL53L1A1_ResetId(Id, 0);
	vl53l1_dev.I2cHandle = &hi2c3;
	vl53l1_dev.I2cDevAddr = 0x52;
	HAL_Delay(1);
	FinalAddress=0x52;
	status = VL53L1A1_ResetId(Id, 1);
	HAL_Delay(1);
	status = VL53L1_RdByte(&vl53l1_dev, 0x010F, &byteData);
//	uart_printf("VL53L1X Model_ID: %X\r\n", byteData);
	status = VL53L1_RdByte(&vl53l1_dev, 0x0110, &byteData);
//	uart_printf("VL53L1X Module_Type: %X\r\n", byteData);
	status = VL53L1_RdWord(&vl53l1_dev, 0x010F, &wordData);
//	uart_printf("VL53L1X: %X\r\n", wordData);


    return status;
}

int8_t TOF_53l1_ResetAndDetectSensor(uint16_t Id){
    int nSensor;
    nSensor = TOF_53l1_DetectSensors(Id);
	if(nSensor!=VL53L1_ERROR_NONE)
	{
		return nSensor;
	}
	/*VL53L1X_SensorInit
	nSensor = TOF_53l1_SetupSingleShot(Id,HIGH_SPEED);
	if(nSensor!=VL53L0X_ERROR_NONE)
	{
		return nSensor;
	}*/
//	TOF_SetCalibration();
//	TOF_Setoffset();
//	TOF_Calibration();
//	TOF_Calibration();
//	while(1);

	return VL53L1_ERROR_NONE;
}

int8_t TOF_53l1_MeasureStep1(void) // Set StartMeasurement
{
	VL53L1X_ERROR status;
	uint8_t byteData, sensorState=0;
	status = VL53L1X_BootState(vl53l1_dev, &sensorState);

  return status;
}



uint8_t TOF_53l1_MeasureRange(uint16_t Id)
{
	static  uint32_t LoopNb;
	uint8_t returnval=0;
	static uint8_t measureStep = VL53l1_DETECT;
	VL53L1X_ERROR ret;
	uint8_t dataReady;
	uint16_t Distance;
	uint16_t SignalRate;
	uint16_t AmbientRate;
	uint8_t RangeStatus;
	if(hi2c3.Instance == NULL)
	{
		MX_I2C3_Init();
	}

	switch(measureStep)
	{
		case VL53l1_DETECT:
			ret =  TOF_53l1_ResetAndDetectSensor(Id);
			if(ret == VL53L1_ERROR_NONE)
			{
				measureStep++;
			}
		break;
		case VL53l1_BOOT:
			ret = TOF_53l1_MeasureStep1();/* Set start register */
			ret = VL53L1X_SensorInit(vl53l1_dev);
			VL53L1X_SetDistanceMode(vl53l1_dev,2);
			if(ret == VL53L1_ERROR_NONE)
			{
			  measureStep++;
			  LoopNb=0;
			}
			else
			{
				LoopNb++;
				if (LoopNb >= 100) {
					measureStep = VL53l1_DETECT;
				}
			}
		break;
		case VL53l1_INITSTART:
			ret = VL53L1X_StartRanging(vl53l1_dev);   /* This function has to be called to enable the ranging */
			if(ret == VL53L1_ERROR_NONE)
			{
			  measureStep++;
			  LoopNb=0;
			}
			else 
			{
			//uart_printf(",");
			   LoopNb++;
				if (LoopNb >= 100) {
					measureStep = VL53l1_BOOT;
				}
			}
		break;
		case VL53l1_WAIT_MEASURE:
			returnval=VL53l1_WAIT_MEASURE;
		    ret = VL53L1X_CheckForDataReady(vl53l1_dev, &dataReady);
			if(ret == VL53L1_ERROR_NONE)  //0? ?? ??? ???? ??. ?? ???..
			{

			  if(dataReady!=0)
			  {
  			    LoopNb=0;
			  	measureStep++;
			  }
			  else
			  {
				 	LoopNb++;
					if (LoopNb >= 100) {
						measureStep = VL53l1_BOOT;
					}
			  }
			}
			else
			{	
				  measureStep = VL53l1_DETECT;
			}
			break;	//?? ???....
		case VL53l1_GET_MEASURE:
			ret = VL53L1X_GetRangeStatus(vl53l1_dev, &RangeStatus);
			ret |= VL53L1X_GetDistance(vl53l1_dev, &Distance);
			ret |= VL53L1X_GetSignalRate(vl53l1_dev, &SignalRate);
			ret |= VL53L1X_GetAmbientRate(vl53l1_dev, &AmbientRate);
			ret |= VL53L1X_ClearInterrupt(vl53l1_dev); /* clear interrupt has to be called to enable next interrupt*/
			//uart_printf("%u, %05u, %05u, %d\r\n", RangeStatus, Distance, SignalRate, ret);
			// int16_t rang=0;
			tohi.tof=Distance;
			measureStep = VL53l1_DETECT;
			if(ret == VL53L1_ERROR_NONE)
			{
				
				returnval=VL53l1_GET_MEASURE;
				measureStep = VL53l1_WAIT_MEASURE;
			}
		break;

	}

 // MX_FMPI2C1_DeInit();
	return returnval;
}

