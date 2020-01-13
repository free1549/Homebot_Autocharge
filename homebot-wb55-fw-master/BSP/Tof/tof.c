#include "stm32wbxx_hal.h"
#include "main.h"
#include "tof.h"
#include "uart_trace.h"
//nclude "Motor.h"

/*
VL53L0X_DeviceParameters_t CurrentParameters;
int UseSensorsMask = 1<<XNUCLEO53L0A1_DEV_CENTER;//|1<<XNUCLEO53L0A1_DEV_LEFT;
RangingConfig_e RangingConfig = HIGH_ACCURACY;
ResetAndDetectSensor(0);
*/
extern	SYSTEM_key	sys;
//extern	Sensor_Data_TypeDef  Sensor;

VL53L0X_RangingMeasurementData_t RangingMeasurementData;

VL53L0X_Dev_t VL53L0XDevs[]=
{
	{.Id=XNUCLEO53L0A1_DEV_CENTER, .DevLetter='c', .I2cHandle=&hi2c2, .I2cDevAddr=0x52},
	{.Id=XNUCLEO53L0A1_DEV_DLEFT,  .DevLetter='r'+1, .I2cHandle=&hi2c2, .I2cDevAddr=0x52},
	{.Id=XNUCLEO53L0A1_DEV_DRIGHT, .DevLetter='l'+1, .I2cHandle=&hi2c2, .I2cDevAddr=0x52},
	{.Id=XNUCLEO53L0A1_DEV_LEFT,   .DevLetter='l', .I2cHandle=&hi2c2, .I2cDevAddr=0x52},
	{.Id=XNUCLEO53L0A1_DEV_RIGHT,  .DevLetter='r', .I2cHandle=&hi2c2, .I2cDevAddr=0x52}
};


extern void MX_I2C2_Init(void);

/*Calibration ??.*/
void TOF_Calibration(uint16_t Id)
{
	uint32_t Xtalk=0;
	VL53L0X_perform_xtalk_calibration(&VL53L0XDevs[Id], 450*65535, &Xtalk);
//	uart_printf("%d ---\r\n",Xtalk);

	/********************/
	// EEPROM? ?? ??? ?? ??...
}

/*Calibration ? ??..*/
void TOF_SetCalibration(uint16_t Id)
{
	//EEPROM?? ??? ???? ??.
	uint32_t Xtalk=(int)(0.0004*(1<<16));
	
	VL53L0X_SetXTalkCompensationEnable(&VL53L0XDevs[Id], 1);
	VL53L0X_SetXTalkCompensationRateMegaCps(&VL53L0XDevs[Id],Xtalk);
}

void TOF_offset_calibration(uint16_t Id)
{
	int32_t Offset;
	VL53L0X_perform_offset_calibration(&VL53L0XDevs[Id], 100*65535, &Offset);
}

void TOF_Setoffset(uint16_t Id)
{
	VL53L0X_SetOffsetCalibrationDataMicroMeter(&VL53L0XDevs[Id], -6000);
//	VL53L0X_SetOffsetCalibrationDataMicroMeter(&VL53L0XDevs, 300000);
}

/**
 *  Setup all detected sensors for single shot mode and setup ranging configuration
 */
VL53L0X_Error TOF_SetupSingleShot(uint16_t Id,RangingConfig_e rangingConfig)
{
  int status;
  uint8_t VhvSettings;
  uint8_t PhaseCal;
  uint32_t refSpadCount;
    uint8_t isApertureSpads;
    FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
    FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
    uint32_t timingBudget = 33000;//Timing Budget Set
    uint8_t preRangeVcselPeriod = 14;//PreVcsel
    uint8_t finalRangeVcselPeriod = 10;//FinalVcsel
    FixPoint1616_t range_ignore_threshold = (FixPoint1616_t)(0.4*65536);

  if(VL53L0XDevs[Id].Present)
  {
    status=VL53L0X_StaticInit(&VL53L0XDevs[Id]);
    if( status )
    {
      return -100; //연결 DISCONNECT
    }

    status = VL53L0X_PerformRefCalibration(&VL53L0XDevs[Id], &VhvSettings, &PhaseCal);

    status = VL53L0X_PerformRefSpadManagement(&VL53L0XDevs[Id], &refSpadCount, &isApertureSpads);

    status = VL53L0X_SetDeviceMode(&VL53L0XDevs[Id], VL53L0X_DEVICEMODE_GPIO_DRIVE); // Setup in single ranging mode

    status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs[Id], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit

    status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs[Id], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC, VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE, VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS

    status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs[Id], VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1); // Enable RANGE_IGNORE

    

    /* Ranging configuration */
    switch(rangingConfig)
    {
    case LONG_RANGE:
        signalLimit = (FixPoint1616_t)(0.1*65536);
        sigmaLimit = (FixPoint1616_t)(60*65536);
        timingBudget = 33000;
        preRangeVcselPeriod = 18;
        finalRangeVcselPeriod = 14;
        break;
    case HIGH_ACCURACY:
        signalLimit = (FixPoint1616_t)(0.25*65536);
        sigmaLimit = (FixPoint1616_t)(18*65536);
        timingBudget = 200000;
        preRangeVcselPeriod = 14;
        finalRangeVcselPeriod = 10;
        break;
    case HIGH_SPEED:
        signalLimit = (FixPoint1616_t)(0.25*65536);
        sigmaLimit = (FixPoint1616_t)(32*65536);
        timingBudget = 20000;
        preRangeVcselPeriod = 14;
        finalRangeVcselPeriod = 10;
        break;
    default:
      break;
    }

    
    //signalLimit=40000;
    //sigmaLimit = (FixPoint1616_t)(33*65536);
    range_ignore_threshold =(FixPoint1616_t)(0.006*65536);
    VL53L0X_SetLimitCheckValue(&VL53L0XDevs[Id],  VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, range_ignore_threshold);
    



    
    status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs[Id],  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);

    status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs[Id],  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);

    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&VL53L0XDevs[Id],  timingBudget);

    status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs[Id],  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);

    status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs[Id],  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);

    status = VL53L0X_PerformRefCalibration(&VL53L0XDevs[Id], &VhvSettings, &PhaseCal);
    VL53L0XDevs[Id].LeakyFirst=1;

  }

  return VL53L0X_ERROR_NONE;
}

int TOF_DetectSensors(uint16_t Id) {
    int status = VL53L0X_ERROR_NONE;
    int FinalAddress;

    /* Reset all */
    status = XNUCLEO53L0A1_ResetId(Id, 0);
    VL53L0X_Dev_t *pDev;
    pDev = &VL53L0XDevs[Id];
    pDev->I2cDevAddr = 0x52;
    pDev->Present = 0;
    pDev->Id = Id;
    HAL_Delay(1);
    FinalAddress=0x52;
	status = XNUCLEO53L0A1_ResetId(Id, 1);
    do {
    	/* Set I2C standard mode (400 KHz) before doing the first register access */
    	if (status == VL53L0X_ERROR_NONE)
    		status = VL53L0X_WrByte(pDev, 0x88, 0x00); //I2C ??? standard mode? ??

    	/* Try to read one register using default 0x52 address */
        status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id); //DEV ID? ?? I2C ?? ?? 
        if (status) {
         //   uart_printf("#%d Read id fail\r\n", i);
            break;
        }
        if (Id == 0xEEAA) {
			/* Sensor is found => Change its I2C address to final one */
            status = VL53L0X_SetDeviceAddress(pDev,FinalAddress); //??? ??? ??
            if (status != 0) {
                uart_printf("VL53L0X_SetDeviceAddress fail\r\n");
                break;
            }
            pDev->I2cDevAddr = FinalAddress;
            /* Check all is OK with the new I2C address and initialize the sensor */
            status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);//?? ??? ??? ???? ?? ??? 
            if (status != 0) {
				uart_printf("VL53L0X_RdWord fail\r\n");
				break;
			}

            status = VL53L0X_DataInit(pDev); //?? ?? ??? 
            if( status == 0 ){
                pDev->Present = 1;
            }
            else{
                uart_printf("VL53L0X_DataInit fail\r\n");
                break;
            }
        //    uart_printf("VL53L0X %d Present and initiated to final 0x%x\r\n", pDev->Id, pDev->I2cDevAddr);
            pDev->Present = 1;
        }
        else {
        	
               	 status = 1;
        }
    } while (0);

    return status;
}

int8_t TOF_ResetAndDetectSensor(uint16_t Id){
    int nSensor;
    nSensor = TOF_DetectSensors(Id);
	if(nSensor!=VL53L0X_ERROR_NONE)
	{
		return nSensor;
	}
	nSensor = TOF_SetupSingleShot(Id,HIGH_SPEED);
	if(nSensor!=VL53L0X_ERROR_NONE)
	{
		return nSensor;
	}
//	TOF_SetCalibration();
//	TOF_Setoffset();
//	TOF_Calibration();
//	TOF_Calibration();
//	while(1);

	return VL53L0X_ERROR_NONE;
}


int8_t TOF_MeasureStep1(uint16_t Id) // Set StartMeasurement
{
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;

  //Status = VL53L0X_StartMeasurement(&VL53L0XDevs);
	Status = VL53L0X_WrByte(&VL53L0XDevs[Id], 0x80, 0x01);
  if(Status != VL53L0X_ERROR_NONE)
  {
    return Status;
  }

	Status = VL53L0X_WrByte(&VL53L0XDevs[Id], 0xFF, 0x01);
  if(Status != VL53L0X_ERROR_NONE)
  {
    return Status;
  }

	Status = VL53L0X_WrByte(&VL53L0XDevs[Id], 0x00, 0x00);
  if(Status != VL53L0X_ERROR_NONE)
  {
    return Status;
  }

	Status = VL53L0X_WrByte(&VL53L0XDevs[Id], 0x91, VL53L0XDevs[Id].Data.StopVariable);
  if(Status != VL53L0X_ERROR_NONE)
  {
    return Status;
  }

	Status = VL53L0X_WrByte(&VL53L0XDevs[Id], 0x00, 0x01);
  if(Status != VL53L0X_ERROR_NONE)
  {
    return Status;
  }

	Status = VL53L0X_WrByte(&VL53L0XDevs[Id], 0xFF, 0x00);
  if(Status != VL53L0X_ERROR_NONE)
  {
    return Status;
  }

	Status = VL53L0X_WrByte(&VL53L0XDevs[Id], 0x80, 0x00);
  if(Status != VL53L0X_ERROR_NONE)
  {
    return Status;
  }

	Status = VL53L0X_WrByte(&VL53L0XDevs[Id], VL53L0X_REG_SYSRANGE_START, 0x01);
  if(Status != VL53L0X_ERROR_NONE)
  {
    return Status;
  }

  return VL53L0X_ERROR_NONE;
}

int8_t TOF_MeasureStep2(uint16_t Id) 			/* Wait until start bit has been cleared */
{
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t StartStopByte = VL53L0X_REG_SYSRANGE_MODE_START_STOP;
	uint8_t Byte;

	Status = VL53L0X_RdByte(&VL53L0XDevs[Id], VL53L0X_REG_SYSRANGE_START, &Byte);
  if(Status != VL53L0X_ERROR_NONE)
  {
    return Status;
  }

  if(((Byte & StartStopByte) != StartStopByte))
  {
    return VL53L0X_ERROR_NONE;
  }

  // Data is not ready
  return 1;
}

int8_t TOF_MeasureStep3(uint16_t Id) 			/* Wait measurement completion */
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t NewDataReady = 0;

  Status = VL53L0X_GetMeasurementDataReady(&VL53L0XDevs[Id], &NewDataReady);
  if(Status != VL53L0X_ERROR_NONE)
  {
    return Status;/* the error is set */
  }

  if (NewDataReady == 1)
  {
    return VL53L0X_ERROR_NONE; /* done note that status == 0 */
}
  return 1;
}

int8_t TOF_MeasureStep4(uint16_t Id) 			/* Get Data*/
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

  Status = VL53L0X_GetRangingMeasurementData(&VL53L0XDevs[Id],
    &RangingMeasurementData);
  if(Status != VL53L0X_ERROR_NONE)
  {
    return Status;
  }

  return VL53L0X_ERROR_NONE;
}


//int LeakyFactorFix8 = (int)( 0.7 *256);
void TOF_Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange){
    int LeakyFactorFix8 = (int)(0.7*256);

	if( pRange->RangeStatus == 0 ){
		if( pDev->LeakyFirst ){
			pDev->LeakyFirst = 0;
			pDev->LeakyRange = pRange->RangeMilliMeter;
		}
		else{
			pDev->LeakyRange = (pDev->LeakyRange*LeakyFactorFix8 + (256-LeakyFactorFix8)*pRange->RangeMilliMeter)>>8;
		}
	}
	else{
		pDev->LeakyFirst = 1;
	}
	
}
uint8_t TOF_MeasureRange(uint16_t Id)
{
	static  uint32_t LoopNb;
	uint8_t returnval=0;
	static uint8_t measureStep = TOF_START;
	int8_t ret;
	if(hi2c2.Instance == NULL)
	{
		MX_I2C2_Init();
	}

  switch(measureStep)
  {
  /*case TOF_RESET:
  //  TOF_XSHUT_ON;
  //  measureStep++;
  //  break;*/
  case TOF_DETECT:
  
   	ret =  TOF_ResetAndDetectSensor(Id);
    if(ret == VL53L0X_ERROR_NONE)
    {
		measureStep = TOF_START;
    }
    break;
  case TOF_START:
    ret = TOF_MeasureStep1(Id);/* Set start register */
    if(ret == VL53L0X_ERROR_NONE)
    {
      measureStep++;
	  LoopNb=0;
    }
    else
    {
    //  TOF_XSHUT_OFF;
    	//?? ?? ???. ??? ?? 
	  if(measureStep==TOF_START)measureStep=TOF_DETECT;
	  else    measureStep = TOF_START;
    }
    break;
  case TOF_WAIT_START:
    ret = TOF_MeasureStep2(Id);
    if(ret == VL53L0X_ERROR_NONE)
    {
      measureStep++;
	  LoopNb=0;
    }
    else if(ret != 1) /* Wait until start bit has been cleared */
    {
	//uart_printf(",");
      LoopNb++;
		if (LoopNb >= 100) {
			measureStep = TOF_START;
		}
    }
    break;
  case TOF_WAIT_MEASURE:
    ret = TOF_MeasureStep3(Id);/* Wait measurement completion */
    if(ret == VL53L0X_ERROR_NONE)  //0? ?? ??? ???? ??. ?? ???..
    {
      measureStep++;  //?????..?? ?? ????...
    }
	else
	{	
		//uart_printf(".");
		LoopNb++;
		if (LoopNb >= 100) {
			measureStep = TOF_START;
		}
		break;  //?? ???....
	}
  case TOF_GET_MEASURE:
    ret = TOF_MeasureStep4(Id);
   // int16_t rang=0;
    if(ret == VL53L0X_ERROR_NONE)
    {
		returnval=TOF_GET_MEASURE;
		//TOF_Sensor_SetNewRange(&VL53L0XDevs[Id],&RangingMeasurementData);
		/*rang=(int)RangingMeasurementData.RangeMilliMeter;//-400;
		if(rang<0)rang=0;
		RangingMeasurementData.RangeMilliMeter=rang;*/
    	VL53L0X_ClearInterruptMask(&VL53L0XDevs[Id], 0);
    	if(RangingMeasurementData.RangeStatus==0)//||RangingMeasurementData.RangeStatus==2)
    	{	
    	
			VL53L0XDevs[Id].LeakyRange=RangingMeasurementData.RangeMilliMeter/10;
			
    	/*	sys.tof_org_rang=RangingMeasurementData.RangeMilliMeter/10;
    		sys.tof_rang = VL53L0XDevs[Id].LeakyRange/10;//RangingMeasurementData.RangeMilliMeter;
    		
			if(RangingMeasurementData.RangeMilliMeter<60)
			{
				if(++sys.tofRawErrorCount>38) //2초 이상. 
				{
					sys.tofRawErrorCount=100;
					sys.tofrawerror=1;
				}
			}
			else 
			{
				sys.tofRawErrorCount=0;
				sys.tofrawerror=0;
			}*/
    	}
    	else 
    	{	
    		VL53L0XDevs[Id].LeakyRange=0xff;
    		/*sys.tof_org_rang=0xff;
    		VL53L0XDevs[Id].LeakyRange=100;
    		sys.tof_rang = 0xff;
			sys.tofRawErrorCount=0;
			sys.tofrawerror=0;*/
		//	Sensor.data.TOF=0xff;
    	}
    	 
		uart_printf("[%02d] %04d %04d  %04d  %04d==========\r\n",RangingMeasurementData.RangeStatus,RangingMeasurementData.RangeMilliMeter,VL53L0XDevs[Id].LeakyRange,sys.tof_rang,RangingMeasurementData.SignalRateRtnMegaCps);
       measureStep = TOF_START;
    }
    else
    {
    //  TOF_XSHUT_OFF;
      measureStep = TOF_START;
    }
    break;

  }

 // MX_FMPI2C1_DeInit();
	return returnval;
}

