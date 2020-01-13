/* This sample code teaches developer how to operate the MEMSIC alogrithm library
 * to get Yaw, Pitch and Roll respectively. if developer want to use it as a part
 * of his codes, he need to declare and define 3 functions by himself:
 * ReadPara;
 * Savepara;
 * please NOTE it is ONLY a sample code and can be changed freely by developers.
 */
//#include "stm32f4xx_hal_def.h"

//#include <math.h>


#include "MMC3630KJ.h"  
#include "MemsicNineAxisFusion.h"
#include "MemsicCustomSpec.h"
#include "main.h"
#include "uart_trace.h"
#include "Customer.h"
//#include <arm_math.h>
#include "filter.h"


#include "LSM6DS3_ACC_GYRO_driver_HL.h"
#include "x_nucleo_iks01a1.h"
#include "x_nucleo_iks01a1_accelero.h"
#include "x_nucleo_iks01a1_gyro.h"
#include "simple.pb-c.h"

//extern struct TIMER_BIT	 TIMER_SYSTEM[TIMER_SYSTEM_QTY];
extern UART_HandleTypeDef huart1;
//extern motor_TypeDef DCMOTOR;
extern RTC_HandleTypeDef hrtc;
//extern struct TIMER_BIT	 TIMER_SYSTEM[TIMER_SYSTEM_QTY];
//extern Sensor_Data_TypeDef  Sensor;
extern SYSTEM_key	sys;
extern ToHi tohi;

#define M_LAYOUT	8
#define A_LAYOUT	8
#define G_LAYOUT	8

/* These variables are used to save the calibrated magnetic sensor output. */	
float magX, magY, magZ;		

/* These variables are used to save the calibrated orientation output. */	
float fAzimuth, fPitch, fRoll;
float des_fAzimuth;

/* This variable is used to save the calibration accuracy. */	
int iAccuracy;

/* if first start ot restart the algo, iRestart should be 1. */
int iRestart = 1; 

int iAlgoCal = 1; //if enable the algorithm calibrate real-time, iAlgoCal should be 1.

/* sensor sampling rate(Hz). */
int iSamplingRate = 50;

/* algo state, open or close. */
int iAlgoState = 0;

/*  
 * return the sampling rate; 
 */
int GetSamplingRate(void);

/*  
 * set the sampling rate; 
 */	
void SetSamplingRate(int t);

/* return the sensor state
 */
int GetAlgoState(void);

/* set the sensor state, open or close 
 */
void SetAlgoState(int state);

/**
 * @brief Convert the sensor coordinate to right-front-up coordinate system;
 */
void acc_coordinate_raw_to_real(int layout, float *in, float *out);

/**
 * @brief Convert the sensor coordinate to right-front-up coordinate system;
 */
void mag_coordinate_raw_to_real(int layout, float *in, float *out);

/**
 * @brief Convert the sensor coordinate to right-front-up coordinate system;
 */
void gyr_coordinate_raw_to_real(int layout, float *in, float *out);


/******************************************************************************************************/
static void *LSM6DS3_X_0_handle   = ( void * )0;//NULL
static void *LSM6DS3_G_0_handle   = ( void * )0;//NULL
static volatile uint8_t X_LSM6DS3_DIL24   = 0;
static volatile uint8_t G_LSM6DS3_DIL24   = 0;
/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
void initializeAllSensors(void)
{
//	BSP_ACCELERO_DeInit(&LSM6DS3_X_0_handle);
  if (BSP_ACCELERO_Init(LSM6DS3_X_0, &LSM6DS3_X_0_handle) == COMPONENT_OK)
  {
    X_LSM6DS3_DIL24 = 1;
  }
  else
  {
    sys.imuiniterror = 1;
  }

  if (BSP_GYRO_Init(LSM6DS3_G_0, &LSM6DS3_G_0_handle) == COMPONENT_OK)
  {
    G_LSM6DS3_DIL24 = 1;
  }  else 
  {
 	 sys.imuiniterror=1;
  }
 uart_printf("%d %d %d\r\n",X_LSM6DS3_DIL24,G_LSM6DS3_DIL24,HAL_RCC_GetHCLKFreq());
}
/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
void ACCAnalogFilterBandWifth(uint16_t cutoff)
{
	//float odr;
	uint8_t data,BW_XL;
	switch(cutoff)
	{
		case 50:
			BW_XL=0x03;
		break;
		case 100:
			BW_XL=0x02;
		break;
		case 200:
			BW_XL=0x01;
		break;
		case 400:
			BW_XL=0x00;
		break;
	}
  if (X_LSM6DS3_DIL24)
  {
    BSP_ACCELERO_Read_Reg(LSM6DS3_X_0_handle,LSM6DS3_ACC_GYRO_CTRL1_XL,&data);
//	uart_printf("LSM6DS3_ACC_GYRO_CTRL1_XL =%x \r\n",data);
	data|=BW_XL;
    BSP_ACCELERO_Write_Reg(LSM6DS3_X_0_handle,LSM6DS3_ACC_GYRO_CTRL1_XL,data); //50hz bandwidth
    BSP_ACCELERO_Read_Reg(LSM6DS3_X_0_handle,LSM6DS3_ACC_GYRO_CTRL1_XL,&data);
//	uart_printf("LSM6DS3_ACC_GYRO_CTRL1_XL =%x \r\n",data);
    BSP_ACCELERO_Write_Reg(LSM6DS3_X_0_handle,LSM6DS3_ACC_GYRO_CTRL4_C,0x80);
    BSP_ACCELERO_Read_Reg(LSM6DS3_X_0_handle,LSM6DS3_ACC_GYRO_CTRL4_C,&data);
//	uart_printf("LSM6DS3_ACC_GYRO_CTRL4_C =%x \r\n",data);
  }
  
//  BSP_ACCELERO_Read_Reg(LSM6DS3_X_0_handle,LSM6DS3_ACC_GYRO_CTRL8_XL,&data);
//  uart_printf("LSM6DS3_ACC_GYRO_CTRL8_XL =%x \r\n",data);
//  BSP_ACCELERO_Write_Reg(LSM6DS3_X_0_handle,LSM6DS3_ACC_GYRO_CTRL1_XL,0x84); //50hz bandwidth
// while(1);
}
/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
void GYROAnalogFilterBandWifth(uint16_t cutoff)
{
	//float odr;
	uint8_t data,BW_XL;
	switch(cutoff)
	{
		case 50:
			BW_XL=0x03;
		break;
		case 100:
			BW_XL=0x02;
		break;
		case 200:
			BW_XL=0x01;
		break;
		case 400:
			BW_XL=0x00;
		break;
	}
  if (X_LSM6DS3_DIL24)
  {
    BSP_ACCELERO_Read_Reg(LSM6DS3_X_0_handle,LSM6DS3_ACC_GYRO_CTRL1_XL,&data);
//	uart_printf("LSM6DS3_ACC_GYRO_CTRL1_XL =%x \r\n",data);
	data|=BW_XL;
    BSP_ACCELERO_Write_Reg(LSM6DS3_X_0_handle,LSM6DS3_ACC_GYRO_CTRL1_XL,data); //50hz bandwidth
    BSP_ACCELERO_Read_Reg(LSM6DS3_X_0_handle,LSM6DS3_ACC_GYRO_CTRL1_XL,&data);
//	uart_printf("LSM6DS3_ACC_GYRO_CTRL1_XL =%x \r\n",data);
    BSP_ACCELERO_Write_Reg(LSM6DS3_X_0_handle,LSM6DS3_ACC_GYRO_CTRL4_C,0x80);
    BSP_ACCELERO_Read_Reg(LSM6DS3_X_0_handle,LSM6DS3_ACC_GYRO_CTRL4_C,&data);
//	uart_printf("LSM6DS3_ACC_GYRO_CTRL4_C =%x \r\n",data);
  }
  
//  BSP_ACCELERO_Read_Reg(LSM6DS3_X_0_handle,LSM6DS3_ACC_GYRO_CTRL8_XL,&data);
//  uart_printf("LSM6DS3_ACC_GYRO_CTRL8_XL =%x \r\n",data);
//  BSP_ACCELERO_Write_Reg(LSM6DS3_X_0_handle,LSM6DS3_ACC_GYRO_CTRL1_XL,0x84); //50hz bandwidth
// while(1);
}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
void enableAllSensors(void)
{
	float odr;
	//uint8_t data;
	
	float fullscale=0;
	//LSM6DS3_X_Get_Sensitivity( handle, &fullscale );
  if (X_LSM6DS3_DIL24)
  {
	  //416hz 8g setting
    BSP_ACCELERO_Sensor_Enable(LSM6DS3_X_0_handle);
	ACCAnalogFilterBandWifth(50);
 //   BSP_ACCELERO_Enable_Free_Fall_Detection_Ext(LSM6DS3_X_0_handle,INT2_PIN);
  //  BSP_ACCELERO_Set_Free_Fall_Threshold_Ext(LSM6DS3_X_0_handle,LSM6DS3_ACC_GYRO_FF_THS_13);
  //  BSP_ACCELERO_Set_Tap_Threshold_Ext(LSM6DS3_X_0_handle,);
  //	LSM6DS3_X_Enable_Tilt_Detection
  //	BSP_ACCELERO_Enable_Tilt_Detection_Ext(LSM6DS3_X_0_handle,INT2_PIN);
    BSP_ACCELERO_Enable_Single_Tap_Detection_Ext(LSM6DS3_X_0_handle,INT1_PIN);
	BSP_ACCELERO_Get_ODR(LSM6DS3_X_0_handle,&odr);
//	uart_printf("BSP_ACCELERO_Get_ODR =%f \r\n",odr);
	BSP_ACCELERO_Get_Sensitivity( LSM6DS3_X_0_handle, &fullscale );
//	uart_printf("BSP_ACCELERO_Get_Sensitivity =%f\r\n",fullscale);
  }
  if (G_LSM6DS3_DIL24)
  {
	  //416hz 1000dps setting
    BSP_GYRO_Sensor_Enable(LSM6DS3_G_0_handle);
//	odr=416.0f;
//	BSP_GYRO_Set_ODR_Value(LSM6DS3_G_0_handle,odr);

    BSP_GYRO_Get_ODR(LSM6DS3_G_0_handle,&odr);
//	uart_printf("BSP_GYRO_Get_ODR =%f \r\n",odr);
    BSP_GYRO_Get_Sensitivity(LSM6DS3_G_0_handle,&fullscale);
//	uart_printf("BSP_GYRO_Get_Sensitivity =%f \r\n",fullscale);
  }
// while(1);
}

/**
 * @brief  Disable all sensors
 * @param  None
 * @retval None
 */
#if 0
static void disableAllSensors(void)
{
  if (X_LSM6DS3_DIL24)
  {
    BSP_ACCELERO_Sensor_Disable(LSM6DS3_X_0_handle);
  }
  if (G_LSM6DS3_DIL24)
  {
    BSP_GYRO_Sensor_Disable(LSM6DS3_G_0_handle);
  }
}
#endif

void ACCELERO_Get_Axes( float *acceleration )
{
	SensorAxes_t acc_raw_data;
	
#if 0
	BSP_ACCELERO_Get_Axes(LSM6DS3_X_0_handle, &acc_raw_data);

	
if ( LSM6DS3_X_Get_Axes_Raw( handle, dataRaw ) == COMPONENT_ERROR )
{
  return COMPONENT_ERROR;
}

/* Get LSM6DS3 actual sensitivity. */
if ( LSM6DS3_X_Get_Sensitivity( handle, &sensitivity ) == COMPONENT_ERROR )
{
  return COMPONENT_ERROR;
}


/* Calculate the data. */
acceleration->AXIS_X = ( int32_t )( dataRaw[0] * sensitivity );
acceleration->AXIS_Y = ( int32_t )( dataRaw[1] * sensitivity );
acceleration->AXIS_Z = ( int32_t )( dataRaw[2] * sensitivity );

#else



	
	SensorAxesRaw_t acc_raw_data1;
	float Sensitivity=0.061f;
	BSP_ACCELERO_Get_AxesRaw(LSM6DS3_X_0_handle, &acc_raw_data1);
	BSP_ACCELERO_Get_Sensitivity(LSM6DS3_X_0_handle, &Sensitivity);//°íÁ¤°ª 0.061
	
/*	Sensor.data.XACC[0]=(acc_raw_data1.AXIS_X>>8)&0xff;
	Sensor.data.XACC[1]=acc_raw_data1.AXIS_X&0xff;
	Sensor.data.YACC[0]=(acc_raw_data1.AXIS_Y>>8)&0xff;
	Sensor.data.YACC[1]=acc_raw_data1.AXIS_Y&0xff;
	Sensor.data.ZACC[0]=(acc_raw_data1.AXIS_Z>>8)&0xff;
	Sensor.data.ZACC[1]=acc_raw_data1.AXIS_Z&0xff;
	*/
	acc_raw_data.AXIS_X=(int32_t)(acc_raw_data1.AXIS_X * Sensitivity );
	acc_raw_data.AXIS_Y=(int32_t)(acc_raw_data1.AXIS_Y * Sensitivity );
	acc_raw_data.AXIS_Z=(int32_t)(acc_raw_data1.AXIS_Z * Sensitivity );
//	uart_printf("%f \r\n",Sensitivity);

#endif
	acceleration[0]=(float)acc_raw_data.AXIS_X*9.8f/1000.0f;//g
	acceleration[1]=(float)acc_raw_data.AXIS_Y*9.8f/1000.0f;
	acceleration[2]=(float)acc_raw_data.AXIS_Z*9.8f/1000.0f;
}

/*
rad/s
*/
void GYRO_Get_Axes(float *angular_velocity)
{
	 SensorAxes_t gyr_r_data;
#if 1
	BSP_GYRO_Get_Axes(LSM6DS3_G_0_handle, &gyr_r_data); 
	//float Sensitivity;
	#else
	
	 SensorAxesRaw_t gyr_r_data1;
	 SensorAxesRaw_t gyr_rfi_data1;
	 float Sensitivity;
	 Axis3i16 accelPrim;
	 Axis3i16 accelPrimLPF;
	 static Axis3i32 accelPrimStoredFilterValues;
	 static uint8_t sensorsAccLpfAttFactor=IMU_ACC_IIR_LPF_ATT_FACTOR;
	char datalog[100]; 	   /*!< DataOut Frame */
	BSP_GYRO_Get_Sensitivity(LSM6DS3_G_0_handle, &Sensitivity);
	BSP_GYRO_Get_AxesRaw(LSM6DS3_G_0_handle, &gyr_r_data1);
	gyr_rfi_data1.AXIS_X = iirLPFilterSingle(gyr_r_data1.AXIS_X, sensorsAccLpfAttFactor, &accelPrimStoredFilterValues.x);
	gyr_rfi_data1.AXIS_Y = iirLPFilterSingle(gyr_r_data1.AXIS_Y, sensorsAccLpfAttFactor, &accelPrimStoredFilterValues.y);
	gyr_rfi_data1.AXIS_Z = iirLPFilterSingle(gyr_r_data1.AXIS_Z, sensorsAccLpfAttFactor, &accelPrimStoredFilterValues.z);
	gyr_r_data.AXIS_X=( int32_t )( gyr_rfi_data1.AXIS_X * Sensitivity );
	gyr_r_data.AXIS_Y=( int32_t )( gyr_rfi_data1.AXIS_Y * Sensitivity );
	gyr_r_data.AXIS_Z=( int32_t )( gyr_rfi_data1.AXIS_Z * Sensitivity );

	/*if(TIMER_SYSTEM[Timer_40ms].On)
	{
		Timer_Reset(Timer_40ms);
		//sprintf(datalog,"%05d	%05d \r\n",gyr_raw_data1.AXIS_X,gyr_raw_data.AXIS_X);
	//	sprintf(datalog,"TT %d\r\n",gyr_r_data1.AXIS_X);
	//	HAL_UART_Transmit_DMA(&huart1, (uint8_t*) datalog, strlen(datalog));
		uart_printf("%05d	%05d \r\n",gyr_r_data1.AXIS_X,gyr_rfi_data1.AXIS_X);
	}*/

	#endif
	//Sensor.data.DT
	//BSP_GYRO_Get_Sensitivity(LSM6DS3_G_0_handle, &Sensitivity);
	angular_velocity[0]=(float)gyr_r_data.AXIS_X/1000.0f;//dps/s
	angular_velocity[1]=(float)gyr_r_data.AXIS_Y/1000.0f;
	angular_velocity[2]=(float)gyr_r_data.AXIS_Z/1000.0f;

	
//	angular_velocity[0]=angular_velocity[0]*PI/180.0f;//rad/s
//	angular_velocity[1]=angular_velocity[1]*PI/180.0f;
//	angular_velocity[2]=angular_velocity[2]*PI/180.0f;
	angular_velocity[0]=angular_velocity[0]*0.01745f;//rad/s
	angular_velocity[1]=angular_velocity[1]*0.01745f;
	angular_velocity[2]=angular_velocity[2]*0.01745f;
	

}
void Calibration_Gyro()//(float *Calgyro)
{
	float gyr_raw_data[3]={0.0};
	float sumgyr_raw_data[3]={0.0};
	int i=0;
	for(i=0;i<200;i++)
	{
		GYRO_Get_Axes(gyr_raw_data);
		HAL_Delay(5);
		sumgyr_raw_data[0]+=gyr_raw_data[0];
		sumgyr_raw_data[1]+=gyr_raw_data[1];
		sumgyr_raw_data[2]+=gyr_raw_data[2];
	}
	gyr_raw_data[0]=sumgyr_raw_data[0]/200;
	gyr_raw_data[1]=sumgyr_raw_data[1]/200;
	gyr_raw_data[2]=sumgyr_raw_data[2]/200;
	//printf("%8.4f %8.4f %8.4f \r\n",gyr_raw_data[0],gyr_raw_data[1],gyr_raw_data[2]);
	for(i=0;i<400;i++)
	{
		GYRO_Get_Axes(gyr_raw_data);
		HAL_Delay(5);
		sumgyr_raw_data[0]+=gyr_raw_data[0];
		sumgyr_raw_data[1]+=gyr_raw_data[1];
		sumgyr_raw_data[2]+=gyr_raw_data[2];
	}
	
	gyr_raw_data[0]=sumgyr_raw_data[0]/400;
	gyr_raw_data[1]=sumgyr_raw_data[1]/400;
	gyr_raw_data[2]=sumgyr_raw_data[2]/400;
	//printf("%8.4f %8.4f %8.4f \r\n",gyr_raw_data[0],gyr_raw_data[1],gyr_raw_data[2]);
}
/*
before  ms
current
*/
float TimeStamp_Gap(uint16_t before,uint16_t current,uint8_t type)
{	
	if(type==1) return 0.2f;
	int16_t gap=0;
	gap = ((int16_t)current-(int16_t)before);
	if(gap<0)gap+=1000;
	//uart_printf("%f\r\n",(float)((float)gap/1000.0f));
	return	(float)((float)gap/1000.0f); //sec
	
}


/*
 * init 9d compass algorithm
 */
void Ini_Algo(void)
{
    IniPara iniPara;

	iniPara.t = ts0;
	iniPara.iniCalPara = iniCalPara;
	iniPara.iniAccuracy = iniAccuracy;
	iniPara.si = (float*)si;
	iniPara.magVar = magVar;
	iniPara.enableGyroCal = enableGyroCal;
	iniPara.outlierGyroRestart = outlierGyroRestart;
	iniPara.iniGyroBias = iniGyroBias;
	iniPara.gyroGrade = gyroGrade;
	iniPara.extraSampleCount = extraSampleCount;

	InitializeAlgo(iniPara);
}
/*  
 * Sart or restart the algo, need to call this function. 
 */
void Enable_Algo(void)
{
	iRestart = 1;
}

/*  
 * Sart or restart the algo, need to call this function. 
 return ts ½Ã°£ ms´ÜÀ§.
 */
uint16_t AlgoPoll(void)
{
	//LEDR_G_ON;
	//LEDR_G_OFF;
	/* This variable is used to store the acc, mag and gyro raw data.
	 * please NOTE that the data is going to store here MUST has been transmitted 
	 * to match the Right-Handed coordinate sytem already.
	 */
	//uint16_t subSec = 0,curTimeStamp=0;
	uint16_t curTimeStamp=0;
	static uint16_t preTimeStamp=0;
	//char datalog[256];		  /*!< DataOut Frame */
	float acc_raw_data[3] = {0.0};	//accelerometer field vector, unit is m/s^2
	float acc_real_data[3] = {0.0}; 
	
	float mag_raw_data[3] = {0.0};	//magnetic field vector, unit is uT
	float mag_real_data[3] = {0.0};
	
	float gyr_raw_data[3] = {0.0};	//gyroscope field vector, unit is rad/s
	float gyr_real_data[3] = {0.0};	
	
	//int sampling_rate = 50;	//sampling rate, unit is Hz.
	float ts = 0.02f; //sampling interval, unit is second.
	
	/* This variable is used to save the calibrated mag sensor XYZ output. */
	float caliMag[3] = {0};
	
	/* This variable is used to save the calibrated orientation data. */
	float caliOri[3] = {0};		

	
	float LinearAcc[3] = {0.0};	
	float gravi_data[3] = {0.0};	
	//float quaternion[4] = {0.0};	
	/* Get the accelerometer sensor data, unit is m/s^2*/
	ACCELERO_Get_Axes(acc_raw_data);
	//acc_raw_data[0] = ;	//unit m/s^2	
	//acc_raw_data[1] = ;	
	//acc_raw_data[2] = ;	
	
	/* get the magnetic sensor data, unit is uT */	
	MMC3630KJ_GetData(mag_raw_data); //unit is Gauss;
	mag_raw_data[0] = mag_raw_data[0]*100.0f; //convert to uT;
	mag_raw_data[1] = mag_raw_data[1]*100.0f;
	mag_raw_data[2] = mag_raw_data[2]*100.0f;

	/* get the gyroscope sensor data, unit is rad/s */	
	GYRO_Get_Axes(gyr_raw_data);
	//gyr_raw_data[0] = ;	//unit rad/s
	//gyr_raw_data[1] = ;
	//gyr_raw_data[2] = ;
	
	/* convert the coordinate system */		
	acc_coordinate_raw_to_real(A_LAYOUT, acc_raw_data, acc_real_data);
	mag_coordinate_raw_to_real(M_LAYOUT, mag_raw_data, mag_real_data);
	gyr_coordinate_raw_to_real(G_LAYOUT, gyr_raw_data, gyr_real_data);	
	sys.accx=acc_real_data[0];
	sys.accy=acc_real_data[1];
	sys.accz=acc_real_data[2];
	sys.gyrox=gyr_raw_data[0];
	sys.gyroy=gyr_raw_data[1];
	sys.gyroz=gyr_raw_data[2];

	tohi.accx=sys.accx;
	tohi.accy=sys.accy;
	tohi.accz=sys.accz;
	tohi.gyro_x=sys.gyrox;
	tohi.gyro_y=sys.gyroy;
	tohi.gyro_z=sys.gyroz;

	//float curg=0.0f;
	//curg=sqrt(acc_real_data[0]*acc_real_data[0]+acc_real_data[1]*acc_real_data[1]+acc_real_data[2]*acc_real_data[2]);
	//	if(curg>2.0f)return;
	

	/* get the dlta time, unit is second */
	curTimeStamp = GetSystemTimeStamp();
	ts = TimeStamp_Gap(preTimeStamp,curTimeStamp,iRestart);
	//ts = curTimeStamp - preTimeStamp;
	preTimeStamp = curTimeStamp;
	//sampling_rate = GetSamplingRate();
	//ts = 1.0f/(float)sampling_rate;
    ts = 0.02f;

	/* below functions are algorithm interface. 
	 * input acc, mag data into algorithm
	 * make sure that acc and mag XYZ data meet the right-hand(right-front-up) coordinate system
	 */
	MainAlgoProcess(acc_real_data, mag_real_data, gyr_real_data, ts, iRestart,iAlgoCal);
	
	iRestart = 0;
	
	/* get corrected mag data */
	GetCalibratedMag(caliMag);
	
	/* calibrated magnetic sensor output */
	magX = caliMag[0];	//unit is uT
	magY = caliMag[1];	//unit is uT
	magZ = caliMag[2];	//unit is uT
	
	/* get corrected ori data */
	GetOri(caliOri);
		/* Get the accuracy of the algorithm */
	sys.imu_iAccuracy = iAccuracy = GetAccuracy();
	
	/* Get the fAzimuth Pitch Roll Accuracy for the eCompass */
	sys.yaw		=fAzimuth  = caliOri[0];
	sys.pitch	=fPitch    = caliOri[1];
	sys.roll	=fRoll     = caliOri[2];	
	
	tohi.yaw=sys.yaw;
	tohi.pitch=sys.pitch;
	tohi.roll=sys.roll;
	/*if(sys.yaw_req)
	{	
		sys.yaw_req=0;
		sys.yaw_req_ok=1;
		sys.cmd_yaw=sys.yaw;
	}*/


/*	if(iAccuracy==3)
	{
		Sensor.data.DT= (uint8_t)(ts *1000);
	}*/
//	int st=0;
//	static int k=1;
//	st=GetGyroCalPara(gravi_data);
	GetLinearAcc(LinearAcc);
	GetGravity(gravi_data);
	//GetRotVec(quaternion);
	/*if(st==2 && k)
	{k=0;
		iRestart=1;
		iniGyroBias[0] = gravi_data[0];//ôøã·öíÕ¢ßÈø¶£¬rad/s
		iniGyroBias[1] = gravi_data[1];//ôøã·öíÕ¢ßÈø¶£¬rad/s
		iniGyroBias[2] = gravi_data[2];//ôøã·öíÕ¢ßÈø¶£¬rad/s
	}*/
	float curg=0.0f;
	curg=sqrt(gravi_data[0]*gravi_data[0]+gravi_data[1]*gravi_data[1]+gravi_data[2]*gravi_data[2]);
	//GetGravity(gravi_data);
	/* Get the SET Flag from algorithm */   
	if(GetMagSatStatus())  
	{
	//	uart_printf("MMC3630KJ_SET\r\n");
		//MEMSIC_Magnetic_Sensor_SET();	//Do SET action
		MMC3630KJ_SET();
		I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_CTRL0, MMC3630KJ_CMD_TM_M);
		//iRestart = 1;
		//DCMOTOR.Value.FLAG=0;
	}		  
	//uart_printf("[%03d][%4.3f][%4.3f] fAzimuth:%8.2f, fPitch:%8.2f, fRoll:%8.2f\r\n",iAccuracy,ts,curg,fAzimuth,fPitch,fRoll);hwp
	//if(TIMER_SYSTEM[Timer_40ms].On)
	{
	    /*
        Sensor.data.LEN_0 = 16;
        Sensor.data.STATUS=0;
        Sensor.data.STATUS=sys.acc_irq_1;
        Sensor.data.STATUS|=(iAccuracy<<1);
        Sensor.data.STATUS|= (uint8_t)(ts *1000)<<3;
        Sensor.data.TOF=sys.tof_org_rang;
        Sensor.data.XACC=(int16_t)(acc_real_data[0]*1000);
        Sensor.data.YACC=(int16_t)(acc_real_data[1]*1000); //
        Sensor.data.ZACC=(int16_t)(acc_real_data[2]*1000);
        Sensor.data.YAW=(int16_t)sys.yaw;
        Sensor.data.PITCH=(int16_t)sys.pitch;
		Sensor.data.ROLL=(int16_t)sys.roll;
		Sensor.data.ZGYRO=(int16_t)(gyr_raw_data[2]*1000); //

		//uart_printf("YAW:%10f[%04d] PITCH:%10f[%04d] \r\n",sys.yaw,Sensor.data.YAW,sys.pitch,Sensor.data.PITCH);
		if(sys.funckey != 92 && sys.acc_irq_1==SET) sys.acc_irq_1=0;
		//Timer_Reset(Timer_40ms);
		if(sys.ai_on==1)
		{
		//Sensor.data.CNT++;
		//	if(Sensor.data.CNT==10)Sensor.data.CNT=0;
			Send_Ble(&Sensor.Buf[0],6+14); //cmd¸¸ ¸®ÅÏÇÑ´Ù.
		}
	//	curg=sqrt(acc_real_data[0]*acc_real_data[0]+acc_real_data[1]*acc_real_data[1]+acc_real_data[2]*acc_real_data[2]);
	//	sprintf(datalog,"[%03d][%4.3f][%4.3f] fAzimuth:%8.2f, fPitch:%8.2f, fRoll:%8.2f\r\n",iAccuracy,ts,curg,fAzimuth,fPitch,fRoll);
	//	sprintf(datalog,"%d, g:%10f, a:%10f, aZ:%10f, aZ:%10f\r\n",st,gravi_data[2],LinearAcc[2],gravi_data[2],acc_raw_data[2]);
	//	sprintf(datalog,"gX:%10f, gY:%10f, gZ:%10f\r\n",gyr_raw_data[0],gyr_raw_data[1],gyr_raw_data[2]);
	//	sprintf(datalog,"mX:%10f, mY:%10f, mZ:%10f\r\n",mag_real_data[0],mag_real_data[1],mag_real_data[2]);
	
	//	sprintf(datalog,"LX:%10f, LY:%10f, LZ:%10f\r\n",LinearAcc[0],LinearAcc[1],LinearAcc[2]);
	//	sprintf(datalog,"AX:%10f, AY:%10f, AZ:%10f fAzimuth:%8.2f\r\n",acc_raw_data[0],acc_raw_data[1],acc_raw_data[2],fAzimuth);
	//	HAL_UART_Transmit_DMA(&huart1, (uint8_t*) datalog, strlen(datalog));
//		uart_printf("AX:%10f, AY:%10f, AZ:%10f \r\n",acc_raw_data[0],acc_raw_data[1],acc_raw_data[2]);
	//	uart_printf("[%03d][%4.3f][%4.3f] fAzimuth:%8.2f, fPitch:%8.2f, fRoll:%8.2f :%d %d\r\n",iAccuracy,ts,curg,fAzimuth,fPitch,fRoll,Sensor.data.YAW[3],Sensor.data.YAW[4]);
		int16_t gz=(int16_t)Sensor.data.YAW[1];
		gz|=(int16_t)Sensor.data.YAW[0]<<8;
		uart_printf("GX:%10f, GY:%10f, GZ:%10f [%10d]\r\n",gyr_real_data[0]/0.01745f,gyr_real_data[1]/0.01745f,gyr_real_data[2]/0.01745f,gz);
	*/
		//uart_printf("GX:%10f, GY:%10f, GZ:%10f [%10d]\r\n",gyr_real_data[0]/0.01745f,gyr_real_data[1]/0.01745f,gyr_real_data[2]/0.01745f,gz);

/*	
	InUseduart2=1;
	HAL_UART_Transmit_DMA(&huart2,buf,Remainder+5);//,100);
*/
	#if 0
	char buf[200]={0};
//	sprintf(buf, "{\"fAzimuth\":%8.2f,\"fPitch\":%8.2f,\"fRoll\":%8.2f}",fAzimuth,fPitch,fRoll=0.0f);
	sprintf(buf, "{\"D\":[%5.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f]}",ts,acc_real_data[0],acc_real_data[1],acc_real_data[2] \
	,LinearAcc[0],LinearAcc[1],LinearAcc[2],fAzimuth,fPitch,fRoll);
	char sss[3];
	sss[0] = 0x8a;
	sss[1] = 0x8a;
	sss[2] = strlen(buf);
	HAL_UART_Transmit(&huart1, (uint8_t*)sss,3,20);
	uart_printf("%s",buf);
	#endif
//	 uart_printf("{\"fAzimuth\":%8.2f,\"fPitch\":%8.2f,\"fRoll\":%8.2f}",fAzimuth,fPitch,fRoll);
		//uart_printf("[%03d]%8.2f	%8.2f	%8.2f	%8.2f\r\n",iAccuracy,quaternion[0],quaternion[1],quaternion[2],quaternion[3]);
		//uart_printf("LX:%10f, LY:%10f, LZ:%10f\r\n",LinearAcc[0],LinearAcc[1],LinearAcc[2]);
	}

    
	return (uint16_t)(ts *1000);
}

void AlgoPoll_IRQ(void)
{
	//LEDR_G_ON;
	//LEDR_G_OFF;
	/* This variable is used to store the acc, mag and gyro raw data.
	 * please NOTE that the data is going to store here MUST has been transmitted 
	 * to match the Right-Handed coordinate sytem already.
	 */
	//uint16_t subSec = 0,curTimeStamp=0;
	uint16_t curTimeStamp=0;
	static uint16_t preTimeStamp=0;
	//char datalog[256];		  /*!< DataOut Frame */
	float acc_raw_data[3] = {0.0};	//accelerometer field vector, unit is m/s^2
	float acc_real_data[3] = {0.0}; 
	
	float mag_raw_data[3] = {0.0};	//magnetic field vector, unit is uT
	float mag_real_data[3] = {0.0};
	
	float gyr_raw_data[3] = {0.0};	//gyroscope field vector, unit is rad/s
	float gyr_real_data[3] = {0.0};	
	
	//int sampling_rate = 50;	//sampling rate, unit is Hz.
	float ts = 0.02f; //sampling interval, unit is second.
	
	/* This variable is used to save the calibrated mag sensor XYZ output. */
	float caliMag[3] = {0};
	
	/* This variable is used to save the calibrated orientation data. */
	float caliOri[3] = {0};		

	
	float LinearAcc[3] = {0.0};	
	float gravi_data[3] = {0.0};	
	/* Get the accelerometer sensor data, unit is m/s^2*/
	ACCELERO_Get_Axes(acc_raw_data);
	//acc_raw_data[0] = ;	//unit m/s^2	
	//acc_raw_data[1] = ;	
	//acc_raw_data[2] = ;	
	
	/* get the magnetic sensor data, unit is uT */	
	MMC3630KJ_GetData(mag_raw_data); //unit is Gauss;
	mag_raw_data[0] = mag_raw_data[0]*100.0f; //convert to uT;
	mag_raw_data[1] = mag_raw_data[1]*100.0f;
	mag_raw_data[2] = mag_raw_data[2]*100.0f;

	/* get the gyroscope sensor data, unit is rad/s */	
	GYRO_Get_Axes(gyr_raw_data);
	//gyr_raw_data[0] = ;	//unit rad/s
	//gyr_raw_data[1] = ;
	//gyr_raw_data[2] = ;
	
	/* convert the coordinate system */		
	acc_coordinate_raw_to_real(A_LAYOUT, acc_raw_data, acc_real_data);
	mag_coordinate_raw_to_real(M_LAYOUT, mag_raw_data, mag_real_data);
	gyr_coordinate_raw_to_real(G_LAYOUT, gyr_raw_data, gyr_real_data);	

	/* get the dlta time, unit is second */
	curTimeStamp = GetSystemTimeStamp();
	ts = TimeStamp_Gap(preTimeStamp,curTimeStamp,iRestart);
	//ts = curTimeStamp - preTimeStamp;
	preTimeStamp = curTimeStamp;
	//sampling_rate = GetSamplingRate();
	//ts = 1.0f/(float)sampling_rate;
	//ts = 0.02f;
	//float curg=0.0f;
	//curg=arm_sqrt(acc_real_data[0]*acc_real_data[0]+acc_real_data[1]*acc_real_data[1]+acc_real_data[2]*acc_real_data[2]);
//	if(curg>2.0f)return;

	/* below functions are algorithm interface. 
	 * input acc, mag data into algorithm
	 * make sure that acc and mag XYZ data meet the right-hand(right-front-up) coordinate system
	 */
	MainAlgoProcess(acc_real_data, mag_real_data, gyr_real_data, ts, iRestart,iAlgoCal);
	
	iRestart = 0;
	
	/* get corrected mag data */
	GetCalibratedMag(caliMag);
	
	/* calibrated magnetic sensor output */
	magX = caliMag[0];	//unit is uT
	magY = caliMag[1];	//unit is uT
	magZ = caliMag[2];	//unit is uT
	
	/* get corrected ori data */
	GetOri(caliOri);
	
	/* Get the fAzimuth Pitch Roll Accuracy for the eCompass */
	fAzimuth  = caliOri[0];
	fPitch    = caliOri[1];
	fRoll     = caliOri[2];	

	/* Get the accuracy of the algorithm */
	iAccuracy = GetAccuracy();
	//int st=0;
	//static int k=1;
//	st=GetGyroCalPara(gravi_data);
	GetLinearAcc(LinearAcc);
	GetGravity(gravi_data);
	/*if(st==2 && k)
	{k=0;
		iRestart=1;
		iniGyroBias[0] = gravi_data[0];//ôøã·öíÕ¢ßÈø¶£¬rad/s
		iniGyroBias[1] = gravi_data[1];//ôøã·öíÕ¢ßÈø¶£¬rad/s
		iniGyroBias[2] = gravi_data[2];//ôøã·öíÕ¢ßÈø¶£¬rad/s
	}*/
//	curg=sqrt(gravi_data[0]*gravi_data[0]+gravi_data[1]*gravi_data[1]+gravi_data[2]*gravi_data[2]);
	//GetGravity(gravi_data);
	/* Get the SET Flag from algorithm */   
	if(GetMagSatStatus())  
	{
	//	uart_printf("MMC3630KJ_SET\r\n");
		//MEMSIC_Magnetic_Sensor_SET();	//Do SET action
		MMC3630KJ_SET();
		/* Write 0x01 to register 0x08, set TM_M bit high */
		I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_CTRL0, MMC3630KJ_CMD_TM_M);
		iRestart = 1;
	}		
	/*if(TIMER_SYSTEM[Timer_40ms].On)
	{
		Timer_Reset(Timer_40ms);
		sprintf(datalog,"[%03d][%4.3f][%4.3f] fAzimuth:%8.2f, fPitch:%8.2f, fRoll:%8.2f\r\n",iAccuracy,ts,curg,fAzimuth,fPitch,fRoll);
	//	sprintf(datalog,"%d, g:%10f, a:%10f, aZ:%10f, aZ:%10f\r\n",st,gravi_data[2],LinearAcc[2],gravi_data[2],acc_raw_data[2]);
	//	sprintf(datalog,"gX:%10f, gY:%10f, gZ:%10f\r\n",gyr_raw_data[0],gyr_raw_data[1],gyr_raw_data[2]);
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*) datalog, strlen(datalog));
	}*/
	return;
}


int Imu_mag_init(void)
{	
	int ret=-1;
	/* initial the 9d compass algorithm */	 
	Ini_Algo();
	/* initial the 9d compass algorithm */	 
	Enable_Algo();
	/* initial the sensor when power on. */	
	while(ret<0)
	{
		ret=MMC3630KJ_Initialization(); 
		uart_printf("MMC3630KJ_Initialization=%d\r\n",ret);
		if(ret < 0)sys.maginiterror= 1;
	}
	/* enable the sensor when from pown down mode to normal mode. */
	MMC3630KJ_Enable();

	return 0;
}

/*  
 * return the sampling rate; 
 */	
int GetSamplingRate(void)
{
	return iSamplingRate;
}

/*  
 * set the sampling rate; 
 */		
void SetSamplingRate(int t)
{
	iSamplingRate = t;
	
	return;
}

/* return the sensor state
 */
int GetAlgoState(void)
{
	iAlgoState = 1;
	
	return iAlgoState;	
}

/* return the sensor state
 */
void SetAlgoState(int state)
{	
	/* 
	.
	. Need to be implemented by user. 
	. If need to open the algorithm, then make iAlgoState = 1.
	. If need to close the algorithm, then make iAlgoState = 0.
	*/
	iAlgoState = state;
	
	return; 	
}

/* Convert the sensor coordinate to right-front-up coordinate system;
 */
void acc_coordinate_raw_to_real(int layout, float *in, float *out)
{
	switch (layout) {
	case 0:
		out[0] =  in[0];
		out[1] =  in[1];
		out[2] =  in[2];
		break;
	case 1:
		out[0] = -in[1];
		out[1] =  in[0];
		out[2] =  in[2];
		break;
	case 2:
		out[0] = -in[0];
		out[1] = -in[1];
		out[2] =  in[2];
		break;
	case 3:
		out[0] =  in[1];
		out[1] = -in[0];
		out[2] =  in[2];
		break;
	case 4:
		out[0] =  in[1];
		out[1] =  in[0];
		out[2] = -in[2];
		break;
	case 5:
		out[0] = -in[0];
		out[1] =  in[1];
		out[2] = -in[2];
		break;
	case 6:
		out[0] = -in[1];
		out[1] = -in[0];
		out[2] = -in[2];
		break;
	case 7:
		out[0] =  in[0];
		out[1] = -in[1];
		out[2] = -in[2];
		break;
	default:
		out[0] =  in[2];
		out[1] =  -in[1];
		out[2] =  in[0];
		break;
	}
}
/* Convert the sensor(MMC3630KJ) coordinate to right-front-up coordinate system;
 */
void mag_coordinate_raw_to_real(int layout, float *in, float *out)
{
	switch (layout) {
	case 0:
		out[0] =  in[0];
		out[1] =  in[1];
		out[2] =  in[2];
		break;
	case 1:
		out[0] = -in[1];
		out[1] =  in[0];
		out[2] =  in[2];
		break;
	case 2:
		out[0] = -in[0];  
		out[1] = -in[1];
		out[2] =  in[2];
		break;
	case 3:
		out[0] =  in[1];
		out[1] = -in[0];
		out[2] =  in[2];
		break;
	case 4:
		out[0] =  in[1];
		out[1] =  in[0];
		out[2] = -in[2];
		break;
	case 5:
		out[0] = -in[0];
		out[1] =  in[1];
		out[2] = -in[2];
		break;
	case 6:
		out[0] = -in[1];
		out[1] = -in[0];
		out[2] = -in[2];
		break;
	case 7:
		out[0] =  in[0];
		out[1] = -in[1];
		out[2] = -in[2];
		break;		
	default:
		out[0] =  in[2];
		out[1] =  in[0];
		out[2] =  in[1];
		break;
	}
}

/* Convert the sensor coordinate to right-front-up coordinate system;
 */
void gyr_coordinate_raw_to_real(int layout, float *in, float *out)
{
	switch (layout) {
	case 0:
		out[0] = in[0];
		out[1] = in[1];
		out[2] = in[2];
		break;
	case 1:
		out[0] = -in[1];
		out[1] =  in[0];
		out[2] =  in[2];
		break;
	case 2:
		out[0] = -in[0];
		out[1] = -in[1];
		out[2] =  in[2];
		break;
	case 3:
		out[0] =  in[1];
		out[1] = -in[0];
		out[2] =  in[2];
		break;
	case 4:
		out[0] =  in[1];
		out[1] =  in[0];
		out[2] = -in[2];
		break;
	case 5:
		out[0] = -in[0];
		out[1] =  in[1];
		out[2] = -in[2];
		break;
	case 6:
		out[0] = -in[1];
		out[1] = -in[0];
		out[2] = -in[2];
		break;
	case 7:
		out[0] =  in[0];
		out[1] = -in[1];
		out[2] = -in[2];
		break;
	default:
		out[0] =  in[2];
		out[1] =  -in[1];
		out[2] =  in[0];
		break;
	}
}

