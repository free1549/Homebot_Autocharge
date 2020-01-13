#include "Customer.h"
#include "main.h"

#include "stm32wbxx_hal.h"
//#include "stm32f4xx_hal_def.h"
#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1
extern I2C_HandleTypeDef hi2c1;

void Delay_Ms(int cnt)
{
	/*  cnt is the time to wait */
	HAL_Delay(cnt);
	/* 
	.
	. Need to be implemented by user. 
	.
	*/
	
	return;
}
void Delay_Us(int cnt)
{
	/*  cnt is the time to wait */
	
	/* 
	.
	. Need to be implemented by user. 
	.
	*/

	return;
}
int I2C_Write_Reg(unsigned char i2c_add, unsigned char reg_add, unsigned char cmd)
{
	/* i2c_add is the 7-bit i2c address of the sensor
	 * reg_add is the register address to wtite
	 * cmd is the value that need to be written to the register
	 * I2C operating successfully, return 1, otherwise return 0;
	 */
	
	/* 
	.
	. Need to be implemented by user. 
	.
	*/	 
	
	HAL_StatusTypeDef status;
	uint8_t data[5];
	data[0]=reg_add;
	data[1]=cmd;
    status = HAL_I2C_Master_Transmit(&hi2c1, i2c_add, data, 2, I2C_TIME_OUT_BASE);
    if (status) {
        //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
        //XNUCLEO6180XA1_I2C1_Init(&hi2c1);
        return 0;
    }
	return 1;
}
int I2C_Read_Reg(unsigned char i2c_add, unsigned char reg_add, unsigned char *data)
{
	/* i2c_add is the 7-bit i2c address of the sensor
	 * reg_add is the register address to read
	 * data is the first address of the buffer that need to store the register value
	 * I2C operating successfully, return 1, otherwise return 0;	
	 */
	 	
	/* 
	.
	. Need to be implemented by user. 
	.
	*/	
	 
	HAL_StatusTypeDef status;
	status = HAL_I2C_Master_Transmit(&hi2c1,i2c_add,&reg_add, 1, I2C_TIME_OUT_BASE);//0xd6
	 if (status) {
        return 0;
    }
    status = HAL_I2C_Master_Receive(&hi2c1, i2c_add|0x01, data, 1, I2C_TIME_OUT_BASE);
    if (status) {
        //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
        //XNUCLEO6180XA1_I2C1_Init(&hi2c1);
        return 0;
    }
	return 1;
}
int I2C_MultiRead_Reg(unsigned char i2c_add, unsigned char reg_add, int num, unsigned char *data)
{
	/* i2c_add is the 7-bit i2c address of the sensor
	 * reg_add is the first register address to read
	 * num is the number of the register to read	
	 * data is the first address of the buffer that need to store the register value
	 * I2C operating successfully, return 1, otherwise return 0;	
	 */
	
	/* 
	.
	. Need to be implemented by user. 
	.
	*/
	 int i2c_time_out = I2C_TIME_OUT_BASE+ num* I2C_TIME_OUT_BYTE;
	 HAL_StatusTypeDef status;
	 status = HAL_I2C_Master_Transmit(&hi2c1,i2c_add,&reg_add, 1, I2C_TIME_OUT_BASE);//0xd6
	  if (status) {
		 return 0;
	 }
	 status = HAL_I2C_Master_Receive(&hi2c1, i2c_add|0x01, data, num, i2c_time_out);
	 
	// status = HAL_I2C_Mem_Read(&hi2c1, i2c_add|0x01, reg_add, I2C_MEMADD_SIZE_8BIT, data, num, 50);
	 if (status) {
		 //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
		 //XNUCLEO6180XA1_I2C1_Init(&hi2c1);
		 return 0;
	 }
	return 1;
}
