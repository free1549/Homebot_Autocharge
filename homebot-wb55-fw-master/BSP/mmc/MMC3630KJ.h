/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information and source code
 *  contained herein is confidential. The software including the source code
 *  may not be copied and the information contained herein may not be used or
 *  disclosed except with the written permission of MEMSIC Inc. (C) 2017
 *****************************************************************************/

/**
 * @brief
 * This file define magnetic sensor driver APIs. Data:2017-10-26
 */
 
typedef   signed char  int8_t; 		// signed 8-bit number    (-128 to +127)
typedef unsigned char  uint8_t; 	// unsigned 8-bit number  (+0 to +255)
typedef   signed short int16_t; 	// signed 16-bt number    (-32768 to +32767)
typedef unsigned short uint16_t; 	// unsigned 16-bit number (+0 to +65535)
typedef   signed int   int32_t; 	// signed 32-bt number    (-2,147,483,648 to +2,147,483,647)
typedef unsigned int   uint32_t; 	// unsigned 32-bit number (+0 to +4,294,967,295)

#define MMC3630KJ_7BITI2C_ADDRESS	0x60

#define MMC3630KJ_REG_DATA			0x00
#define MMC3630KJ_REG_XL			0x00
#define MMC3630KJ_REG_XH			0x01
#define MMC3630KJ_REG_YL			0x02
#define MMC3630KJ_REG_YH			0x03
#define MMC3630KJ_REG_ZL			0x04
#define MMC3630KJ_REG_ZH			0x05
#define MMC3630KJ_REG_TEMP			0x06
#define MMC3630KJ_REG_STATUS		0x07
#define MMC3630KJ_REG_CTRL0			0x08
#define MMC3630KJ_REG_CTRL1			0x09
#define MMC3630KJ_REG_CTRL2			0x0A
#define MMC3630KJ_REG_X_THD			0x0B
#define MMC3630KJ_REG_Y_THD			0x0C
#define MMC3630KJ_REG_Z_THD			0x0D
#define MMC3630KJ_REG_SELFTEST		0x0E
#define MMC3630KJ_REG_PASSWORD		0x0F
#define MMC3630KJ_REG_OTPMODE		0x12
#define MMC3630KJ_REG_TESTMODE		0x13
#define MMC3630KJ_REG_SR_PWIDTH		0x20
#define MMC3630KJ_REG_OTP			0x2A
#define MMC3630KJ_REG_PRODUCTID		0x2F
 
#define MMC3630KJ_CMD_REFILL		0x20
#define MMC3630KJ_CMD_RESET         0x10
#define MMC3630KJ_CMD_SET			0x08
#define MMC3630KJ_CMD_TM_M			0x01
#define MMC3630KJ_CMD_TM_T			0x02
#define MMC3630KJ_CMD_START_MDT		0x04
#define MMC3630KJ_CMD_100HZ			0x00
#define MMC3630KJ_CMD_200HZ			0x01
#define MMC3630KJ_CMD_400HZ			0x02
#define MMC3630KJ_CMD_600HZ			0x03
#define MMC3630KJ_CMD_CM_14HZ		0x01
#define MMC3630KJ_CMD_CM_5HZ		0x02
#define MMC3630KJ_CMD_CM_1HZ		0x04
#define MMC3630KJ_CMD_SW_RST		0x80
#define MMC3630KJ_CMD_PASSWORD		0xE1
#define MMC3630KJ_CMD_OTP_OPER		0x11
#define MMC3630KJ_CMD_OTP_MR		0x80
#define MMC3630KJ_CMD_OTP_ACT		0x80
#define MMC3630KJ_CMD_OTP_NACT		0x00
#define MMC3630KJ_CMD_STSET_OPEN	0x02
#define MMC3630KJ_CMD_STRST_OPEN	0x04
#define MMC3630KJ_CMD_ST_CLOSE		0x00
#define MMC3630KJ_CMD_INT_MD_EN		0x40
#define MMC3630KJ_CMD_INT_MDT_EN	0x20

#define MMC3630KJ_PRODUCT_ID		0x0A
#define MMC3630KJ_OTP_READ_DONE_BIT	0x10
#define MMC3630KJ_PUMP_ON_BIT		0x08
#define MMC3630KJ_MDT_BIT			0x04
#define MMC3630KJ_MEAS_T_DONE_BIT	0x02
#define MMC3630KJ_MEAS_M_DONE_BIT	0x01

// 16-bit mode, null field output (32768)
#define MMC3630KJ_OFFSET			32768.0
#define MMC3630KJ_SENSITIVITY		1024.0
#define MMC3630KJ_T_ZERO			(-75.0)
#define MMC3630KJ_T_SENSITIVITY		0.8

#define OTP_CONVERT(REG)		 ((float)((REG) >=32 ? (32 - (REG)) : (REG)) * 0.006)

/**
 * @brief Initialization first of all
 */
int MMC3630KJ_Initialization(void);

/**
 * @brief Enable the sensor
 */
void MMC3630KJ_Enable(void);

/**
 * @brief Disable the sensor
 */
void MMC3630KJ_Disable(void);

/**
 * @brief SET operation
 */
void MMC3630KJ_SET(void);

/**
 * @brief RESET operation
 */
void MMC3630KJ_RESET(void);

/**
 * @brief Get the temperature output
 * @param t_out[0] is the temperature, unit is degree Celsius
 */
void MMC3630KJ_GetTemperature(float *t_out);

/**
 * @brief Get sensor data
 * @param mag_out is the magnetic field vector, unit is gauss
 */
void MMC3630KJ_GetData(float *mag_out);

