/*
 * ICM20948.h
 *
 *  Created on: Nov 21, 2024
 *      Author: Alvin
 */

#ifndef INC_ICM20948_H_
#define INC_ICM20948_H_

// include hal code for i2c
#include "stm32wbxx_hal.h"

// I2C Address
#define ICM20948_ADDR        0x69<<1 // Default I2C address when AD0 is low (0x68). Left-shifted for STM32 HAL.
#define Device_ID			 0xEA    // Device ID


// Used Register Addresses
//Bank 0
#define WHO_AM_I             0x00     // Device ID register, default value should be 0xEA.
#define PWR_MGMT_1           0x06     // Power Management 1 register.
#define PWR_MGMT_2           0x07     // Power Management 2 register.
// Accelerometer Output Data Registers
#define ACCEL_XOUT_H 		 0x2D  	  // Accelerometer X-axis high byte
#define ACCEL_XOUT_L 		 0x2E  	  // Accelerometer X-axis low byte
#define ACCEL_YOUT_H 		 0x2F     // Accelerometer Y-axis high byte
#define ACCEL_YOUT_L 		 0x30     // Accelerometer Y-axis low byte
#define ACCEL_ZOUT_H 		 0x31     // Accelerometer Z-axis high byte
#define ACCEL_ZOUT_L 		 0x32     // Accelerometer Z-axis low byte
//Bank 2
#define ACCEL_CONFIG         0x14     // Accelerometer Configuration register.
#define ACCEL_CONFIG_2       0x15     // Accelerometer Configuration 2 register (digital low-pass filter settings).
#define ACCEL_SMPLRT_DIV_1   0x10     //High byte of accelerometer sample rate divider
#define ACCEL_SMPLRT_DIV_2   0x11     //Low byte of accelerometer sample rate divider

// Register Bank Selection
#define REG_BANK_SEL          0x7F  // Register to select banks


typedef struct{
	//I2C handle
	I2C_HandleTypeDef *i2cHandle;

	//Acceleration data in mps2 (x,y,z)
	float acc_mps2[3];
}ICM;

//Initialization
uint8_t ICM_Initialise( ICM *dev,I2C_HandleTypeDef *i2cHandle); //will return 0 if no errors
																//Set all the configuration for the sensor


//Data acquisition

HAL_StatusTypeDef ICM_ReadAcceleration(ICM *dev);



//Low level functions

HAL_StatusTypeDef ICM_ReadRegister(ICM *dev,uint8_t reg, uint8_t *data); //read from one register
HAL_StatusTypeDef ICM_ReadRegisters(ICM *dev,uint8_t reg, uint8_t *data, uint8_t length); //read from multiple registers

HAL_StatusTypeDef ICM_WriteRegister(ICM *dev,uint8_t reg, uint8_t *data); //write to a register

HAL_StatusTypeDef ICM_SelectBank(ICM *dev, uint8_t data); //select bank before W/R registers

#endif /* INC_ICM20948_H_ */
