/*
 * ICM20948.c
 *
 *  Created on: Nov 21, 2024
 *      Author: Alvin
 */

#include "icm20948.h"
#include <stdio.h>
#include <string.h>

uint8_t bank;


uint8_t ICM_Initialise( ICM *dev,I2C_HandleTypeDef *i2cHandle){

	dev->i2cHandle=i2cHandle;
	dev->acc_mps2[0]=0.0f;
	dev->acc_mps2[1]=0.0f;
	dev->acc_mps2[2]=0.0f;


	//store number of errors
	uint8_t errNum =0;
	HAL_StatusTypeDef status; //will check the status of the method
	HAL_StatusTypeDef status1;
	HAL_StatusTypeDef status2;

	//Check Id of the device
	uint8_t check;

	uint8_t regData;


	uint8_t ID;
	uint8_t krono[9];

	bank=0;//select bank
	status=ICM_SelectBank(dev, bank);
	errNum= errNum + (status!=HAL_OK); // add error number if there is error


	status= ICM_ReadRegister(dev, WHO_AM_I, &ID);
	errNum= errNum + (status!=HAL_OK); // add error number if there is error

	if (ID!=Device_ID){

		return 255; //Device Id doesn't match
	}



	//Configuring the sensor
	//SELECT BANK0

	status=ICM_SelectBank(dev, bank);
	errNum= errNum + (status!=HAL_OK); // add error number if there is error
	if (status!=HAL_OK){
		check=1;
		krono[check]=1;
	}

	// Power Management 1 register.
 	regData=0x81;  //Reset registers,off_sleep,off_lowpower,Enable temp sensor,use best clock source
	status= ICM_WriteRegister(dev, PWR_MGMT_1, &regData);
	errNum= errNum + (status!=HAL_OK); // add error number if there is error
	if (status!=HAL_OK){
		check=2;
		krono[check]=1;
	}

	// Power Management 2 register.
	regData=0x00;  //Enable acc and gyro on all axes
	status= ICM_WriteRegister(dev, PWR_MGMT_2, &regData);
	errNum= errNum + (status!=HAL_OK); // add error number if there is error
	if (status!=HAL_OK){
		check=3;
		krono[check]=1;
		status1=status;
	}




	//SELECT BANK2
	bank=2;
	status= ICM_SelectBank(dev, bank);
	errNum= errNum + (status!=HAL_OK); // add error number if there is error
	if (status!=HAL_OK){
		check=4;
		krono[check]=1;
	}

	// Accelerometer Configuration register.
	regData=0x31;  //Enable low pass filter with 5.7Hz cutt-off for low noise
	status= ICM_WriteRegister(dev, ACCEL_CONFIG, &regData);
	errNum= errNum + (status!=HAL_OK); // add error number if there is error
	if (status!=HAL_OK){
		check=5;
		krono[check]=1;
		status2=status;


	}

	// Accelerometer Configuration 2 register
	regData=0x02;  //Enable averaging samples every 16 samples
	status= ICM_WriteRegister(dev, ACCEL_CONFIG_2, &regData);
	errNum= errNum + (status!=HAL_OK); // add error number if there is error
	if (status!=HAL_OK){
		check=6;
		krono[check]=1;
	}

	//High byte of accelerometer sample rate divider
	regData=0x00;  //[10:8] MSB of sample rate divider to get 127
	status= ICM_WriteRegister(dev, ACCEL_SMPLRT_DIV_1, &regData);
	errNum= errNum + (status!=HAL_OK); // add error number if there is error
	if (status!=HAL_OK){
		check=7;
		krono[check]=1;
	}

	//Low byte of accelerometer sample rate divider
	regData=0x7F;  //[7:0] LSB of sample rate divider to get 127
	status= ICM_WriteRegister(dev, ACCEL_SMPLRT_DIV_2, &regData);
	errNum= errNum + (status!=HAL_OK); // add error number if there is error
	if (status!=HAL_OK){
		check=8;
		krono[check]=1;
	}


	return errNum;
}


//Data acquisition

HAL_StatusTypeDef ICM_ReadAcceleration(ICM *dev){
	uint8_t errNum =0;
	uint8_t regData[6];
	HAL_StatusTypeDef status; //will check the status of the method

	//Read raw acc and combine into bytes
	uint16_t tempRaw[3];
	bank=0;//select bank
	status=ICM_SelectBank(dev, bank);
	errNum= errNum + (status!=HAL_OK); // add error number if there is error
	status=ICM_ReadRegisters(dev, ACCEL_XOUT_H, regData, 6); //read register for H and L byte
	errNum= errNum + (status!=HAL_OK); // add error number if there is error
	tempRaw[0]= ( (regData[0]<<8) | regData[1] ); //combining the H and L bytes off raw acc X
	tempRaw[1]= ( (regData[2]<<8) | regData[3] ); //combining the H and L bytes off raw acc Y
	tempRaw[2]= ( (regData[4]<<8) | regData[5] ); //combining the H and L bytes off raw acc Z

	//Convert raw to measurement
	dev->acc_mps2[0]=  (float)tempRaw[0] / 16384.0f;  // acc_measurement= raw_acc/ acc_sensitivity //2g=16,384 4g= 8,192
	dev->acc_mps2[1]=  (float)tempRaw[1] / 16384.0f;
	dev->acc_mps2[2]=  (float)tempRaw[2] / 16384.0f;

	return status;
}


//Low level functions

HAL_StatusTypeDef ICM_ReadRegister(ICM *dev,uint8_t reg, uint8_t *data){  //read from one register
	HAL_Delay(1);
	return HAL_I2C_Mem_Read(dev->i2cHandle, ICM20948_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1,HAL_MAX_DELAY);
};
HAL_StatusTypeDef ICM_ReadRegisters(ICM *dev,uint8_t reg, uint8_t *data, uint8_t length){ //read from multiple registers
	HAL_Delay(1);
	return HAL_I2C_Mem_Read(dev->i2cHandle, ICM20948_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length,HAL_MAX_DELAY);
}

HAL_StatusTypeDef ICM_WriteRegister(ICM *dev,uint8_t reg, uint8_t *data){ //write to a register
	HAL_Delay(1);
	return HAL_I2C_Mem_Write(dev->i2cHandle, ICM20948_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ICM_SelectBank(ICM *dev, uint8_t data){ //accept inputs from 0-3
	HAL_Delay(1);

    if (data > 3) {
        return HAL_ERROR; // Invalid bank
    }
    uint8_t shifted_data = (data) << 4; //shift by 4 because bank address is [5:4]
	return HAL_I2C_Mem_Write(dev->i2cHandle, ICM20948_ADDR, REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT, &shifted_data, 1, HAL_MAX_DELAY);
}

