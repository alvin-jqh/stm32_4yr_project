/*
 * bno055.c
 *
 *  Created on: Feb 1, 2025
 *      Author: User
 */

#include "bno055.h"


uint8_t GPwrMode 	= NormalG;   		// Gyro power mode
uint8_t Gscale 		= GFS_2000DPS; 	// Gyro full scale
//uint8_t Godr	 	= GODR_250Hz;  	// Gyro sample rate
uint8_t Gbw 			= GBW_230Hz;    // Gyro bandwidth
//
uint8_t Ascale 		= AFS_16G;      // Accel full scale
uint8_t APwrMode 	= NormalA;   		// Accel power mode
uint8_t Abw 		= ABW_250Hz;    // Accel bandwidth, accel sample rate divided by ABW_divx
//
//uint8_t Mscale 	= MFS_4Gauss;		// Select magnetometer full-scale resolution
uint8_t MOpMode 	= EnhancedRegular;    	// Select magnetometer perfomance mode
uint8_t MPwrMode 	= Normal;    		// Select magnetometer power mode
uint8_t Modr 		= MODR_30Hz;    // Select magnetometer ODR when in BNO055 bypass mode

uint8_t PWRMode 	= Normalpwr;  	// Select BNO055 power mode
uint8_t OPRMode 	= NDOF;    	// specify operation mode for sensors [ACCONLY|MAGONLY|GYROONLY|ACCMAG|ACCGYRO|MAGGYRO|AMG|NDOF|NDOF_FMC_OFF]

uint8_t status;               // BNO055 data status register
float aRes, gRes, mRes; 			// scale resolutions per LSB for the sensors

// IMU calibration variables
uint8_t cal_sys 	= 0;
uint8_t cal_gyro 	= 0;
uint8_t cal_acc 	= 0;
uint8_t cal_mag 	= 0;
uint8_t cal_imu 	= 0;


//Initialization
uint8_t BNO_INIT( BNO *dev,I2C_HandleTypeDef *i2cHandle){


	dev->i2cHandle=i2cHandle;
	dev->acc_mps2[0]=0.0f;
	dev->acc_mps2[1]=0.0f;
	dev->acc_mps2[2]=0.0f;


	//store number of errors
	uint8_t errNum =0;
	HAL_StatusTypeDef status; //will check the status of the method

	//Check Id of the device

	uint8_t regData;
	//Check chip ID
	status= BNO_ReadRegister(dev, BNO055_CHIP_ID, &regData);
	errNum= errNum + (status!=HAL_OK); // add error number if there is error
	if (regData!=CHIP_ID){

		return 255; //Device Id doesn't match
	}

	//ACC ID
	status= BNO_ReadRegister(dev, BNO055_ACC_ID, &regData);
	errNum= errNum + (status!=HAL_OK); // add error number if there is error
	if (regData!=ACC_ID){

		return 255; //Device Id doesn't match
	}

	//MAG ID
	status= BNO_ReadRegister(dev, BNO055_MAG_ID, &regData);
	errNum= errNum + (status!=HAL_OK); // add error number if there is error
	if (regData!=MAG_ID){

		return 255; //Device Id doesn't match
	}

	//GYR ID
	status= BNO_ReadRegister(dev, BNO055_GYRO_ID, &regData);
	errNum= errNum + (status!=HAL_OK); // add error number if there is error
	if (regData!=GYR_ID){

		return 255; //Device Id doesn't match
	}

	// Select BNO055 config mode
	uint8_t opr_config_mode=CONFIGMODE;
	BNO_WriteRegister(dev, BNO055_OPR_MODE, &opr_config_mode);


	// Select page 1 to configure sensors
	uint8_t conf_page1=0x01;
	BNO_WriteRegister(dev, BNO055_PAGE_ID, &conf_page1);


	// Configure ACC (Page 1; 0x08)
	uint8_t conf_acc=APwrMode << 5 | Abw << 2 | Ascale;
	BNO_WriteRegister(dev, BNO055_ACC_CONFIG, &conf_acc);


	// Configure GYR
	uint8_t conf_gyro=Gbw << 3 | Gscale;
	BNO_WriteRegister(dev, BNO055_GYRO_CONFIG_0, &conf_gyro);


	uint8_t conf_gyro_pwr = GPwrMode;
	BNO_WriteRegister(dev, BNO055_GYRO_CONFIG_1, &conf_gyro_pwr);


//	// Configure MAG
	uint8_t conf_mag_pwr =  Modr | MPwrMode << 5 | MOpMode << 3 | Modr;
	BNO_WriteRegister(dev, BNO055_MAG_CONFIG, &conf_mag_pwr);



	// Select page 0
	uint8_t conf_page0 =  0x00;
	BNO_WriteRegister(dev, BNO055_PAGE_ID, &conf_page0);


	// Select BNO055 sensor units (Page 0; 0x3B, default value 0x80)
	/*- ORIENTATION_MODE		 - Android					(default)
		- VECTOR_ACCELEROMETER - m/s^2  					(default)
		- VECTOR_MAGNETOMETER  - uT							(default)
		- VECTOR_GYROSCOPE     - rad/s        v				(must be configured)
		- VECTOR_EULER         - degrees					(default)
		- VECTOR_LINEARACCEL   - m/s^2        v				(default)
		- VECTOR_GRAVITY       - m/s^2						(default)
	*/

	// Select BNO055 system power mode (Page 0; 0x3E)
	uint8_t pwr_pwrmode = PWRMode;
	BNO_WriteRegister(dev, BNO055_PWR_MODE, &pwr_pwrmode);


	// Select BNO055 system operation mode (Page 0; 0x3D)
	uint8_t opr_oprmode =  OPRMode;
	BNO_WriteRegister(dev, BNO055_OPR_MODE, &opr_oprmode);


return errNum;

}

//Data acquisition
HAL_StatusTypeDef BNO_ReadAcceleration(BNO *dev){
	uint8_t regData[6];
	HAL_StatusTypeDef status; //will check the status of the method

	//Read raw acc and combine into bytes
	int16_t tempRaw[3];
	status=BNO_ReadRegisters(dev, BNO055_ACC_DATA_X_LSB, regData, 6); //read register for H and L byte
	tempRaw[0]= ( (regData[1]<<8) | regData[0] ); //combining the H and L bytes off raw acc X
	tempRaw[1]= ( (regData[3]<<8) | regData[2] ); //combining the H and L bytes off raw acc Y
	tempRaw[2]= ( (regData[5]<<8) | regData[4] ); //combining the H and L bytes off raw acc Z

	//Convert raw to measurement
	dev->acc_mps2[0]=  (float)tempRaw[0] / 100.0f;  // 1 m/s2 = 100LSB
	dev->acc_mps2[1]=  (float)tempRaw[1] / 100.0f;  // 1 mg = 1LSB
	dev->acc_mps2[2]=  (float)tempRaw[2] / 100.0f;

	return status;
}

HAL_StatusTypeDef BNO_ReadGyro(BNO *dev){
	uint8_t regData[6];

	HAL_StatusTypeDef status; //will check the status of the method

	//Read raw gyro and combine into bytes
	int16_t tempRaw[3];
	status=BNO_ReadRegisters(dev, BNO055_GYR_DATA_X_LSB, regData, 6); //read register for H and L byte
	tempRaw[0]= ( (regData[1]<<8) | regData[0] ); //combining the H and L bytes off raw acc X
	tempRaw[1]= ( (regData[3]<<8) | regData[2] ); //combining the H and L bytes off raw acc Y
	tempRaw[2]= ( (regData[5]<<8) | regData[4] ); //combining the H and L bytes off raw acc Z

	//Convert raw to measurement
	dev->gyro_dps[0]=  (float)tempRaw[0] / 16.0f;  // 1 Dps = 16LSB
	dev->gyro_dps[1]=  (float)tempRaw[1] / 16.0f;  // 1 Rps = 900LSB
	dev->gyro_dps[2]=  (float)tempRaw[2] / 16.0f;

	return status;
}

HAL_StatusTypeDef BNO_ReadMag(BNO *dev){
	uint8_t regData[6];

	HAL_StatusTypeDef status; //will check the status of the method

	//Read raw mag and combine into bytes
	int16_t tempRaw[3];
	status=BNO_ReadRegisters(dev, BNO055_MAG_DATA_X_LSB, regData, 6); //read register for H and L byte
	tempRaw[0]= ( (regData[1]<<8) | regData[0] ); //combining the H and L bytes off raw acc X
	tempRaw[1]= ( (regData[3]<<8) | regData[2] ); //combining the H and L bytes off raw acc Y
	tempRaw[2]= ( (regData[5]<<8) | regData[4] ); //combining the H and L bytes off raw acc Z

	//Convert raw to measurement
	dev->mag_uT[0]=  (float)tempRaw[0] / 16.0f;  // 1 uT = 16LSB
	dev->mag_uT[1]=  (float)tempRaw[1] / 16.0f;
	dev->mag_uT[2]=  (float)tempRaw[2] / 16.0f;

	return status;
}

HAL_StatusTypeDef BNO_ReadEul(BNO *dev){
	uint8_t regData[6];

	HAL_StatusTypeDef status; //will check the status of the method

	//Read raw orientation eul and combine into bytes
	int16_t tempRaw[3];
	status=BNO_ReadRegisters(dev, BNO055_EUL_HEADING_LSB, regData, 6); //read register for H and L byte
	tempRaw[0]= ( (regData[1]<<8) | regData[0] ); //Heading(Yaw)
	tempRaw[1]= ( (regData[3]<<8) | regData[2] ); //Roll
	tempRaw[2]= ( (regData[5]<<8) | regData[4] ); //Pitch

	//Convert raw to measurement
	dev->eul_deg[0]=  (float)tempRaw[0] / 16.0f;  // 1 degree = 16LSB
	dev->eul_deg[1]=  (float)tempRaw[1] / 16.0f;
	dev->eul_deg[2]=  (float)tempRaw[2] / 16.0f;

	return status;
}

HAL_StatusTypeDef BNO_ReadQuart(BNO *dev){
	uint8_t regData[8];

	HAL_StatusTypeDef status; //will check the status of the method

	//Read raw orientation eul and combine into bytes
	int16_t tempRaw[3];
	status=BNO_ReadRegisters(dev, BNO055_QUA_DATA_W_LSB, regData, 8); //read register for H and L byte
	tempRaw[0]= ( (regData[1]<<8) | regData[0] ); //W
	tempRaw[1]= ( (regData[3]<<8) | regData[2] ); //X
	tempRaw[2]= ( (regData[5]<<8) | regData[4] ); //Y
	tempRaw[3]= ( (regData[7]<<8) | regData[6] ); //Y

	//Convert raw to measurement
	dev->quart[0]=  (float)tempRaw[0] / 16384.0f;  // 1 quaternion = 2^14LSB (
	dev->quart[1]=  (float)tempRaw[1] / 16384.0f;
	dev->quart[2]=  (float)tempRaw[2] / 16384.0f;
	dev->quart[3]=  (float)tempRaw[3] / 16384.0f;

	return status;
}

HAL_StatusTypeDef BNO_ReadCal(BNO *dev){
	uint8_t regData[1];

	HAL_StatusTypeDef status; //will check the status of the method

	//Read raw orientation eul and combine into bytes
	status=BNO_ReadRegister(dev, BNO055_CALIB_STAT, regData); //read register for H and L byte


	//Convert raw to measurement
	dev->cal_stat[0] = (uint8_t)((regData[0] & 0b11000000) >> 6);  // SYS (bits 7-6)
	dev->cal_stat[1] = (uint8_t)((regData[0] & 0b00110000) >> 4);  // GYR (bits 5-4)
	dev->cal_stat[2] = (uint8_t)((regData[0] & 0b00001100) >> 2);  // ACC (bits 3-2)
	dev->cal_stat[3] = (uint8_t)(regData[0]  & 0b00000011);        // MAG (bits 1-0)
	dev->cal_stat[4] = (uint8_t)regData[0];
	return status;
}

Quaternion quaternion_multiply(const Quaternion *q1, const Quaternion *q2) {
    Quaternion result;

    result.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;

    return result;
}

Quaternion quaternion_conjugate(const Quaternion *q) {
    Quaternion result;
    result.w = q->w;
    result.x = -q->x;
    result.y = -q->y;
    result.z = -q->z;
    return result;
}

//Low level functions

HAL_StatusTypeDef BNO_ReadRegister(BNO *dev,uint8_t reg, uint8_t *data){  //read from one register

	return HAL_I2C_Mem_Read(dev->i2cHandle, BNO055_I2C_ADDR_LO, reg, I2C_MEMADD_SIZE_8BIT, data, 1,HAL_MAX_DELAY);
}
HAL_StatusTypeDef BNO_ReadRegisters(BNO *dev,uint8_t reg, uint8_t *data, uint8_t length){ //read from multiple registers

	return HAL_I2C_Mem_Read(dev->i2cHandle, BNO055_I2C_ADDR_LO, reg, I2C_MEMADD_SIZE_8BIT, data, length,HAL_MAX_DELAY);
}

HAL_StatusTypeDef BNO_WriteRegister(BNO *dev,uint8_t reg, uint8_t *data){ //write to a register

	return HAL_I2C_Mem_Write(dev->i2cHandle, BNO055_I2C_ADDR_LO, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
HAL_StatusTypeDef BNO_WriteRegisters(BNO *dev,uint8_t reg, uint8_t *data, uint8_t length){ //write to a registers
	return HAL_I2C_Mem_Write(dev->i2cHandle, BNO055_I2C_ADDR_LO, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

