#include "ball.h"

HAL_StatusTypeDef trackball_read_register(trackball *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Read(dev->i2cHandle, I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
HAL_StatusTypeDef trackball_read_registers(trackball *dev, uint8_t reg, uint8_t *data, uint8_t length){

	return HAL_I2C_Mem_Read(dev->i2cHandle, I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef trackball_write_register(trackball *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Write(dev->i2cHandle, I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

uint8_t trackball_init(trackball *dev, I2C_HandleTypeDef *i2cHandle){
	// initalise parameters for the struct
	dev->i2cHandle = i2cHandle;

	dev->left = 0;
	dev->right = 0;
	dev->up = 0;
	dev->down = 0;

	dev->sw_pressed = 0;
	dev->sw_changed = 0;

	// checking for errors
	uint8_t errorNum = 0;
	HAL_StatusTypeDef status;

	uint8_t regData[2];
	uint16_t chip_id = 0;

	status = trackball_read_registers(dev, TRACKBALL_REG_CHIP_ID_L , regData, 2);
	errorNum += (status != HAL_OK);

	chip_id = ( regData[1] << 8 | regData[0]);

	if (chip_id != CHIP_ID){
		return 255;
	}

	return errorNum;
}

HAL_StatusTypeDef trackball_read_states(trackball *dev){
	uint8_t sw_state;
	uint8_t regData[5];

	HAL_StatusTypeDef status = trackball_read_registers(dev, TRACKBALL_REG_LEFT, regData, 5);

	dev->left = regData[0];
	dev->right = regData[1];
	dev->up = regData[2];
	dev->down = regData[3];

	sw_state = regData[4];

	dev->sw_changed = sw_state & ~TRACKBALL_MSK_SWITCH_STATE;
	dev->sw_pressed = (sw_state & TRACKBALL_MSK_SWITCH_STATE) > 0;

	return status;
}

HAL_StatusTypeDef setRed(trackball *dev, uint8_t redBrightness){
	return trackball_write_register(dev, TRACKBALL_REG_LED_RED, &redBrightness);
}

HAL_StatusTypeDef  setGreen(trackball *dev, uint8_t greenBrightness){
	return trackball_write_register(dev, TRACKBALL_REG_LED_GREEN, &greenBrightness);
}

HAL_StatusTypeDef  setBlue(trackball *dev, uint8_t blueBrightness){
	return trackball_write_register(dev, TRACKBALL_REG_LED_BLUE, &blueBrightness);
}

HAL_StatusTypeDef  setWhite(trackball *dev, uint8_t whiteBrightness){
	return trackball_write_register(dev, TRACKBALL_REG_LED_WHITE, &whiteBrightness);
}

uint8_t setRGBW(trackball *dev, uint8_t r, uint8_t g, uint8_t b, uint8_t w){
	uint8_t errorNum = 0;
	HAL_StatusTypeDef status;

	status = setRed(dev, r);
	errorNum += (status != HAL_OK);

	status = setGreen(dev, g);
	errorNum += (status != HAL_OK);

	status = setBlue(dev, b);
	errorNum += (status != HAL_OK);

	status = setWhite(dev, w);
	errorNum += (status != HAL_OK);

	return errorNum;
}

