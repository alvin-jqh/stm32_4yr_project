/*
 * ball.h
 *
 *  Created on: Nov 16, 2024
 *      Author: Alvin
 */

#ifndef INC_BALL_H_
#define INC_BALL_H_

#include "stm32wbxx_hal.h"	// needed for I2C

#define I2C_ADDRESS (0x0A << 1)	// address of the trackball
#define CHIP_ID 0xBA11	// chip ID of the trackball

// REGISTERS
#define TRACKBALL_REG_LED_RED 0x00
#define TRACKBALL_REG_LED_GREEN 0x01
#define TRACKBALL_REG_LED_BLUE 0x02
#define TRACKBALL_REG_LED_WHITE 0x03

#define TRACKBALL_REG_LEFT 0x04
#define TRACKBALL_REG_RIGHT 0x05
#define TRACKBALL_REG_UP 0x06
#define TRACKBALL_REG_DOWN 0x07
#define TRACKBALL_REG_SWITCH 0x08

#define TRACKBALL_REG_USER_FLASH 0xD0
#define TRACKBALL_REG_FLASH_PAGE 0xF0
#define TRACKBALL_REG_INT 0xF9

// CHIP ID STORED IN 2 REGISTERS
#define TRACKBALL_REG_CHIP_ID_L 0xFA
#define TRACKBALL_RED_CHIP_ID_H 0xFB

#define TRACKBALL_REG_VERSION 0xFC
#define TRACKBALL_REG_I2C_ADDR 0xFD
#define TRACKBALL_REG_CTRL 0xFE

// MASKS
#define TRACKBALL_MSK_INT_TRIGGERED 0b00000001
#define TRACKBALL_MSK_INT_OUT_EN 0b00000010

#define TRACKBALL_MSK_CTRL_SLEEP 0b00000001
#define TRACKBALL_MSK_CTRL_RESET 0b00000010
#define TRACKBALL_MSK_CTRL_FREAD 0b00000100
#define TRACKBALL_MSK_CTRL_FWRITE 0b00001000
#define TRACKBALL_MSK_SWITCH_STATE 0b10000000

// defining boolean
typedef uint8_t bool;
#define true 1
#define false 0

// struct including the data for the trackball
typedef struct{
	I2C_HandleTypeDef *i2cHandle;

	uint8_t left;
	uint8_t right;
	uint8_t up;
	uint8_t down;
	bool sw_changed;
	bool sw_pressed;
}trackball;

// intialisation
uint8_t trackball_init(trackball *dev, I2C_HandleTypeDef *i2cHandle);

// setting LEDs
HAL_StatusTypeDef  setRed(trackball *dev, uint8_t redBrightness);
HAL_StatusTypeDef  setGreen(trackball *dev, uint8_t greenBrightness);
HAL_StatusTypeDef  setBlue(trackball *dev, uint8_t blueBrightness);
HAL_StatusTypeDef  setWhite(trackball *dev, uint8_t whiteBrightness);
uint8_t setRGBW(trackball *dev, uint8_t r, uint8_t g, uint8_t b, uint8_t w);

// reading data
HAL_StatusTypeDef trackball_read_states(trackball *dev);

// low level functions
HAL_StatusTypeDef trackball_read_register(trackball *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef trackball_read_registers(trackball *dev, uint8_t reg, uint8_t *data, uint8_t length);

HAL_StatusTypeDef trackball_write_register(trackball *dev, uint8_t reg, uint8_t *data);


#endif /* INC_BALL_H_ */
