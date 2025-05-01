/*
 * PMW3610DM-SUDU.c
 *
 *  Created on: Mar 18, 2025
 *      Author: User
 */


#include "PMW3610DM-SUDU.h"

static SPI_HandleTypeDef *spi;
static GPIO_TypeDef *cs_port;
static uint16_t cs_pin;

 void PMW3610DM_CS_Select(void) {
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
}

 void PMW3610DM_CS_Deselect(void) {
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

 void PMW3610DM_WriteReg(uint8_t reg, uint8_t value) {
    uint8_t data[2] = { reg | PMW3610DM_WRITE, value }; //MSB needs to be 1 for write
    PMW3610DM_CS_Select();
    HAL_SPI_Transmit(spi, data, 2, HAL_MAX_DELAY);
    PMW3610DM_CS_Deselect();
}

 uint8_t PMW3610DM_ReadReg(uint8_t reg) {
    uint8_t tx = reg & 0x7F;
    uint8_t rx;
    PMW3610DM_CS_Select();
    HAL_SPI_Transmit(spi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi, &rx, 1, HAL_MAX_DELAY);
    PMW3610DM_CS_Deselect();
    return rx;
}

void PMW3610DM_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_Port, uint16_t CS_Pin) {
    spi = hspi;
    cs_port = CS_Port;
    cs_pin = CS_Pin;

    PMW3610DM_CS_Select();
    HAL_Delay(10);

    PMW3610DM_WriteReg(REG_SPI_CLK_ON_REQ, 0xBA); // Enable SPI clock
    HAL_Delay(1);

    // Optional sensor settings, adjust CPI resolution etc.
    PMW3610DM_WriteReg(REG_RES_STEP, 0x06); // Default to 1200 CPI

    PMW3610DM_WriteReg(REG_SPI_CLK_ON_REQ, 0xB5); // Disable SPI clock to save power
    PMW3610DM_CS_Deselect();
}

void PMW3610DM_ReadMotion(PMW3610DM_MotionData *data) {
    uint8_t burstData[6];

    PMW3610DM_CS_Select();
    uint8_t tx = PMW3610DM_MOTION_BURST;
    HAL_SPI_Transmit(spi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi, burstData, 6, HAL_MAX_DELAY);
    PMW3610DM_CS_Deselect();

    data->deltaX = (int16_t)((burstData[3] & 0xF0) << 4 | burstData[1]);
    if (data->deltaX & 0x800) data->deltaX |= 0xF000; // Sign extend 12-bit

    data->deltaY = (int16_t)((burstData[3] & 0x0F) << 8 | burstData[2]);
    if (data->deltaY & 0x800) data->deltaY |= 0xF000; // Sign extend 12-bit

    data->squal = burstData[4];
    data->shutter = ((uint16_t)burstData[5]) << 8 | burstData[0];
}
