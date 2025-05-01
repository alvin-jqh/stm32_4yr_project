/*
 * PMW3610DM-SUDU.h
 *
 *  Created on: Mar 18, 2025
 *      Author: User
 */

#ifndef INC_PMW3610DM_H_
#define INC_PMW3610DM_H_


#include "stm32wbxx_hal.h"

// PMW3610DM SPI commands
#define PMW3610DM_READ           0x00
#define PMW3610DM_WRITE          0x80
#define PMW3610DM_MOTION_BURST   0x12

// PMW3610DM Registers
#define REG_PRODUCT_ID           0x00
#define REG_REVISION_ID          0x01
#define REG_MOTION               0x02
#define REG_DELTA_X_L            0x03
#define REG_DELTA_Y_L            0x04
#define REG_DELTA_XY_H           0x05
#define REG_SQUAL                0x06
#define REG_SHUTTER_HIGHER       0x07
#define REG_SHUTTER_LOWER        0x08
#define REG_PERFORMANCE          0x11
#define REG_SPI_CLK_ON_REQ       0x41
#define REG_RES_STEP             0x85

typedef struct {
    int16_t deltaX;
    int16_t deltaY;
    uint8_t squal;
    uint16_t shutter;
} PMW3610DM_MotionData;

// Public API
void PMW3610DM_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_Port, uint16_t CS_Pin);
void PMW3610DM_ReadMotion(PMW3610DM_MotionData *data);
void PMW3610DM_CS_Select();
void PMW3610DM_CS_Deselect();
void PMW3610DM_WriteReg(uint8_t reg, uint8_t value);
uint8_t PMW3610DM_ReadReg(uint8_t reg);


#endif /* INC_PMW3610DM_H_ */
