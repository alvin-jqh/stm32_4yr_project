/*
 * ball_board.h
 *
 *  Created on: Feb 25, 2025
 *      Author: Alvin
 */

#ifndef INC_BALL_BOARD_H_
#define INC_BALL_BOARD_H_

#include "stm32wbxx_hal.h"

typedef struct{
  uint16_t UP;
  uint16_t DOWN;
  uint16_t LEFT;
  uint16_t RIGHT;
}ball;

void ball_reset(ball *trackball);

#endif /* INC_BALL_BOARD_H_ */
