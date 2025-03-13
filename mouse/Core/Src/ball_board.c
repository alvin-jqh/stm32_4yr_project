/*
 * ball_board.c
 *
 *  Created on: Feb 25, 2025
 *      Author: Alvin
 */

#include "ball_board.h"

void ball_reset(ball *trackball){
  // function that resets all the directional counts
  trackball->UP = 0;
  trackball->DOWN = 0;
  trackball->LEFT = 0;
  trackball->RIGHT = 0;
}
