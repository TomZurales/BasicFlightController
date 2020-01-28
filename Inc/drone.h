/*
 * drone.h
 *
 *  Created on: Jan 13, 2020
 *      Author: root
 */

#ifndef DRONE_H_
#define DRONE_H_

#include <stdint.h>
#include "MY_LIS3DSH.h"
#include "stm32f4xx_hal_tim.h"

// Pitch axis index
#define PITCH 0

// Roll axis index
#define ROLL 1

// Throttle value for 1ms pulse
#define MIN_THROTTLE 183

// Throttle value for 2ms pulse
#define MAX_THROTTLE 366

// Minimum value from accelerometer
#define MIN_ACCEL -1000

// Maximum value from accelerometer
#define MAX_ACCEL 1000

// PID parameters
struct {
  int16_t xGoal;
  int16_t yGoal;
  float pGain;
  float iGain;
  float dGain;
} params;

// axis offsets between goal and accelerometer value
struct {
  int16_t x;
  int16_t y;
} errorData;

// Sum of axis error values
struct {
  int16_t x;
  int16_t y;
} integralData;

// Change in axis error values
struct {
  int16_t x;
  int16_t y;
} derivativeData;

// Calculated PID values for each axis
struct {
  int16_t xp;
  int16_t xi;
  int16_t xd;
  int16_t yp;
  int16_t yi;
  int16_t yd;
} controller;

void set_accel_data(LIS3DSH_DataScaled);
void calculate_proportion(void);
void calculate_integral(void);
void calculate_derivative(void);
void set_motors(TIM_HandleTypeDef * timer);

#endif /* DRONE_H_ */
