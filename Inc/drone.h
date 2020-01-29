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

// Throttle value for 1ms pulse
#define MIN_THROTTLE 183

// Throttle value for 2ms pulse
#define MAX_THROTTLE 366

// Minimum value from accelerometer
#define MIN_ACCEL -1000

// Maximum value from accelerometer
#define MAX_ACCEL 1000

typedef struct {
  uint8_t p_gain;
  uint8_t i_gain;
  uint8_t d_gain;
} DroneInitStruct;

struct Axes {
  int16_t ROLL;
  int16_t PITCH;
};

struct {
  uint8_t P;
  uint8_t I;
  uint8_t D;
} control_gains;

// Goal parameters
struct Axes goal;

// Accelerometer offsets
struct Axes sensor;

// Difference between goal and sensor values
struct Axes err;

// Sum of axis error values
struct Axes err_sum;

// Change in axis error values
struct Axes err_change;

// Previously read error
struct Axes err_prev;

void drone_init(DroneInitStruct*);
void calculate_PID(LIS3DSH_DataScaled);
void set_motors(TIM_HandleTypeDef * timer);

#endif /* DRONE_H_ */
