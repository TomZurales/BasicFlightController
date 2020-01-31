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

// Axes Indexes
#define ROLL 0
#define PITCH 1

// Filter constants
#define FILTER_SIZE 10
#define FILTER_MODE_NONE 0
#define FILTER_MODE_AVERAGE 1

typedef struct {
  float p_gain;
  float i_gain;
  float d_gain;
  uint8_t filter_mode;
} DroneInitStruct;

typedef struct {
  float roll_goal;
  float pitch_goal;
  float throttle;
} DroneInputStruct;

struct Axes {
  float roll;
  float pitch;
};

struct {
  float p;
  float i;
  float d;
} control_gains;

struct {
  uint8_t filter_mode;
} control_params;

struct {
  float roll[FILTER_SIZE];
  float pitch[FILTER_SIZE];
} filter;

// Goal parameters
struct{
  float roll;
  float pitch;
  float throttle;
} goal;

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

void set_input(DroneInputStruct*);

void set_motors(TIM_HandleTypeDef * timer);

float get_filtered_value(uint8_t axis, float new_value);

#endif /* DRONE_H_ */
