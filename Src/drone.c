/*
 * drone.c
 *
 *  Created on: Jan 13, 2020
 *      Author: root
 */

#include "drone.h"
#include "utils.h"

void drone_init(DroneInitStruct * init){
  control_gains.p = init->p_gain;
  control_gains.i = init->i_gain;
  control_gains.d = init->d_gain;

  control_params.filter_mode = init->filter_mode;
}

void calculate_PID(LIS3DSH_DataScaled accelData){
  sensor.roll = get_filtered_value(ROLL, accelData.x);
  sensor.pitch = get_filtered_value(PITCH, accelData.y);

  //TODO: Verify trimming used correctly here
  err.pitch = trim(goal.pitch - sensor.pitch, MIN_ACCEL, MAX_ACCEL);
  err.roll = trim(goal.roll - sensor.roll, MIN_ACCEL, MAX_ACCEL);

  err_sum.pitch = trim(err_sum.pitch + err.pitch * control_gains.i, 0, MAX_ACCEL);
  err_sum.roll = trim(err_sum.roll + err.roll * control_gains.i, 0, MAX_ACCEL);

  err_change.pitch = trim(err.pitch - err_prev.pitch, MIN_ACCEL, MAX_ACCEL);
  err_change.roll = trim(err.roll - err_prev.roll, MIN_ACCEL, MAX_ACCEL);
  err_prev.pitch = err.pitch;
  err_prev.roll = err.roll;
}

void set_motors(TIM_HandleTypeDef * timer){

//    A
//    |
//  B---C
//    |
//    D

  int16_t a_setpoint = trim((-err.pitch * control_gains.p) + (-err_sum.pitch) + (-err_change.pitch * control_gains.d) + goal.throttle, 0, 3 * MAX_ACCEL);
  timer->Instance->CCR3 = map(a_setpoint, 0, 3 * MAX_ACCEL, MIN_THROTTLE, MAX_THROTTLE);
  int16_t b_setpoint = trim((err.roll * control_gains.p) + (err_sum.roll) + (err_change.roll * control_gains.d) + goal.throttle, 0, 3 * MAX_ACCEL);
  timer->Instance->CCR2 = map(b_setpoint, 0, 3 * MAX_ACCEL, MIN_THROTTLE, MAX_THROTTLE);
  int16_t c_setpoint = trim((-err.roll * control_gains.p) + (-err_sum.roll) + (-err_change.roll * control_gains.d) + goal.throttle, 0, 3 * MAX_ACCEL);
  timer->Instance->CCR1 = map(c_setpoint, 0, 3 * MAX_ACCEL, MIN_THROTTLE, MAX_THROTTLE);
  int16_t d_setpoint = trim((err.pitch * control_gains.p) + (err_sum.pitch) + (err_change.pitch * control_gains.d) + goal.throttle, 0, 3 * MAX_ACCEL);
  timer->Instance->CCR4 = map(d_setpoint, 0, 3 * MAX_ACCEL, MIN_THROTTLE, MAX_THROTTLE);
}

void set_input(DroneInputStruct* input){
  goal.roll = input-> roll_goal;
  goal.pitch = input->pitch_goal;
  goal.throttle = input->throttle;
}

float get_filtered_value(uint8_t axis, float new_value){
  if(control_params.filter_mode == FILTER_MODE_NONE){
    return new_value;
  }

  float* filter_axis = axis == PITCH ? filter.pitch : filter.roll;
  if(control_params.filter_mode == FILTER_MODE_AVERAGE){
    shift(filter_axis, FILTER_SIZE);
    filter_axis[0] = new_value;
    return average(filter_axis, FILTER_SIZE);
  }

  // If an unrecognized filter value is read, default to no filtering
  return new_value;
}
