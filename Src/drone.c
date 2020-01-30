/*
 * drone.c
 *
 *  Created on: Jan 13, 2020
 *      Author: root
 */

#include "drone.h"
#include "utils.h"

void drone_init(DroneInitStruct * init){
  control_gains.P = init->p_gain;
  control_gains.I = init->i_gain;
  control_gains.D = init->d_gain;
}

void calculate_PID(LIS3DSH_DataScaled accelData){
  sensor.PITCH = accelData.y;
  sensor.ROLL = accelData.x;

  //TODO: Verify trimming used correctly here
  err.PITCH = trim(goal.PITCH - sensor.PITCH, MIN_ACCEL, MAX_ACCEL);
  err.ROLL = trim(goal.ROLL - sensor.ROLL, MIN_ACCEL, MAX_ACCEL);

  err_sum.PITCH = trim(err_sum.PITCH + err.PITCH * control_gains.I, MIN_ACCEL, MAX_ACCEL);
  err_sum.ROLL = trim(err_sum.ROLL + err.ROLL * control_gains.I, MIN_ACCEL, MAX_ACCEL);

  err_change.PITCH = trim(err.PITCH - err_prev.PITCH, MIN_ACCEL, MAX_ACCEL);
  err_change.ROLL = trim(err.ROLL - err_prev.ROLL, MIN_ACCEL, MAX_ACCEL);
  err_prev.PITCH = err.PITCH;
  err_prev.ROLL = err.ROLL;
}

void set_motors(TIM_HandleTypeDef * timer){

//    A
//    |
//  B---C
//    |
//    D

  int16_t a_setpoint = trim((-err.PITCH * control_gains.P) + (-err_sum.PITCH) + (-err_change.PITCH * control_gains.D), 0, 3 * MAX_ACCEL);
  timer->Instance->CCR3 = map(a_setpoint, 0, 3 * MAX_ACCEL, MIN_THROTTLE, MAX_THROTTLE);
  int16_t b_setpoint = trim((err.ROLL * control_gains.P) + (err_sum.ROLL) + (err_change.ROLL * control_gains.D), 0, 3 * MAX_ACCEL);
  timer->Instance->CCR2 = map(b_setpoint, 0, 3 * MAX_ACCEL, MIN_THROTTLE, MAX_THROTTLE);
  int16_t c_setpoint = trim((-err.ROLL * control_gains.P) + (-err_sum.ROLL) + (-err_change.ROLL * control_gains.D), 0, 3 * MAX_ACCEL);
  timer->Instance->CCR1 = map(c_setpoint, 0, 3 * MAX_ACCEL, MIN_THROTTLE, MAX_THROTTLE);
  int16_t d_setpoint = trim((err.PITCH * control_gains.P) + (err_sum.PITCH) + (err_change.PITCH * control_gains.D), 0, 3 * MAX_ACCEL);
  timer->Instance->CCR4 = map(d_setpoint, 0, 3 * MAX_ACCEL, MIN_THROTTLE, MAX_THROTTLE);
}

