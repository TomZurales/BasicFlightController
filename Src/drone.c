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

  err_sum.PITCH = trim(err_sum.PITCH + err.PITCH, MIN_ACCEL, MAX_ACCEL);
  err_sum.ROLL = trim(err_sum.ROLL + err.ROLL, MIN_ACCEL, MAX_ACCEL);

  err_change.PITCH = trim(err.PITCH - err_prev.PITCH, MIN_ACCEL, MAX_ACCEL);
  err_change.ROLL = trim(err.ROLL - err_prev.ROLL, MIN_ACCEL, MAX_ACCEL);
  err_prev.PITCH = err.PITCH;
  err_prev.ROLL = err.ROLL;
}

void set_motors(TIM_HandleTypeDef * timer){
  int16_t motor_set = trim((-err.ROLL * control_gains.P) + (-err_sum.ROLL * control_gains.I) + (err_change.ROLL * control_gains.D), 0, 3 * MAX_ACCEL);
  timer->Instance->CCR1 = map(motor_set, 0, 3 * MAX_ACCEL, MIN_THROTTLE, MAX_THROTTLE);
//  timer->Instance->CCR1 = map(-controller.xp - controller.xi - controller.xd, -3000 - params.xGoal, 3000 - params.xGoal, MIN_THROTTLE, MAX_THROTTLE);
//  timer->Instance->CCR4 = map(controller.yp + controller.yi + controller.yd, -3000 - params.yGoal, 3000 - params.yGoal, MIN_THROTTLE, MAX_THROTTLE);
//  timer->Instance->CCR3 = map(-controller.yp - controller.yi - controller.yd, -3000 - params.yGoal, 3000 - params.yGoal, MIN_THROTTLE, MAX_THROTTLE);
}

