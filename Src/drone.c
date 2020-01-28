/*
 * drone.c
 *
 *  Created on: Jan 13, 2020
 *      Author: root
 */

#include "drone.h"
#include "utils.h"

void calculate_proportion(void){
  controller.xp = trim(-errorData.x * params.pGain, MIN_ACCEL - params.xGoal, MAX_ACCEL - params.xGoal);
  controller.yp = trim(-errorData.y * params.pGain, MIN_ACCEL - params.yGoal, MAX_ACCEL - params.yGoal);
}

void calculate_integral(void){
  integralData.x = trim(integralData.x - (errorData.x * params.iGain), MIN_ACCEL, MAX_ACCEL);
  integralData.y = trim(integralData.y - (errorData.y * params.iGain), MIN_ACCEL, MAX_ACCEL);
  controller.xi = integralData.x;
  controller.yi = integralData.y;
}

void calculate_derivative(void){
  controller.xd = trim(-(errorData.x - derivativeData.x) * params.dGain, MIN_ACCEL, MAX_ACCEL);
  controller.yd = trim(-(errorData.y - derivativeData.y) * params.dGain, MIN_ACCEL, MAX_ACCEL);
  derivativeData.x = errorData.x;
  derivativeData.y = errorData.y;
}

void set_motors(TIM_HandleTypeDef * timer){
  timer->Instance->CCR2 = map(controller.xp + controller.xi + controller.xd, -3000 - params.xGoal, 3000 - params.xGoal, MIN_THROTTLE, MAX_THROTTLE);
  timer->Instance->CCR1 = map(-controller.xp - controller.xi - controller.xd, -3000 - params.xGoal, 3000 - params.xGoal, MIN_THROTTLE, MAX_THROTTLE);
  timer->Instance->CCR4 = map(controller.yp + controller.yi + controller.yd, -3000 - params.yGoal, 3000 - params.yGoal, MIN_THROTTLE, MAX_THROTTLE);
  timer->Instance->CCR3 = map(-controller.yp - controller.yi - controller.yd, -3000 - params.yGoal, 3000 - params.yGoal, MIN_THROTTLE, MAX_THROTTLE);
}

void set_accel_data(LIS3DSH_DataScaled accelData){
  errorData.x = trim(accelData.x - params.xGoal, MIN_ACCEL - params.xGoal, MAX_ACCEL - params.xGoal);
  errorData.y = trim(accelData.y - params.yGoal, MIN_ACCEL - params.yGoal, MAX_ACCEL - params.yGoal);
}

