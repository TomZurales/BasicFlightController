/*
 * drone.c
 *
 *  Created on: Jan 13, 2020
 *      Author: root
 */

#include "drone.h"
#include "utils.h"

void calculate_proportion(void){
  controller.xp = trim(-errorData.x * params.pGain, -1000 - params.xGoal, 1000 - params.xGoal);
  controller.yp = trim(-errorData.y * params.pGain, -1000 - params.yGoal, 1000 - params.yGoal);
}

void calculate_integral(void){
  integralData.x = trim(integralData.x - (errorData.x * params.iGain), -1000, 1000);
  integralData.y = trim(integralData.y - (errorData.y * params.iGain), -1000, 1000);
  controller.xi = integralData.x;
  controller.yi = integralData.y;
}

void calculate_derivative(void){
  controller.xd = trim(-(errorData.x - derivativeData.x) * params.dGain, -1000, 1000);
  controller.yd = trim(-(errorData.y - derivativeData.y) * params.dGain, -1000, 1000);
  derivativeData.x = errorData.x;
  derivativeData.y = errorData.y;
}

void set_motors(TIM_HandleTypeDef * timer){
  set_motor_y(timer, map(controller.xp + controller.xi + controller.xd, -3000 - params.xGoal, 3000 - params.xGoal, min_thrust, max_thrust));
  set_motor_y(timer, map(-controller.xp - controller.xi - controller.xd, -3000 - params.xGoal, 3000 - params.xGoal, min_thrust, max_thrust));
  set_motor_y(timer, map(controller.yp + controller.yi + controller.yd, -3000 - params.yGoal, 3000 - params.yGoal, min_thrust, max_thrust));
  set_motor_y(timer, map(-controller.yp - controller.yi - controller.yd, -3000 - params.yGoal, 3000 - params.yGoal, min_thrust, max_thrust));
}

void set_motor_x(TIM_HandleTypeDef * timer, uint32_t val){
  timer->Instance->CCR2 = val;
}

void set_motor_inv_x(TIM_HandleTypeDef * timer, uint32_t val){
  timer->Instance->CCR1 = val;
}

void set_motor_y(TIM_HandleTypeDef * timer, uint32_t val){
  timer->Instance->CCR4 = val;
}

void set_motor_inv_y(TIM_HandleTypeDef * timer, uint32_t val){
  timer->Instance->CCR3 = val;
}

void set_accel_data(LIS3DSH_DataScaled accelData){
  errorData.x = trim(accelData.x - params.xGoal, -1000 - params.xGoal, 1000 - params.xGoal);
  errorData.y = trim(accelData.y - params.yGoal, -1000 - params.yGoal, 1000 - params.yGoal);
}
