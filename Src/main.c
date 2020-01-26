/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "MY_LIS3DSH.h"
#include "string.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
LIS3DSH_DataScaled accelData;
UART_HandleTypeDef huart2;
int EXTI0_FLAG = 0;
int min_thrust = 183; //180
int max_thrust = 366; //370
int min_accel = -1000;
int max_accel = 1000;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
void EXTI0_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
  EXTI0_FLAG = 1;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void UART_Config(void);
void calibrate_accel(void);
void init_motors(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
int16_t map(int16_t, int16_t, int16_t, int16_t, int16_t);
int16_t trim(int16_t, int16_t, int16_t);
void set_motor_x(uint32_t);
void set_motor_inv_x(uint32_t);
void set_motor_y(uint32_t);
void set_motor_inv_y(uint32_t);
//float shift(float*, int);
//float average(float*, int);
void config_accel(void);
void config_control_params(void);
void set_accel_data(LIS3DSH_DataScaled);
void calculate_proportion(void);
void calculate_integral(void);
void calculate_derivative(void);
void set_motors(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
int16_t txData[4];

/**
  * @brief  The application entry point.
  * @retval int
  */

LIS3DSH_InitTypeDef accelConfigDef;
int main(void)
{
  SystemClock_Config();
  MX_GPIO_Init();
  HAL_Init();
  config_accel();
  config_control_params();
  UART_Config();
  MX_SPI1_Init();
  MX_TIM4_Init();
  LIS3DSH_Init(&hspi1, &accelConfigDef);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  init_motors();

//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//
//  float p_gain = .6;
//  float i_gain = .5;
//  float x_goal = 0;
//  float y_goal = 0;
//  float x1_error//  set_motor_x(max_thrust);
  //  set_motor_inv_x(max_thrust);
  //  set_motor_y(max_thrust);
  //  set_motor_inv_y(max_thrust);
  //  HAL_Delay(10000);//  set_motor_x(max_thrust);
  //  set_motor_inv_x(max_thrust);
  //  set_motor_y(max_thrust);
  //  set_motor_inv_y(max_thrust);
  //  HAL_Delay(10000);;
//  float x2_error;
//  float y1_error;
//  float y2_error;
//
//  float px1 = 0;
//  float px2 = 0;
//  float py1 = 0;
//  float py2 = 0;
//  float x1int = 0;
//  float x2int = 0;
//  float y1int = 0;
//  float y2int = 0;
//  float ix1 = 0;
//  float ix2 = 0;
//  float iy1 = 0;
//  float iy2 = 0;
//  const int NUM_AVG = 5;
//  float dy1_errors_new[5] = {0};
//  float dy1_errors_old[5] = {0};
//  float dy1_new_avg;
//  float dy1_old_avg;

//  float throttle = 0;
  int count = 0;

  while (1) {
    count += 1;
    if (LIS3DSH_PollDRDY(1000) == true) {
      set_accel_data(LIS3DSH_GetDataScaled());
      calculate_proportion();
      calculate_integral();
      if(count % 15 == 0){
        calculate_derivative();
        count = 0;
      }
      set_motors();
//
//      //d calculations
//      if(count % 10 == 0){
//        shift(dy1_errors_old, NUM_AVG);
//        dy1_errors_old[0] = shift(dy1_errors_new, NUM_AVG);
//        dy1_errors_new[0] = y1_error;
//        dy1_new_avg = average(dy1_errors_new, NUM_AVG);
//        dy1_old_avg = average(dy1_errors_old, NUM_AVG);
//        count = 0;
//      }
//
//      px1 = map(x1_error, min_accel, max_accel, min_thrust, max_thrust);
//      px2 = map(x2_error, min_accel, max_accel, min_thrust, max_thrust);
//      py1 = map(y1_error, min_accel, max_accel, min_thrust, max_thrust);
//      py2 = map(y2_error, min_accel, max_accel, min_thrust, max_thrust);
//
//      x1int = trim(x1int + (x1_error / 2000), 0, max_thrust - min_thrust);
//      x2int = trim(x2int + (x2_error / 2000), 0, max_thrust - min_thrust);
//      y1int = trim(y1int + (y1_error / 2000), 0, max_thrust - min_thrust);
//      y2int = trim(y2int + (y2_error / 2000), 0, max_thrust - min_thrust);
//
//      ix1 = x1int;
//      ix2 = x2int;
//      iy1 = y1int;
//      iy2 = y2int;
//
      txData[0] = controller.xp;
      txData[1] = controller.xi;
      txData[2] = controller.xd;
////      txData[3] = trim(py2 + iy2, min_thrust, max_thrust);
//
      HAL_UART_Transmit(&huart2, (uint8_t *) txData, sizeof txData, 10);
////      HAL_Delay(250);
//
//
//
//      if(throttle < (max_thrust - min_thrust) / 6){
//        throttle += .01;
//      }
    }
  }
  /* USER CODE END 3 */
}

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

void set_motors(void){
  set_motor_y(map(controller.xp + controller.xi + controller.xd, -3000 - params.xGoal, 3000 - params.xGoal, min_thrust, max_thrust));
  set_motor_y(map(-controller.xp - controller.xi - controller.xd, -3000 - params.xGoal, 3000 - params.xGoal, min_thrust, max_thrust));
  set_motor_y(map(controller.yp + controller.yi + controller.yd, -3000 - params.yGoal, 3000 - params.yGoal, min_thrust, max_thrust));
  set_motor_y(map(-controller.yp - controller.yi - controller.yd, -3000 - params.yGoal, 3000 - params.yGoal, min_thrust, max_thrust));
}

void set_accel_data(LIS3DSH_DataScaled accelData){
  errorData.x = trim(accelData.x - params.xGoal, -1000 - params.xGoal, 1000 - params.xGoal);
  errorData.y = trim(accelData.y - params.yGoal, -1000 - params.yGoal, 1000 - params.yGoal);
}

float shift(float* arr, int arrLen){
    float ret = arr[arrLen - 1];
    for(int i = arrLen - 2; i >= 0; i--){
        arr[i + 1] = arr[i];
    }
    return ret;
}

float average(float* arr, int arrLen){
    float sum = 0;
    for(int i = 0; i < arrLen; i++){
        sum += arr[i];
    }
    return sum / arrLen;
}

void config_accel(void){
  accelConfigDef.dataRate = LIS3DSH_DATARATE_400;
  accelConfigDef.fullScale = LIS3DSH_FULLSCALE_4;
  accelConfigDef.antiAliasingBW = LIS3DSH_FILTER_BW_50;
  accelConfigDef.enableAxes = LIS3DSH_XYZ_ENABLE;
  accelConfigDef.interruptEnable = false;
}

void config_control_params(void){
  params.xGoal = 0;
  params.yGoal = 0;
  params.pGain = 2;
  params.iGain = .02;
  params.dGain = 7;

  integralData.x = 0;
  integralData.y = 0;
  derivativeData.x = 0;
  derivativeData.y = 0;

}

/**
  * @brief System Clock Configuration
  *
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB an  // Error increasing, positive slope
  //  Slow down motors
  // Error increasing negative slope
  //  Speed up motorsd APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 67;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3650;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ACCEL_CS_Pin */
  GPIO_InitStruct.Pin = ACCEL_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACCEL_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PIN_RESET;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

void calibrate_accel(void)
{
  float min_x, min_y = 0, max_x , max_y;
  if (LIS3DSH_PollDRDY(1000) == true) {
    accelData = LIS3DSH_GetDataScaled();
    min_x = accelData.x;
    max_x = accelData.x;
    min_y = accelData.y;
    max_y = accelData.y;
  }
  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET){
    if (LIS3DSH_PollDRDY(1000) == true) {
      accelData = LIS3DSH_GetDataScaled();
      if(accelData.x < min_x){
        min_x = accelData.x;
      }
      if(accelData.y < min_y){
        min_y = accelData.y;
      }
      if(accelData.x > min_x){
        max_x = accelData.x;
      }
      if(accelData.y > min_y){
        max_y = accelData.y;
      }
    }
  }
  LIS3DSH_X_calibrate(min_x, max_x);
  LIS3DSH_Y_calibrate(min_y, max_y);
  HAL_Delay(50);
}

void init_motors(void)
{
//  set_motor_x(max_thrust);
//  set_motor_inv_x(max_thrust);
//  set_motor_y(max_thrust);
//  set_motor_inv_y(max_thrust);
//  HAL_Delay(10000);
  set_motor_x(min_thrust);
  set_motor_inv_x(min_thrust);
  set_motor_y(min_thrust);
  set_motor_inv_y(min_thrust);
  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET){
    HAL_Delay(50);
  }
}

void UART_Config(void){
  __HAL_RCC_GPIOA_CLK_ENABLE();

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

void set_motor_x(uint32_t val){
  htim4.Instance->CCR2 = val;
}

void set_motor_inv_x(uint32_t val){
  htim4.Instance->CCR1 = val;
}

void set_motor_y(uint32_t val){
  htim4.Instance->CCR4 = val;
}

void set_motor_inv_y(uint32_t val){
  htim4.Instance->CCR3 = val;
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
