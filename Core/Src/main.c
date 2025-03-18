/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_A_EN_PIN GPIO_PIN_9
#define MOTOR_A_EN_PORT GPIOE
#define MOTOR_B_EN_PIN GPIO_PIN_11
#define MOTOR_B_EN_PORT GPIOE

#define UART_CONFIG_ERROR 1
#define BLE_AT_HANDSHAKE_ERROR 2
#define BLE_WAKE_ERROR 3
#define BLE_AT_COMMAND_ERROR 4
#define TIM_INIT_ERROR 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint8_t current_error = 0;
uint32_t current_HAL_status = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */
void BLE_Send_AT_Command(char *msg, UART_HandleTypeDef *huart, char* response, uint8_t expected_max_length);
void BLE_Send_Handshake(UART_HandleTypeDef *huart, char* response);
uint8_t BLE_Wake_From_Sleep(UART_HandleTypeDef *huart, char* response);

void print_msg(char *msg);

void Motor_A_Control(int8_t direction, int16_t speed);
void Motor_B_Control(int8_t direction, int16_t speed);
void Motor_Direct_Test(void);
void Motor_PWM_Test(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  // Motor_Direct_Test();
  // Motor_PWM_Test();

  //bluetooth handshake:
  char response[50];
  BLE_Send_Handshake(&huart6, response);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 180;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // For Motor A
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // For Motor B
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, MOTOR_B_IN1_Pin|MOTOR_A_IN2_Pin|MOTOR_B_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_B_IN1_Pin MOTOR_A_IN2_Pin MOTOR_B_IN2_Pin */
  GPIO_InitStruct.Pin = MOTOR_B_IN1_Pin|MOTOR_A_IN2_Pin|MOTOR_B_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_A_IN1_Pin */
  GPIO_InitStruct.Pin = MOTOR_A_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_A_IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void print_msg(char *msg) {
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
}


void Motor_A_Control(int8_t direction, int16_t speed) {
  // clamp speed to 1000
  if (speed > 1000)
    speed = 1000;

  if (direction == 1) { 
    // forward
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
  } else if (direction == -1) {
    // reverse
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_SET);
  } else {
    // stop
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
  }

  // set speed
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
}

void Motor_B_Control(int8_t direction, int16_t speed) {
  // clamp speed to 1000
  if (speed > 1000)
    speed = 1000;

  if (direction == 1) { 
    // forward
    HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_RESET);
  } else if (direction == -1) {
    // reverse
    HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_SET);
  } else {
    // stop
    HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_RESET);
  }

  // set speed
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
}

void Motor_PWM_Test(void) {
  // test motor control
  print_msg("Motor test control\r\n");

  // 30% 
  print_msg("Moving forward 0.50\r\n");
  Motor_A_Control(1, 500);
  Motor_B_Control(1, 500);
  HAL_Delay(5000);

  // 70%
  print_msg("Moving forward 0.90\r\n"); 
  Motor_A_Control(1, 900);
  Motor_B_Control(1, 900);
  HAL_Delay(5000);

  // Stop
  print_msg("Stopping\r\n");
  Motor_A_Control(0, 0);
  Motor_B_Control(0, 0);
  HAL_Delay(5000);

  // -30%
  print_msg("Moving backward 0.50\r\n");
  Motor_A_Control(-1, 500);
  Motor_B_Control(-1, 500);
  HAL_Delay(5000);

  // -70%
  print_msg("Moving backward 0.90\r\n");
  Motor_A_Control(-1, 900);
  Motor_B_Control(-1, 900);
  HAL_Delay(5000);

  // Stop
  print_msg("Stopping\r\n");
  Motor_A_Control(0, 0);
  Motor_B_Control(0, 0);
  HAL_Delay(5000);
}


// NOT FOR NORMAL USE
// THIS FUNCTION IS HERE FOR IF PWM CONTROL IS NOT WORKING
// AND THERE IS NEED TO TEST WITH DIRECT GPIO CONTROL
// BY DRIVING EN PINS HIGH OR LOW
void Motor_Direct_Test(void) {
  // Reconfigure EN pins as regular GPIO outputs
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = MOTOR_A_EN_PIN | MOTOR_B_EN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // Test with direct GPIO control
  print_msg("Setting direction pins\r\n");
  HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_SET);   // Forward direction
  HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_SET);   // Forward direction
  HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_RESET);

  print_msg("Enabling motors directly\r\n");
  HAL_GPIO_WritePin(MOTOR_A_EN_PORT, MOTOR_A_EN_PIN, GPIO_PIN_SET);     // Enable motor A
  HAL_GPIO_WritePin(MOTOR_B_EN_PORT, MOTOR_B_EN_PIN, GPIO_PIN_SET);     // Enable motor B
  HAL_Delay(5000);

  print_msg("Stopping motors\r\n");
  HAL_GPIO_WritePin(MOTOR_A_EN_PORT, MOTOR_A_EN_PIN, GPIO_PIN_RESET);   // Disable motor A
  HAL_GPIO_WritePin(MOTOR_B_EN_PORT, MOTOR_B_EN_PIN, GPIO_PIN_RESET);   // Disable motor B
  HAL_Delay(2000);

  // Optional: Test reverse
  print_msg("Testing reverse direction\r\n");
  HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET); // Reverse direction
  HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_RESET); // Reverse direction
  HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(MOTOR_A_EN_PORT, MOTOR_A_EN_PIN, GPIO_PIN_SET);     // Enable motor A
  HAL_GPIO_WritePin(MOTOR_B_EN_PORT, MOTOR_B_EN_PIN, GPIO_PIN_SET);     // Enable motor B
  HAL_Delay(5000);

  print_msg("Stopping motors\r\n");
  HAL_GPIO_WritePin(MOTOR_A_EN_PORT, MOTOR_A_EN_PIN, GPIO_PIN_RESET);   // Disable motor A
  HAL_GPIO_WritePin(MOTOR_B_EN_PORT, MOTOR_B_EN_PIN, GPIO_PIN_RESET);   // Disable motor B
}


void BLE_Send_AT_Command(char *msg, UART_HandleTypeDef *huart, char* response, uint8_t expected_max_length) {
  char cmd[50];
  sprintf(cmd, "AT+%s", msg);
  memset(response, 0, expected_max_length);

  //transmit command
  current_HAL_status = HAL_UART_Transmit(huart, (uint8_t *)cmd, strlen(cmd), 500);
  if (current_HAL_status != HAL_OK) {
    print_msg("Transmit error\r\n");
    current_error = BLE_AT_COMMAND_ERROR;
    Error_Handler();
  }

  HAL_Delay(100);

  //receive response
  current_HAL_status = HAL_UART_Receive(huart, (uint8_t*)response, expected_max_length, 2000); 
  if (current_HAL_status == HAL_OK || current_HAL_status == HAL_TIMEOUT) {
    print_msg("Received data\r\n");
  } else {
    current_error = BLE_AT_COMMAND_ERROR;
    print_msg("Receive error\r\n");
    Error_Handler();
  }

  // print whatever we received
  char r[100];
  sprintf(r, "Response: '%s'\r\n", response);
  print_msg(r);
}

void BLE_Send_Handshake(UART_HandleTypeDef *huart, char* response) {
  char cmd[] = "AT";
  // uint16_t received_length;
  memset(response, 0, 50);
  
  current_HAL_status = HAL_UART_Transmit(huart, (uint8_t *)cmd, strlen(cmd), 500); 
  if (current_HAL_status != HAL_OK) {
    print_msg("Transmit error\r\n");

    current_error = BLE_AT_HANDSHAKE_ERROR;
    Error_Handler();
  }
  
  // HAL_Delay(100);

  current_HAL_status = HAL_UART_Receive(huart, (uint8_t*)response, 2, 2000);
  if (current_HAL_status == HAL_OK || current_HAL_status == HAL_TIMEOUT) {
    print_msg("Received data\r\n");
  } else {
    current_error = BLE_AT_HANDSHAKE_ERROR;
    print_msg("Receive error\r\n");
    Error_Handler();
  }
  
  // Print whatever we received
  char msg[100];
  sprintf(msg, "Response: '%s'\r\n", response);
  print_msg(msg);
}

uint8_t BLE_Wake_From_Sleep(UART_HandleTypeDef *huart, char* response) {
  char cmd[] = "JonahJDDiamondJonah";

  current_HAL_status = HAL_UART_Transmit(huart, (uint8_t *)cmd, strlen(cmd), 100);
  if (current_HAL_status != HAL_OK) {
    current_error = BLE_WAKE_ERROR;
    Error_Handler();
  }

  HAL_Delay(50);

  current_HAL_status = HAL_UART_Receive(huart, (uint8_t *)response, 7, 500);
  if (current_HAL_status != HAL_OK) {
    current_error = BLE_WAKE_ERROR;
    Error_Handler();
  }

  response[8] = '\0';

  if (strstr(response, "OK+WAKE") != NULL) {
    return 1;
  } else {
    return 0;
  }

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == USER_Btn_Pin) {
    // simple debounce
    HAL_Delay(500);

    print_msg("Button pressed!\r\n");

    // TO BE USED IN CASE OF PWM FAILURE:
    // Motor_Direct_Test();

    // Motor test:
    Motor_PWM_Test();

    
    // Bluetooth handshake test:
    // char response[50];
    // BLE_Send_Handshake(&huart6, response);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    switch (current_error) {
      case UART_CONFIG_ERROR:
        print_msg("HAL_UART_Init failed for USART6\r\n");
        break;
      case BLE_AT_HANDSHAKE_ERROR:
        print_msg("AT handshake failed\r\n");
        break;
      case BLE_WAKE_ERROR:
        print_msg("BLE wake failed\r\n");
        break;
      case BLE_AT_COMMAND_ERROR:
        print_msg("AT command failed\r\n");
        break;
      case TIM_INIT_ERROR:
        print_msg("TIM init failed\r\n");
        break;
      default:
        print_msg("Error\r\n");
        break;
    }
    switch (current_HAL_status) {
      case HAL_OK:
        print_msg("HAL OK\r\n");
        break;
      case HAL_ERROR:
        print_msg("HAL ERROR\r\n");
        break;
      case HAL_BUSY:
        print_msg("HAL BUSY\r\n");
        break;
      case HAL_TIMEOUT:
        print_msg("HAL TIMEOUT\r\n");
        break;
      default:
        print_msg("HAL UNKNOWN\r\n");
        break;
    }

    HAL_Delay(100);
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
