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
#include "stm32f4xx_hal_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

#define BLE_UART6_RX_BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint8_t current_error = 0;
uint32_t current_HAL_status = 0;
char print_buffer[300];

char ble1_rx_buffer[BLE_UART6_RX_BUFFER_SIZE];
uint8_t ble1_rx_data;
uint16_t ble1_rx_write_pos = 0;
uint8_t ble1_connected = 0;

// HC-SR04 variables
volatile uint32_t echo_rise_time = 0;
volatile uint32_t echo_fall_time = 0;
volatile uint8_t echo_capture_complete = 0;
volatile uint32_t distance_cm = 0;
uint32_t distance_buffer[3] = {0, 0, 0};
uint8_t distance_buffer_index = 0;
uint32_t current_distance = 0;

uint8_t jonahvinav = 0;
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
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void BLE_Send_AT_Command(char *msg, UART_HandleTypeDef *huart, char* response, uint8_t expected_max_length);
void BLE_Send_Handshake(UART_HandleTypeDef *huart, char* response);
uint8_t BLE_Wake_From_Sleep(UART_HandleTypeDef *huart, char* response);
void BLE_InitConfigure(void);  
void BLE_Process_Data(uint8_t ble_num);

void print_msg(char *msg);

void Motor_A_Control(int8_t direction, int16_t speed);
void Motor_B_Control(int8_t direction, int16_t speed);
void Motors_Turn(uint16_t direction);
void Motor_Direct_Test(void);
void Motor_PWM_Test(void);

void USS_Delay_us(uint16_t us);
void USS_Init(void);
void USS_Trigger(void);
uint8_t USS_Read(uint32_t* distance);
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Motor_Direct_Test();
  // Motor_PWM_Test();

  BLE_InitConfigure();

  // Initialize HC-SR04
  USS_Init();
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Receive_IT(&huart6, &ble1_rx_data, 1);

  // HAL_GPIO_WritePin(USS_1_TRIG_GPIO_Port, USS_1_TRIG_Pin, GPIO_PIN_RESET);
  // Motors_Turn(0);
  while (1)
  {
    BLE_Process_Data(1);

    // HAL_Delay(500);
    // print RSSI
    // char response[50];
    // BLE_Send_AT_Command("RSSI?", &huart6, response, 50);
    // sprintf(print_buffer, "RSSI: %s\r\n", response);
    // print_msg(print_buffer);
    // HAL_Delay(500);
    
    // print address
    // char response[50];
    // BLE_Send_AT_Command("ADDR?", &huart6, response, 50);
    // sprintf(print_buffer, "ADDR: %s\r\n", response);
    // print_msg(print_buffer);
    // HAL_Delay(100);


    //trigger measurement
    USS_Trigger();
    //wait for echo with timeout
    uint32_t start_time = HAL_GetTick();
    while (!echo_capture_complete) {
      if (HAL_GetTick() - start_time > 100) { // 100ms timeout
        break;
      }
    }
    // Read distance
    uint32_t measured_distance = 0;
    if (USS_Read(&measured_distance)) {
      distance_buffer[distance_buffer_index] = measured_distance;
      distance_buffer_index = (distance_buffer_index + 1) > 2 ? 0 : distance_buffer_index + 1;
      current_distance = (distance_buffer[0] + distance_buffer[1] + distance_buffer[2]) / 3;
      // sprintf(print_buffer, "%lu cm\r\n", current_distance);
      // print_msg(print_buffer);
    } else {
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    }


    if (jonahvinav) {
      if (current_distance < 15) {
        Motor_A_Control(0, 0);
        Motor_B_Control(0, 0);
      } else {
        Motor_A_Control(1, 750);
        Motor_B_Control(1, 750);
      }
    } else {
      Motor_A_Control(0, 0);
      Motor_B_Control(0, 0);
    }
    
    // Wait before next measurement
    HAL_Delay(20);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, USS_1_TRIG_Pin|MOTOR_B_IN1_Pin|MOTOR_A_IN2_Pin|MOTOR_B_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USS_1_TRIG_Pin MOTOR_B_IN1_Pin MOTOR_A_IN2_Pin MOTOR_B_IN2_Pin */
  GPIO_InitStruct.Pin = USS_1_TRIG_Pin|MOTOR_B_IN1_Pin|MOTOR_A_IN2_Pin|MOTOR_B_IN2_Pin;
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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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

void Motors_Turn(uint16_t direction) {
  uint16_t time = 80 / 36 * direction;;

  Motor_A_Control(-1, 1000);
  Motor_B_Control(1, 1000);
  HAL_Delay(time);
  Motor_A_Control(0, 0);
  Motor_B_Control(0, 0);
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

  // abort reception
  HAL_UART_AbortReceive_IT(huart);

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

  //receive response
  uint8_t string_length = 0;
  current_HAL_status = HAL_OK;
  char b[1];
  do {
    current_HAL_status = HAL_UART_Receive(huart, (uint8_t*)b, 1, 100);
    if (current_HAL_status == HAL_OK) {
      response[string_length++] = *b;
    } else if (current_HAL_status == HAL_TIMEOUT) {
      break;
    } else {
      current_error = BLE_AT_HANDSHAKE_ERROR;
      print_msg("Receive error\r\n");
      Error_Handler();
    }
  } while (current_HAL_status == HAL_OK);
  
  // add null terminator
  response[string_length] = '\0';

  // restart reception
  HAL_UART_Receive_IT(huart, &ble1_rx_data, 1);
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

  // receive response
  uint8_t string_length = 0;
  current_HAL_status = HAL_OK;
  char b[1];
  do {
    current_HAL_status = HAL_UART_Receive(huart, (uint8_t*)b, 1, 100);
    if (current_HAL_status == HAL_OK) {
      response[string_length++] = *b;
    } else if (current_HAL_status == HAL_TIMEOUT) {
      break;
    } else {
      current_error = BLE_AT_HANDSHAKE_ERROR;
      print_msg("Receive error\r\n");
      Error_Handler();
    }
  } while (current_HAL_status == HAL_OK);

  
  // add null terminator
  response[string_length] = '\0';
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

  if (strstr(response, "OK+WAKE") != NULL) {
    return 1;
  } else {
    return 0;
  }

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == USER_Btn_Pin) {
    // simple debounce
    HAL_Delay(50);
    print_msg("Button pressed!\r\n");

    // TO BE USED IN CASE OF PWM FAILURE:
    // Motor_Direct_Test();

    // Motor test:
    // Motor_PWM_Test();

    
    // Bluetooth handshake test:
    // char response[50];
    // BLE_Send_Handshake(&huart6, response);
  }
}

void BLE_InitConfigure(void) {
  //bluetooth handshake:
  char response[50];
  BLE_Send_Handshake(&huart6, response);
  sprintf(print_buffer, "HANDSHAKE: %s\r\n", response);
  print_msg(print_buffer);
  if (strcmp(response, "OK+LOST\r\n") == 0) {
    print_msg("Disconnected from previous connection\r\n");
    HAL_Delay(500);
  }

  // print address, passkey
  BLE_Send_AT_Command("ADDR?", &huart6, response, 50);
  sprintf(print_buffer, "ADDR: %s\r\n", response);
  print_msg(print_buffer);
  BLE_Send_AT_Command("PASS?", &huart6, response, 50);
  sprintf(print_buffer, "PASS: %s\r\n", response);
  print_msg(print_buffer);

  // set name
  BLE_Send_AT_Command("NAMEstummon_ble_1", &huart6, response, 50);
  sprintf(print_buffer, "NAME: %s\r\n", response);
  print_msg(print_buffer);

  // set slave
  BLE_Send_AT_Command("ROLE0", &huart6, response, 50);
  sprintf(print_buffer, "ROLE: %s\r\n", response);
  print_msg(print_buffer);

  // wait for reboot
  HAL_Delay(500);

  // set advi
  BLE_Send_AT_Command("ADVI0", &huart6, response, 50);
  sprintf(print_buffer, "ADVI: %s\r\n", response);
  print_msg(print_buffer);

  // query mode
  BLE_Send_AT_Command("MODE?", &huart6, response, 50);
  sprintf(print_buffer, "MODE: %s\r\n", response);
  print_msg(print_buffer);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart != &huart6)
    return;

  // Put byte in buffer and increment write position
  ble1_rx_buffer[ble1_rx_write_pos++] = ble1_rx_data;
  ble1_rx_buffer[ble1_rx_write_pos] = '\0'; // Null-terminate

  // Check for connection messages with proper line endings
  if (strstr(ble1_rx_buffer, "OK+CONN") != NULL) {
    ble1_connected = 1;
    print_msg("Connected\r\n");
    ble1_rx_write_pos = 0; // Reset buffer
    ble1_rx_buffer[0] = '\0';
  } 
  else if (strstr(ble1_rx_buffer, "OK+LOST") != NULL) {
    ble1_connected = 0;
    print_msg("Disconnected\r\n");
    ble1_rx_write_pos = 0; // Reset buffer
    ble1_rx_buffer[0] = '\0';
  }

  // Safety check - if buffer is getting too full, reset it
  if (ble1_rx_write_pos >= BLE_UART6_RX_BUFFER_SIZE - 2) {
    ble1_rx_write_pos = 0;
    ble1_rx_buffer[0] = '\0';
  }

  // Restart the interrupt reception
  HAL_UART_Receive_IT(&huart6, &ble1_rx_data, 1);
}

void BLE_Process_Data(uint8_t ble_num) {
  if (ble_num == 1 && ble1_rx_write_pos > 0) {
    // Check if we have a complete message (ends with \n)
    if (strstr(ble1_rx_buffer, "\n") != NULL) {
      sprintf(print_buffer, "BLE1 RX: %s\r\n", ble1_rx_buffer);
      print_msg(print_buffer);

      if (strstr(ble1_rx_buffer, "F1") != NULL) {
        Motor_A_Control(1, 500);
        Motor_B_Control(1, 500);
      } else if (strstr(ble1_rx_buffer, "F2") != NULL) {
        Motor_A_Control(1, 750);
        Motor_B_Control(1, 750);
      } else if (strstr(ble1_rx_buffer, "F3") != NULL) {
        Motor_A_Control(1, 1000);
        Motor_B_Control(1, 1000);
      } else if (strstr(ble1_rx_buffer, "S") != NULL) {
        Motor_A_Control(0, 0);
        Motor_B_Control(0, 0);
        jonahvinav = 0;
      } else if (strstr(ble1_rx_buffer, "B1") != NULL) {
        Motor_A_Control(-1, 500);
        Motor_B_Control(-1, 500);
      } else if (strstr(ble1_rx_buffer, "B2") != NULL) {
        Motor_A_Control(-1, 750);
        Motor_B_Control(-1, 750);
      } else if (strstr(ble1_rx_buffer, "B3") != NULL) {
        Motor_A_Control(-1, 1000);
        Motor_B_Control(-1, 1000);
      } else if (strstr(ble1_rx_buffer, "T") != NULL ) {
        char direction_buffer[4];
        for (int i = 1; i < 5; i++) {
          if (ble1_rx_buffer[i] != '\n') {
            direction_buffer[i - 1] = ble1_rx_buffer[i];
          } else {
            direction_buffer[i - 1] = '\0';
            break;
          }
        }
        uint16_t direction = atoi(direction_buffer);
        Motors_Turn(direction);
      } else if (strstr(ble1_rx_buffer, "GO") != NULL) {
        jonahvinav = 1;
      }
      
      ble1_rx_write_pos = 0;
      ble1_rx_buffer[0] = '\0';
    }
  }
}

//USS ultrasonic sensor functions
void USS_Delay_us(uint16_t us){
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  while (__HAL_TIM_GET_COUNTER(&htim3) < us);
}
void USS_Init(void){
  // Start the timer base
  HAL_TIM_Base_Start(&htim3);
  
  // Start input capture in interrupt mode
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
}
void USS_Trigger(void){
  // Reset capture complete flag
  echo_capture_complete = 0;
  
  // Generate 10us pulse on TRIG pin
  HAL_GPIO_WritePin(USS_1_TRIG_GPIO_Port, USS_1_TRIG_Pin, GPIO_PIN_SET);
  USS_Delay_us(10);
  HAL_GPIO_WritePin(USS_1_TRIG_GPIO_Port, USS_1_TRIG_Pin, GPIO_PIN_RESET);
}
uint8_t USS_Read(uint32_t* distance){
  if (!echo_capture_complete)
    return 0;
    
  // Calculate pulse width
  uint32_t pulse_width;
  if (echo_fall_time > echo_rise_time) {
    pulse_width = echo_fall_time - echo_rise_time;
  } else {
    // Handle timer overflow
    pulse_width = ((0xFFFF - echo_rise_time) + echo_fall_time + 1);
  }
  
  // Calculate distance: d = (t × 343 m/s) ÷ 2
  // For cm: d = (t(μs) × 0.0343 cm/μs) ÷ 2 = t × 0.01715
  // Simplified with integer math: d = t × 343 ÷ 20000
  *distance = (pulse_width * 343) / 20000;
  
  return 1;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  // if it is tim3 (USS) and it is the echo channel (tim3 channel 1)
  if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    if (HAL_GPIO_ReadPin(USS_1_ECHO_GPIO_Port, USS_1_ECHO_Pin) == GPIO_PIN_SET) {
      // Rising edge detected
      echo_rise_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      // Change polarity to capture falling edge
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    } else {
      // Falling edge detected
      echo_fall_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      // Change back to capture rising edge
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
      echo_capture_complete = 1;
    }
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
