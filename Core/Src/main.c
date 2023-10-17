/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> // sprintf
#include <string.h> // strlen
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
  RESET_BTN,
  WAIT,
  ARMING,
  ARMED,
  DRAWN,
  TRIGGER,
  HOLD,
  LOW_WAIT
}button_state_t;

typedef enum{
  mode1,
  mode2,
  mode3
}ledmode_t;

typedef enum{
  state1mode2,
  state2mode2,
  state3mode2
}ledmode2;

typedef enum{
  state1mode3,
  state2mode3,
  state3mode3,
  state4mode3
}ledmode3;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
button_state_t state_button1_t = RESET_BTN;
button_state_t state_button2_t = RESET_BTN;
button_state_t pre_state_button1_t = RESET_BTN;
button_state_t pre_state_button2_t = RESET_BTN;
GPIO_PinState state_button1 = GPIO_PIN_SET;
GPIO_PinState state_button2 = GPIO_PIN_SET;
uint32_t t_s1 = 0;
uint32_t t_s2 = 0;
uint32_t t_0_s1 = 0;
uint32_t t_0_s2 = 0;
uint32_t t_diff_s1 = 0;
uint32_t t_diff_s2 = 0;
uint32_t t_press_s1 = 0;
uint32_t t_press_s2 = 0;
uint32_t bounce_delay_s = 50;
uint32_t hold_delay_s = 500;

uint8_t initialled1 = 1;

uint32_t tim3scaler = 7200 - 1;
uint32_t tim3period = 10000 - 1;


ledmode_t stateofled = mode1;
ledmode2 mode2forled = state1mode2;
ledmode3 mode3forled = state1mode3;

uint8_t ishold1 = 0;
uint8_t ishold2 = 0;

uint8_t message[35] = {'\0'};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void initLED(GPIO_PinState ledbuiltin, GPIO_PinState ledr, GPIO_PinState ledg, GPIO_PinState ledb){
  HAL_GPIO_WritePin(ledbuiltin_GPIO_Port, ledbuiltin_Pin, ledbuiltin);
  HAL_GPIO_WritePin(ledred_GPIO_Port, ledred_Pin, ledr);
  HAL_GPIO_WritePin(ledgreen_GPIO_Port, ledgreen_Pin, ledg);
  HAL_GPIO_WritePin(ledblue_GPIO_Port, ledblue_Pin, ledb);
}

void function_mode1(){
  if (initialled1 == 1){
    initLED(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
    initialled1 = 0;
  }else{
    HAL_GPIO_TogglePin(ledbuiltin_GPIO_Port, ledbuiltin_Pin);
    HAL_GPIO_TogglePin(ledred_GPIO_Port, ledred_Pin);
    HAL_GPIO_TogglePin(ledgreen_GPIO_Port, ledgreen_Pin);
    HAL_GPIO_TogglePin(ledblue_GPIO_Port, ledblue_Pin);
  }

}

void function_mode2(){
  switch (mode2forled)
  {
  case state1mode2:
    initLED(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
    mode2forled = state2mode2;
    break;
  case state2mode2:
    initLED(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);
    mode2forled = state3mode2;
    break;
  case state3mode2:
    initLED(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);
    mode2forled = state1mode2;
    break; 
  default:
    break;
  }
}

void function_mode3(){
  switch (mode3forled)
  {
  case state1mode3:
    initLED(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
    mode3forled = state2mode3;
    break;
  case state2mode3:
    initLED(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
    mode3forled = state3mode3;
    break;
  case state3mode3:
    initLED(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_RESET);
    mode3forled = state4mode3;
    break; 
  case state4mode3:
    initLED(GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET);
    mode3forled = state1mode3;
    break;
  default:
    break;
  }
}

void changemodeled() {
  switch (stateofled)
  {
    case mode1:
      initialled1 = 1;
      stateofled = mode2;
      break;
    case mode2:
      stateofled = mode3;
      break;
    case mode3:
      stateofled = mode1;
      break;
    default:
      break;
  }
}

void SM_ledmode(void){
  switch (stateofled)
    {
    case mode1:
      function_mode1();
      break;
    case mode2:
      function_mode2();
      break;
    case mode3:
      function_mode3();
      break;
    default:
      break;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */

  if(htim == &htim3){
    SM_ledmode();
  }
}

void SM_button1(void){
  state_button1 = HAL_GPIO_ReadPin(button1_GPIO_Port, button1_Pin);
  pre_state_button1_t = state_button1_t;
  switch(state_button1_t){
    case RESET_BTN:
      state_button1_t = WAIT;
      break;
    case WAIT:
      if (state_button1 == GPIO_PIN_RESET){
        state_button1_t = ARMING;
      }
      break;
    case ARMING:
      t_0_s1 = HAL_GetTick();
      state_button1_t = ARMED;
      break;
    case ARMED:
      t_s1 = HAL_GetTick();
      t_diff_s1 = t_s1 - t_0_s1;
      if (state_button1 == GPIO_PIN_SET){
        state_button1_t = RESET_BTN;
      }
      if(t_diff_s1 > bounce_delay_s){
        state_button1_t = DRAWN;
      }
      break;
    case DRAWN:
      t_s1 = HAL_GetTick();
      t_diff_s1 = t_s1 - t_0_s1;
      if (state_button1 == GPIO_PIN_SET){
        state_button1_t = TRIGGER;
      }
      if (t_diff_s1 > hold_delay_s){
        state_button1_t = HOLD;
      }
      break;
    case TRIGGER:
      state_button1_t = RESET_BTN;
      break;
    case HOLD:
      state_button1_t = LOW_WAIT;
      break;
    case LOW_WAIT:
      if (state_button1 == GPIO_PIN_SET){
        t_s1 = HAL_GetTick();
        t_press_s1 = t_s1 - t_0_s1;
        state_button1_t = RESET_BTN;
      }
      break;
    default:
    break;
  }
}

void SM_button2(void){
  state_button2 = HAL_GPIO_ReadPin(button2_GPIO_Port, button2_Pin);
  pre_state_button2_t = state_button2_t;
  switch(state_button2_t){
    case RESET_BTN:
      state_button2_t = WAIT;
      break;
    case WAIT:
      if (state_button2 == GPIO_PIN_RESET){
        state_button2_t = ARMING;
      }
      break;
    case ARMING:
      t_0_s2 = HAL_GetTick();
      state_button2_t = ARMED;
      break;
    case ARMED:
      t_s2 = HAL_GetTick();
      t_diff_s2 = t_s2 - t_0_s2;
      if (state_button2 == GPIO_PIN_SET){
        state_button2_t = RESET_BTN;
      }
      if(t_diff_s2 > bounce_delay_s){
        state_button2_t = DRAWN;
      }
      break;
    case DRAWN:
      t_s2 = HAL_GetTick();
      t_diff_s2 = t_s2 - t_0_s2;
      if (state_button2 == GPIO_PIN_SET){
        state_button2_t = TRIGGER;
      }
      if (t_diff_s2 > hold_delay_s){
        state_button2_t = HOLD;
      }
      break;
    case TRIGGER:
      state_button2_t = RESET_BTN;
      break;
    case HOLD:
      state_button2_t = LOW_WAIT;
      break;
    case LOW_WAIT:
      if (state_button2 == GPIO_PIN_SET){
        t_s2 = HAL_GetTick();
        t_press_s2 = t_s2 - t_0_s2;
        state_button2_t = RESET_BTN;
      }
      break;
    default:
    break;
  }
}

void print_uart(char *message){
  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 100);
}

void button1_pressed(void){
  if (HAL_TIM_Base_Stop_IT(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Base_DeInit(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if(tim3period == 999){
    tim3period = 20000 - 1;
  }else{
    tim3period -= 1000;
  }
  MX_TIM3_Init();
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sprintf((char*)message, "Period of timer 3: %ld\r\n", tim3period);
  print_uart((char*)message);
  print_uart("BUTTON1_TRIGGERED!\r\n");
}

void button1_holdtime(void){
  if(ishold1 != 0){
    uint8_t divsion_time = t_press_s1 / 200;
    uint16_t decrease_time = divsion_time * 1000;
    if (HAL_TIM_Base_Stop_IT(&htim3) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIM_Base_DeInit(&htim3) != HAL_OK)
    {
      Error_Handler();
    }
    if((int)tim3period - (int)decrease_time > 0){
      tim3period -= decrease_time;
    }else{
      tim3period = 20000 - 1;
    }
    MX_TIM3_Init();
    if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
    {
      Error_Handler();
    }
    sprintf((char*)message, "Period of timer 3: %ld\r\n", tim3period);
    print_uart((char*)message);
    sprintf((char*)message, "Time pressing button: %ld\r\n", t_press_s1);
    print_uart((char*)message);
  }      
  ishold1 = 0;
}

void button2_pressed(void){
  changemodeled();
  print_uart("BUTTON2_TRIGGERED!\r\n");
}

void button2_holdtime(void){
  if(ishold2 != 0){
    uint8_t divsion_time = t_press_s2 / 200;
    uint16_t increase_time = divsion_time * 1000;
    if (HAL_TIM_Base_Stop_IT(&htim3) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIM_Base_DeInit(&htim3) != HAL_OK)
    {
      Error_Handler();
    }
    if(tim3period + increase_time > 20000 - 1){
      tim3period = 20000 - 1;
    }else{
      tim3period += increase_time;
    }  
    MX_TIM3_Init();
    if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
    {
      Error_Handler();
    }
    sprintf((char*)message, "Period of timer 3: %ld\r\n", tim3period);
    print_uart((char*)message);
    sprintf((char*)message, "Time pressing button: %ld\r\n", t_press_s2);
    print_uart((char*)message);
  }      
  ishold2 = 0;
}

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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    Error_Handler();
  }  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    SM_button1();
    SM_button2();
    if (state_button1_t == TRIGGER){
      button1_pressed();
    }
    if (state_button1_t == HOLD){
      print_uart("BUTTON1_HOLD!\r\n");
      ishold1 = 1;
    }
    if (state_button1_t == RESET_BTN){
      button1_holdtime();
    }

    if (state_button2_t == TRIGGER){
      button2_pressed();
    }
    if (state_button2_t == HOLD){
      print_uart("BUTTON2_HOLD!\r\n");
      ishold2 = 1;
    }
    if (state_button2_t == RESET_BTN){
      button2_holdtime();
    }

    if (DEBUG){
      if(pre_state_button1_t != state_button1_t){
        sprintf((char*)message, "State Of Button1: %d\r\n", state_button1_t);
        print_uart((char*)message);
      }

      if(pre_state_button2_t != state_button2_t){
        sprintf((char*)message, "State Of Button2: %d\r\n", state_button2_t);
        print_uart((char*)message);
      }
    }
    
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM3_Init 1 */
  htim3.Init.Prescaler = tim3scaler;
  htim3.Init.Period = tim3period;
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ledbuiltin_GPIO_Port, ledbuiltin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ledred_Pin|ledgreen_Pin|ledblue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : lebbuiltin_Pin */
  GPIO_InitStruct.Pin = ledbuiltin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ledbuiltin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : button1_Pin button2_Pin */
  GPIO_InitStruct.Pin = button1_Pin|button2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ledred_Pin ledgreen_Pin ledblue_Pin */
  GPIO_InitStruct.Pin = ledred_Pin|ledgreen_Pin|ledblue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  { 
    print_uart("ERROR!\r\n");
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
