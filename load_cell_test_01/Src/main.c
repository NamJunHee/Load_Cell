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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hx711.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int test =0;

int Load_cell_number = 8;
int Load_cell_Txbuffer_number = 35;
uint8_t Load_cell_ID[8] = {0,1,2,3,4,5,6,7};

int value[8]  = {0,};
int value_[8] = {0,};
int Value_Gain[8] = {4,32,32,32,32,32,32,32};

HX711 data_0;
HX711 data_1;
HX711 data_2;
HX711 data_3;
HX711 data_4;
HX711 data_5;
HX711 data_6;
HX711 data_7;

#define HX711_NUM 8
HX711 HX711_data[HX711_NUM];

uint8_t Tx_buffer[35] = {0,};
int counter = 2;

int8_t Checksum  = 0;
int8_t Checksum_ = 0;

uint8_t Rxbuffer[11] = {0,};
uint8_t Value_Gain_buffer[8] = {0,};
uint8_t Rx_Checksum_ = 0;
uint8_t Rx_Checksum  = 0 ;
int Rx_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  
  HAL_UART_Receive_IT(&huart2, (uint8_t*)Rxbuffer,11);
  
  /* USER CODE BEGIN 2 */
  
  HX711_data[0].gpioData = GPIOB;
  HX711_data[0].pinData  = GPIO_PIN_1;
  HX711_data[0].gpioSck  = GPIOB;
  HX711_data[0].pinSck   = GPIO_PIN_0;
  HX711_data[0].gain     = Value_Gain[0];
  
  HX711_data[1].gpioData = GPIOB;
  HX711_data[1].pinData  = GPIO_PIN_2;
  HX711_data[1].gpioSck  = GPIOB;
  HX711_data[1].pinSck   = GPIO_PIN_0;
  HX711_data[1].gain     = Value_Gain[1];
  
  HX711_data[2].gpioData = GPIOB;
  HX711_data[2].pinData  = GPIO_PIN_4;
  HX711_data[2].gpioSck  = GPIOB;
  HX711_data[2].pinSck   = GPIO_PIN_0;
  HX711_data[2].gain     = Value_Gain[2];
  
  HX711_data[3].gpioData = GPIOB;
  HX711_data[3].pinData  = GPIO_PIN_5;
  HX711_data[3].gpioSck  = GPIOB;
  HX711_data[3].pinSck   = GPIO_PIN_0;
  HX711_data[3].gain     = Value_Gain[3];
  
  HX711_data[4].gpioData = GPIOB;
  HX711_data[4].pinData  = GPIO_PIN_6;
  HX711_data[4].gpioSck  = GPIOB;
  HX711_data[4].pinSck   = GPIO_PIN_0;
  HX711_data[4].gain     = Value_Gain[4];
  
  HX711_data[5].gpioData = GPIOB;
  HX711_data[5].pinData  = GPIO_PIN_7;
  HX711_data[5].gpioSck  = GPIOB;
  HX711_data[5].pinSck   = GPIO_PIN_0;
  HX711_data[5].gain     = Value_Gain[5];
  
  HX711_data[6].gpioData = GPIOB;
  HX711_data[6].pinData  = GPIO_PIN_8;
  HX711_data[6].gpioSck  = GPIOB;
  HX711_data[6].pinSck   = GPIO_PIN_0;
  HX711_data[6].gain     = Value_Gain[6];
  
  HX711_data[7].gpioData = GPIOB;
  HX711_data[7].pinData  = GPIO_PIN_9;
  HX711_data[7].gpioSck  = GPIOB;
  HX711_data[7].pinSck   = GPIO_PIN_0;
  HX711_data[7].gain     = Value_Gain[7];
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    //==================Value_Gain_Rx================//
    if(Rxbuffer[0] == 0xFF && Rxbuffer[1] == 0xFF)
    {
      for(int i = 2; i < 10; i++)
      {
        Rx_Checksum_ += Rxbuffer[i];
      }
      Rx_Checksum = ~Rx_Checksum_;
      
      if(Rx_Checksum == Rxbuffer[10])
      {
        for(int i = 2; i < 10; i++)
        {
          Value_Gain_buffer[i-2] = Rxbuffer[i];
        }
      }
    }
  
    if(Value_Gain_buffer[0] != 0)
    {
      for(int i = 0; i < 8; i++)
      {
        Value_Gain[i] = Value_Gain_buffer[i];
      }
    }
    //==========================================//
    
    HAL_Delay(1);
  
    HX711_Init(HX711_data[0]);
    value_[0] = HX711_Value(HX711_data[0]);

    HX711_Init(HX711_data[1]);
    value_[1] = HX711_Value(HX711_data[1]);
    
    HX711_Init(HX711_data[2]);    
    value_[2] = HX711_Value(HX711_data[2]);
    
    HX711_Init(HX711_data[3]);
    value_[3] = HX711_Value(HX711_data[3]);
    
    HX711_Init(HX711_data[4]);
    value_[4] = HX711_Value(HX711_data[4]);
    
    HX711_Init(HX711_data[5]);
    value_[5] = HX711_Value(HX711_data[5]);
    
    HX711_Init(HX711_data[6]);
    value_[6] = HX711_Value(HX711_data[6]);
    
    HX711_Init(HX711_data[7]);
    value_[7] = HX711_Value(HX711_data[7]);
        
    for(int i = 0; i < Load_cell_number; i++)
    {
      value[i] = value_[i];
      
      if(value[i] > 16000000)
        value[i] = value[i] / 2;
    }
    
    //=======================LOAD_CELL_DATA_TX========================//
    Tx_buffer[0] = 0xFF;
    Tx_buffer[1] = 0xFF;    
    
    for(int i = 0; i < Load_cell_number; i++)
    {
      Tx_buffer[counter] = Load_cell_ID[i];
      Tx_buffer[counter+1] = value[i] & 0xFF;
      Tx_buffer[counter+2] = (value[i] >> 8) & 0xFF;
      Tx_buffer[counter+3] = (value[i] >> 16) & 0xFF;
      counter += 4;
    }
    counter = 2;
    
    for(int i = 2; i < Load_cell_Txbuffer_number - 1 ; i++)
    {
      Checksum_ += Tx_buffer[i];
    }
    
    Checksum = ~Checksum_;
    Tx_buffer[34] = Checksum;
    
    Checksum  = 0;
    Checksum_ = 0;
    
    HAL_UART_Transmit(&huart2, Tx_buffer,35,100);     
    //===============================================================//
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  Rx_counter++;
   
  HAL_UART_Receive_IT(&huart2, (uint8_t*)Rxbuffer,11);
}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 90-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB4 PB5 
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int HX711_cnt=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

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
