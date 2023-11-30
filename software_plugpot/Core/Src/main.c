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
#include "digitalsynth_plug_i2c.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//I2C defines
#define TXSIZE 13
#define RXSIZE  3
#define TOGGLEDELAY     1000    //1s -   num/ms/s
#define TOGGLEDELAY2     10      //10ms -   num/ms/s

#define ADC_BUF_LEN 2*8


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

//I2C global variables
uint8_t TxData[TXSIZE] = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1};
uint8_t txcount = 0;
uI2CPlugPotControls sCont;
uint32_t u32LastToggleTick = 0;
HAL_StatusTypeDef status;

//adc
uint8_t arru8adc_buf[ADC_BUF_LEN];
uint8_t * pu8adc_bufcurr;

//SW
uint32_t u32SWReadTick;
GPIO_TypeDef* arrSWGPIOPort[8] = 
{SW1p_GPIO_Port, SW2p_GPIO_Port, SW3p_GPIO_Port, SW4p_GPIO_Port, SW5p_GPIO_Port, SW6p_GPIO_Port, SW7p_GPIO_Port, SW8p_GPIO_Port};
uint16_t arrSWPin[8] = 
{SW1p_Pin, SW2p_Pin, SW3p_Pin, SW4p_Pin, SW5p_Pin, SW6p_Pin, SW7p_Pin, SW8p_Pin};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void i2c_reset();
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  //JUST FOR TESTING
  for(int i = 0; i < 13; i++){
    sCont.au8I2CPlugPotByteAccess[i] = TxData[i];
  }
  
  //adc
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)arru8adc_buf, ADC_BUF_LEN);
  
  
  //HAL_TIM_Base_Start_IT(&htim14);
  u32LastToggleTick = HAL_GetTick();
  u32SWReadTick = HAL_GetTick();
  uint8_t u8ErrorFlag = 0;
  uint32_t u32ErrorTick = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    //I2C communication
    status = HAL_I2C_Slave_Transmit(&hi2c1, sCont.au8I2CPlugPotByteAccess, TXSIZE,30);
    
    //if the line is busy, reset
    if( (status != HAL_OK) & (u8ErrorFlag==0) ){
      u8ErrorFlag = 1;
      u32ErrorTick = HAL_GetTick();
    } 
    if( ( status != HAL_OK ) & ( HAL_GetTick() > (u32ErrorTick+250) ) & (u8ErrorFlag==1) ){
      i2c_reset(&hi2c1);
      u8ErrorFlag = 0;

    }
      
    //toggle status led    
    while ((HAL_GetTick() - u32LastToggleTick) > TOGGLEDELAY){
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
      u32LastToggleTick = HAL_GetTick();
    }
    
    
    sCont.sI2CPlugPotControl.u1SW1 = (HAL_GPIO_ReadPin(arrSWGPIOPort[0], arrSWPin[0]) == GPIO_PIN_SET);
    
    //SW read
    while ((HAL_GetTick() - u32SWReadTick) > TOGGLEDELAY2){
      sCont.sI2CPlugPotControl.u1SW1 = (HAL_GPIO_ReadPin(arrSWGPIOPort[0], arrSWPin[0]) == GPIO_PIN_SET);
      sCont.sI2CPlugPotControl.u1SW2 = (HAL_GPIO_ReadPin(arrSWGPIOPort[1], arrSWPin[1]) == GPIO_PIN_SET);
      sCont.sI2CPlugPotControl.u1SW3 = (HAL_GPIO_ReadPin(arrSWGPIOPort[2], arrSWPin[2]) == GPIO_PIN_SET);
      sCont.sI2CPlugPotControl.u1SW4 = (HAL_GPIO_ReadPin(arrSWGPIOPort[3], arrSWPin[3]) == GPIO_PIN_SET);
      sCont.sI2CPlugPotControl.u1SW5 = (HAL_GPIO_ReadPin(arrSWGPIOPort[4], arrSWPin[4]) == GPIO_PIN_SET);
      sCont.sI2CPlugPotControl.u1SW6 = (HAL_GPIO_ReadPin(arrSWGPIOPort[5], arrSWPin[5]) == GPIO_PIN_SET);
      sCont.sI2CPlugPotControl.u1SW7 = (HAL_GPIO_ReadPin(arrSWGPIOPort[6], arrSWPin[6]) == GPIO_PIN_SET);
      sCont.sI2CPlugPotControl.u1SW8 = (HAL_GPIO_ReadPin(arrSWGPIOPort[7], arrSWPin[7]) == GPIO_PIN_SET);
      u32SWReadTick = HAL_GetTick();
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 38;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STATUS_LED_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1p_Pin */
  GPIO_InitStruct.Pin = SW1p_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1p_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW2p_Pin */
  GPIO_InitStruct.Pin = SW2p_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW2p_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW3p_Pin SW4p_Pin SW5p_Pin SW6p_Pin
                           SW7p_Pin SW8p_Pin */
  GPIO_InitStruct.Pin = SW3p_Pin|SW4p_Pin|SW5p_Pin|SW6p_Pin
                          |SW7p_Pin|SW8p_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void i2c_reset(I2C_HandleTypeDef *hi2c){
  SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
  HAL_Delay( 10 );
  CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
  MX_I2C1_Init();
}

//adc double buffering
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
  sCont.sI2CPlugPotControl.u8Plugpot[0] = arru8adc_buf[0];
  sCont.sI2CPlugPotControl.u8Plugpot[1] = arru8adc_buf[1];
  sCont.sI2CPlugPotControl.u8Plugpot[2] = arru8adc_buf[2];
  sCont.sI2CPlugPotControl.u8Plugpot[3] = arru8adc_buf[3];
  sCont.sI2CPlugPotControl.u8Plugpot[4] = arru8adc_buf[4];
  sCont.sI2CPlugPotControl.u8Plugpot[5] = arru8adc_buf[5];
  sCont.sI2CPlugPotControl.u8Plugpot[6] = arru8adc_buf[6];
  sCont.sI2CPlugPotControl.u8Plugpot[7] = arru8adc_buf[7];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  sCont.sI2CPlugPotControl.u8Plugpot[0] = arru8adc_buf[8];
  sCont.sI2CPlugPotControl.u8Plugpot[1] = arru8adc_buf[9];
  sCont.sI2CPlugPotControl.u8Plugpot[2] = arru8adc_buf[10];
  sCont.sI2CPlugPotControl.u8Plugpot[3] = arru8adc_buf[11];
  sCont.sI2CPlugPotControl.u8Plugpot[4] = arru8adc_buf[12];
  sCont.sI2CPlugPotControl.u8Plugpot[5] = arru8adc_buf[13];
  sCont.sI2CPlugPotControl.u8Plugpot[6] = arru8adc_buf[14];
  sCont.sI2CPlugPotControl.u8Plugpot[7] = arru8adc_buf[15];
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
