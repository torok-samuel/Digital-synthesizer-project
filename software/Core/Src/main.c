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
#include <math.h>
#include <stdio.h>
#include <string.h>

//project headers
#include "digitalsynth_main_i2c.h"
//#include "samples.h"
#include "digitalsynth_dsp.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 256;

#define TICKDELAY       10        //10ms - num/ms/s
#define ONEBITTIMEI2C   1000/48000        //1 bit time for i2c

#define SLIDEPOT_ADDR 0x12<<1
#define PLUGPOT_ADDR 0x13<<1
#define KEYPOT_ADDR 0x14<<1

#define SLIDEPOT_RXSIZE 12
#define PLUGPOT_RXSIZE 13
#define KEY_RXSIZE 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

//HAL_StatusTypeDef status;
//hi2c1->Instance->SR1&

//PCM1753 SPI setup
//R16-0x10FF, R18-0x1200, 0x1300, 0x1404, 0x1600
const uint16_t SPI_SETUP_REG[] = {
0x10FF, //R16
0x11FF, //R17
0x1200, //R18
0x1300, //R19
0x1405,         //R20, LEFT - 0X1405, I2S - 0x1404, RIGHT - 0x1400
0x1600  //R22
};

//variables
//I2c
HAL_StatusTypeDef status;
uint8_t u8TimerI2C_counter;
uint16_t slaveADDR;
uint8_t * RxData;
uI2CMainControls ui2cControl;
uint8_t i2cRxSize;

uint32_t u32LastReadTick = 0;
//dsp
int32_t arri32AudioBuffer2[2048];
uint16_t u16WaveformNum = 0;
uint8_t u8Waveform = 0;
uint32_t u32AudioAmp = 0;
uint8_t u8WaveformFlag = 0;
uint8_t u8Range = 4;
uint8_t u8PButton = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void ll_spi_setup();
void ll_send_data();
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s1);
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s1);
void i2c_1bitcycle();
void reset_i2c(I2C_HandleTypeDef *hi2c);
void i2c_rst2();
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
  MX_I2C1_Init();
  MX_I2S1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  

  //set nss high, no spi comm yet
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  //spi setup
  ll_spi_setup();
  
  //i2s dma tx start
  u8Range = 4;
  u8Waveform = ui2cControl.sI2CMainControl.u2ButtWaveform1;
  u8PButton = ui2cControl.sI2CMainControl.u8TestKey1;
  u32AudioAmp = 0x7FFFFFFF;
  singen_def(u32AudioAmp);
  sawgen_def(u32AudioAmp);
  trigen_def(u32AudioAmp);
  sqrgen_def(u32AudioAmp);
  dsp2(u8Range, u8PButton, u8Waveform, u32AudioAmp, arri32AudioBuffer2);
  status = HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t*)arri32AudioBuffer2, 2048);
  
  
  //I2C timer
  u8TimerI2C_counter = 0;
  //HAL_TIM_Base_Start_IT(&htim6);
  
  u32LastReadTick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if ((HAL_GetTick() - u32LastReadTick) > TICKDELAY){
      switch(u8TimerI2C_counter){
      case 0:
        //SlidePot
        RxData = &ui2cControl.au8I2CMainByteAccess[0];
        slaveADDR = SLIDEPOT_ADDR;
        i2cRxSize = SLIDEPOT_RXSIZE;
        break;
      case 1:
        //Plugpot
        RxData = &ui2cControl.au8I2CMainByteAccess[12];
        slaveADDR = PLUGPOT_ADDR;
        i2cRxSize = PLUGPOT_RXSIZE;      
        break;
      case 2:
        //Keyboard
        RxData = &ui2cControl.au8I2CMainByteAccess[25];
        slaveADDR = KEYPOT_ADDR;
        i2cRxSize = KEY_RXSIZE;  
        break;
      default:
        //SlidePot
        RxData = &ui2cControl.au8I2CMainByteAccess[0];
        slaveADDR = SLIDEPOT_ADDR;
        i2cRxSize = SLIDEPOT_RXSIZE;  
        break;
      }
      status = HAL_I2C_Master_Receive(&hi2c1, slaveADDR, RxData, i2cRxSize, 30);
      if(u8TimerI2C_counter >= 2){
        u8TimerI2C_counter = 0;
      }
      else{
        u8TimerI2C_counter++;
      }
    u32LastReadTick = HAL_GetTick();
    }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s1.Init.Standard = I2S_STANDARD_MSB;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_32B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PC1   ------> SPI2_MOSI
  PB10   ------> SPI2_SCK
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_Init 1 */
  
  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim6.Init.Prescaler = 16-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000-1;
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
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void ll_spi_setup(){
  //HAL_Delay(10);
  unsigned int SPI_SETUP_SIZE = sizeof(SPI_SETUP_REG)/sizeof(SPI_SETUP_REG[0]);
  //spi enable
  LL_SPI_Enable(SPI2);
  //send datas
  ll_send_data(SPI_SETUP_REG, SPI_SETUP_SIZE);
  
  //nss soft??

  
}

void ll_send_data(uint16_t* data, unsigned int size){
  //*********
  //SOURCES
  //https://hackaday.com/2022/10/24/bare-metal-stm32-setting-up-and-using-spi/
  //https://usermanual.wiki/Document/Reference20manualF446RE.1190689300/view
  //http://www.disca.upv.es/aperles/arm_cortex_m3/llibre/st/STM32F439xx_User_Manual/group__spi__ll__em__write__read.html
  //*********
  //sending "size" number of datas
  for(uint16_t i=0; i<size; i++)
  {  
    //set nss low to start communication
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

    //wait while transfer register is empty
    while(!(SPI2->SR & LL_SPI_SR_TXE));
    //Write data (8-16 bits) into SPI_DR
    //SPI2->DR = data[i];
    LL_SPI_TransmitData16(SPI2, data[i]);
  
    //wait while transfer register is empty
    while(!(SPI2->SR & LL_SPI_SR_TXE));
    //Wait for SPI_SR_BSY (status register: bus busy) to become false.
    while((SPI2->SR & LL_SPI_SR_BSY)); 
    //nss low, end of spi comm
    HAL_Delay(2);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(10);
  }
  //Clear overrun flag????
  HAL_Delay(10);
  
  
  //other ideas, mainly trash
  //while(LL_SPI_ReadReg(hi2s1.Instance->SR, LL_SPI_SR_TXE));
  //Write data (8-16 bits) into SPI_DR
  //LL_SPI_TransmitData16(hi2s1.Instance, data);
  
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s1){ 
  dsp2(u8Range, u8PButton, u8Waveform, u32AudioAmp, arri32AudioBuffer2);
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s1){
  dsp2(u8Range, u8PButton, u8Waveform, u32AudioAmp, &arri32AudioBuffer2[1024]);
}

void i2c_1bitcycle(){
  unsigned u32CycleReadTick = 0;
  //SCL=0
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  //wait 1 i2c bit time
  u32CycleReadTick = HAL_GetTick();
  while((HAL_GetTick() - u32CycleReadTick) > 1);
  //SCL=1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  //wait 1 i2c bit time
  u32CycleReadTick = HAL_GetTick();
  while((HAL_GetTick() - u32CycleReadTick) > 1);
}


void reset_i2c(I2C_HandleTypeDef *hi2c){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  //SCL
  HAL_GPIO_WritePin( GPIOB, GPIO_PIN_6, GPIO_PIN_SET );
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  //GPIO_InitStruct.Alternate = ;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  //SDA
  HAL_GPIO_WritePin( GPIOB, GPIO_PIN_7, GPIO_PIN_SET );
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  i2c_1bitcycle();
  
  uint32_t u32ClockCycles = 0;
  while(!((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET))){
  //while(READ_BIT(hi2c->Instance->SR2, I2C_SR2_BUSY)){
      i2c_1bitcycle();
      u32ClockCycles++;
      if( u32ClockCycles > 8 )
      {
        //TODO: hibakezelÚs
        break;
      }
  }
  
  // Start: SDA falledg while SCL High 
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(1);
  //Stop: SDA risedg while SCL High 
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(1);
  
  SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
  HAL_Delay( 10 );
  CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
  
  MX_GPIO_Init();
  MX_I2C1_Init();
}
//not working, the bitbanging...
void i2c_rst2()
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint8_t u8Cycles;
 
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 
  HAL_Delay( 1u );
  if( LL_GPIO_IsInputPinSet( GPIOB, LL_GPIO_PIN_6 ) )  // SCL == 1
  {
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);  // SCL = 0
    HAL_Delay( 1u );
  }
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);  // SDA = 1
  HAL_Delay( 1u );
 
  for( u8Cycles = 0u; u8Cycles < 9u; u8Cycles++ )
  {
    // If the slave released SDA
    if( LL_GPIO_IsInputPinSet( GPIOB, LL_GPIO_PIN_7 ) )  // SDA == 1
    {
      break;
    }
    else  // slave currently pulls the SDA line
    {
      // Generate one SCL cycle
      LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);  // SCL = 1
      HAL_Delay( 1u );
      LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);  // SCL = 0
      HAL_Delay( 1u );
    }
  }
 
  // Generate stop condition
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);  // SDA = 0
  HAL_Delay( 1u );
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);  // SCL = 1
  HAL_Delay( 1u );
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);  // SDA = 1
  HAL_Delay( 1u );
  
  //MX_GPIO_Init();
  //MX_I2C1_Init();
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
