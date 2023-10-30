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
#include <stdbool.h>
#include "math.h"

#include "digitalsynth_slide_i2c.h"
#include "digitalsynth_slide_butt.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//WS2812b defines
#define MAX_LED 25                 //25
#define USE_BRIGHTNESS 0
#define PI 3.14159265
#define WS2812B_RST_TIME 20     //50us<

//I2C defines
#define TXSIZE 12
#define RXSIZE  3
#define TESTING_LED_NUM 2
#define TICKDELAY       20          //20ms - num/ms/s
#define TOGGLEDELAY     1000        //1s -   num/ms/s
#define BUTTDELAY       10          //10ms - num/ms/s


//Button defines
#define ROWSIZE 5
#define COLSIZE 5

    
    
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim17_ch1;

/* USER CODE BEGIN PV */

//WS2812b global variables
uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];    //for brightness
int datasentflag=0;
  
uint8_t green = 0;
uint8_t red = 0;
uint8_t blue = 0;

uint16_t pwmData[(24*MAX_LED)+WS2812B_RST_TIME];

//I2C global variables
uint8_t RxData[RXSIZE];
uint8_t rxcount = 0;
int countAddr = 0;
//int is_first_recvd = 0;
//int countrxcplt = 0;
uint8_t TxData[TXSIZE] = {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1};
uint8_t txcount = 0;
int counterror = 0;
uI2CSlidePotControls sCont;
uint32_t u32LastReadTick = 0;
uint32_t u32LastToggleTick = 0;
uint32_t u32LastButtTick = 0;
HAL_StatusTypeDef status;

//Button
uint8_t aKey_State[ROWSIZE][COLSIZE] = {{0, 0, 0, 0, 0}, 
                                        {0, 0, 0, 0, 0},
                                        {0, 0, 0, 0, 0}, 
                                        {0, 0, 0, 0, 0}, 
                                        {0, 0, 0, 0, 0}};

GPIO_PinState testing_gpio = GPIO_PIN_RESET;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void send(uint8_t Green, uint8_t Red, uint8_t Blue);
void Set_LED(int LEDnum, int Red, int Green, int Blue);
void Set_Brightness(int brightness);
void WS2812_Send (void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
  //HAL_TIM_PWM_Stop_DMA(&htim17, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop_DMA(&htim17, TIM_CHANNEL_1);
  datasentflag=1;  
}




#pragma optimize=none
void send(uint8_t Green, uint8_t Red, uint8_t Blue){
  
  //lol
  uint32_t data1 = 0;
  uint32_t data3 = 0;
  
  uint32_t data = (Green<<16) | (Red<<8) | Blue;
    
  uint16_t pwmData2[92];
  int indx = 0;
  
  //data1 
  for(int i=23; i>=0;i--){
    if(data1&(1<<i))
      pwmData2[indx] = 13;          //12.8
    else
      pwmData2[indx] = 6;           //6.4
    
    indx++;
  }
    
  //62.5ns steps, 
  for(int i=23; i>=0;i--){
    if(data&(1<<i))
      pwmData2[indx] = 13;          //12.8
    else
      pwmData2[indx] = 6;           //6.4
    
    indx++;
  }
  
  //data3
  for(int i=23; i>=0;i--){
    if(data3&(1<<i))
      pwmData2[indx] = 13;          //12.8
    else
      pwmData2[indx] = 6;           //6.4
    
    indx++;
  }
  
  for (int i=0; i<20; i++){
    pwmData2[indx] = 0;
    indx++;
  }
  
  HAL_TIMEx_PWMN_Start_DMA(&htim17, TIM_CHANNEL_1, (uint32_t *)pwmData2, indx);
  //HAL_TIM_PWM_Start_DMA(&htim17, TIM_CHANNEL_1,(uint32_t *) pwmData, 124);
  while (!datasentflag){};
  datasentflag = 0;
}

void Set_LED(int LEDnum, int Red, int Green, int Blue){
  LED_Data[LEDnum][0] = LEDnum;
  LED_Data[LEDnum][1] = Green;
  LED_Data[LEDnum][2] = Red;
  LED_Data[LEDnum][3] = Blue;
}

//brightness állítás linearitása miatt 
void Set_Brightness(int brightness)
{
#if USE_BRIGHTNESS
  if(brightness > 45)
    brightness = 45;
  for(int i = 0; i < MAX_LED; i++){
    LED_Mod[i][0] = LED_Data[i][0];
    for(int j = 1; j < 4; j++){
      float angle = 90*brightness;      //in degrees
      angle = angle * PI / 180;         //in rad
      LED_Mod[i][j] = (LED_Data[i][j]/(tan(angle)));
    }
  }
#endif
}

#pragma optimize=none
void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
#if USE_BRIGHTNESS
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else
		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = 13;  //12.8
			}

			else pwmData[indx] = 6;  //6.4

			indx++;
		}

	}

	for (int u=0; u<WS2812B_RST_TIME; u++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	//HAL_TIM_PWM_Start_DMA(&htim17, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
        HAL_TIMEx_PWMN_Start_DMA(&htim17, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
	while (!datasentflag){};
	datasentflag = 0;
}


//*******
//I2C functions
//*******

/*
void process_data(){
  //TODO
  //memcpy(mainbuf, RXDATA+1, rxcount-1);
};

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c){
  HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode){
  if(TransferDirection == I2C_DIRECTION_TRANSMIT){
    rxcount = 0;
    countAddr++;
    HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_FIRST_FRAME);
  }
  else{
    txcount = 0;
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, TxData+txcount, 1, I2C_FIRST_FRAME);
  }
*/
/*  
  if(TransferDirection == I2C_DIRECTION_TRANSMIT){
    if(is_first_recvd == 0){
      rxcount = 0;
      countAddr++;
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_FIRST_FRAME);
    }
  }
  else{
    txcount = 0;
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, TxData+txcount, 1, I2C_FIRST_FRAME);
  }
*/
/*
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c){
  txcount++;
  HAL_I2C_Slave_Seq_Transmit_IT(hi2c, TxData+txcount, 1, I2C_NEXT_FRAME);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
  rxcount++;
  if(rxcount < RXSIZE){
    if(rxcount == RXSIZE - 1){
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_LAST_FRAME);
    }
    else{
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_NEXT_FRAME);
    }
  }
  
  if(rxcount == RXSIZE){
    process_data();
  }
  //countrxcplt++;
*/
  /*
  if(is_first_recvd == 0){
    rxcount++;
    is_first_recvd = 1;
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+rxcount, RxData[0], I2C_LAST_FRAME);
  }
  else{
    rxcount = rxcount+RxData[0];
    is_first_recvd=0;
    process_data();
  }
*/
/*
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
  counterror++;
  uint32_t errorcode = HAL_I2C_GetError(hi2c);
  if(errorcode == 4){
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);    //clear AF Flag
    if(txcount == 0){
      process_data();
    }
    else{       //error while slave is transmitting
      txcount = txcount - 1;
    }
    process_data();
  }
  HAL_I2C_EnableListen_IT(hi2c);
  
  //testing with LED
  //Set_LED(TESTING_LED_NUM, 1, 0, 0);
  //WS2812_Send();
}
*/

void i2c_reset(I2C_HandleTypeDef *hi2c){
  SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
  HAL_Delay( 10 );
  CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
  //MX_GPIO_Init();
  MX_I2C1_Init();
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM17_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  
  //testing pwm timer
  //uint32_t ff_var = 20; 
  //uint32_t var0 = 1;  
  //HAL_TIM_PWM_Start_DMA(&htim17, TIM_CHANNEL_1, &ff_var, 1);
  
  //Set_LED(0, 0, 1, 0);
  Set_LED(1, 0, 0, 1);
  //Set_LED(2, 255, 255, 255);
  //Set_LED(3, 255, 255, 255);
  //Set_LED(4, 255, 255, 255);
  //Set_LED(5, 255, 255, 255);
  //Set_LED(6, 255, 255, 255);
  //Set_LED(7, 255, 255, 255);
  //Set_LED(8, 255, 255, 255);
  //Set_LED(9, 255, 255, 255);
  //Set_LED(10, 255, 255, 255);
  //Set_LED(11, 255, 255, 255);
  //Set_LED(12, 255, 255, 255);
  //Set_LED(13, 255, 255, 255);
  //Set_LED(14, 255, 255, 255);
  //Set_LED(15, 255, 255, 255);
  //Set_LED(16, 255, 255, 255);
  //Set_LED(17, 255, 255, 255);
  //Set_LED(18, 255, 255, 255);
  //Set_LED(19, 255, 255, 255);
  //Set_LED(22, 255, 255, 255);
  //Set_LED(21, 255, 255, 255);
  //Set_LED(22, 255, 255, 255);
  //Set_LED(23, 255, 255, 255);
  //Set_LED(24, 255, 255, 255);

  
  //Set_Brightness(20);
  WS2812_Send();
  
  //uint8_t array[] = {0, 1, 2, 4, 8, 16, 32, 64, 128};
  //send(green, red, blue);
  /*
  for(int j = 0; j < 3; j++){
    red = 0;
    green = 0;
    blue = 0;      
    for(int k = 0; k < 9; k++){
      switch(j){
      case 0: 
        green=array[k];
        break;
      case 1:
        red=array[k];
        break;
      case 2:
        blue=array[k];
        break;
      default:
        break;
      }
      send(green, red, blue);
      HAL_Delay(1000);      
    }
  }
*/

  //I2C setup
  //if( HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK){
  //  Error_Handler();
  //}
  
  
  //HAL_I2C_EnableListen_IT(&hi2c1);
  /*
  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK){
    //Transfer error in reception process
    Error_Handler();
  }*/
  
  sCont.sI2CSlidePotControl.u2ButtWaveform1 = 0;
  
  for(int i = 0; i < 12; i++)
    sCont.au8I2CSlidePotByteAccess[i] = 0;
  

  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_Delay(1000);
  
  HAL_TIM_Base_Start_IT(&htim14);
  u32LastReadTick = HAL_GetTick();
  u32LastToggleTick = HAL_GetTick();
  u32LastButtTick = HAL_GetTick();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //toggle pin things
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    //HAL_Delay(1000);
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
    //HAL_Delay(1000);
    
    /*
    if (Xfer_Complete ==1){
      HAL_Delay(10);
      //##- Put I2C peripheral in listen mode process ###########################
      if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK){
        // Transfer error in reception process
        Error_Handler();
      }
      Xfer_Complete =0;
    }

*/


    //I2C communication
    status = HAL_I2C_Slave_Transmit(&hi2c1, sCont.au8I2CSlidePotByteAccess, TXSIZE,100);
    if(status != HAL_OK)
      i2c_reset(&hi2c1);
      /*
      if((HAL_GetTick() - u32LastReadTick) > TOGGLEDELAY){
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
      }*/
      
    
    
    while ((HAL_GetTick() - u32LastToggleTick) > TOGGLEDELAY){
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
      u32LastToggleTick = HAL_GetTick();
    }
    
    //button
    //getNewState();
    //setButtonState();
    //refreshOldState();
    

    
    /*
    if(hi2c1.State == HAL_I2C_STATE_LISTEN){
      Set_LED(3, 0, 1, 0);
    }
    else {
      Set_LED(3, 1, 0, 0);
    }
    
    if(aKey_State[0][0] == 1){
      Set_LED(4, 1, 0, 0);
    }
    else{
      Set_LED(4, 0, 0, 0);
    }
    
    Set_LED(TESTING_LED_NUM, RxData[0], RxData[1], RxData[2]);
    */
    
    
    //testing buttons
    //noooo
    //HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);
    //testing_gpio = HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin);
    /*if (testing_gpio == GPIO_PIN_SET){
      Set_LED(4, 1, 0, 0);
    }
    else{
      Set_LED(4, 0, 0, 0);
    }*/
    //HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);

    
    //nooo
    //HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);
    //testing_gpio = HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin);
    //HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);
    
    //noooo
    //HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_SET);
    //testing_gpio = HAL_GPIO_ReadPin(R1_GPIO_Port, R1_Pin);
    //HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);
    
    //yaaaaas
    //HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);
    //testing_gpio = HAL_GPIO_ReadPin(R1_GPIO_Port, R1_Pin);
    //HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_SET);
    /*
    HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);
    testing_gpio = HAL_GPIO_ReadPin(R1_GPIO_Port, R1_Pin);
    if (testing_gpio == GPIO_PIN_RESET){
      Set_LED(4, 1, 0, 0);
    }
    else{
      Set_LED(4, 0, 0, 0);
    }
    HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_SET);
*/
    
    if ((HAL_GetTick() - u32LastButtTick) > BUTTDELAY){
      switch(sCont.sI2CSlidePotControl.u2ButtWaveform1){
        case 0:
          Set_LED(0, 0, 0, 0);
          break;
        case 1:
          Set_LED(0, 1, 0, 0);
          break;
        case 2:
          Set_LED(0, 0, 1, 0);
          break;
        case 3:
          Set_LED(0, 0, 0, 1);
          break;
        default:
          Set_LED(0, 0, 0, 0);        
      }
      WS2812_Send();
      u32LastButtTick = HAL_GetTick();
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
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
  hi2c1.Init.OwnAddress1 = 36;
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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 16-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 10000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 20-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim17, TIM_CHANNEL_1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, C5_Pin|C1_Pin|C2_Pin|C3_Pin
                          |C4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : C5_Pin C1_Pin C2_Pin C3_Pin
                           C4_Pin */
  GPIO_InitStruct.Pin = C5_Pin|C1_Pin|C2_Pin|C3_Pin
                          |C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : STATUS_LED_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R5_Pin R3_Pin R2_Pin R1_Pin */
  GPIO_InitStruct.Pin = R5_Pin|R3_Pin|R2_Pin|R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : R4_Pin */
  GPIO_InitStruct.Pin = R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(R4_GPIO_Port, &GPIO_InitStruct);

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
