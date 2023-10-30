//description
/*
      (+) Receive in master mode an amount of data in non-blocking mode using HAL_I2C_Master_Receive_IT()
      (+) At reception end of transfer, HAL_I2C_MasterRxCpltCallback() is executed and user can
           add his own code by customization of function pointer HAL_I2C_MasterRxCpltCallback()
*/

//typedef

//include
//#include "main.h"
//#include "digitalsynth_main_i2c.h"
//#include "stm32f4xx.h"       
//#include "stm32f4xx_hal_gpio.h"

//variables




/*
//include
#include "main.h"
#include "digitalsynth_main_i2c.h"

//defines
#define SLIDEPOT_ADDR 0x12<<1
#define PLUGPOT_ADDR 0x13<<1
#define KEYPOT_ADDR 0x14<<1

#define SLIDEPOT_RXSIZE 12
#define PLUGPOT_RXSIZE 13
#define KEY_RXSIZE 2


//variables
extern TIM_HandleTypeDef htim6;
extern I2C_HandleTypeDef hi2c1;
extern HAL_StatusTypeDef status;
extern uint8_t u8TimerI2C_counter;
extern uint16_t slaveADDR;
extern uint8_t * RxData;
extern uI2CMainControls ui2cControl;
extern uint8_t i2cRxSize;



//header func

//func..

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  //Check timer
  if(htim == &htim6){
    switch(u8TimerI2C_counter){
    case 0:
      //SlidePot
      RxData = &ui2cControl.au8I2CMainByteAccess[0];
      slaveADDR = SLIDEPOT_ADDR;
      i2cRxSize = SLIDEPOT_RXSIZE;
      //Plugpot
      //RxData = &ui2cControl.au8I2CMainByteAccess[0];
      //slaveADDR = PLUGPOT_ADDR;
      //i2cRxSize = PLUGPOT_RXSIZE;
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
    status = HAL_I2C_Master_Receive(&hi2c1, slaveADDR, RxData, i2cRxSize, 1000);
    if(u8TimerI2C_counter >= 2 | u8TimerI2C_counter < 0){
    //if(u8TimerI2C_counter >= 1 | u8TimerI2C_counter < 0){
      u8TimerI2C_counter = 0;
    }
    else{
      u8TimerI2C_counter++;
    }
    //u8TimerI2C_counter = 0;
  }
}





*/
