//description

//include
#include <stdbool.h>
#include "main.h"
#include "digitalsynth_key_i2c.h"
#include "digitalsynth_keyit.h"





//define
#define SWITCHDELAY 30/1000*16000000    //30ms - num/ms/s

//variable
bool bStateButt1 = false;
extern uI2CKeyControls sCont;
uint32_t switchstart = 0;
//test
extern int sw1_test;

//func
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(HAL_GPIO_ReadPin(SW12_GPIO_Port, SW12_Pin) == GPIO_PIN_RESET & bStateButt1 == false){
    bStateButt1 = true;
    switchstart = HAL_GetTick();
    //for switch oscillating
    while ((HAL_GetTick() - switchstart) > SWITCHDELAY){}        
  }
  else if(bStateButt1 == true & HAL_GPIO_ReadPin(SW12_GPIO_Port, SW12_Pin) == GPIO_PIN_SET){
    bStateButt1 = false;
    switchstart = HAL_GetTick();
    //for switch oscillating
    while ((HAL_GetTick() - switchstart) > SWITCHDELAY){}
  }
  else{
  }
  __disable_irq();
  sCont.sI2CKeyControl.u1Keys9 = bStateButt1;
  __enable_irq();
}