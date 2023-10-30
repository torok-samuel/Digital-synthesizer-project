//desciption

//include
#include "main.h"
#include "digitalsynth_slide_i2c.h"
#include "digitalsynth_slide_butt.h"
#include "stm32g030xx.h"

#define ROWSIZE 5
#define COLSIZE 5

// Button state
bool bButtonStateCurr[ ROWSIZE ][ COLSIZE ] = {{false, false, false, false, false}, 
                                               {false, false, false, false, false},
                                               {false, false, false, false, false}, 
                                               {false, false, false, false, false}, 
                                               {false, false, false, false, false}};
bool bButtonStateOld[ ROWSIZE ][ COLSIZE ] = {{false, false, false, false, false}, 
                                              {false, false, false, false, false},
                                              {false, false, false, false, false}, 
                                              {false, false, false, false, false}, 
                                              {false, false, false, false, false}};

extern uI2CSlidePotControls sCont;


//Row pin
uint16_t u16RowPin[ROWSIZE] = {R1_Pin, R2_Pin, R3_Pin, R4_Pin, R5_Pin};
//Row port
GPIO_TypeDef* gpioRowPort[ROWSIZE] = {R1_GPIO_Port, R2_GPIO_Port, R3_GPIO_Port, R4_GPIO_Port, R5_GPIO_Port};
//Col pin
uint16_t u16ColPin[COLSIZE] = {C1_Pin, C2_Pin, C3_Pin, C4_Pin, C5_Pin}; 
//Col port
GPIO_TypeDef* gpioColPort[COLSIZE] = {C1_GPIO_Port, C2_GPIO_Port, C3_GPIO_Port, C4_GPIO_Port, C5_GPIO_Port};

void getNewState(){
  for(int i = 0; i < COLSIZE; i++)
  {
    HAL_GPIO_WritePin(gpioColPort[i], u16ColPin[i], GPIO_PIN_RESET);
    for( int j = 0; j < ROWSIZE; j++ )
    {
        bButtonStateCurr[ i ][ j ] = ( HAL_GPIO_ReadPin(gpioRowPort[j], u16RowPin[j]) == GPIO_PIN_RESET ) ? true : false;
    }
    HAL_GPIO_WritePin(gpioColPort[i], u16ColPin[i], GPIO_PIN_SET);
  }
}


void setButtonState(){
  for(int i = 0; i < COLSIZE; i++)
  {
    for( int j = 0; j < ROWSIZE; j++ )
    {
      if( (true == bButtonStateCurr[i][j]) && (false == bButtonStateOld[i][j]) )
      {
        //BUTTON PUSHED
        if(i == 0 & j == 0)
          sCont.sI2CSlidePotControl.u2ButtWaveform1++;
      }
      else if( (false == bButtonStateCurr[i][j]) && (true == bButtonStateOld[i][j]) )
      {
        //BUTTON RELEASED
      }
    }
  }
}


void refreshOldState(){
  for(int i = 0; i < COLSIZE; i++)
  {
          for( int j = 0; j < ROWSIZE; j++ )
          {
              bButtonStateOld[ i ][ j ] = bButtonStateCurr[ i ][ j ];
          }
  }
}

//func
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  getNewState();
  setButtonState();
  refreshOldState();
  //__disable_irq();
  //write pointer
  //__enable_irq();
}