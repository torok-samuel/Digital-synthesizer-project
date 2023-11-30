#ifndef __DIGITALSYNTH_PLUG_I2C_H
#define __DIGITALSYNTH_PLUG_I2C_H

#include "main.h"
#include <stdint.h>

//PlugPot header
typedef union{
  struct{
    //************
    //*PlugPot uC*
    //************
    uint8_t u8Plugpot[12];
    //DPDT Switches
    uint8_t u1SW1:1;
    uint8_t u1SW2:1;
    uint8_t u1SW3:1;
    uint8_t u1SW4:1;
    uint8_t u1SW5:1;
    uint8_t u1SW6:1;
    uint8_t u1SW7:1;
    uint8_t u1SW8:1;
    //end of SW
  }sI2CPlugPotControl;
  uint8_t au8I2CPlugPotByteAccess[13];
}uI2CPlugPotControls;

/*
//func
void process_data();
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
*/


#endif /* __DIGITALSYNTH_PLUG_I2C_H */