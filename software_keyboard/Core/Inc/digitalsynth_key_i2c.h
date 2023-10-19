#ifndef __DIGITALSYNTH_KEY_I2C_H
#define __DIGITALSYNTH_KEY_I2C_H

#include "main.h"
#include <stdint.h>

//keyboard header
typedef union{
	struct{
		//*************
		//*Keyboard uC*
		//*************
		//Keys 1st byte
		uint8_t u1Keys1:1;
		uint8_t u1Keys2:1;
		uint8_t u1Keys3:1;
		uint8_t u1Keys4:1;
		uint8_t u1Keys5:1;
		uint8_t u1Keys6:1;
		uint8_t u1Keys7:1;
		uint8_t u1Keys8:1;
		//Keys 2nd byte
		uint8_t u1Keys9:1;
		uint8_t u1Keys10:1;
		uint8_t u1Keys11:1;
		uint8_t u1Keys12:1;
		uint8_t u1KeysZ1:1;	//Zero bit
		uint8_t u1KeysZ2:1;	//Zero bit
		uint8_t u1KeysZ3:1;	//Zero bit
		uint8_t u1KeysZ4:1;	//Zero bit
		//end of Keys
	}sI2CKeyControl;
	uint8_t au8I2CKeyByteAccess[2];
}uI2CKeyControls;


//func
/*
void process_data();
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
*/


#endif /* __DIGITALSYNTH_PLUG_I2C_H */