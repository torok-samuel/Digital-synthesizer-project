#ifndef __DIGITALSYNTH_SLIDE_I2C_H
#define __DIGITALSYNTH_SLIDE_I2C_H

#include <stdint.h>

//SlidePot header
typedef union{
	struct{
		//*************
		//*SlidePot uC*
		//*************
		uint8_t u8SlidePot[8];
		//Button 1st byte
		uint8_t u2ButtWaveform1:2;
		//uint8_t u1Button2:1;
		uint8_t u1Button3:1;
		uint8_t u1Button4:1;
		uint8_t u1Button5:1;
		uint8_t u1Button6:1;
		uint8_t u1Button7:1;
		uint8_t u1Button8:1;
		//Button 2nd byte
		uint8_t u1Button9:1;
		uint8_t u1Button10:1;
		uint8_t u1Button11:1;
		uint8_t u1Button12:1;
		uint8_t u1Button13:1;
		uint8_t u1Button14:1;
		uint8_t u1Button15:1;
		uint8_t u1Button16:1;
		//Button 3rd byte
		uint8_t u1Button17:1;
		uint8_t u1Button18:1;
		uint8_t u1Button19:1;
		uint8_t u1Button20:1;
		uint8_t u1Button21:1;
		uint8_t u1Button22:1;
		uint8_t u1Button23:1;
		uint8_t u1Button24:1;
		//Button 4th byte
		uint8_t u1Button25:1;
		uint8_t u1ButtonZ1:1;	//Zero bit
		uint8_t u1ButtonZ2:1;	//Zero bit
		uint8_t u1ButtonZ3:1;	//Zero bit
		uint8_t u1ButtonZ4:1;	//Zero bit
		uint8_t u1ButtonZ5:1;	//Zero bit
		uint8_t u1ButtonZ6:1;	//Zero bit
		uint8_t u1ButtonZ7:1;	//Zero bit
		//end of Buttons struct
	}sI2CSlidePotControl;
	uint8_t au8I2CSlidePotByteAccess[12];
}uI2CSlidePotControls;

/*
//func
void process_data();
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
*/


#endif /* __DIGITALSYNTH_SLIDE_I2C_H */