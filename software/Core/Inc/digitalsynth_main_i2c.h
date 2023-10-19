#ifndef __DIGITALSYNTH_MAIN_I2C_H
#define __DIGITALSYNTH_MAIN_I2C_H

#include <stdint.h>

//main func header
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
		
		//************
		//*PlugPot uC*
		//************
		uint8_t Plugpot[12];
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
		
		//*************
		//*Keyboard uC*
		//*************
		//Keys 1st byte
                uint8_t u8TestKey1;
		//uint8_t u1Keys1:1;
		//uint8_t u1Keys2:1;
		//uint8_t u1Keys3:1;
		//uint8_t u1Keys4:1;
		//uint8_t u1Keys5:1;
		//uint8_t u1Keys6:1;
		//uint8_t u1Keys7:1;
		//uint8_t u1Keys8:1;
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
	}sI2CMainControl;
	uint8_t au8I2CMainByteAccess[27];
}uI2CMainControls;


/*
void i2c_datatype_testing(uI2CMainControls ui2cControl);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
*/

void reset_i2c(I2C_HandleTypeDef *hi2c);

#endif /* __DIGITALSYNTH_MAIN_I2C_H */