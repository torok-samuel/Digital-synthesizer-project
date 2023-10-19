#ifndef __DIGITALSYNTH_SLIDE_BUTT_H
#define __DIGITALSYNTH_SLIDE_BUTT_H

#include <stdint.h>
#include <stdbool.h>


//func
void getNewState();
void setButtonState();
void refreshOldState();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* __DIGITALSYNTH_SLIDE_I2C_H */