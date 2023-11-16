#ifndef __DIGITALSYNTH_DSP_H
#define __DIGITALSYNTH_DSP_H

#include <stdint.h>

uint32_t endianness32b(uint32_t i32AudioData);

void adsr();

//DDS
void singen_def(uint32_t u32AudioAmp);
void dsp2(uint8_t u8Range, uint8_t u8PButton, uint8_t u8Waveform, uint32_t u32AudioAmp, int32_t* arri32AudioBuffer);
void sqrgen_def(uint32_t u32AudioAmp);
void sawgen_def(uint32_t u32AudioAmp);
void trigen_def(uint32_t u32AudioAmp);

//BLIST
void dsp_blist(uint8_t u8Range, uint8_t u8PButton, uint8_t u8Waveform, uint32_t u32AudioAmp, int32_t* arri32AudioBuffer);
void blistbspline(float fd, uint32_t u32AudioAmp);
void biblistbspline(float fdbi, uint8_t u8Flagbi, uint32_t u32AudioAmp);
void leakyintegrator(int32_t* xn, int32_t* yn, float E, int32_t n1stelement);
void dcoff(float fd, int32_t* yndcoff, int32_t* ynin, uint32_t u32AudioAmp);

#endif /* __DIGITALSYNTH_MAIN_I2C_H */