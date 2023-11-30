#ifndef __DIGITALSYNTH_DSP_H
#define __DIGITALSYNTH_DSP_H

#include <stdint.h>

uint32_t endianness32b(uint32_t i32AudioData);

float lfo(uint32_t u32AudioAmp);
void adsr();

//DDS
void singen_def(uint32_t u32AudioAmp);
void dsp2(uint8_t u8Range, uint8_t u8PButton, uint8_t u8Waveform, uint32_t u32AudioAmp, int32_t* arri32AudioBuffer);
void sqrgen_def(uint32_t u32AudioAmp);
void sawgen_def(uint32_t u32AudioAmp);
void trigen_def(uint32_t u32AudioAmp);

//BLIST
void dsp_blitbspline(uint8_t u8Range, uint8_t u8PButton, uint8_t u8Waveform, uint32_t u32AudioAmp, int32_t* arri32AudioBuffer);
void blistbspline(float fd, uint32_t u32AudioAmp);
void biblistbspline(float fd, uint32_t u32AudioAmp);
float leakyintegrator(float* xn, float* yn, float i32n1stelement, float E);
void dcoff(float fd, float* yndcoff, float* ynin, uint32_t u32AudioAmp);
void dds(int32_t* arri32AudioBuffer);

#endif /* __DIGITALSYNTH_MAIN_I2C_H */