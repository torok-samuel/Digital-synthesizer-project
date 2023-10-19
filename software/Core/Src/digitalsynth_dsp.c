//description

//include
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "digitalsynth_main_i2c.h"
#include "digitalsynth_dsp.h"
#include "samples.h"



//defines
#define I2S_FREQ 48000
#define AUDIO_HIGH_MASK 0xFFFF0000
#define AUDIO_LOW_MASK 0x0000FFFF
#define PI 3.14159
#define SQR_WIDTH 0.5
#define WAVEFOMRNUM 512

//variables
extern uint16_t u16WaveformNum;
//extern int32_t* arri32AudioBuffer;
uint32_t i32AudioData = 0;
uint16_t u16Index = 0;
extern uI2CMainControls ui2cControl;
int32_t* arri32AudioBuffer22;
int32_t arri32AudioBuffer22sin[512];
int32_t arri32AudioBuffer22sqr[512];
int32_t arri32AudioBuffer22saw[512];
int32_t arri32AudioBuffer22tri[512];
double dq;
double dDdsNum = 0;

//test
int32_t i32AudioDatatest[10000];
int test = 0;



//func



uint32_t endianness32b(uint32_t i32AudioData){
  uint32_t u32AudioDatah = i32AudioData & AUDIO_HIGH_MASK;
  uint32_t u32AudioDatal = i32AudioData & AUDIO_LOW_MASK;
  uint32_t i32NewAudioData = (u32AudioDatah>>16) | (u32AudioDatal<<16);
  return i32NewAudioData;
}



/////*************

void singen_def(uint32_t u32AudioAmp){
  for(u16Index = 0; u16Index < WAVEFOMRNUM; u16Index++){
    //waveform generaion
    arri32AudioBuffer22sin[ u16Index] = (int32_t)( u32AudioAmp * sin( 2*PI*u16Index / WAVEFOMRNUM ));
  }
}

void sqrgen_def(uint32_t u32AudioAmp){
  for(u16Index = 0; u16Index < WAVEFOMRNUM; u16Index++){
    //waveform generaion
    if(u16Index <= (WAVEFOMRNUM*SQR_WIDTH))
      arri32AudioBuffer22sqr[ u16Index] = (int32_t)u32AudioAmp;
    else
      arri32AudioBuffer22sqr[ u16Index] = 0;
  }
}

void sawgen_def(uint32_t u32AudioAmp){
  for(u16Index = 0; u16Index < WAVEFOMRNUM; u16Index++){
    //waveform generaion
    if(u16Index == (WAVEFOMRNUM-1))
      arri32AudioBuffer22saw[ u16Index] = 0;
    else
      arri32AudioBuffer22saw[ u16Index] = (int32_t)( u32AudioAmp * (u16Index / (float)(WAVEFOMRNUM) ));
  }
}

void trigen_def(uint32_t u32AudioAmp){
  for(u16Index = 0; u16Index < WAVEFOMRNUM; u16Index++){
    //waveform generaion
    if(u16Index <= (WAVEFOMRNUM/2))
      arri32AudioBuffer22tri[ u16Index] = (int32_t)( u32AudioAmp * ( u16Index / (float)(WAVEFOMRNUM/2) ));
    else
      arri32AudioBuffer22tri[ u16Index] = (int32_t)( u32AudioAmp * (1 - ( (u16Index-(WAVEFOMRNUM/2)) / (float)(WAVEFOMRNUM/2) )));
  }
}

void dsp2(uint8_t u8Range, uint8_t u8PButton, uint8_t u8Waveform, uint32_t u32AudioAmp, int32_t* arri32AudioBuffer){


  uint16_t u16Index = 0; 
  //get the frequency
  
  float fFreq = arrfFreqs[u8Range][ui2cControl.sI2CMainControl.u8TestKey1];
  //float fFreq = 187.5;
  //u16WaveformNum = (uint16_t)(I2S_FREQ / fFreq);
  //u16WaveformNum = 512;
  //singen2(u32AudioAmp);
  
  dq = (fFreq * WAVEFOMRNUM) / (float)I2S_FREQ;
  dq = 4;

  switch(ui2cControl.sI2CMainControl.u2ButtWaveform1){
    //sin
    case 0:
      arri32AudioBuffer22 = arri32AudioBuffer22sin;
      break;
    //tri
    case 1:
      arri32AudioBuffer22 = arri32AudioBuffer22tri;
      break;
    //saw
    case 2:
      arri32AudioBuffer22 = arri32AudioBuffer22saw;
      break;
    //sqr
    case 3:
      arri32AudioBuffer22 = arri32AudioBuffer22sqr;
      break;
    //who knows
    default:
      arri32AudioBuffer22 = arri32AudioBuffer22sin;
      break;
  }  

  //for debug
  int idds = (int)dDdsNum;
  for(u16Index = 0; u16Index < WAVEFOMRNUM; u16Index++){

    
    
    idds = (int)dDdsNum;

    i32AudioData = arri32AudioBuffer22[(int)dDdsNum];
    
    if(test< 10000){
      test++;
      i32AudioDatatest[test] = i32AudioData;
    }
    else{
      test++;
    }
  
    i32AudioData = endianness32b(i32AudioData);
    //write buffer
    arri32AudioBuffer[ u16Index*2 + 0 ] = i32AudioData;
    arri32AudioBuffer[ u16Index*2 + 1 ] = i32AudioData;
    
    dDdsNum += dq;
    if(dDdsNum >= WAVEFOMRNUM)
      dDdsNum = dDdsNum - WAVEFOMRNUM;
  }


  return;
}



/////*************



void singen(uint32_t u32AudioAmp, int32_t* arri32AudioBuffer){
  for(u16Index = 0; u16Index < u16WaveformNum; u16Index++){
    //waveform generaion
    i32AudioData = (int32_t)( u32AudioAmp * sin( 2*PI*u16Index / u16WaveformNum ));
    //endianness
    i32AudioData = endianness32b(i32AudioData);
    //write buffer
    arri32AudioBuffer[ u16Index*2 + 0 ] = i32AudioData;
    arri32AudioBuffer[ u16Index*2 + 1 ] = i32AudioData;
  }
}


void trigen(uint32_t u32AudioAmp, int32_t* arri32AudioBuffer){
  for(u16Index = 0; u16Index < u16WaveformNum; u16Index++){
    //waveform generaion
    if(u16Index <= (u16WaveformNum/2))
      i32AudioData = (int32_t)( u32AudioAmp * ( u16Index / (float)(u16WaveformNum/2) ));
    else
      i32AudioData = (int32_t)( u32AudioAmp * (1 - ( (u16Index-(u16WaveformNum/2)) / (float)(u16WaveformNum/2) )));
    //endianness
    i32AudioData = endianness32b(i32AudioData);
    //write buffer
    arri32AudioBuffer[ u16Index*2 + 0 ] = i32AudioData;
    arri32AudioBuffer[ u16Index*2 + 1 ] = i32AudioData;
  }
}

void sawgen(uint32_t u32AudioAmp, int32_t* arri32AudioBuffer){
  for(u16Index = 0; u16Index < u16WaveformNum; u16Index++){
    //waveform generaion
    if(u16Index == (u16WaveformNum-1))
      i32AudioData = 0;
    else
      i32AudioData = (int32_t)( u32AudioAmp * (u16Index / (float)(u16WaveformNum) ));
    //endianness
    i32AudioData = endianness32b(i32AudioData);
    //write buffer
    arri32AudioBuffer[ u16Index*2 + 0 ] = i32AudioData;
    arri32AudioBuffer[ u16Index*2 + 1 ] = i32AudioData;
  }
}

void sqrgen(uint32_t u32AudioAmp, int32_t* arri32AudioBuffer){
  for(u16Index = 0; u16Index < u16WaveformNum; u16Index++){
    //waveform generaion
    if(u16Index <= (u16WaveformNum*SQR_WIDTH))
      i32AudioData = (int32_t)u32AudioAmp;
    else
      i32AudioData = 0;
    //endianness
    i32AudioData = endianness32b(i32AudioData);
    //write buffer
    arri32AudioBuffer[ u16Index*2 + 0 ] = i32AudioData;
    arri32AudioBuffer[ u16Index*2 + 1 ] = i32AudioData;
  }
}


void dsp(uint8_t u8Range, uint8_t u8PButton, uint8_t u8Waveform, uint32_t u32AudioAmp, int32_t* arri32AudioBuffer){
  //get the frequency
  
  float fFreq = arrfFreqs[u8Range][ui2cControl.sI2CMainControl.u8TestKey1];
  //float fFreq = 187.5;
  //float fFreq = 93.75;
  u16WaveformNum = (uint16_t)(I2S_FREQ / fFreq);
    switch(ui2cControl.sI2CMainControl.u2ButtWaveform1){
    //sin
    case 0:
      singen(u32AudioAmp, arri32AudioBuffer);
      break;
    //tri
    case 1:
      trigen(u32AudioAmp, arri32AudioBuffer);
      break;
    //saw
    case 2:
      sawgen(u32AudioAmp, arri32AudioBuffer);
      break;
    //sqr
    case 3:
      sqrgen(u32AudioAmp, arri32AudioBuffer);
      break;
    //who knows
    default:
      memset(arri32AudioBuffer, 0, 10000);
      break;
  }

  return;
}









