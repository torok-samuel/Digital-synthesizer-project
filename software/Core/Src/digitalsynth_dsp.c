//description

//include
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
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
#define U32MAX 4,294,967,295
#define U8MAX 255

enum ADSR_STATE{
  NONE,
  ATTACK,
  DECAY,
  SUSTAIN,
  RELEASE
};

struct KEY_STATE{
  enum ADSR_STATE state;
  uint32_t u32AttackTime;
  uint32_t u32DecayTime;
  float fSustainLevel;
  uint32_t u32ReleaseTime;
  bool u8AdsrKeyPressed_Old;
  uint32_t u32AdsrTime;
  float fAmpl;
};


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
float fd;
float fDdsNum = 0;

//ADSR
enum ADSR_STATE ADSR_STATE = NONE;
struct KEY_STATE testkey = {
  .state = NONE,
  .u8AdsrKeyPressed_Old = 0,
  .fAmpl = 0,
  .u32AttackTime = 1000,
  .u32DecayTime = 1000,
  .fSustainLevel = 0.5,
  .u32ReleaseTime = 1000
};
//bool au8AdsrKeyPressed_Old[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//uint32_t u32AdsrTime;

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
  
  fd = (fFreq * WAVEFOMRNUM) / (float)I2S_FREQ;
  //fd = 4;

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
  int idds;
  adsr();
  for(u16Index = 0; u16Index < WAVEFOMRNUM; u16Index++){

    
    
    idds = (int)floorf( fDdsNum );

    i32AudioData = (int32_t)floorf( (float)arri32AudioBuffer22[idds] * (float)testkey.fAmpl);
    
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
    
    fDdsNum += fd;
    if(fDdsNum >= WAVEFOMRNUM)
      fDdsNum = fDdsNum - WAVEFOMRNUM;
  }


  return;
}


void adsr(){
  
  
  uint8_t u8AdsrKeyPressed = ui2cControl.sI2CMainControl.u1Keys9;
  
  //uint32_t u32AdsrLastTime;
  
  

  switch(testkey.state){
    case NONE:
      if( u8AdsrKeyPressed == 1){
        testkey.u32AdsrTime = HAL_GetTick();
        testkey.state = ATTACK;
        break;
      }
      testkey.fAmpl = 0;
      break;
    case ATTACK:
      if(HAL_GetTick() > (testkey.u32AttackTime + testkey.u32AdsrTime ) ){
        testkey.u32AdsrTime = HAL_GetTick();
        testkey.state = DECAY;
        break;
      }
      testkey.fAmpl = sqrtf((float)(HAL_GetTick() - testkey.u32AdsrTime))/sqrtf((float)testkey.u32DecayTime);
      break;
    case DECAY:
      if(HAL_GetTick() > (testkey.u32DecayTime + testkey.u32AdsrTime) ){
        testkey.u32AdsrTime = HAL_GetTick();
        testkey.state = SUSTAIN;
        break;
      }
      testkey.fAmpl = (1 - testkey.fSustainLevel) * powf(testkey.u32DecayTime - (HAL_GetTick() - testkey.u32AdsrTime), 2)/powf(testkey.u32DecayTime, 2) + testkey.fSustainLevel;
      break;
    case SUSTAIN:
      if( u8AdsrKeyPressed == 0 ){
        testkey.u32AdsrTime = HAL_GetTick();
        testkey.state = RELEASE;
        break;
      }   
      testkey.fAmpl = testkey.fSustainLevel;
      break;
    case RELEASE:
      if(HAL_GetTick() > (testkey.u32ReleaseTime + testkey.u32AdsrTime) )
        testkey.state = NONE;
      testkey.fAmpl = testkey.fSustainLevel * powf(testkey.u32ReleaseTime - (HAL_GetTick() - testkey.u32AdsrTime), 2)/powf(testkey.u32ReleaseTime, 2);
      if(testkey.fAmpl > testkey.fSustainLevel)
        testkey.fAmpl = 0;
      break;  
    default:
      testkey.state = NONE;
      testkey.fAmpl = 0;
  }
  
  //testkey.u8AdsrKeyPressed_Old = u8AdsrKeyPressed;
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









