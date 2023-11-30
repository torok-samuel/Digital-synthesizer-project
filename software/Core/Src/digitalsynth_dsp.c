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
//BLIST B-Slave
float fBlistDelay;
float fBlistIndex = 2;
float arri32BlistBuffer[512];
float arri32BiBlistBuffer[512];
float arri32BlistBufferDC[512];
float arri32BlistBufferOut[512];
float arri32BlistBufferOut2[512];
int32_t u32AudioAmpBlist = 0x7FFFFFFF;
float fDStep;
int32_t i32BlistIndexFloor;
float fBlistNum;
float i32n1stelement = -0.5;
float i32n1stelement1 = -0.5;
float i32n1stelement2 = -0.5;
int32_t ii;
int8_t u8Flagbi = 1;

float arri32BlistBuffernext[3];
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

//lfo
float fLFOFreq = 0;
float fLFOFd = 0;
float fLfoNum = 0;
int iLfoNum;
float fLfoRes;
float fLfoResult;
float fLfoRate;
float fLFORateFd;
//func



uint32_t endianness32b(uint32_t i32AudioData){
  uint32_t u32AudioDatah = i32AudioData & AUDIO_HIGH_MASK;
  uint32_t u32AudioDatal = i32AudioData & AUDIO_LOW_MASK;
  uint32_t i32NewAudioData = (u32AudioDatah>>16) | (u32AudioDatal<<16);
  return i32NewAudioData;
}

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
  //get the frequency
  //float fFreq = arrfFreqs[u8Range][ui2cControl.sI2CMainControl.u8TestKey1];
  float fFreq = 2217.46;
  fd = (fFreq * WAVEFOMRNUM) / (float)I2S_FREQ;
  float fdBlist = (float)I2S_FREQ / fFreq;
  float fE=0.01f;
  //uint16_t u16Index;
  
  //ui2cControl.sI2CMainControl.u2ButtWaveform1 = 2;

  switch(ui2cControl.sI2CMainControl.u2ButtWaveform1){
    //dds sin
    case 0:
      arri32AudioBuffer22 = arri32AudioBuffer22sin;
      dds(arri32AudioBuffer);
      break;
    //dds tri
    case 1:
      arri32AudioBuffer22 = arri32AudioBuffer22tri;
      dds(arri32AudioBuffer);
      break;
    //dds saw
    case 3:
      arri32AudioBuffer22 = arri32AudioBuffer22saw;
      dds(arri32AudioBuffer);
      break;
    //dds sqr
    case 5:
      arri32AudioBuffer22 = arri32AudioBuffer22sqr;
      dds(arri32AudioBuffer);
      break;
    //blit saw
    case 4:
      blistbspline(fdBlist, u32AudioAmp);
      dcoff(fdBlist, arri32BlistBufferDC, arri32BlistBuffer, u32AudioAmp);
      i32n1stelement = leakyintegrator(arri32BlistBufferDC, arri32BlistBufferOut, i32n1stelement, fE);
      uint16_t u16Index;
      for(u16Index = 0; u16Index < WAVEFOMRNUM; u16Index++){
        //i32AudioData = (int32_t)floorf( (float)arri32BlistBufferOut[u16Index] * (float)testkey.fAmpl);
        i32AudioData = (int32_t)floorf( arri32BlistBufferOut[u16Index] * (float)u32AudioAmp);
        i32AudioData = endianness32b(i32AudioData);
        //write buffer
        arri32AudioBuffer[ u16Index*2 + 0 ] = i32AudioData;
        arri32AudioBuffer[ u16Index*2 + 1 ] = i32AudioData;
      }
      break;
    //blit sqr
    case 6:
      biblistbspline(fdBlist/2, u32AudioAmp);
      i32n1stelement = leakyintegrator(arri32BiBlistBuffer, arri32BlistBufferOut, i32n1stelement, fE);
      for(u16Index = 0; u16Index < WAVEFOMRNUM; u16Index++){
        //i32AudioData = (int32_t)floorf( (float)arri32BlistBufferOut[u16Index] * (float)testkey.fAmpl);
        i32AudioData = (int32_t)floorf( arri32BlistBufferOut[u16Index] * (float)u32AudioAmp);
        i32AudioData = endianness32b(i32AudioData);
        //write buffer
        arri32AudioBuffer[ u16Index*2 + 0 ] = i32AudioData;
        arri32AudioBuffer[ u16Index*2 + 1 ] = i32AudioData;
      }
      break;
    //blit tri
    case 2:
      biblistbspline(fdBlist/2, u32AudioAmp);
      i32n1stelement = leakyintegrator(arri32BiBlistBuffer, arri32BlistBufferOut2, i32n1stelement, fE);  
      i32n1stelement2 = leakyintegrator(arri32BlistBufferOut2, arri32BlistBufferOut, i32n1stelement2, fE);
      for(u16Index = 0; u16Index < WAVEFOMRNUM; u16Index++){
        //i32AudioData = (int32_t)floorf( (float)arri32BlistBufferOut[u16Index] * (float)testkey.fAmpl);
        i32AudioData = (int32_t)floorf( arri32BlistBufferOut[u16Index] * (float)u32AudioAmp);
        i32AudioData = endianness32b(i32AudioData);
        //write buffer
        arri32AudioBuffer[ u16Index*2 + 0 ] = i32AudioData;
        arri32AudioBuffer[ u16Index*2 + 1 ] = i32AudioData;
      }
      break;
    //who knows
    default:
      arri32AudioBuffer22 = arri32AudioBuffer22sin;
      dds(arri32AudioBuffer);
      break;
  }  


  return;
}

void dds(int32_t* arri32AudioBuffer){
  int idds;
  //adsr();
  //fLfoResult = lfo(u32AudioAmp);
  for(u16Index = 0; u16Index < WAVEFOMRNUM; u16Index++){
    idds = (int)floorf( fDdsNum );
    //i32AudioData = (int32_t)floorf( (float)arri32AudioBuffer22[idds] * (float)testkey.fAmpl);
    i32AudioData = arri32AudioBuffer22[idds];
    i32AudioData = endianness32b(i32AudioData);
    //write buffer
    arri32AudioBuffer[ u16Index*2 + 0 ] = i32AudioData;
    arri32AudioBuffer[ u16Index*2 + 1 ] = i32AudioData;
    //fLfoResult = lfo(u32AudioAmp);
    //fDdsNum += (fd + fLfoResult);
    fDdsNum += fd;
    if(fDdsNum >= WAVEFOMRNUM)
      fDdsNum = fDdsNum - WAVEFOMRNUM;
  }
}

void adsr(){
  
  
  uint8_t u8AdsrKeyPressed = ui2cControl.sI2CMainControl.u1Keys9;
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
}


float lfo(uint32_t u32AudioAmp){
  //0-20Hz
  fLFOFreq = 10;
  fLfoRate = 25; 
  fLFOFd = (fLFOFreq * WAVEFOMRNUM) / (float)I2S_FREQ;
  fLFORateFd = (fLfoRate * WAVEFOMRNUM) / (float)I2S_FREQ;
  
  
  iLfoNum = (int)floorf( fLfoNum );
  fLfoRes = (float)arri32AudioBuffer22[iLfoNum] * fLFORateFd / u32AudioAmp;
  
  fLfoNum += (fLFOFd* WAVEFOMRNUM);
  if(fLfoNum >= WAVEFOMRNUM)
    fLfoNum = fLfoNum - WAVEFOMRNUM;
  
  return fLfoRes;
  
}




//switch statement
//fBlistDelay = (float)(I2S_FREQ / fFreq);
//fd = (fFreq * WAVEFOMRNUM) / (float)I2S_FREQ;
void dsp_blitbspline(uint8_t u8Range, uint8_t u8PButton, uint8_t u8Waveform, uint32_t u32AudioAmp, int32_t* arri32AudioBuffer){
  //float fFreq = arrfFreqs[u8Range][ui2cControl.sI2CMainControl.u8TestKey1];
  float fFreq = 2217.46;
  float fdBlist = (float)I2S_FREQ / fFreq;
  
  float fE=0.01f;


  switch(ui2cControl.sI2CMainControl.u2ButtWaveform1){
    //saw
    case 0:
      blistbspline(fdBlist, u32AudioAmp);
      dcoff(fdBlist, arri32BlistBufferDC, arri32BlistBuffer, u32AudioAmp);
      i32n1stelement = leakyintegrator(arri32BlistBufferDC, arri32BlistBufferOut, i32n1stelement, fE);
      break;
    //sqr
    case 1:
      biblistbspline(fdBlist, u32AudioAmp);
      i32n1stelement = leakyintegrator(arri32BiBlistBuffer, arri32BlistBufferOut, i32n1stelement, fE);
      break;
    //tri
    case 2:
      blistbspline(fdBlist, u32AudioAmp);
      dcoff(fdBlist, arri32BlistBufferDC, arri32BlistBuffer, u32AudioAmp);      
      i32n1stelement = leakyintegrator(arri32BlistBufferDC, arri32BlistBufferOut,i32n1stelement, fE);
      break;
    //notthingham
    case 3:
      for (ii = 0; ii < WAVEFOMRNUM; ii++) {
        arri32BlistBufferOut[ii] = 0.0f;
      }  
      break;
    //who knows
    default:
      break;
  }
  

  //SAWTOOTH
  
  
  
  //SQR
  //biblistbspline(fdBlist, u32AudioAmp);
  //float fEsqr=0.01f;
  //leakyintegrator(arri32BiBlistBuffer, arri32BlistBufferOut, fEsqr);  
  
  //TRI
  //biblistbspline(fdBlist, u32AudioAmp);
  //float fEtri=0.01f;
  //leakyintegrator(arri32BiBlistBuffer, arri32BlistBufferOut2, fEtri);  
  //leakyintegrator(arri32BlistBufferOut2, arri32BlistBufferOut, fEtri); 
  
  //uint16_t u16Index;
  //for(u16Index = 0; u16Index < WAVEFOMRNUM; u16Index++){
  //  arri32BlistBufferOut[u16Index] = (float) (arri32BlistBufferOut[u16Index] - 0.5f);
  //  if (arri32BlistBufferOut[u16Index] > 0.5f)
  //    arri32BlistBufferOut[u16Index] = 0.5f;
  //  else if (arri32BlistBufferOut[u16Index] < -0.5f)
  //    arri32BlistBufferOut[u16Index] = -0.5f;
  //}
  //adsr();
  
  //Output
  uint16_t u16Index;
  for(u16Index = 0; u16Index < WAVEFOMRNUM; u16Index++){
    //i32AudioData = (int32_t)floorf( (float)arri32BlistBufferOut[u16Index] * (float)testkey.fAmpl);
    i32AudioData = (int32_t)floorf( arri32BlistBufferOut[u16Index] * (float)u32AudioAmp);
    i32AudioData = endianness32b(i32AudioData);
    //write buffer
    arri32AudioBuffer[ u16Index*2 + 0 ] = i32AudioData;
    arri32AudioBuffer[ u16Index*2 + 1 ] = i32AudioData;
  }

}


void blistbspline(float fd, uint32_t u32AudioAmp){
  for (ii = 0; ii < WAVEFOMRNUM; ii++) {
    arri32BlistBuffer[ii] = 0.0f;
  }
  //WAVEFOMRNUM???
  //add previous values
  arri32BlistBuffer[0] = arri32BlistBuffernext[0];
  arri32BlistBuffer[1] = arri32BlistBuffernext[1];
  arri32BlistBuffer[2] = arri32BlistBuffernext[2];
  //set to null
  arri32BlistBuffernext[0] = 0.0f;
  arri32BlistBuffernext[1] = 0.0f;
  arri32BlistBuffernext[2] = 0.0f;
  while (fBlistIndex < (WAVEFOMRNUM + 2)) {
    i32BlistIndexFloor = (int)floorf(fBlistIndex);
    //0 <= t/Ts < 1
    fBlistNum = fBlistIndex - i32BlistIndexFloor;  
    //for the next sample buffer
    if (i32BlistIndexFloor == (511 + 0)) {
      //+1
      arri32BlistBuffernext[0] = 0.16666667f * (2.0f - (fBlistNum + 1.0f)) * (2.0f - (fBlistNum + 1.0f)) * (2.0f - (fBlistNum + 1.0f));
    }
    else if (i32BlistIndexFloor == (511 + 1)) {
      //+0
      arri32BlistBuffernext[0] = 0.66666667f - (fBlistNum + 0.0f) * (fBlistNum + 0.0f) + 0.5f * (fBlistNum + 0.0f) * (fBlistNum + 0.0f) * (fBlistNum + 0.0f);
      //+1
      arri32BlistBuffernext[1] = 0.16666667f * (2.0f - (fBlistNum + 1.0f)) * (2.0f - (fBlistNum + 1.0f)) * (2.0f - (fBlistNum + 1.0f));
    }
    else if (i32BlistIndexFloor == (511 + 2)) {
      //-1
      arri32BlistBuffernext[0] = 0.66666667f - (fBlistNum - 1.0f) * (fBlistNum - 1.0f) - 0.5f * (fBlistNum - 1.0f) * (fBlistNum - 1.0f) * (fBlistNum - 1.0f);
      //+0
      arri32BlistBuffernext[1] = 0.66666667f - (fBlistNum + 0.0f) * (fBlistNum + 0.0f) + 0.5f * (fBlistNum + 0.0f) * (fBlistNum + 0.0f) * (fBlistNum + 0.0f);
      //+1
      arri32BlistBuffernext[2] = 0.16666667f * (2.0f - (fBlistNum + 1.0f)) * (2.0f - (fBlistNum + 1.0f)) * (2.0f - (fBlistNum + 1.0f));
    }  
    //bspline
    if (((i32BlistIndexFloor - 2) >= 0) & ((i32BlistIndexFloor - 2) < WAVEFOMRNUM))
      arri32BlistBuffer[i32BlistIndexFloor - 2] += 0.16666667f * (2.0f + (fBlistNum - 2.0f)) * (2.0f + (fBlistNum - 2.0f)) * (2.0f + (fBlistNum - 2.0f));
    if (((i32BlistIndexFloor - 1) >= 0) & ((i32BlistIndexFloor - 1) < WAVEFOMRNUM))
      arri32BlistBuffer[i32BlistIndexFloor - 1] += 0.66666667f - (fBlistNum - 1.0f) * (fBlistNum - 1.0f) - 0.5f * (fBlistNum - 1.0f) * (fBlistNum - 1.0f) * (fBlistNum - 1.0f);
    if (((i32BlistIndexFloor + 0) >= 0) & ((i32BlistIndexFloor - 0) < WAVEFOMRNUM))
      arri32BlistBuffer[i32BlistIndexFloor - 0] += 0.66666667f - (fBlistNum + 0.0f) * (fBlistNum + 0.0f) + 0.5f * (fBlistNum + 0.0f) * (fBlistNum + 0.0f) * (fBlistNum + 0.0f);
    if (((i32BlistIndexFloor + 1) >= 0) & ((i32BlistIndexFloor + 1) < WAVEFOMRNUM))
      arri32BlistBuffer[i32BlistIndexFloor + 1] += 0.16666667f * (2.0f - (fBlistNum + 1.0f)) * (2.0f - (fBlistNum + 1.0f)) * (2.0f - (fBlistNum + 1.0f));
    fBlistIndex += fd;
  }
  fBlistIndex -= WAVEFOMRNUM;
}

//u8Flagbi = -1 * u8Flagbi;
//void biblistbspline(float fdbi, uint32_t u32AudioAmp){
void biblistbspline(float fd, uint32_t u32AudioAmp){
  float fdbi = fd/2;
  for (ii = 0; ii < WAVEFOMRNUM; ii++) {
    arri32BiBlistBuffer[ii] = 0.0f;
  }
  //WAVEFOMRNUM???
  //add previous values
  arri32BiBlistBuffer[0] = arri32BlistBuffernext[0];
  arri32BiBlistBuffer[1] = arri32BlistBuffernext[1];
  arri32BiBlistBuffer[2] = arri32BlistBuffernext[2];
  //set to null
  arri32BlistBuffernext[0] = 0.0f;
  arri32BlistBuffernext[1] = 0.0f;
  arri32BlistBuffernext[2] = 0.0f;
  while (fBlistIndex < (WAVEFOMRNUM + 2)) {
    i32BlistIndexFloor = (int)floorf(fBlistIndex);
    //0 <= t/Ts < 1
    fBlistNum = fBlistIndex - i32BlistIndexFloor;  
    //for the next sample buffer
    if (i32BlistIndexFloor == (511 + 0)) {
      //+1
      arri32BlistBuffernext[0] = 0.16666667f * (2.0f - (fBlistNum + 1.0f)) * (2.0f - (fBlistNum + 1.0f)) * (2.0f - (fBlistNum + 1.0f)) * u8Flagbi;
    }
    else if (i32BlistIndexFloor == (511 + 1)) {
      //+0
      arri32BlistBuffernext[0] = (0.66666667f - (fBlistNum + 0.0f) * (fBlistNum + 0.0f) + 0.5f * (fBlistNum + 0.0f) * (fBlistNum + 0.0f) * (fBlistNum + 0.0f)) * u8Flagbi;
      //+1
      arri32BlistBuffernext[1] = 0.16666667f * (2.0f - (fBlistNum + 1.0f)) * (2.0f - (fBlistNum + 1.0f)) * (2.0f - (fBlistNum + 1.0f)) * u8Flagbi;
    }
    else if (i32BlistIndexFloor == (511 + 2)) {
      //-1
      arri32BlistBuffernext[0] = (0.66666667f - (fBlistNum - 1.0f) * (fBlistNum - 1.0f) - 0.5f * (fBlistNum - 1.0f) * (fBlistNum - 1.0f) * (fBlistNum - 1.0f)) * u8Flagbi;
      //+0
      arri32BlistBuffernext[1] = (0.66666667f - (fBlistNum + 0.0f) * (fBlistNum + 0.0f) + 0.5f * (fBlistNum + 0.0f) * (fBlistNum + 0.0f) * (fBlistNum + 0.0f)) * u8Flagbi;
      //+1
      arri32BlistBuffernext[2] = (0.16666667f * (2.0f - (fBlistNum + 1.0f)) * (2.0f - (fBlistNum + 1.0f)) * (2.0f - (fBlistNum + 1.0f))) * u8Flagbi;
    }  
    //bspline
    if (((i32BlistIndexFloor - 2) >= 0) & ((i32BlistIndexFloor - 2) < WAVEFOMRNUM))
      arri32BiBlistBuffer[i32BlistIndexFloor - 2] += (0.16666667f * (2.0f + (fBlistNum - 2.0f)) * (2.0f + (fBlistNum - 2.0f)) * (2.0f + (fBlistNum - 2.0f))) * u8Flagbi;
    if (((i32BlistIndexFloor - 1) >= 0) & ((i32BlistIndexFloor - 1) < WAVEFOMRNUM))
      arri32BiBlistBuffer[i32BlistIndexFloor - 1] += (0.66666667f - (fBlistNum - 1.0f) * (fBlistNum - 1.0f) - 0.5f * (fBlistNum - 1.0f) * (fBlistNum - 1.0f) * (fBlistNum - 1.0f)) * u8Flagbi;
    if (((i32BlistIndexFloor + 0) >= 0) & ((i32BlistIndexFloor - 0) < WAVEFOMRNUM))
      arri32BiBlistBuffer[i32BlistIndexFloor - 0] += (0.66666667f - (fBlistNum + 0.0f) * (fBlistNum + 0.0f) + 0.5f * (fBlistNum + 0.0f) * (fBlistNum + 0.0f) * (fBlistNum + 0.0f)) * u8Flagbi;
    if (((i32BlistIndexFloor + 1) >= 0) & ((i32BlistIndexFloor + 1) < WAVEFOMRNUM))
      arri32BiBlistBuffer[i32BlistIndexFloor + 1] += (0.16666667f * (2.0f - (fBlistNum + 1.0f)) * (2.0f - (fBlistNum + 1.0f)) * (2.0f - (fBlistNum + 1.0f))) * u8Flagbi;
    fBlistIndex += fdbi;
    u8Flagbi = -1 * u8Flagbi;
  }
  fBlistIndex -= WAVEFOMRNUM;
}


//y(n) = x(n) + (1-E) * y(n-1)
float leakyintegrator(float* xn, float* yn, float i32n1stelement, float E){
  uint16_t u16LeakyIndex;
    float fE1 = (1.0f - E);
    for (u16LeakyIndex = 0; u16LeakyIndex < WAVEFOMRNUM; u16LeakyIndex++) {
      if (u16LeakyIndex == 0)
        yn[u16LeakyIndex] = xn[u16LeakyIndex] + fE1 * i32n1stelement;
      else
        yn[u16LeakyIndex] = xn[u16LeakyIndex] + fE1 * yn[u16LeakyIndex - 1];      
      if (u16LeakyIndex == (WAVEFOMRNUM - 1))
        i32n1stelement = yn[u16LeakyIndex];
    }
    return i32n1stelement;
}

void dcoff(float fd, float* yndcoff, float* ynin, uint32_t u32AudioAmp){
  uint16_t u16Index;
  float fDcoffFd = 1.0f / fd;
  for(u16Index = 0; u16Index < WAVEFOMRNUM; u16Index++)
    yndcoff[u16Index] = ynin[u16Index] - fDcoffFd;
}